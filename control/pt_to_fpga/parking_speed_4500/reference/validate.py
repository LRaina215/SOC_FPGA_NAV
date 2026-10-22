"""Export the offline fixed-point assets, golden vectors, and numerical reports."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
import torch
from fixed_policy import FixedPolicy, LAYER_NAMES, SCALE, quantize, write_mem

ROOT = Path(__file__).resolve().parents[1]
EXPORTER = ROOT / 'export_policy.py'


def load_exporter():
    spec = importlib.util.spec_from_file_location('baseline_exporter', EXPORTER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def error_stats(actual, expected):
    delta = actual.astype(np.float64) - expected.astype(np.float64)
    return {'max_abs': float(np.max(np.abs(delta))), 'mean_abs': float(np.mean(np.abs(delta))),
            'rmse': float(np.sqrt(np.mean(delta * delta)))}


def float_layers(policy, obs, history):
    outputs = {}
    with torch.no_grad():
        x = torch.as_tensor(history, dtype=torch.float32)
        for prefix in ('encoder', 'actor'):
            modules = getattr(policy, prefix)
            if prefix == 'actor':
                latent = x
                x = torch.cat((torch.as_tensor(obs, dtype=torch.float32), latent), dim=-1)
            for index in range(0, len(modules), 2):
                x = modules[index](x)
                if index < len(modules) - 1:
                    x = modules[index + 1](x)
                outputs[f'{prefix}_{index}'] = x.numpy()
        return outputs, x.numpy(), torch.clamp(x, -1, 1).numpy(), latent.numpy()


def export_memories(fixed, assets, extraction):
    weights = np.concatenate([x.weight.reshape(-1) for x in fixed.layers])
    biases = np.concatenate([x.bias for x in fixed.layers])
    assert weights.size == 40704 and biases.size == 425
    write_mem(assets / 'weights.mem', weights, 16)
    write_mem(assets / 'bias.mem', biases, 32)
    write_mem(assets / 'elu.mem', fixed.elu_table, 24)
    layers, wo, bo = [], 0, 0
    with np.load(assets / 'inference_fp32.npz', allow_pickle=False) as data:
        for layer in fixed.layers:
            key = layer.name.replace('_', '.')
            layers.append({'name': layer.name, 'input_size': layer.weight.shape[1],
                           'output_size': layer.weight.shape[0], 'activation': layer.activation,
                           'weight_offset': wo, 'weight_count': layer.weight.size,
                           'bias_offset': bo, 'bias_count': layer.bias.size,
                           'expected_layer_output_offset': bo,
                           'weight_quantization_error': error_stats(layer.weight / SCALE, data[key + '.weight']),
                           'bias_quantization_error': error_stats(layer.bias / (SCALE * SCALE), data[key + '.bias'])})
            wo += layer.weight.size
            bo += layer.bias.size
    manifest = {
        'schema': 'full_center_v5.fixed_policy.v1', 'source_checkpoint': extraction['source_checkpoint'],
        'checkpoint_sha256': extraction['checkpoint_sha256'], 'extraction_report': 'extraction_report.json',
        'formats': {key: {'signed_bits': bits, 'fraction_bits': frac} for key, bits, frac in (
            ('input', 24, 12), ('activation', 24, 12), ('output', 24, 12), ('weight', 16, 12),
            ('bias', 32, 24), ('accumulator', 48, 24))},
        'quantization': {'parameters_and_float_inputs': 'nearest ties-even (numpy.rint)',
                         'parameter_saturation': 'reject overflow', 'input_saturation': 'signed24, reported flag/count',
                         'accumulator': 'exact dot plus bias; signed48 overflow rejected',
                         'requantization': 'shift12 nearest ties-away-from-zero, then signed24 saturation BEFORE ELU'},
        'elu': {'file': 'elu.mem', 'words': 3073, 'alpha': 1, 'nonnegative': 'identity',
                'negative_index': 'min(3072, (-x + 8) >> 4)',
                'table': 'np.rint((exp(-i/256)-1)*4096), i=0..3072; signed24 F12'},
        'memory_encoding': "lowercase fixed-width two's-complement hex; one word per line; no header",
        'weight_order': 'layer order below; row-major output neuron first, input index second',
        'memories': {'weights': {'file': 'weights.mem', 'words': 40704, 'hex_digits': 4},
                     'bias': {'file': 'bias.mem', 'words': 425, 'hex_digits': 8},
                     'elu': {'file': 'elu.mem', 'words': 3073, 'hex_digits': 6}},
        'interface': {'obs_size': 28, 'history_size': 140, 'history_order': 'oldest to newest, 5 x 28',
                      'actor_input': 'obs28 followed by latent3', 'obs_history_clipping': 'unchanged simulator-supplied values',
                      'raw_actions': '6 signed24 F12, no action clamp', 'clipped_actions': '6 signed24 F12 clipped +/-4096',
                      'latent': '3 signed24 F12, no action clamp', 'layer_outputs': 425},
        'layers': layers, 'vectors_manifest': 'vectors/manifest.json',
        'verification_scope': 'Offline software reference; not a hardware or closed-loop claim',
    }
    for entry in manifest['memories'].values():
        entry['sha256'] = hashlib.sha256((assets / entry['file']).read_bytes()).hexdigest()
    (assets / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest


def expected_saturation(fixed, obs_q, history_q, result):
    """Per-sample layer-saturation flags; input saturation and action clipping are excluded."""
    flags = np.zeros((obs_q.shape[0], 1), dtype=np.int64)
    if any(layer['pre_elu_saturation_count'] for layer in result.diagnostics['layers'].values()):
        for index, (obs, history) in enumerate(zip(obs_q, history_q)):
            single = fixed.infer_fixed(obs, history)
            flags[index, 0] = int(any(layer['pre_elu_saturation_count']
                                      for layer in single.diagnostics['layers'].values()))
    return flags


def validate_dataset(name, obs, history, fixed, floating):
    obs = np.asarray(obs, dtype=np.float32).reshape(-1, 28)
    history = np.asarray(history, dtype=np.float32).reshape(-1, 140)
    if not obs.shape[0] or obs.shape[0] != history.shape[0]:
        raise ValueError('Dataset must have matching nonempty obs/history batches')
    result = fixed.infer_float(obs, history)
    layers, raw, clipped, latent = float_layers(floating, obs, history)
    obs_q, _ = quantize(obs)
    history_q, _ = quantize(history)
    qlayers, qraw, qclipped, qlatent = float_layers(floating, obs_q / SCALE, history_q / SCALE)
    layer_report = {}
    for layer in LAYER_NAMES:
        value = result.layers[layer] / SCALE
        layer_report[layer] = {
            'float_range': [float(layers[layer].min()), float(layers[layer].max())],
            'fixed_dequantized_range': [float(value.min()), float(value.max())],
            'error_vs_original_float': error_stats(value, layers[layer]),
            'error_vs_float_on_quantized_inputs': error_stats(value, qlayers[layer]),
            **result.diagnostics['layers'][layer],
        }
    report = {'name': name, 'samples': obs.shape[0], 'layers': layer_report,
              'input_saturation': result.diagnostics['input_saturation'],
              'action_clip_count': result.diagnostics['action_clip_count'],
              'outputs': {key: {'error_vs_original_float': error_stats(value / SCALE, target),
                                 'error_vs_float_on_quantized_inputs': error_stats(value / SCALE, qtarget)}
                          for key, value, target, qtarget in (('raw', result.raw, raw, qraw),
                                                            ('clipped', result.clipped, clipped, qclipped),
                                                            ('latent', result.latent, latent, qlatent))}}
    arrays = {'obs': obs, 'history': history, 'obs_q': obs_q, 'history_q': history_q,
              'expected_layers': result.all_layers, 'expected_raw': result.raw,
              'expected_clipped': result.clipped, 'expected_latent': result.latent,
              'expected_saturated': expected_saturation(fixed, obs_q, history_q, result),
              'float_raw': raw, 'float_clipped': clipped, 'float_latent': latent}
    return report, arrays


def emit_vectors(directory, arrays, prefix=''):
    directory.mkdir(parents=True, exist_ok=True)
    np.savez(directory / f'{prefix}fixed_vectors.npz', **arrays)
    values = {'inputs': np.concatenate((arrays['obs_q'], arrays['history_q']), axis=-1),
              **{name: arrays[name] for name in ('expected_layers', 'expected_raw', 'expected_clipped', 'expected_latent', 'expected_saturated')}}
    files = {}
    for name, value in values.items():
        filename = f'{prefix}{name}.mem'
        write_mem(directory / filename, value, 24)
        files[name] = {'file': filename, 'words_per_sample': value.shape[1], 'words': value.size,
                       'sha256': hashlib.sha256((directory / filename).read_bytes()).hexdigest()}
        if name == 'expected_saturated':
            files[name].update(signed_bits=24, fraction_bits=0, hex_digits=6,
                               encoding='000000=false; 000001=true',
                               semantics='any pre-ELU signed24 saturation across seven layers; excludes input saturation and final action clipping')
    return {'samples': arrays['obs_q'].shape[0], 'files': files, 'npz': f'{prefix}fixed_vectors.npz'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets', type=Path, default=ROOT / 'assets')
    parser.add_argument('--checkpoint', type=Path)
    parser.add_argument('--skip-export', action='store_true')
    parser.add_argument('--trajectory', type=Path, action='append', default=[], help='Read-only NPZ with obs/history')
    args = parser.parse_args()
    torch.set_num_threads(2)
    assets = args.assets.resolve()
    exporter = load_exporter()
    if not args.skip_export:
        command = [sys.executable, str(EXPORTER), '--output', str(assets)]
        if args.checkpoint:
            command += ['--checkpoint', str(args.checkpoint)]
        subprocess.run(command, check=True)
        extraction = json.loads((assets / 'manifest.json').read_text())
        (assets / 'extraction_report.json').write_text(json.dumps(extraction, indent=2) + '\n')
    else:
        extraction = json.loads((assets / 'extraction_report.json').read_text())
    with np.load(assets / 'inference_fp32.npz', allow_pickle=False) as data:
        state = {key: torch.from_numpy(data[key].copy()) for key in data.files}
    floating = exporter.Policy(state).eval()
    fixed = FixedPolicy.from_npz(assets / 'inference_fp32.npz')
    export_memories(fixed, assets, extraction)
    restored = FixedPolicy.from_assets(assets)
    vectors = assets / 'vectors'
    manifest = {'schema': 'full_center_v5.fixed_vectors.v1', 'format': 'signed24 F12',
                'ordering': 'sample-major, one hex word/line', 'inputs': 'obs28 then history140 oldest-to-newest',
                'expected_layers': 'encoder_0[128],encoder_2[64],encoder_4[3],actor_0[128],actor_2[64],actor_4[32],actor_6[6]; post-ELU for hidden layers',
                'expected_saturated': 'one logical boolean per sample in a signed24 F0 word, shape(N,1); any pre-ELU signed24 saturation across seven layers, excluding input saturation and final action clipping',
                'layer_offsets': dict(zip(LAYER_NAMES, (0, 128, 192, 195, 323, 387, 419))), 'datasets': {}}
    reports = []
    with np.load(assets / 'synthetic_vectors.npz', allow_pickle=False) as data:
        datasets = [('synthetic', data['obs'].copy(), data['history'].copy(), 'exporter synthetic seed6701')]
    stress_obs, stress_history = np.zeros((8, 28), np.float32), np.zeros((8, 140), np.float32)
    stress_obs[1:5] = np.array([2047.999755859375, -2048, 3000, -3000])[:, None]
    stress_history[1:5] = stress_obs[1:5, :1]
    rng = np.random.default_rng(712)
    stress_obs[5:] = rng.uniform(-2048, 2048, (3, 28))
    stress_history[5:] = rng.uniform(-2048, 2048, (3, 140))
    datasets.append(('stress', stress_obs, stress_history, 'numeric stress, not physical observations'))
    paths = list(args.trajectory)
    for path in sorted((ROOT / 'vectors').glob('rollout_*.npz')):
        if path.resolve() not in [p.resolve() for p in paths]:
            paths.append(path)
    real_parts, real_sources = [], []
    for index, path in enumerate(paths):
        with np.load(path, allow_pickle=False) as data:
            datasets.append((f'trajectory_{index}_{path.stem}', data['obs'].copy(), data['history'].copy(), str(path.resolve())))
    for name, obs, history, source in datasets:
        report, arrays = validate_dataset(name, obs, history, fixed, floating)
        np.testing.assert_array_equal(restored.infer_fixed(arrays['obs_q'], arrays['history_q']).all_layers, arrays['expected_layers'])
        directory = vectors if name == 'synthetic' else vectors / name
        vector_arrays = arrays
        if name.startswith('trajectory_'):
            # Flatten time-major simulator arrays, preserving chronological frame then env order.
            frame_count = min(64, np.asarray(obs).shape[0])
            env_count = int(np.prod(np.asarray(obs).shape[1:-1])) if np.asarray(obs).ndim > 2 else 1
            count = frame_count * env_count
            vector_arrays = {key: value[:count] for key, value in arrays.items()}
            real_parts.append(vector_arrays)
            real_sources.append({'source': source, 'frames': frame_count, 'envs': env_count, 'samples': count})
        entry = emit_vectors(directory, vector_arrays)
        entry.update(directory='.' if name == 'synthetic' else name, source=source)
        manifest['datasets'][name] = entry
        report['source'] = source
        reports.append(report)
    if real_parts:
        real_arrays = {key: np.concatenate([part[key] for part in real_parts], axis=0) for key in real_parts[0]}
        manifest['datasets']['real'] = {**emit_vectors(vectors, real_arrays, 'real_'), 'directory': '.', 'sources': real_sources}
    (vectors / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    numerical = {'schema': 'full_center_v5.numerical_report.v1', 'original_vs_extracted': extraction['extraction_test'],
                 'full_fixed_point_reference_verified': True, 'memory_roundtrip_verified': True,
                 'rtl_verified': False, 'closed_loop_verified': False,
                 'note': 'Synthetic/stress are not real trajectories; real trajectory comparisons are open-loop.',
                 'datasets': reports}
    (vectors / 'numerical_report.json').write_text(json.dumps(numerical, indent=2) + '\n')
    lines = [numerical['original_vs_extracted'], 'Full fixed-point reference and memory roundtrip verified.',
             'No hardware or full closed-loop validation claimed.']
    for report in reports:
        lines.append(f"{report['name']}: {report['samples']} samples; input saturation {report['input_saturation']}")
        for key, output in report['outputs'].items():
            lines.append(f"  {key}: {output['error_vs_original_float']}")
        for name, layer in report['layers'].items():
            lines.append(f"  {name}: float range={layer['float_range']} fixed range={layer['fixed_dequantized_range']} error={layer['error_vs_original_float']} pre-ELU saturations={layer['pre_elu_saturation_count']}")
    text = '\n'.join(lines) + '\n'
    (vectors / 'numerical_report.txt').write_text(text)
    print(text)


if __name__ == '__main__':
    main()
