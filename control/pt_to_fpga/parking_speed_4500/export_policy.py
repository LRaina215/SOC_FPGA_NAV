"""Extract this checkpoint's inference assets; does not build or program an FPGA."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np
import torch
from torch import nn

CHECKPOINT = Path('/home/cyd/fpga/pt_to_fpga/parking_speed_4500/model_4500.pt')
MODEL_SOURCE = Path('/home/cyd/fpga/rl_ws2/Wheel-Legged-Gym-SF_small/wheel_legged_gym/rsl_rl/modules/actor_critic_sequence.py')


def network(state, prefix):
    keys = sorted((k for k in state if k.startswith(prefix + '.') and k.endswith('.weight')),
                  key=lambda k: int(k.split('.')[1]))
    layers = []
    for i, key in enumerate(keys):
        w, b = state[key], state[key.replace('weight', 'bias')]
        layer = nn.Linear(w.shape[1], w.shape[0])
        with torch.no_grad():
            layer.weight.copy_(w)
            layer.bias.copy_(b)
        layers.append(layer)
        if i < len(keys) - 1:
            layers.append(nn.ELU())
    return nn.Sequential(*layers)


class Policy(nn.Module):
    def __init__(self, state):
        super().__init__()
        self.encoder = network(state, 'encoder')
        self.actor = network(state, 'actor')

    def forward(self, obs, history):
        latent = self.encoder(history)
        raw = self.actor(torch.cat((obs, latent), dim=-1))
        return raw, torch.clamp(raw, -1.0, 1.0), latent


def write_hex(path, values, bits):
    mask = (1 << bits) - 1
    path.write_text(''.join(f'{int(x) & mask:0{bits // 4}x}\n' for x in values.reshape(-1)))
    # Round-trip check verifies signed two's-complement and memory ordering.
    recovered = np.array([int(x, 16) for x in path.read_text().splitlines()], dtype=np.int64)
    recovered[recovered >= 1 << (bits - 1)] -= 1 << bits
    np.testing.assert_array_equal(recovered, values.reshape(-1))


def fixtures(count=256):
    """Synthetic input-interface samples, not real hardware calibration data."""
    rng = np.random.default_rng(6701)
    # 28 维布局（v5，2026-10-11）：在 index 9 插入跳跃指令位（本代模型训练时恒 0，
    # 硬件接线也将恒 0），其后所有槽位相对 v4 顺移 +1。
    history = np.zeros((count, 5, 28), dtype=np.float32)
    for t in range(5):
        obs = history[:, t]
        obs[:, :3] = rng.uniform(-3, 3, (count, 3)) * 1.3
        g = np.column_stack((rng.uniform(-.5, .5, (count, 2)), -np.ones(count)))
        obs[:, 3:6] = g / np.linalg.norm(g, axis=1, keepdims=True) * 3
        obs[:, 6] = rng.uniform(-.6, .6, count) * 2
        obs[:, 7] = rng.uniform(-2.9, 2.9, count) * 1.3
        obs[:, 8] = rng.uniform(.085, .19, count) * 5
        obs[:, 9] = 0.0  # 跳跃指令位：训练恒 0
        previous_action = rng.uniform(-1, 1, (count, 6))
        obs[:, 10:16] = previous_action * .03
        obs[:, [12, 15]] = 0  # wheel position slots
        obs[:, [18, 21]] = rng.uniform(-30, 30, (count, 2)) * .15
        obs[:, 22:28] = previous_action
    # Reset/standing fixture: zero action, zero angular speed, repeated history.
    history[0] = 0
    history[0, :, 5] = -3
    history[0, :, 8] = .1305 * 5
    return torch.from_numpy(history[:, -1].copy()), torch.from_numpy(history.reshape(count, 140))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--checkpoint', type=Path, default=CHECKPOINT)
    parser.add_argument('--output', type=Path, default=Path(__file__).parent / 'assets')
    args = parser.parse_args()
    torch.set_num_threads(2)
    checkpoint = torch.load(args.checkpoint, map_location='cpu', weights_only=True)
    config = json.loads(args.checkpoint.with_name('config.json').read_text())
    assert config['train']['policy']['activation'] == 'elu'
    assert config['env']['env']['num_observations'] == 28
    assert config['env']['env']['num_actions'] == 6
    assert config['env']['env']['obs_history_length'] == 5
    state = checkpoint['model_state_dict']
    policy = Policy(state).eval()
    assert policy.encoder[0].in_features == 140 and policy.actor[0].in_features == 31
    args.output.mkdir(parents=True, exist_ok=True)
    float_weights = {k: v.numpy() for k, v in state.items() if k.startswith(('encoder.', 'actor.'))}
    np.savez(args.output / 'inference_fp32.npz', **float_weights)

    spec = importlib.util.spec_from_file_location('original_actor_critic_sequence', MODEL_SOURCE)
    original_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(original_module)
    reference = original_module.ActorCriticSequence(
        num_obs=28, num_critic_obs=state['critic.0.weight'].shape[1], num_actions=6, num_encoder_obs=140,
        latent_dim=3, encoder_hidden_dims=[128, 64], actor_hidden_dims=[128, 64, 32],
        critic_hidden_dims=[256, 128, 64], activation='elu').eval()
    reference.load_state_dict(state, strict=True)
    obs, history = fixtures()
    with torch.no_grad():
        raw, clipped, latent = policy(obs, history)
        original_raw, original_latent = reference.act_inference(obs, history)
        torch.testing.assert_close(raw, original_raw, rtol=0, atol=0)
        torch.testing.assert_close(latent, original_latent, rtol=0, atol=0)
    traced = torch.jit.trace(policy, (obs[:1], history[:1]))
    traced.save(str(args.output / 'policy_reference.pt'))
    with torch.no_grad():
        loaded = torch.jit.load(str(args.output / 'policy_reference.pt'))
        for actual, expected in zip(loaded(obs, history), (raw, clipped, latent)):
            torch.testing.assert_close(actual, expected, rtol=0, atol=0)
    np.savez(args.output / 'synthetic_vectors.npz', obs=obs.numpy(), history=history.numpy(),
             raw_actions=raw.numpy(), clipped_actions=clipped.numpy(), latent=latent.numpy())

    layers = []
    quantized = Policy(state).eval()
    parameter_count = macs = storage_bytes = 0
    for prefix in ('encoder', 'actor'):
        modules = getattr(policy, prefix)
        for index, layer in enumerate(modules):
            if not isinstance(layer, nn.Linear):
                continue
            name = f'{prefix}_{index}'
            w, b = layer.weight.detach().numpy(), layer.bias.detach().numpy()
            qw = np.rint(w.astype(np.float64) * 4096).astype(np.int64)
            qb = np.rint(b.astype(np.float64) * 16777216).astype(np.int64)
            assert np.all((-32768 <= qw) & (qw <= 32767)), name
            assert np.all((-(1 << 31) <= qb) & (qb < (1 << 31))), name
            write_hex(args.output / f'{name}_weights.mem', qw, 16)
            write_hex(args.output / f'{name}_bias.mem', qb, 32)
            qlayer = getattr(quantized, prefix)[index]
            with torch.no_grad():
                qlayer.weight.copy_(torch.tensor(qw / 4096, dtype=torch.float32))
                qlayer.bias.copy_(torch.tensor(qb / 16777216, dtype=torch.float32))
            layers.append(dict(name=name, input_size=w.shape[1], output_size=w.shape[0],
                               activation='elu' if index < len(modules)-1 else 'linear',
                               weights_file=f'{name}_weights.mem', bias_file=f'{name}_bias.mem',
                               weight_order='row-major: output neuron first, input index second',
                               weights_signed_bits=16, weights_fraction_bits=12,
                               bias_signed_bits=32, bias_fraction_bits=24,
                               weight_max_abs_error=float(np.max(np.abs(qw / 4096 - w)))))
            parameter_count += w.size + b.size
            macs += w.size
            storage_bytes += w.size * 2 + b.size * 4
    with torch.no_grad():
        qraw, qclipped, _ = quantized(obs, history)
    delta = (qclipped - clipped).abs()
    report = dict(
        source_checkpoint=str(args.checkpoint.resolve()),
        checkpoint_sha256=hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),
        checkpoint_iter=checkpoint.get('iter'), activation='ELU alpha=1',
        current_observation_size=28, history_size=140, history_order='oldest to newest; 5 x 28',
        outputs=6, inference_parameter_count=parameter_count, macs_per_inference=macs,
        fp32_parameter_bytes=parameter_count*4, candidate_weight_bias_bytes=storage_bytes,
        policy_period_ms=config['env']['sim']['dt']*config['env']['control']['decimation']*1000,
        ideal_mac_time_at_50mhz_single_mac_ms=macs/50000,
        timing_note='Ideal arithmetic lower bound only; excludes memory/activation/FSM latency.',
        extraction_test='256 synthetic samples: exact match to original act_inference and TorchScript reload.',
        quantization_test=dict(
            scope='Weights/biases quantized; all intermediate math and ELU remain floating-point.',
            data='Synthetic interface-shaped observations; not measured trajectories or calibration.',
            clipped_action_max_abs_error=float(delta.max()),
            clipped_action_mean_abs_error=float(delta.mean()),
            raw_action_max_abs_error=float((qraw-raw).abs().max()),
            full_fixed_point_or_rtl_verified=False),
        deployment_status='Export assets only. No inference RTL, .fs, sensor integration or motor deployment.',
        layers=layers,
    )
    (args.output / 'manifest.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    (args.output / 'training_config.json').write_text(json.dumps(config, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'layers'}, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
