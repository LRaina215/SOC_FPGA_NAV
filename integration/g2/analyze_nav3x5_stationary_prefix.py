"""Read saved N1c outputs; do not rerun or change the navigation model."""
from pathlib import Path
import argparse
import datetime
import json
import numpy as np

ROOT = Path('F:/nav_integration_g2_20261023/nav3x5_N1c_SLF128_256_stage1_20261026')


def run(root):
    result = dict(
        recorded_at=datetime.datetime.now().astimezone().isoformat(),
        read_only_existing_outputs=True,
        new_simulation=False,
        fields_source=str(root / 'firmware_source/test/hal_py.c') + ':137',
        match_fields=['applied', 'dx', 'dy', 'reserved0', 'dyaw', 'n_valid',
                      'cost0', 'cost_best', 'degen_x', 'degen_y', 'degen_yaw', 'n_applied'],
        model_window_source=str(root / 'execution_tree/model/slf_match.py') + ':198',
        score_window_source=str(root / 'execution_tree/model/slf_match.py') + ':219',
        confirmed_model_semantics=dict(
            cell_size_approx_mm=50, SN256_window_approx_m=12.8,
            SN128_window_approx_m=6.4, nomap_edge_margin_cells=8,
            SN256_nomap_scoring_core_approx_m=12.0,
            SN128_nomap_scoring_core_approx_m=5.6,
            nomap_view_center='current elevation-map cx/cy, not globally fixed to field center',
            prior_view_origin='fixed from prior package; edge margin zero',
            caveat='geometric field fit does not prove equal observed/gated structure constraints'),
        prefixes=[],
        causal_conclusion=(
            'Smaller window changes usable matching constraints in these saved outputs; '
            'no per-point rejection reason or saved per-frame structure map proves which '
            'combination of view boundary, gate, tag collision or normals caused the '
            'localization drift. Do not claim a unique cause.'))
    for case in ('goal', 'ramp'):
        for sn in (256, 128):
            path = root / 'results' / f'{case}_sn{sn}_nomap_s1' / 'final_maps.npz'
            with np.load(path) as saved:
                matches = saved['match_log'][:20]
                trace = saved['tlog']
                stationary = trace[trace[:, 0] <= 2.000001]
                valid = matches[matches[:, 5] > 0, 5]
                result['prefixes'].append(dict(
                    case=case, SLF_SN=sn, raw_output=str(path),
                    match_log_commit_indices=[0, len(matches)-1],
                    match_log_rows=matches.tolist(), nonzero_n_valid_count=len(valid),
                    nonzero_n_valid_min=int(valid.min()) if len(valid) else None,
                    nonzero_n_valid_median=float(np.median(valid)) if len(valid) else None,
                    nonzero_n_valid_max=int(valid.max()) if len(valid) else None,
                    truth_until_2s=dict(samples=len(stationary),
                        x_range_m=[float(stationary[:, 1].min()), float(stationary[:, 1].max())],
                        y_range_m=[float(stationary[:, 2].min()), float(stationary[:, 2].max())]),
                    timestamp_caveat=(
                        'match_log stores committed sequence without per-row timestamps; '
                        'tlog stationary phase shown separately; index-to-time causal alignment not assumed'),
                    cost_caveat='raw cost sums depend on different n_valid; not directly comparable as quality'))
    target = root / 'stationary_match_prefix_diagnostic.json'
    target.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(target)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT)
    run(parser.parse_args().root)
