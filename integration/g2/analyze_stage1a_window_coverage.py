"""Read-only window and matching-prefix comparison for the stage1a ablation."""
from pathlib import Path
import argparse
import datetime
import json
import numpy as np

ROOT = Path('F:/nav_integration_g2_20261023/nav3x5_N1c_stage1a_window_ablation_20261026')


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def prefix(left, right):
    result = {}
    with np.load(left) as a, np.load(right) as b:
        for name in ('match_log', 'tlog'):
            x, y = a[name], b[name]
            n = min(len(x), len(y))
            indices = np.flatnonzero(np.any(x[:n] != y[:n], axis=1))
            i = int(indices[0]) if len(indices) else None
            result[name] = dict(
                rows_left=len(x), rows_right=len(y),
                exactly_equal_prefix_rows=i if i is not None else n,
                first_different_zero_based_row=i,
                first_different_left=x[i].tolist() if i is not None else None,
                first_different_right=y[i].tolist() if i is not None else None,
                tlog_first_different_time_s=float(y[i, 0]) if i is not None and name == 'tlog' else None)
    result['caveat'] = 'match_log has commit order but no individual timestamps; no row-to-time alignment is assumed'
    return result


def run(root):
    manifest = read(root / 'manifest.json')
    base = {(r['case'], r['SLF_SN']): Path(r['out']) for r in manifest['reused_baselines']}
    result = dict(recorded_at=datetime.datetime.now().astimezone().isoformat(),
                  new_simulation=False, production_inputs_changed=False, profiles=[])
    for cell in manifest['jobs']:
        path = Path(cell['out'])
        if not (path / 'final_maps.npz').exists():
            continue
        audit = read(path / 'window_audit.json')
        builds = [v for v in audit['events'] if v['op'] == 'build_lf']
        scores = [v for v in audit['events'] if v['op'] == 'score']
        updates = [v for v in audit['events'] if v['op'] == 'update']
        contain = []
        for event in builds:
            corners = event['world_scoring_core_corners_m']
            xs = [p[0] for p in corners]
            ys = [p[1] for p in corners]
            contain.append(min(xs) <= 0 and max(xs) >= 5 and min(ys) <= -1.5 and max(ys) >= 1.5)
        effective = [s['effective_n_valid'] for s in scores if s['effective_n_valid'] > 0]
        result['profiles'].append(dict(
            case=cell['case'], profile=cell['profile'],
            actual_profile=audit['profile'], audit_path=str(path / 'window_audit.json'),
            first_build=builds[0], last_build=builds[-1],
            builds_covering_whole_field_core_k=sum(contain), build_n=len(builds),
            positive_effective_n_valid=dict(n=len(effective), min=min(effective),
                median=float(np.median(effective)), max=max(effective)),
            update_calls=len(updates), update_points=sum(v['points'] for v in updates),
            baseline128_prefix=prefix(base[cell['case'], 128] / 'final_maps.npz', path / 'final_maps.npz'),
            baseline256_prefix=prefix(base[cell['case'], 256] / 'final_maps.npz', path / 'final_maps.npz')))
    result['interpretation_limit'] = (
        'Matched early prefixes plus ablation error support a view-coverage contribution; '
        'later differences do not prove all storage/tag/history effects absent. '
        'A whole-field window bound alone does not prove localization acceptance.')
    target = root / 'window_coverage_and_prefix_comparison.json'
    target.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(target)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT)
    run(parser.parse_args().root)
