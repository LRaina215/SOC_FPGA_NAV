import csv
import json
import math
import sys
from pathlib import Path

runs = [('128 forward 1.5 m (Codex)', 'F:/nav_integration_g2_20261023/nav3x5_N1c_final128_long_patrol_20261026/results/patrol330s_best128_s1'),
        ('256 control (Codex)', 'F:/nav_integration_g2_20261023/nav3x5_N1c_final128_long_patrol_20261026/results/patrol330s_control256_s1')]
for d in sorted(Path('F:/nav_integration_g2_20261023/K_patrol_20261027').iterdir()):
    runs.append((d.name, str(d)))

print(f"{'run':30s} {'wall_s':>7s} {'coll':>4s} {'lost':>4s} {'goals':>5s} | err mm at 60/120/200/270/330 s | max   P95   yaw_err_max")
for name, d in runs:
    d = Path(d)
    if not (d / 'metrics.json').exists():
        print(f'{name:30s} (no metrics)')
        continue
    m = json.loads((d / 'metrics.json').read_text(encoding='utf-8'))
    rows = list(csv.DictReader(open(d / 'timeseries.csv', encoding='utf-8')))
    e = [(float(r['t']), math.hypot(float(r['x']) - float(r['ex']), float(r['y']) - float(r['ey'])) * 1000,
          abs(((float(r['eyaw_deg']) - float(r['yaw_deg']) + 180) % 360) - 180), float(r.get('loc_valid', 1))) for r in rows]

    def at(T):
        c = [v for t, v, _, _ in e if t <= T]
        return round(c[-1]) if c else -1
    valid = sorted(v for _, v, _, lv in e if lv >= 0.5)
    p95 = valid[int(0.95 * (len(valid) - 1))] if valid else float('nan')
    goals = m.get('goals_reached', m.get('n_goals', ''))
    pa = d / 'patrol_protocol_audit.json'
    if pa.exists():
        try:
            goals = json.loads(pa.read_text(encoding='utf-8')).get('ordered_native_goal_advances', goals)
        except Exception:
            pass
    print(f"{name:30s} {m.get('wall_s', 0):7.0f} {m.get('collisions', 0):4d} {m.get('n_lost', 0):4d} {str(goals):>5s} | "
          f"{at(60):5d} {at(120):5d} {at(200):5d} {at(270):5d} {at(330):5d} | {max(v for _, v, _, _ in e):5.0f} {p95:5.0f} {max(y for _, _, y, _ in e):6.2f}")
