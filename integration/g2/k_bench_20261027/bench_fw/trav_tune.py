"""Offline terrain-classifier study on a dumped hardware elevation map (elev_dump.py output).

1. Rebuild the model's elevation map from the dump (ages from the per-row frame counter).
2. Run model/trav_v3 with the hardware's current registers and compare with the dumped cost map.
3. Sweep thresholds and report: floor cells judged lethal / blocked (lethal or inflated), and
   obstacle cells (well above the floor) judged lethal.
Usage: python trav_tune.py <dump.json>
"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(r'D:\Desktop\SOC FPGA\nav\model')))
import nav_fixed as nf  # noqa: E402
import trav_v3  # noqa: E402

N = nf.N


class EM:
    pass


def load(path):
    d = json.load(open(path))
    em = EM()
    ctrl = d['regs']['MAP_CTRL']
    em.avg, em.avg_shift_max = (ctrl >> 4) & 1, (ctrl >> 5) & 7
    em.zmin = np.array(d['zmin'], dtype=np.int64)
    if em.avg:
        em.zmin &= 0xFFFF            # 平均模式：高 16 位是无符号 Q3 平均绝对偏差
    em.zmax = np.array(d['zmax'], dtype=np.int64)
    em.cnt = np.array(d['cnt'], dtype=np.int64)
    em.tag = np.array(d['tag'], dtype=np.int64)
    stamp = np.array(d['stamp'], dtype=np.int64)
    fc_row = np.repeat(np.array(d['fc_row'], dtype=np.int64), 128)
    age = (fc_row - stamp) & 0xFF
    em.fc = 0
    em.stamp = (-age) & 0xFF
    em.stale = np.array(d['stale'], dtype=bool)
    return d, em


def cfg_from_regs(r):
    thr0, thr1, w, infl, vc, drop, fill, ctrl = (r[k] for k in ('MAP_THR0', 'MAP_THR1', 'MAP_WEIGHT', 'MAP_INFL',
                                                                 'MAP_VALID_CFG', 'MAP_DROP', 'MAP_FILL', 'MAP_CTRL'))
    return dict(step_max=thr0 & 0xFFFF, rough_max=thr0 >> 16, slope_rise=thr1 & 0xFFFF, jump=(thr1 >> 16) & 0xFF,
                k_slope=w & 0xFF, k_step=(w >> 8) & 0xFF, k_rough=(w >> 16) & 0xFF,
                infl_r=infl & 0xF, infl_disk=(infl >> 4) & 1, age_max=vc & 0xFF, cnt_min=(vc >> 8) & 0xF,
                keep_r=(vc >> 16) & 0xF, drop_k=drop & 0x1F, drop_slope=(drop >> 8) & 0xFF,
                fill_min=fill & 0xF, fill_tol=(fill >> 8) & 0xFF, fill_cost=(fill >> 16) & 0xFF,
                step_high_only=(ctrl >> 3) & 1, avg=(ctrl >> 4) & 1, avg_shift=(ctrl >> 5) & 7, slope_b=4)


def window(a, cx, cy):
    """slot-ordered array -> window (row = y, col = x) around (cx, cy)."""
    w = np.arange(N)
    sx = (cx - N // 2 + w) & (N - 1)
    sy = (cy - N // 2 + w) & (N - 1)
    return a.reshape(N, N)[sy[:, None], sx[None, :]]


def regions(V, H, cx, cy):
    w = np.arange(N)
    X = ((cx - N // 2 + w) + 0.5) * 0.05
    Y = ((cy - N // 2 + w) + 0.5) * 0.05
    XX, YY = np.meshgrid(X, Y)
    roi = (XX > 0.35) & (XX < 2.3) & (np.abs(YY) < 1.0)
    floor = V & roi & (H < 100) & (XX < 1.55) & (np.abs(YY) < 0.4)
    obst = V & roi & (H > 250)
    return floor, obst


def evaluate(em, cx, cy, cfg):
    em2 = EM()
    for k in ('zmin', 'zmax', 'cnt', 'tag', 'stamp', 'stale'):
        setattr(em2, k, getattr(em, k).copy())
    em2.fc = em.fc
    cost, lethal, (slot, V, H, RO) = trav_v3.trav_v3(em2, cx, cy, cfg)
    cw = cost[slot]
    floor, obst = regions(V, H, cx, cy)
    return dict(floor_n=int(floor.sum()), floor_lethal=float(lethal[floor].mean()),
                floor_blocked=float((cw[floor] >= nf.COST_INSCRIBED).mean()),
                obst_n=int(obst.sum()), obst_lethal=float(lethal[obst].mean())), cw, lethal, V, H, RO


def main():
    d, em = load(sys.argv[1])
    cx, cy = d['lat_cx'], d['lat_cy']
    base = cfg_from_regs(d['regs'])
    print('hardware config:', base)
    res, cw, lethal, V, H, RO = evaluate(em, cx, cy, base)
    hw = window(np.array(d['cost']), cx, cy)
    known = (hw != 255) | (cw != 255)
    print('model vs hardware cost map: cells %d, identical %.1f%%, lethal agree %.1f%%'
          % (known.sum(), 100 * (hw[known] == cw[known]).mean(), 100 * ((hw[known] == 254) == (cw[known] == 254)).mean()))
    print('baseline:', res)
    floor, obst = regions(V, H, cx, cy)
    st = []
    for name, m in (('roughness zmax-zmin', RO), ('height zmax', H)):
        v = np.sort(m[floor])
        st.append('%s p50 %d p90 %d p99 %d' % (name, v[len(v) // 2], v[int(len(v) * .9)], v[int(len(v) * .99)]))
    print('floor cells:', '; '.join(st), '; cnt p50', int(np.median(em.cnt[slot_of(cx, cy)][floor])))
    # which criterion fires on floor cells
    c = dict(trav_v3.DEFAULT_V3); c.update(base)
    print('sweep (floor lethal / floor blocked incl. inflation / obstacles lethal):')
    for rough in (50, 70, 90, 120):
        for step in (20, 30, 40, 60):
            for cnt_min in (1, 2, 3):
                cfg = dict(base, rough_max=rough, step_max=step, cnt_min=cnt_min)
                r = evaluate(em, cx, cy, cfg)[0]
                print('  rough %3d step %2d cnt_min %d : floor lethal %5.1f%%  blocked %5.1f%%   obstacles lethal %5.1f%% (n=%d/%d)'
                      % (rough, step, cnt_min, 100 * r['floor_lethal'], 100 * r['floor_blocked'], 100 * r['obst_lethal'],
                         r['floor_n'], r['obst_n']))


def slot_of(cx, cy):
    w = np.arange(N)
    sx = (cx - N // 2 + w) & (N - 1)
    sy = (cy - N // 2 + w) & (N - 1)
    return (sy[:, None] << 7) | sx[None, :]


if __name__ == '__main__':
    main()
