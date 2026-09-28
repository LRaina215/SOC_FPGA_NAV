"""地形可通行性算法单独评测（排除定位误差）：机器人按真值位姿沿路线采集，逐帧建图，
按地形要素分区统计判定正确率。

用法: python sim/closed_loop/eval_trav.py [--trav v1 v2] [--set key=val ...]
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1] / "model"))
import fpga_model  # noqa: E402
import nav_fast as fast  # noqa: E402
import nav_fixed as nf  # noqa: E402
import sim_world as sw  # noqa: E402
import world as W  # noqa: E402

RES = 65536 / 1311 / 1000

# 采集路线（真值），覆盖坡道、平台、台阶带、路沿、箱子、陡坡
ROUTE = [(-1.0, -0.3, 0), (0.0, -0.3, 0), (0.8, -0.3, 0), (1.6, -0.3, 0), (2.6, -0.3, 0), (3.6, -0.3, 0),
         (4.6, -0.3, 0), (4.6, 0.6, 60), (3.8, 0.9, 150), (2.6, 0.9, 180), (0.5, 0.9, 150),
         (0.5, 1.0, 30), (4.8, 1.0, -20), (6.0, 0.0, -60), (6.3, -1.8, -30), (7.5, -2.4, 0), (8.0, -2.3, 0),
         (8.0, 0.2, 60), (6.5, 2.8, 0), (9.0, 2.8, -30)]


def regions(scene, WX, WY):
    """每个栅格属于哪个地形要素（按几何体外扩 10 cm）"""
    reg = np.full(WX.shape, "floor", dtype=object)
    for kind, (x0, y0, x1, y1), v, name in scene.prims:
        if name == "wall":
            m = (WX >= x0 - 0.1) & (WX <= x1 + 0.1) & (WY >= y0 - 0.1) & (WY <= y1 + 0.1)
            reg[m] = "wall"
    for kind, (x0, y0, x1, y1), v, name in scene.prims:
        if name != "wall":
            m = (WX >= x0 - 0.1) & (WX <= x1 + 0.1) & (WY >= y0 - 0.1) & (WY <= y1 + 0.1)
            reg[m] = name
    return reg


def evaluate(trav, cfg_over, verbose=True):
    scene = W.scene_terrain()
    lid = sw.VirtualLidar(noise_mm=4.0, seed=11, rng_max=8.0, step=0.015)
    Rbs, tbs = sw.mount_pose()
    fm = fpga_model.FpgaModel(trav=trav)
    fm.cfg.update(cfg_over)
    stats = {}
    # 路线插值（每 0.25 m 一帧，朝向插值）
    poses = []
    for (x0, y0, a0), (x1, y1, a1) in zip(ROUTE[:-1], ROUTE[1:]):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / 0.25))
        for k in range(n):
            f = k / n
            da = (a1 - a0 + 180) % 360 - 180
            poses.append((x0 + (x1 - x0) * f, y0 + (y1 - y0) * f, a0 + da * f))
    rb = __import__("robot").RobotSim(scene, 0, 0, 0)
    for i, (x, y, a) in enumerate(poses):
        rb.x, rb.y, rb.yaw = x, y, math.radians(a)
        rb.update_attitude()
        Rwb, twb = rb.body_pose()
        Rws, tws = sw.compose(Rwb, twb, Rbs, tbs)
        depth, _ = lid.capture(scene.w, Rws, tws)
        fm.frame(depth)
        Rq, tq = sw.to_fixed_pose(Rws, tws)
        cx, cy = nf.world_cell(int(x * 1000), 1311), nf.world_cell(int(y * 1000), 1311)
        fm.commit([v for r in Rq for v in r] + tq, cx, cy, int(rb.z * 1000) - 600, int(rb.z * 1000) + 1200)
        if i % 3:
            continue
        cw = fast.cost_window(fm.cost, cx, cy)
        ix = np.arange(128) + cx - 64
        iy = np.arange(128) + cy - 64
        WX, WY = np.meshgrid((ix + 0.5) * RES, (iy + 0.5) * RES)
        x0b, y0b, x1b, y1b = scene.bounds
        inb = (WX > x0b + 0.1) & (WX < x1b - 0.1) & (WY > y0b + 0.1) & (WY < y1b - 0.1)
        obs = inb & (cw != 255)
        lab = scene.gt_lookup(WX, WY)
        reg = regions(scene, WX, WY)
        use = obs & (lab != 2)
        pl = cw >= 254
        for r in set(reg[use].tolist()):
            m = use & (reg == r)
            s = stats.setdefault(r, np.zeros(4, dtype=np.int64))   # 真可通行&判可 / 真可&判致命 / 真致命&判可 / 真致命&判致命
            s += [int((m & (lab == 0) & ~pl).sum()), int((m & (lab == 0) & pl).sum()),
                  int((m & (lab == 1) & ~pl).sum()), int((m & (lab == 1) & pl).sum())]
        # 未知格统计（真可通行区里还有多少未知）
        s = stats.setdefault("_unknown_free", np.zeros(4, dtype=np.int64))
        s += [int((inb & (lab == 0) & (cw == 255)).sum()), int((inb & (lab == 0)).sum()), 0, 0]
    tot = np.zeros(4, dtype=np.int64)
    if verbose:
        print(f"== trav {trav} {cfg_over}")
        print(f"{'要素':12s} {'可通行格':>8s} {'误报致命':>8s} {'致命格':>8s} {'漏报':>8s}")
    for r, s in sorted(stats.items()):
        if r.startswith("_"):
            continue
        tot += s
        if verbose:
            fp = s[1] / max(1, s[0] + s[1])
            fn = s[2] / max(1, s[2] + s[3])
            print(f"{r:12s} {s[0] + s[1]:8d} {100 * fp:7.1f}% {s[2] + s[3]:8d} {100 * fn:7.1f}%")
    fp = tot[1] / max(1, tot[0] + tot[1])
    fn = tot[2] / max(1, tot[2] + tot[3])
    u = stats["_unknown_free"]
    if verbose:
        print(f"{'合计':12s} {tot[0] + tot[1]:8d} {100 * fp:7.1f}% {tot[2] + tot[3]:8d} {100 * fn:7.1f}%   "
              f"可通行区未知比例 {100 * u[0] / max(1, u[1]):.1f}%")
    return fp, fn, u[0] / max(1, u[1]), stats


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--trav", nargs="+", default=["v1", "v2"])
    ap.add_argument("--set", nargs="*", default=[])
    a = ap.parse_args()
    over = {k: int(v) for k, v in (s.split("=") for s in a.set)}
    for t in a.trav:
        evaluate(t, over)
