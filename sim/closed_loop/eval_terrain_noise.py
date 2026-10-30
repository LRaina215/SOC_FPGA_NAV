"""地形判定在实测量级噪声下的评测：旧模式（每帧极值）与平均模式（跨帧均值 + 平均绝对偏差）对比。

场景（不跨台阶，只要求能过斜坡与崎岖地面）：10° 坡道上 20 cm 平台再下坡、一片 ±15 mm 起伏的崎岖地面、
3/5/10 cm 低障碍与 45 cm 箱子。机器人按真值路线 3 cm/帧（约 0.3 m/s、10 帧/s）采集。
噪声：沿视线距离噪声按入射角放大（σ_r = R0·r/sqrt(cos 入射角)，掠射地面上 σ_z 约 10 mm，与 10-31 实测地面相当），
外加每帧整帧俯仰/横滚抖动（建图用的位姿与真实位姿不同）。

统计（栅格真值见 world.Scene.gt_label：能滚过 2 cm、坡度 12°）：
  可通行格判致命（254）比例、判不可走（>=253，含膨胀）比例；不可通行格判可走（<253）比例（漏报）。
用法: python sim/closed_loop/eval_terrain_noise.py [--frames-every 1] [--quick]
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
import trav_v3  # noqa: E402
import world as W  # noqa: E402

RES = 65536 / 1311 / 1000
BANDS = ((0.35, 0.7), (0.7, 1.0), (1.0, 1.5), (1.5, 2.0))
# 板上寄存器复位值对应的配置（MAP_THR0/THR1/WEIGHT/INFL/VALID_CFG/DROP/FILL/CTRL）
BOARD = dict(trav_v3.DEFAULT_V3, step_max=20, rough_max=50, slope_rise=57, jump=25, k_slope=160, k_step=48,
             k_rough=32, infl_r=4, infl_disk=1, age_max=60, cnt_min=1, keep_r=12, drop_k=16, drop_slope=4,
             fill_min=5, fill_tol=30, fill_cost=40, step_high_only=1)


def scene():
    s = W.Scene(size=(9.0, 6.0), origin=(-1.5, -3.0))
    s.walls()
    s.ramp(1.0, 2.1, -1.6, -0.4, 0.0, 0.155, "ramp_up")           # 8°（真值栅格在 10° 坡与平台交接处有 1 cm 量化台阶）
    s.box(2.1, -1.6, 3.0, -0.4, 0.155, "platform")
    s.ramp(4.1, 3.0, -1.6, -0.4, 0.0, 0.155, "ramp_down")
    # 崎岖地面：两种尺度的平滑起伏，峰谷约 ±15 mm
    x0, x1, y0, y1 = 1.0, 4.0, 0.6, 1.8
    ix0, iy0 = s.w._idx(np.array(x0), np.array(y0))
    ix1, iy1 = s.w._idx(np.array(x1), np.array(y1))
    xs = s.w.o + (np.arange(ix0, ix1) + 0.5) * s.w.res
    ys = s.w.o + (np.arange(iy0, iy1) + 0.5) * s.w.res
    X, Y = np.meshgrid(xs, ys)
    bump = np.sin(2 * math.pi * X / 0.50) * np.sin(2 * math.pi * Y / 0.43) + 0.6 * np.sin(2 * math.pi * (X + 0.7 * Y) / 0.27)
    bump = bump - bump.min()
    taper = np.minimum.reduce([np.clip((X - x0) / 0.15, 0, 1), np.clip((x1 - X) / 0.15, 0, 1),
                               np.clip((Y - y0) / 0.15, 0, 1), np.clip((y1 - Y) / 0.15, 0, 1)])
    bump *= np.sin(0.5 * math.pi * taper) ** 2                # 边缘 15 cm 内渐变到 0，不形成台阶
    g = np.hypot(*np.gradient(bump, s.w.res))
    bump *= math.tan(math.radians(9.0)) / g.max()          # 最陡处 9°（能力 12° 以内）
    s.w.h[iy0:iy1, ix0:ix1] = np.maximum(s.w.h[iy0:iy1, ix0:ix1], bump)
    s.prims.append(("box", (x0, y0, x1, y1), 0.03, "rough"))
    s.box(5.0, -2.0, 5.3, -1.4, 0.03, "low3")
    s.box(5.0, -0.9, 5.3, -0.3, 0.05, "low5")
    s.box(5.0, 0.2, 5.3, 0.8, 0.10, "low10")
    s.box(6.2, -0.5, 6.8, 0.3, 0.45, "box45")
    return s


ROUTE = [(-0.8, -1.0, 0), (4.6, -1.0, 0), (4.6, -1.0, 90), (4.6, 1.2, 180), (0.2, 1.2, 180), (0.2, 1.2, 0),
         (4.4, 0.0, -10), (4.4, 0.0, 0)]


def poses(step=0.03, turn_deg=6.0):
    out = []
    for (x0, y0, a0), (x1, y1, a1) in zip(ROUTE[:-1], ROUTE[1:]):
        da = (a1 - a0 + 180) % 360 - 180
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step), int(abs(da) / turn_deg))
        for k in range(n):
            f = k / n
            out.append((x0 + (x1 - x0) * f, y0 + (y1 - y0) * f, a0 + da * f))
    return out


class NoisyLidar(sw.VirtualLidar):
    """距离噪声随入射角放大：σ_r = r0 · r / sqrt(max(cos 入射角, 0.05))。"""

    def __init__(self, r0=0.012, **kw):
        super().__init__(noise_mm=0.0, **kw)
        self.r0 = r0

    def capture(self, world, R_ws, t_ws):
        h, w, _ = self.unit.shape
        d_w = (R_ws @ self.unit.reshape(-1, 3).T).T
        r = world.raycast(np.asarray(t_ws, dtype=float), d_w, tmax=self.rng_max, step=self.step)
        ok = np.isfinite(r) & (r >= self.rng_min) & (r <= self.rng_max)
        rr = np.where(ok, r, 1.0)
        p = np.asarray(t_ws) + d_w * rr[:, None]
        # 表面法线：由高程场梯度近似（竖直面上梯度很大，法线接近水平）
        H, res = world.h, world.res
        ix = np.clip(((p[:, 0] - world.o) / res).astype(int), 1, world.n - 2)
        iy = np.clip(((p[:, 1] - world.o) / res).astype(int), 1, world.n - 2)
        gx = (H[iy, ix + 1] - H[iy, ix - 1]) / (2 * res)
        gy = (H[iy + 1, ix] - H[iy - 1, ix]) / (2 * res)
        nrm = np.stack([-gx, -gy, np.ones_like(gx)], 1)
        nrm /= np.linalg.norm(nrm, axis=1, keepdims=True)
        cosi = np.abs((nrm * d_w).sum(1))
        sig = 1000 * self.r0 * rr / np.sqrt(np.maximum(cosi, 0.05))
        mm = np.round(rr * 1000 + self.rng.normal(0, 1, rr.shape) * sig)
        depth = np.where(ok, np.clip(mm, 1, 65535), 0).astype(np.int64).reshape(h, w)
        return depth, depth


def causes(V, H, c):
    sh = fast._shift
    n = nf.N
    step = np.zeros((n, n), dtype=np.int64)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy or dx:
                v, h = sh(V, dy, dx, False), sh(H, dy, dx, 0)
                step = np.maximum(step, np.where(v, np.maximum(H - h, 0), 0))
    rx, _ = trav_v3.gated_rise(V, H, 1, c["slope_b"], c["jump"])
    ry, _ = trav_v3.gated_rise(V, H, 0, c["slope_b"], c["jump"])
    return step, np.maximum(rx, ry) + (np.minimum(rx, ry) >> 1)


def run(cfgs, every=1, r0=0.012, jitter_deg=0.3, seed=5, verbose=True):
    s = scene()
    lid = NoisyLidar(r0=r0, seed=seed, rng_max=6.0, step=0.012)
    Rbs, tbs = sw.mount_pose(height=0.15, pitch_down_deg=2.1)
    rng = np.random.default_rng(seed + 1)
    models = {}
    for name, cfg in cfgs.items():
        fm = fpga_model.FpgaModel(trav="v3", cfg=dict(BOARD, **cfg))
        fm.em = nf.ElevMap(avg=cfg.get("avg", 0), avg_shift_max=cfg.get("avg_shift", 3))
        models[name] = fm
    acc = {name: {} for name in cfgs}
    rb = __import__("robot").RobotSim(s, 0, 0, 0)
    P = poses()[::every]
    for i, (x, y, a) in enumerate(P):
        rb.x, rb.y, rb.yaw = x, y, math.radians(a)
        rb.update_attitude()
        Rwb, twb = rb.body_pose()
        Rws, tws = sw.compose(Rwb, twb, Rbs, tbs)
        depth, _ = lid.capture(s.w, Rws, tws)
        # 建图位姿：真实位姿 + 本帧俯仰/横滚误差
        ep, er = np.radians(rng.normal(0, jitter_deg, 2))
        Rm = Rwb @ sw.roty(ep) @ sw.rotx(er)
        Rms, tms = sw.compose(Rm, twb, Rbs, tbs)
        Rq, tq = sw.to_fixed_pose(Rms, tms)
        cx, cy = nf.world_cell(int(x * 1000), 1311), nf.world_cell(int(y * 1000), 1311)
        for name, fm in models.items():
            fm.frame(depth)
            fm.commit([v for r in Rq for v in r] + tq, cx, cy, int(rb.z * 1000) - 600, int(rb.z * 1000) + 1200)
        if i % 5 or i < 20:
            continue
        ix = np.arange(128) + cx - 64
        iy = np.arange(128) + cy - 64
        WX, WY = np.meshgrid((ix + 0.5) * RES, (iy + 0.5) * RES)
        dist = np.hypot(WX - x, WY - y)
        roi = (dist > 0.35) & (dist < 2.0)
        lab = s.gt_lookup(WX, WY)
        reg = np.full(WX.shape, "floor", dtype=object)
        for kind, (x0, y0, x1, y1), v, nm in s.prims:
            m = (WX >= x0 - 0.1) & (WX <= x1 + 0.1) & (WY >= y0 - 0.1) & (WY <= y1 + 0.1)
            reg[m] = "wall" if nm == "wall" else nm
        for name, fm in models.items():
            cw = fast.cost_window(fm.cost, cx, cy)
            c = dict(trav_v3.DEFAULT_V3)
            c.update(fm.cfg)
            _, V, H, RO = fm.view
            step, rise = causes(V, H, c)
            fl = roi & (lab == 0) & (cw == 254)
            st = acc[name].setdefault("_cause", np.zeros(5, dtype=np.int64))
            st += [int(fl.sum()), int((fl & (step > c["step_max"])).sum()), int((fl & (RO > c["rough_max"])).sum()),
                   int((fl & (rise > c["slope_rise"])).sum()), 0]
            obs = roi & (cw != 255) & (lab != 2)
            for r in set(reg[obs].tolist()):
                m = obs & (reg == r)
                st = acc[name].setdefault(r, np.zeros(5, dtype=np.int64))
                st += [int((m & (lab == 0)).sum()), int((m & (lab == 0) & (cw >= 254)).sum()),
                       int((m & (lab == 0) & (cw >= 253)).sum()), int((m & (lab == 1)).sum()),
                       int((m & (lab == 1) & (cw < 253)).sum())]
            st = acc[name].setdefault("_unknown", np.zeros(5, dtype=np.int64))
            st += [int((roi & (lab == 0)).sum()), int((roi & (lab == 0) & (cw == 255)).sum()), 0, 0, 0]
            for b0, b1 in BANDS:
                m = (dist > b0) & (dist <= b1) & (lab == 0)
                st = acc[name].setdefault(f"_band{b0:.2f}", np.zeros(5, dtype=np.int64))
                st += [int((m & (cw != 255)).sum()), int((m & (cw == 254)).sum()), int((m & (cw == 255)).sum()),
                       int(m.sum()), 0]
    if verbose:
        for name in cfgs:
            print(f"== {name}  {cfgs[name]}")
            print(f"   {'区域':10s} {'可通行格':>8s} {'误判致命':>8s} {'判不可走':>8s} {'不可通行格':>10s} {'漏报':>6s}")
            for r, v in sorted(acc[name].items()):
                if r.startswith("_"):
                    continue
                print(f"   {r:10s} {v[0]:8d} {100 * v[1] / max(1, v[0]):7.1f}% {100 * v[2] / max(1, v[0]):7.1f}% "
                      f"{v[3]:10d} {100 * v[4] / max(1, v[3]):5.1f}%")
            k = acc[name]["_cause"]
            print(f"   可通行格误判致命 {k[0]} 格，其中台阶 {k[1]}、粗糙度 {k[2]}、坡度 {k[3]}（可重叠，其余为落差）")
            u = acc[name]["_unknown"]
            print(f"   近处 0.35-2 m 可通行格中未知 {100 * u[1] / max(1, u[0]):.1f}%")
            print("   按距离：" + "；".join(
                f"{b0:.2f}-{b1:.2f} m 误判致命 {100 * v[1] / max(1, v[0]):.2f}% 未知 {100 * v[2] / max(1, v[3]):.0f}%"
                for (b0, b1), v in ((b, acc[name][f"_band{b[0]:.2f}"]) for b in BANDS)))
    return acc


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--every", type=int, default=1)
    ap.add_argument("--r0", type=float, default=0.012)
    ap.add_argument("--jitter", type=float, default=0.3)
    ap.add_argument("--set", nargs="*", default=[], help="name:key=val,key=val ...")
    a = ap.parse_args()
    cfgs = {"旧模式 现参数": {}, "旧模式 40/90/3": dict(step_max=40, rough_max=90, cnt_min=3),
            "平均 20/30/2": dict(avg=1, step_max=20, rough_max=30, cnt_min=2),
            "平均 30/30/2": dict(avg=1, step_max=30, rough_max=30, cnt_min=2)}
    for spec in a.set:
        nm, kv = spec.split(":", 1)
        cfgs[nm] = {k: int(v) for k, v in (p.split("=") for p in kv.split(","))}
    run(cfgs, every=a.every, r0=a.r0, jitter_deg=a.jitter)
