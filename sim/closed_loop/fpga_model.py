"""FPGA 侧模型：感知链路（与 RTL 逐位一致的 nav_fast）+ 匹配打分（EICM，将来的 match_eval 硬件）。

帧处理两遍（与硬件双缓冲帧一致）：
  1. 匹配遍：按固件给的预测位姿把点变换到里程计系，对每个候选增量统计与"本帧融合前"高程图的区间一致性；
  2. 融合遍：固件修正位姿后提交，点云写入高程图，扫描生成代价图。
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "model"))
import lidar_lut  # noqa: E402
import nav_fast as fast  # noqa: E402
import nav_fixed as nf  # noqa: E402
import trav_v2  # noqa: E402
import trav_v3  # noqa: E402

N = nf.N
SIN = np.array(nf.SIN, dtype=np.int64)

# 匹配参数（硬件寄存器）
M_TOL = 30        # 区间两侧容差 mm
M_EMAX = 150      # 截断残差 mm
M_EIN = 100       # 高度修正内点门限 mm
M_GROUND_ROUGH = 40
M_STRIDE = 4      # 点抽样步长（硬件每 4 个点取 1 个）


class FpgaModel:
    def __init__(self, trav="v1", inv_res=1311, cfg=None):
        self.lut = np.array(lidar_lut.lut_entries(), dtype=np.int64)
        self.em = nf.ElevMap()
        self.inv_res = inv_res
        self.trav = trav
        if cfg is None:
            # v1 的参数即 RTL 寄存器默认值；v2/v3 用各自模块里按机器人能力给出的默认值
            cfg = dict(age_max=60, cnt_min=1, step_max=120, rough_max=120, slope_max=60,
                       k_slope=64, k_step=32, k_rough=32, infl_r=5) if trav == "v1" else dict(infl_r=5)
        self.cfg = cfg
        self.cost = np.full(N * N, 255, dtype=np.int64)
        self.cx = self.cy = 0
        self.seq = 0
        self.xyz = None
        self.pv = None
        self.last_lethal = None
        self.view = None
        self.kf = None          # (cx, cy, valid, zmin, zmax, rough) 窗口坐标，z 已按 8 位 1 cm 量化还原

    # ---------------------------------------------------------------- 帧
    def frame(self, depth):
        self.xyz, self.pv = fast.pc_project(depth, self.lut)

    # ---------------------------------------------------------------- 关键帧
    def keyframe(self, z_ref):
        slot, V, H, RO = fast.window_view(self.em, self.cx, self.cy, dict(trav_v3.DEFAULT_V3, **self.cfg), kill=False)
        def q8(z):     # 相对 z_ref、1 cm 量化、8 位饱和（硬件存储格式），还原为 mm
            return np.clip(np.round((z - z_ref) / 10), -128, 127).astype(np.int64) * 10 + z_ref
        zmin = q8(self.em.zmin[slot].astype(np.int64))
        zmax = q8(self.em.zmax[slot].astype(np.int64))
        self.kf = (self.cx, self.cy, V.copy(), zmin, zmax, zmax - zmin)

    # ---------------------------------------------------------------- 匹配遍
    def match_eval(self, rt, cx_mm, cy_mm, cands):
        """rt: R[9] + t[3]；cands: (K,3) dx dy dyaw。返回 (K,4) n_valid cost n_in sum_dz"""
        R = np.array(rt[:9], dtype=np.int64).reshape(3, 3)
        t = np.array(rt[9:12], dtype=np.int64)
        idx = np.nonzero(self.pv)[0][::M_STRIDE]
        p = fast.pc_transform(self.xyz[idx], R, t)
        # 与融合遍同一高度带（机器人附近地面以下/机身以上的点不参与）
        if self.kf is None:
            return np.zeros((len(cands), 4), dtype=np.int64)
        kcx, kcy, V, zmin, zmax, RO = self.kf
        out = np.zeros((len(cands), 4), dtype=np.int64)
        px, py, pz = p[:, 0] - cx_mm, p[:, 1] - cy_mm, p[:, 2]
        for k, (dx, dy, da) in enumerate(cands):
            i = (int(da) & 0xFFFF) >> 6
            s, c = SIN[i], SIN[(i + 256) & 1023]
            x = cx_mm + ((px * c - py * s) >> 14) + dx
            y = cy_mm + ((px * s + py * c) >> 14) + dy
            wx = fast.world_cell(x, self.inv_res) - (kcx - N // 2)
            wy = fast.world_cell(y, self.inv_res) - (kcy - N // 2)
            inw = (wx >= 0) & (wx < N) & (wy >= 0) & (wy < N)
            wxc, wyc = np.clip(wx, 0, N - 1), np.clip(wy, 0, N - 1)
            ok = inw & V[wyc, wxc]
            lo = zmin[wyc, wxc] - M_TOL
            hi = zmax[wyc, wxc] + M_TOL
            e = np.where(pz < lo, lo - pz, np.where(pz > hi, pz - hi, 0))
            cost = np.minimum(e, M_EMAX)
            ground = ok & (RO[wyc, wxc] < M_GROUND_ROUGH)
            r = pz - zmax[wyc, wxc]
            inl = ground & (np.abs(r) < M_EIN)
            out[k] = (int(ok.sum()), int(cost[ok].sum()), int(inl.sum()), int(r[inl].sum()))
        return out

    # ---------------------------------------------------------------- 融合遍
    def commit(self, rt, cx, cy, zmin, zmax):
        R = np.array(rt[:9], dtype=np.int64).reshape(3, 3)
        t = np.array(rt[9:12], dtype=np.int64)
        p = fast.pc_transform(self.xyz, R, t)
        a, tg, z = fast.pc_filter(p, self.pv, self.inv_res, cx, cy, max(-32768, zmin), min(32767, zmax))
        fast.elev_update(self.em, a, tg, z)
        if self.trav == "v1":
            self.cost, self.last_lethal, self.view = fast.trav_v1(self.em, cx, cy, self.cfg)
        elif self.trav == "v2":
            self.cost, self.last_lethal, self.view = trav_v2.trav_v2(self.em, cx, cy, self.cfg)
        else:
            self.cost, self.last_lethal, self.view = trav_v3.trav_v3(self.em, cx, cy, self.cfg)
        self.cx, self.cy = cx, cy
        self.seq = (self.seq + 1) & 0xFFFF
        return p
