"""FPGA 侧模型：感知链路（nav_fast / trav_v3，与 RTL 逐位一致）+ SLF 匹配（model/slf_match.py，尚无 RTL）。

每帧（兼容旧固件单遍，新固件先匹配再融合）：
  MATCH_ONLY commit：仅提取结构点，保留原始深度，不增加高程帧龄/地图代际；
  match_eval：对缓冲点逐候选增量打分（结构图为此前各帧累积）；
  match_update：按已接受修正更新结构图；FUSE_ONLY commit 用修正后的采集位姿融合一次。
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
import slf_match as slf  # noqa: E402

N = nf.N
SIN = np.array(nf.SIN, dtype=np.int64)

class FpgaModel:
    def __init__(self, trav="v3", inv_res=1311, cfg=None, mount_z=155):
        self.lut = np.array(lidar_lut.lut_entries(), dtype=np.int64)
        self.em = nf.ElevMap()
        self.inv_res = inv_res
        self.trav = trav
        if cfg is None:
            # v1 的参数即 RTL 寄存器默认值；v2/v3 用各自模块里按机器人能力给出的默认值
            cfg = dict(age_max=60, cnt_min=1, step_max=120, rough_max=120, slope_max=60,
                       k_slope=64, k_step=32, k_rough=32, infl_r=5) if trav == "v1" else dict(infl_r=4)
        self.cfg = cfg
        self.cost = np.full(N * N, 255, dtype=np.int64)
        self.cx = self.cy = 0
        self.seq = 0
        self.xyz = None
        self.pv = None
        self.last_lethal = None
        self.view = None
        self.mount_z = mount_z
        self.smap = slf.StructMap(inv_res)
        self._pw = []
        self.zlo, self.zhi = slf.Z_LO, slf.Z_HI
        self.spts = np.zeros((0, 3), dtype=np.int64)     # 本帧结构点缓冲（提交时的位姿下）
        self.stage = 0
        self.frame_id = 0
        self.struct_frame_id = -1
        self.fused_frame_id = -1
        self.match_cx = self.match_cy = 0
        self.coverage = None
        self.clock_us = 0
        self.capture_us = None

    def enable_support_coverage(self):
        from support_coverage import CompactCoverage
        self.coverage = CompactCoverage()

    # ---------------------------------------------------------------- 帧
    def frame(self, depth):
        if self.stage == 1 and self.struct_frame_id == self.frame_id and self.fused_frame_id != self.frame_id:
            raise RuntimeError("Raw frame overwritten before second-pass fusion")
        self.frame_id += 1
        self.xyz, self.pv = fast.pc_project(depth, self.lut)

    # ---------------------------------------------------------------- 结构点
    def struct_points(self, rt):
        R = np.array(rt[:9], dtype=np.int64).reshape(3, 3)
        t = np.array(rt[9:12], dtype=np.int64)
        p = fast.pc_transform(self.xyz, R, t)
        m = slf.structure_mask(p, self.pv, int(t[2]) - self.mount_z, self.zlo, self.zhi)
        idx = slf.subsample(np.nonzero(m)[0])
        return p[idx]

    def match_eval(self, cx_mm, cy_mm, cands):
        """cands: (K,3) dx dy dyaw。返回 (K,4) n_valid cost n_in n_pts（抽样后的结构点总数）"""
        if self.stage in (1, 2) and self.struct_frame_id != self.frame_id:
            raise RuntimeError("SLF match attempted with a previous frame's points")
        o = self.smap.score(self.spts[:, 0], self.spts[:, 1], cands, cx_mm, cy_mm)
        o[:, 3] = len(self.spts)
        return o

    # ---------------------------------------------------------------- 有图模式 / 控制（固件 hal_py 回调）
    def ctl(self, op, a, b, c):
        if op == 0:                                   # 清高程图
            self.em = nf.ElevMap()
            self.cost = np.full(N * N, 255, dtype=np.int64)
            if self.coverage is not None:
                self.coverage.clear()
        elif op == 1:                                 # 先验开始：清结构图
            self.smap = slf.StructMap(self.inv_res)
            self._pw = []
        elif op == 2:                                 # 写槽字
            self._pw.append((a & 0xFFFF, b & 0x3FFFF))
        elif op == 3:                                 # 有图开关
            if a:
                w = np.array(self._pw, dtype=np.int64).reshape(-1, 2)
                self.smap.load_prior(b, c, w[:, 0], w[:, 1])
            else:
                self.smap = slf.StructMap(self.inv_res)
            self._pw = []
        elif op == 4:                                 # 门控
            self.smap.nogate = not a
        elif op == 5:                                 # 结构点高度带（相对足底）
            self.zlo, self.zhi = a, b
        elif op == 6:                                 # 原始帧保留/重放；旧固件始终为0
            if a not in (0, 1, 2): raise ValueError("Invalid lidar phase")
            self.stage = a

    def match_update(self, dx, dy, dyaw, cx_mm, cy_mm):
        if self.smap.prior:                           # 有图：结构图固定，只释放点缓冲
            self.spts = self.spts[:0]
            return
        if len(self.spts):
            x, y = slf.apply_delta(self.spts[:, 0], self.spts[:, 1], dx, dy, dyaw, cx_mm, cy_mm)
            self.smap.update(np.stack([x, y], axis=1))
        # RTL completes LF update with MATCH_ONLY's global center held,
        # before FUSE_ONLY installs the continuous terrain-frame center.
        cx,cy=(self.match_cx,self.match_cy) if self.cfg.get('local_frame',0) else (self.cx,self.cy)
        self.smap.build_lf(cx,cy)
        self.spts = self.spts[:0]

    # ---------------------------------------------------------------- 融合遍
    def commit(self, rt, cx, cy, zmin, zmax):
        if self.stage == 1:
            self.match_cx,self.match_cy=cx,cy
            self.spts = self.struct_points(rt) if zmin <= zmax else self.spts[:0]
            self.struct_frame_id = self.frame_id
            return None                               # 不写高度，不增加地图帧龄/代际
        if self.stage == 2 and self.struct_frame_id != self.frame_id:
            raise RuntimeError("Fusion replay doesn't belong to retained raw frame")
        self.fused_frame_id = self.frame_id
        R = np.array(rt[:9], dtype=np.int64).reshape(3, 3)
        t = np.array(rt[9:12], dtype=np.int64)
        p = fast.pc_transform(self.xyz, R, t)
        pv = self.pv
        if self.cfg.get("fuse_r", 0) > 0:            # 融合距离门限（水平距离，mm）：远处点的高度受姿态误差放大，只用于定位
            r2 = (p[:, 0] - t[0]) ** 2 + (p[:, 1] - t[1]) ** 2
            pv = pv & (r2 <= self.cfg["fuse_r"] ** 2)
        a, tg, z = fast.pc_filter(p, pv, self.inv_res, cx, cy, max(-32768, zmin), min(32767, zmax))
        fast.elev_update(self.em, a, tg, z)
        if self.coverage is not None:
            valid = pv & (p[:, 2] >= max(-32768, zmin)) & (p[:, 2] <= min(32767, zmax))
            # Missing capture metadata must not manufacture current observations.
            if self.capture_us is None:
                valid = np.zeros_like(valid)
            self.coverage.update(p, valid, cx, cy,
                                 self.clock_us if self.capture_us is None else self.capture_us,
                                 self.clock_us)
        if self.trav == "v1":
            self.cost, self.last_lethal, self.view = fast.trav_v1(self.em, cx, cy, self.cfg)
        elif self.trav == "v2":
            self.cost, self.last_lethal, self.view = trav_v2.trav_v2(self.em, cx, cy, self.cfg)
        else:
            self.cost, self.last_lethal, self.view = trav_v3.trav_v3(self.em, cx, cy, self.cfg)
        self.cx, self.cy = cx, cy
        self.seq = (self.seq + 1) & 0xFFFF
        if self.stage != 2:
            self.spts = self.struct_points(rt) if zmin <= zmax else self.spts[:0]
        return p
