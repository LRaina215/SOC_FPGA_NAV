"""仿真世界：2.5D 高程场景 + 虚拟 XT-M60 + 位姿工具。

用于在没有实机时产生"真实格式"的雷达数据，驱动 RTL 测试与固件主机测试。
坐标：世界/里程计系 X 前 Y 左 Z 上，单位 m（对外接口换算为 mm）。
"""
import math

import numpy as np

import lidar_lut


class World:
    """高程图场景：res 米/格，原点在地图中心。"""

    def __init__(self, size_m=16.0, res=0.01):
        self.res = res
        self.n = int(round(size_m / res))
        self.h = np.zeros((self.n, self.n), dtype=np.float32)   # [iy, ix]
        self.o = -size_m / 2

    def _idx(self, x, y):
        return (np.floor((x - self.o) / self.res).astype(np.int64),
                np.floor((y - self.o) / self.res).astype(np.int64))

    def box(self, x0, y0, x1, y1, z):
        ix0, iy0 = self._idx(np.array(min(x0, x1)), np.array(min(y0, y1)))
        ix1, iy1 = self._idx(np.array(max(x0, x1)), np.array(max(y0, y1)))
        self.h[iy0:iy1, ix0:ix1] = np.maximum(self.h[iy0:iy1, ix0:ix1], z)

    def ramp_x(self, x0, x1, y0, y1, z0, z1, plateau_to=None):
        """沿 +x 从 z0 升到 z1 的坡；plateau_to 给出坡顶平台延伸到的 x。"""
        ix0, iy0 = self._idx(np.array(x0), np.array(min(y0, y1)))
        ix1, iy1 = self._idx(np.array(x1), np.array(max(y0, y1)))
        for ix in range(ix0, ix1):
            z = z0 + (z1 - z0) * (ix - ix0) / max(1, ix1 - ix0)
            self.h[iy0:iy1, ix] = np.maximum(self.h[iy0:iy1, ix], z)
        if plateau_to is not None:
            self.box(x1, y0, plateau_to, y1, z1)

    def height(self, x, y):
        ix, iy = self._idx(x, y)
        inside = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
        out = np.full(np.shape(x), 3.0, dtype=np.float32)          # 场景外视为高墙
        out[inside] = self.h[iy[inside], ix[inside]]
        return out

    def raycast(self, origin, dirs, tmax=20.0, step=0.01):
        """origin (3,), dirs (M,3) 单位向量 -> 命中距离 (M,)，未命中为 inf。"""
        m = dirs.shape[0]
        hit = np.full(m, np.inf)
        alive = np.ones(m, dtype=bool)
        t = step
        prev_above = np.ones(m, dtype=bool)
        while t <= tmax and alive.any():
            p = origin[None, :] + dirs * t
            below = p[:, 2] <= self.height(p[:, 0], p[:, 1])
            newhit = alive & below
            hit[newhit] = t
            alive &= ~newhit
            t += step
        return hit


def default_scene():
    """测试场景：围墙、箱子、10° 坡道 + 平台、8 cm 台阶、20 cm 路沿。"""
    w = World()
    w.box(-8, -8, 8, -7.8, 1.5)
    w.box(-8, 7.8, 8, 8, 1.5)
    w.box(-8, -8, -7.8, 8, 1.5)
    w.box(7.8, -8, 8, 8, 1.5)
    w.box(3.0, 1.0, 3.5, 1.6, 0.4)          # 箱子
    w.box(2.0, -2.2, 2.4, -1.6, 0.8)        # 高箱
    w.ramp_x(4.0, 5.13, -1.0, 0.5, 0.0, 0.2, plateau_to=6.5)   # 10° 坡（1.13 m 升 0.2 m）
    w.box(1.5, 2.5, 3.0, 4.0, 0.08)          # 8 cm 台阶（可通过）
    w.box(-1.0, -3.5, 3.0, -3.3, 0.20)       # 20 cm 路沿（不可通过）
    return w


def rotz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def roty(pitch):
    c, s = math.cos(pitch), math.sin(pitch)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rotx(roll):
    c, s = math.cos(roll), math.sin(roll)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


class VirtualLidar:
    """虚拟 XT-M60：输出 H×W 原始深度（mm, uint16）与幅值。"""

    def __init__(self, depth_def="z", intr=None, rng_min=0.1, rng_max=20.0, noise_mm=0.0, seed=0, step=0.01):
        self.step = step
        self.depth_def = depth_def
        self.dirs_body = lidar_lut.rays(intr=intr, depth_def="z")        # 前向分量 = 1
        self.unit = self.dirs_body / np.linalg.norm(self.dirs_body, axis=-1, keepdims=True)
        self.rng_min, self.rng_max = rng_min, rng_max
        self.noise = noise_mm
        self.rng = np.random.default_rng(seed)

    def capture(self, world, R_ws, t_ws):
        """R_ws/t_ws：雷达本体系 -> 世界系。返回 depth(uint16 H×W), amp(uint16 H×W)。"""
        h, w, _ = self.unit.shape
        d_w = (R_ws @ self.unit.reshape(-1, 3).T).T
        r = world.raycast(np.asarray(t_ws, dtype=float), d_w, tmax=self.rng_max, step=self.step)
        r = r.reshape(h, w)
        if self.depth_def == "z":
            val = r * self.unit[..., 0]                   # 光轴方向分量
        else:
            val = r
        mm = np.round(val * 1000 + (self.rng.normal(0, self.noise, val.shape) if self.noise else 0))
        ok = np.isfinite(r) & (r >= self.rng_min) & (r <= self.rng_max)
        depth = np.where(ok, np.clip(mm, 1, 65535), 0).astype(np.int64)
        amp = np.where(ok, np.clip(3000.0 / (1.0 + np.nan_to_num(r, posinf=99)), 0, 65535), 0).astype(np.int64)
        return depth, amp


def mount_pose(height=0.2, pitch_down_deg=10.0, x=0.1):
    """雷达相对机身的安装：前移 x、高 height、向下俯 pitch_down（绕 Y 正转为低头）。"""
    return roty(math.radians(pitch_down_deg)), np.array([x, 0.0, height])


def compose(R_wb, t_wb, R_bs, t_bs):
    return R_wb @ R_bs, R_wb @ t_bs + t_wb


def to_fixed_pose(R, t_m):
    """浮点位姿 -> RTL 寄存器值：R Q1.14，t mm。"""
    Rq = [[int(round(v * 16384)) for v in row] for row in R]
    tq = [int(round(v * 1000)) for v in t_m]
    return Rq, tq
