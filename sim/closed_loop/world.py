"""闭环仿真场景：由带语义的几何体搭建（便于给出真实可通行性标签）。

几何体：
  box(x0,y0,x1,y1,h)          实心块（墙、箱子、台阶、路沿都是 box，按高度区分能否跨越）
  ramp(x0,x1,y0,y1,h0,h1)     沿 +x 或 -x 的斜坡
真实标签（5 cm 栅格）：由真实高程场按机器人能力判定（Robot.step_climb、Robot.slope_max_deg）：
  0 可通行，1 不可通行，2 忽略带（不可通行区外扩 1 格，5 cm 量化下边界有歧义，不计入统计）
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "model"))
import sim_world as sw  # noqa: E402


class Robot:
    step_climb = 0.10          # 可跨越台阶高度 m
    slope_max_deg = 20.0       # 可通行坡度
    length = 0.40              # 轮距（前后）
    width = 0.36               # 轮距（左右）
    radius = 0.25              # 外接圆半径


class Scene:
    def __init__(self, size=(12.0, 10.0), origin=(-2.0, -5.0), res=0.01):
        self.bounds = (origin[0], origin[1], origin[0] + size[0], origin[1] + size[1])
        ext = max(abs(v) for v in self.bounds) + 1.0
        self.w = sw.World(size_m=2 * ext, res=res)      # 以原点为中心的正方形高程场
        self.prims = []

    def box(self, x0, y0, x1, y1, h, name=""):
        self.w.box(x0, y0, x1, y1, h)
        self.prims.append(("box", (min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)), h, name))

    def ramp(self, x0, x1, y0, y1, h0, h1, name=""):
        """x0->x1 高度 h0->h1（x1 可小于 x0 表示反向坡）"""
        lo, hi = min(x0, x1), max(x0, x1)
        ix0, iy0 = self.w._idx(np.array(lo), np.array(min(y0, y1)))
        ix1, iy1 = self.w._idx(np.array(hi), np.array(max(y0, y1)))
        for ix in range(ix0, ix1):
            xm = self.w.o + (ix + 0.5) * self.w.res
            t = (xm - x0) / (x1 - x0)
            z = h0 + (h1 - h0) * t
            self.w.h[iy0:iy1, ix] = np.maximum(self.w.h[iy0:iy1, ix], z)
        ang = math.degrees(math.atan2(abs(h1 - h0), abs(x1 - x0)))
        self.prims.append(("ramp", (lo, min(y0, y1), hi, max(y0, y1)), ang, name))

    def walls(self, t=0.2, h=1.2):
        x0, y0, x1, y1 = self.bounds
        self.box(x0 - t, y0 - t, x1 + t, y0, h, "wall")
        self.box(x0 - t, y1, x1 + t, y1 + t, h, "wall")
        self.box(x0 - t, y0, x0, y1, h, "wall")
        self.box(x1, y0, x1 + t, y1, h, "wall")

    def height(self, x, y):
        return self.w.height(np.asarray(x, dtype=float), np.asarray(y, dtype=float))

    def gt_label(self, xs, ys, robot=Robot, cell=0.05):
        """xs, ys：栅格中心坐标（m，同形状）。直接由 1 cm 真实高程场判定：
          · 栅格中心 ±5 cm 区域内存在高度突变（相邻 1 cm 采样差 > 2 cm）且区域高差 > 可跨高度 -> 不可通行
          · 平滑部分的最大坡度 > 坡度上限 -> 不可通行
          · 不可通行区外扩 1 格为忽略带（5 cm 量化下边界本身有歧义），不计入统计
        """
        res = self.w.res
        H = self.w.h
        dxh = np.abs(np.diff(H, axis=1, append=H[:, -1:]))
        dyh = np.abs(np.diff(H, axis=0, append=H[-1:, :]))
        jump = (dxh > 0.02) | (dyh > 0.02)
        gx = np.gradient(H, res, axis=1)
        gy = np.gradient(H, res, axis=0)
        g = np.hypot(gx, gy)
        # 突变附近 2 cm 内的梯度不计入坡度
        jb = jump.copy()
        for k in (1, 2):
            jb |= np.roll(jump, k, 0) | np.roll(jump, -k, 0) | np.roll(jump, k, 1) | np.roll(jump, -k, 1)
        g = np.where(jb, 0.0, g)
        half = int(round(1.0 * cell / res))       # 以栅格中心 ±1 格（与 5 cm 分辨率一致）
        lab = np.zeros(xs.shape, dtype=np.int8)
        tmax = math.tan(math.radians(robot.slope_max_deg))
        flat = lab.reshape(-1)
        X, Y = xs.reshape(-1), ys.reshape(-1)
        for n in range(X.size):
            ix, iy = self.w._idx(np.array(X[n]), np.array(Y[n]))
            ix, iy = int(ix), int(iy)
            if ix < half or iy < half or ix >= self.w.n - half or iy >= self.w.n - half:
                flat[n] = 1
                continue
            blk = H[iy - half:iy + half + 1, ix - half:ix + half + 1]
            if jump[iy - half:iy + half + 1, ix - half:ix + half + 1].any() and blk.max() - blk.min() > robot.step_climb:
                flat[n] = 1
            elif g[iy - half:iy + half + 1, ix - half:ix + half + 1].max() > tmax:
                flat[n] = 1
        lab = flat.reshape(xs.shape)
        # 忽略带
        le = lab == 1
        dil = le.copy()
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                dil |= np.roll(np.roll(le, dy, 0), dx, 1)
        lab = np.where(le, 1, np.where(dil, 2, 0)).astype(np.int8)
        # 被不可通行区完全包围的可通行区（如高箱顶面）：局部平坦但到不了，标为忽略
        if xs.ndim == 2:
            from scipy import ndimage
            comp, n = ndimage.label(lab == 0)
            if n > 1:
                sizes = ndimage.sum(np.ones_like(comp), comp, index=np.arange(1, n + 1))
                main = 1 + int(np.argmax(sizes))
                lab[(comp != main) & (lab == 0)] = 2
        return lab

def _gt_lookup(self, wx, wy, cell=0.05):
    """任意坐标的真值标签：查预先计算的 5 cm 栅格（首次调用时生成并缓存）"""
    if not hasattr(self, "_gt"):
        x0, y0, x1, y1 = self.bounds
        gx = np.arange(x0 - 0.5, x1 + 0.5, cell) + cell / 2
        gy = np.arange(y0 - 0.5, y1 + 0.5, cell) + cell / 2
        X, Y = np.meshgrid(gx, gy)
        self._gt = self.gt_label(X, Y, cell=cell)
        self._gt_o = (gx[0] - cell / 2, gy[0] - cell / 2, cell)
    ox, oy, c = self._gt_o
    ix = np.clip(((np.asarray(wx) - ox) / c).astype(int), 0, self._gt.shape[1] - 1)
    iy = np.clip(((np.asarray(wy) - oy) / c).astype(int), 0, self._gt.shape[0] - 1)
    return self._gt[iy, ix]


Scene.gt_lookup = _gt_lookup


def scene_terrain():
    """地形综合场景：10° 坡道 + 20 cm 平台、8 cm 台阶带（可跨）、20 cm 路沿（不可跨，留缺口）、箱子。"""
    s = Scene(size=(12.0, 8.0), origin=(-2.0, -4.0))
    s.walls()
    s.ramp(1.0, 2.13, -1.2, 0.6, 0.0, 0.20, "ramp_up")          # 10°
    s.box(2.13, -1.2, 3.2, 0.6, 0.20, "platform")
    s.ramp(4.33, 3.2, -1.2, 0.6, 0.0, 0.20, "ramp_down")        # 反向 10°
    s.box(1.0, 1.4, 4.4, 2.2, 0.08, "step8")                     # 可跨越的 8 cm 台阶
    s.box(5.2, -4.0, 5.4, 1.0, 0.20, "curb20")                   # 不可跨越的路沿，y>1 有缺口
    s.box(6.5, -1.0, 7.1, -0.4, 0.45, "box1")
    s.box(7.5, 1.5, 8.1, 2.3, 0.45, "box2")
    s.ramp(8.5, 9.1, -3.0, -1.5, 0.0, 0.25, "steep")            # 22.6° 陡坡（不可通行）
    s.box(9.1, -3.0, 9.6, -1.5, 0.25, "steep_top")
    return s


def scene_loop():
    """定位回环场景：矩形跑道，中间一排障碍，四周墙体提供几何约束。"""
    s = Scene(size=(10.0, 7.0), origin=(-1.5, -3.5))
    s.walls()
    s.box(1.0, -1.0, 6.0, 1.0, 0.6, "island")
    s.box(-0.8, 2.2, -0.2, 2.8, 0.4, "c1")
    s.box(7.2, -2.9, 7.8, -2.3, 0.4, "c2")
    s.box(3.0, 2.6, 3.6, 3.2, 0.4, "c3")
    s.ramp(2.0, 3.13, -3.4, -1.8, 0.0, 0.2, "r1")
    s.box(3.13, -3.4, 4.0, -1.8, 0.2, "r1top")
    s.ramp(5.13, 4.0, -3.4, -1.8, 0.0, 0.2, "r1down")
    return s


SCENES = {"terrain": scene_terrain, "loop": scene_loop}
