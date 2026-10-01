"""XT-M60 方向查找表：每个像素的视线单位向量（雷达本体系 X 前 Y 左 Z 上），RTL pc_project 用它把距离换成点。

依据厂家 SDK（github.com/XT-Toffuture/xtsdk_cpp：cartesianTransform.cpp、frame.cpp）核实：
  · 深度是沿视线的径向距离：点 = 距离 × 单位向量（SDK: z = d/sqrt(1+x'^2+y'^2)，x = x'z，y = y'z）。
  · M60 查内参时列号左右翻转：数据第 q 列对应标定坐标第 W-1-q 列（行不翻转）。
  · 去畸变为 Brown-Conrady 模型（同 OpenCV undistortPoints），内参顺序 fx fy cx cy k1 k2 k3 p1 p2。
    每帧帧尾 FrameInfo 带着这台雷达的内参；读不到时 SDK 用下面的 M60_DEFAULT。
  · 厂家上位机对 M60 默认切四角（cut_corner=120，M60 取 1/4 即半径 30 像素的倒角），这些像素方向置零 = 无效。
坐标：标定坐标系按 OpenCV 约定（x 右、y 下、z 前），换到本体系 X=z、Y=-x、Z=-y。左右正负待实物核对（docs/2_interface/icd.md）。

定点：每分量 12 位有符号 Q0.11（值/2048，饱和到 ±2047），36 位字 {rz, ry, rx}。
板上由固件 fw/nav/lidar_cal.c 按雷达自带内参生成同一张表（与本文件相差不超过 1 LSB，见 fw/test）。

用法: python model/lidar_lut.py --out lut.hex [--fx .. --fy .. --cx .. --cy .. --k1 ..] [--no-flip] [--cut 30]
"""
import argparse
import math

import numpy as np

W, H = 160, 60
LUT_FRAC = 11
LUT_MAX = (1 << LUT_FRAC) - 1
CUT_R = 30

# SDK cartesianTransform.cpp camParamsM60default
M60_DEFAULT = dict(fx=78.212524, fy=79.055527, cx=80.701973, cy=26.804274,
                   k1=-0.302470, k2=0.074865, k3=0.0, p1=-0.002307, p2=-0.001400)
LENS_KEYS = ("fx", "fy", "cx", "cy", "k1", "k2", "k3", "p1", "p2")
# 旧仿真用的理想针孔（水平 120°、垂直 45°，无畸变），只留作对比实验
IDEAL_120x45 = dict(fx=(W / 2) / math.tan(math.radians(60)), fy=(H / 2) / math.tan(math.radians(22.5)),
                    cx=(W - 1) / 2, cy=(H - 1) / 2, k1=0.0, k2=0.0, k3=0.0, p1=0.0, p2=0.0)


def undistort(xd, yd, k1, k2, k3, p1, p2, iters=4, newton=3):
    """Brown-Conrady 反解：畸变归一化坐标 -> 理想归一化坐标。
    先做几次不动点迭代（同 OpenCV）给初值，再用牛顿法收敛：畸变大时不动点迭代在图像四角收敛很慢
    （k1=-0.38 时 30 次仍差 3e-3），牛顿法 3 步即到机器精度。固件 lidar_cal.c 用同样的步骤。"""
    x, y = xd.copy(), yd.copy()
    for _ in range(iters):
        r2 = x * x + y * y
        rad = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        x = (xd - dx) / rad
        y = (yd - dy) / rad
    for _ in range(newton):
        r2 = x * x + y * y
        rad = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        drad = k1 + 2 * k2 * r2 + 3 * k3 * r2 * r2          # d rad / d r2
        fx = x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x) - xd
        fy = y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y - yd
        jxx = rad + 2 * x * x * drad + 2 * p1 * y + 6 * p2 * x
        jxy = 2 * x * y * drad + 2 * p1 * x + 2 * p2 * y        # 雅可比矩阵对称
        jyy = rad + 2 * y * y * drad + 6 * p1 * y + 2 * p2 * x
        det = jxx * jyy - jxy * jxy
        x = x - (jyy * fx - jxy * fy) / det
        y = y - (jxx * fy - jxy * fx) / det
    return x, y


def corner_mask(r=CUT_R, w=W, h=H):
    """厂家 setcutcorner 的倒角（标定坐标系下），True = 切除。r=0 不切。"""
    m = np.zeros((h, w), dtype=bool)
    if r <= 0:
        return m
    for v in range(h):
        for u in range(w):
            if u < r:
                if v < r and math.hypot(r - u, r - v) > r:
                    m[v, u] = True
                if v > h - r and math.hypot(r - u, v - (h - r)) > r:
                    m[v, u] = True
            if u > w - r:
                if v < r and math.hypot(u - (w - r), r - v) > r:
                    m[v, u] = True
                if v > h - r and math.hypot(u - (w - r), v - (h - r)) > r:
                    m[v, u] = True
    return m


def rays(intr=None, flip_u=True, flip_v=False, cut=CUT_R, w=W, h=H):
    """(h, w, 3) 单位向量（本体系 前/左/上），按数据的行列排布；切除的像素为零向量。"""
    p = dict(M60_DEFAULT)
    if intr:
        p.update(intr)
    q, r = np.meshgrid(np.arange(w), np.arange(h))
    us = (w - 1) - q if flip_u else q                 # 数据列 -> 标定列
    vs = (h - 1) - r if flip_v else r
    xd = (us - p["cx"]) / p["fx"]
    yd = (vs - p["cy"]) / p["fy"]
    xn, yn = undistort(xd, yd, p["k1"], p["k2"], p["k3"], p["p1"], p["p2"])
    d = np.stack([np.ones_like(xn), -xn, -yn], axis=-1)
    d /= np.linalg.norm(d, axis=-1, keepdims=True)
    if cut:
        d[corner_mask(cut, w, h)[vs, us]] = 0.0
    return d


def quantize(d):
    return np.clip(np.round(d * (1 << LUT_FRAC)), -LUT_MAX, LUT_MAX).astype(np.int64)


def lut_entries(**kw):
    q = quantize(rays(**kw)).reshape(-1, 3)
    return [tuple(int(a) for a in row) for row in q]


def pack(rx, ry, rz):
    return ((rz & 0xFFF) << 24) | ((ry & 0xFFF) << 12) | (rx & 0xFFF)


def write_hex(path, entries):
    with open(path, "w") as f:
        for rx, ry, rz in entries:
            f.write(f"{pack(rx, ry, rz):09x}\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    for k in LENS_KEYS:
        ap.add_argument(f"--{k}", type=float)
    ap.add_argument("--no-flip", action="store_true", help="数据列与标定列不翻转")
    ap.add_argument("--cut", type=int, default=CUT_R, help="四角倒角半径（像素），0 不切")
    a = ap.parse_args()
    intr = {k: getattr(a, k) for k in LENS_KEYS if getattr(a, k) is not None}
    e = lut_entries(intr=intr, flip_u=not a.no_flip, cut=a.cut)
    write_hex(a.out, e)
    n0 = sum(1 for t in e if t == (0, 0, 0))
    print(f"LUT: {len(e)} 项（切除 {n0}）-> {a.out}")


if __name__ == "__main__":
    main()
