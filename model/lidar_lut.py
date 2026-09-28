"""XT-M60 方向查找表生成：内参 + 畸变 -> 每像素方向向量（雷达本体系 X 前 Y 左 Z 上，Q2.13）。

内参来源：雷达 TCP 命令 18 返回 fx fy cx cy k1 k2 k3 p1 p2（float32）。
需实机确认、这里做成参数的三件事：
  depth_def   'z'：深度是光轴方向距离；'radial'：沿视线距离（用 SDK 输出点云比对即可确定）
  flip_u/v    像素列/行增大方向与图像右/下是否相反
坐标轴换算：相机模式 (Z 前, X 左, Y 上) 由本表直接换到本体系，RTL 不再处理。

用法: python model/lidar_lut.py --out rtl/lidar/lut_default.hex [--fx .. --fy .. --cx .. --cy .. --k1 ..]
"""
import argparse
import math

import numpy as np

W, H = 160, 60
LUT_FRAC = 13

# 仿真默认内参：理想针孔，水平 120°、垂直 45°
DEFAULT = dict(fx=(W / 2) / math.tan(math.radians(60)), fy=(H / 2) / math.tan(math.radians(22.5)),
               cx=(W - 1) / 2, cy=(H - 1) / 2, k1=0.0, k2=0.0, k3=0.0, p1=0.0, p2=0.0)


def undistort(xd, yd, k1, k2, k3, p1, p2, iters=20):
    """Brown-Conrady 反解：畸变归一化坐标 -> 理想归一化坐标（迭代）。"""
    x, y = xd.copy(), yd.copy()
    for _ in range(iters):
        r2 = x * x + y * y
        rad = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        x = (xd - dx) / rad
        y = (yd - dy) / rad
    return x, y


def rays(intr=None, depth_def="z", flip_u=False, flip_v=False, w=W, h=H):
    """返回 (h, w, 3) 浮点方向：depth_def='z' 时前向分量为 1；'radial' 时为单位向量。"""
    p = dict(DEFAULT)
    if intr:
        p.update(intr)
    u, v = np.meshgrid(np.arange(w, dtype=float), np.arange(h, dtype=float))
    if flip_u:
        u = (w - 1) - u
    if flip_v:
        v = (h - 1) - v
    xd = (u - p["cx"]) / p["fx"]
    yd = (v - p["cy"]) / p["fy"]
    xn, yn = undistort(xd, yd, p["k1"], p["k2"], p["k3"], p["p1"], p["p2"])
    # 相机系 (X 左, Y 上, Z 前) 中，图像右 = -X，图像下 = -Y
    d = np.stack([np.ones_like(xn), -xn, -yn], axis=-1)     # 本体系 (前, 左, 上)
    if depth_def == "radial":
        d /= np.linalg.norm(d, axis=-1, keepdims=True)
    return d


def quantize(d):
    q = np.round(d * (1 << LUT_FRAC)).astype(np.int64)
    return np.clip(q, -32768, 32767)


def lut_entries(**kw):
    q = quantize(rays(**kw)).reshape(-1, 3)
    return [tuple(int(a) for a in row) for row in q]


def write_hex(path, entries):
    with open(path, "w") as f:
        for rx, ry, rz in entries:
            f.write(f"{((rz & 0xFFFF) << 32) | ((ry & 0xFFFF) << 16) | (rx & 0xFFFF):012x}\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    for k in DEFAULT:
        ap.add_argument(f"--{k}", type=float)
    ap.add_argument("--depth-def", default="z", choices=["z", "radial"])
    ap.add_argument("--flip-u", action="store_true")
    ap.add_argument("--flip-v", action="store_true")
    a = ap.parse_args()
    intr = {k: getattr(a, k) for k in DEFAULT if getattr(a, k) is not None}
    e = lut_entries(intr=intr, depth_def=a.depth_def, flip_u=a.flip_u, flip_v=a.flip_v)
    write_hex(a.out, e)
    print(f"LUT: {len(e)} 项 -> {a.out}")


if __name__ == "__main__":
    main()
