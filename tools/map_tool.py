"""先验地图转换工具：赛前扫图得到的地图点云 -> 板上用的先验地图包（.nvmp），并出一张检查图。

流程：推着车（或遥控）在场地里走一圈，录 XT-M60 + IMU（+ 轮速），在 Linux 上用现成的激光惯性 SLAM
（FAST-LIO 等）建图，导出整张地图点云（地图系，单位 m）；地图系就是之后下发目标点、初始位姿用的坐标系。
地面应大致水平（SLAM 的 z 轴向上）。

用法:
  python tools/map_tool.py build map.pcd -o field.nvmp [--bounds x0,y0,x1,y1] [--step 100] [--slope 20]
  python tools/map_tool.py show field.nvmp -o field.png
支持的点云：.pcd（ascii / binary，字段 x y z）、.ply（ascii / binary_little_endian）、.npy（N×3）、.xyz/.txt（每行 x y z）。
限制：场地（含墙）不超过 12.8 m × 12.8 m（片上结构图）；代价图 19.2 m 以内。
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "model"))
import prior_map as pm  # noqa: E402


def load_cloud(path):
    p = Path(path)
    ext = p.suffix.lower()
    if ext == ".npy":
        return np.load(p)[:, :3].astype(np.float64)
    if ext in (".xyz", ".txt"):
        return np.loadtxt(p, usecols=(0, 1, 2))
    raw = p.read_bytes()
    if ext == ".pcd":
        head, _, body = raw.partition(b"DATA ")
        kind, _, data = body.partition(b"\n")
        h = {}
        for ln in head.decode("ascii", "replace").splitlines():
            t = ln.split()
            if t:
                h[t[0].upper()] = t[1:]
        fields = h["FIELDS"]
        sizes = [int(v) for v in h["SIZE"]]
        types = h["TYPE"]
        counts = [int(v) for v in h.get("COUNT", ["1"] * len(fields))]
        n = int(h["POINTS"][0])
        if kind.strip() == b"ascii":
            a = np.loadtxt(data.decode().splitlines())
            cols = []
            k = 0
            for f, c in zip(fields, counts):
                cols.append((f, k))
                k += c
            idx = {f: i for f, i in cols}
            return a[:, [idx["x"], idx["y"], idx["z"]]]
        if kind.strip() != b"binary":
            raise ValueError("只支持 ascii / binary 的 PCD（binary_compressed 请先用 pcl_convert_pcd_ascii_binary 转换）")
        dt = []
        for f, s, t, c in zip(fields, sizes, types, counts):
            code = {"F": "f", "I": "i", "U": "u"}[t] + str(s)
            dt.append((f if f not in [d[0] for d in dt] else f + "_", code, (c,)) if c > 1 else (f, code))
        a = np.frombuffer(data[:n * np.dtype(dt).itemsize], dtype=np.dtype(dt))
        return np.stack([a["x"], a["y"], a["z"]], axis=1).astype(np.float64)
    if ext == ".ply":
        head, _, data = raw.partition(b"end_header\n")
        lines = head.decode("ascii", "replace").splitlines()
        fmt = [ln for ln in lines if ln.startswith("format")][0].split()[1]
        n = 0
        props = []
        in_vertex = False
        for ln in lines:
            t = ln.split()
            if t[:2] == ["element", "vertex"]:
                n, in_vertex = int(t[2]), True
            elif t[:1] == ["element"]:
                in_vertex = False
            elif t[:1] == ["property"] and in_vertex:
                props.append((t[-1], t[1]))
        if fmt == "ascii":
            a = np.loadtxt(data.decode().splitlines()[:n])
            idx = {name: i for i, (name, _) in enumerate(props)}
            return a[:, [idx["x"], idx["y"], idx["z"]]]
        tmap = {"float": "<f4", "float32": "<f4", "double": "<f8", "float64": "<f8", "uchar": "u1", "uint8": "u1",
                "int": "<i4", "int32": "<i4", "uint": "<u4", "short": "<i2", "ushort": "<u2"}
        dt = np.dtype([(name, tmap[t]) for name, t in props])
        a = np.frombuffer(data[:n * dt.itemsize], dtype=dt)
        return np.stack([a["x"], a["y"], a["z"]], axis=1).astype(np.float64)
    raise ValueError(f"不支持的点云格式 {ext}")


def show(pkg_bytes, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
    d = pm.parse_package(pkg_bytes)
    w = np.zeros(pm.SN * pm.SN, dtype=np.int64)
    ox, oy = d["ox"], d["oy"]
    wy, wx = np.mgrid[0:pm.SN, 0:pm.SN]
    s = (((wy + oy) & 255) << 8) | ((wx + ox) & 255)
    w[d["slot"]] = d["word"]
    ww = w[s]
    occ, gate = (ww >> 7) & 1, ww & 1
    fig, ax = plt.subplots(1, 2, figsize=(16, 7))
    c = 0.05
    ax[0].imshow(occ * 2 + gate, origin="lower", extent=(ox * c, (ox + 256) * c, oy * c, (oy + 256) * c), cmap="viridis")
    ax[0].set_title(f"定位图：竖直结构 {int(occ.sum())} 格（亮）与门控区（暗）")
    g = d["gcost"].astype(float)
    gx, gy = d["gox"] * 0.2, d["goy"] * 0.2
    ax[1].imshow(np.where(g == 255, np.nan, g), origin="lower", cmap="magma_r", vmin=0, vmax=254,
                 extent=(gx, gx + d["gnx"] * 0.2, gy, gy + d["gny"] * 0.2))
    ax[1].set_title("全局代价图（20 cm；黑=致命，白=未知）")
    for a in ax:
        a.set_aspect("equal")
        a.set_xlabel("x / m")
    fig.savefig(out, dpi=90)
    print("保存", out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["build", "show"])
    ap.add_argument("src")
    ap.add_argument("-o", "--out")
    ap.add_argument("--bounds", help="场地范围 x0,y0,x1,y1（m），缺省取点云范围")
    ap.add_argument("--step", type=int, default=100, help="可跨越台阶 mm")
    ap.add_argument("--slope", type=float, default=20.0, help="可通行坡度 °")
    ap.add_argument("--min-count", type=int, default=3, help="结构格最少点数（稀疏点云可调小）")
    a = ap.parse_args()
    if a.cmd == "show":
        show(Path(a.src).read_bytes(), a.out or Path(a.src).with_suffix(".png"))
        return 0
    cloud = load_cloud(a.src) * 1000.0
    cloud = cloud[np.isfinite(cloud).all(1)]
    b = None
    if a.bounds:
        b = tuple(float(v) * 1000 for v in a.bounds.split(","))
        m = (cloud[:, 0] >= b[0]) & (cloud[:, 0] <= b[2]) & (cloud[:, 1] >= b[1]) & (cloud[:, 1] <= b[3])
        cloud = cloud[m]
    print(f"点云 {len(cloud)} 点，范围 x {cloud[:, 0].min() / 1000:.2f}..{cloud[:, 0].max() / 1000:.2f} m，"
          f"y {cloud[:, 1].min() / 1000:.2f}..{cloud[:, 1].max() / 1000:.2f} m")
    sp = pm.struct_points_from_cloud(cloud.astype(np.int64))
    pkg, info = pm.build_package(sp, cloud.astype(np.int64), bounds_mm=b, step_max=a.step, slope_max_deg=a.slope,
                                 min_count=a.min_count)
    out = Path(a.out or Path(a.src).with_suffix(".nvmp"))
    out.write_bytes(pkg)
    print(f"结构格 {info['n_struct']}，槽字 {info['n_words']}，代价图 {info['gnx']}×{info['gny']}，包 {len(pkg)} 字节 -> {out}")
    show(pkg, out.with_suffix(".png"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
