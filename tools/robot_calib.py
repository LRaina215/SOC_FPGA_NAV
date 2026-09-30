"""由整车 URDF + STL（SolidWorks 导出，机械组提供）标定导航用的几何参数：雷达安装位姿、轮半径、轮距、外形尺寸、质心。

URDF 是树结构，每侧腿的"前大腿+前小腿""后大腿+后小腿"各被合成一个刚体，两条链在轮轴处的闭环（五连杆）表达不了，
所以这里不按关节角去摆腿，只用导出时的装配姿态取几何量；站姿按平衡车的稳态处理：两个髋关节中点正下方是轮轴
（虚拟腿竖直），机身水平（机身质心几乎就在髋关节中点，见输出）。腿长 L0（髋关节中点到轮轴）由底盘控制给定，作为参数。

坐标：URDF base 系 X 向右、Y 向前、Z 向上（由 Front/Back、Left/Right 关节位置推出）；
导航机体系 X 向前、Y 向左、Z 向上，原点在两轮轴线中点正上方的地面。

用法:
  python tools/robot_calib.py [URDF 路径] [--leg 130] [--json config/robot_calib.json]
"""
import argparse
import json
import math
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

DEFAULT_URDF = r"D:\Desktop\SOC FPGA\Structure\car\car\urdf\car.urdf"
M60_W, M60_H = 65.0, 38.0          # 雷达正面宽 × 高（mm，手册）


def load_stl(p):
    b = Path(p).read_bytes()
    n = struct.unpack_from("<I", b, 80)[0]
    a = np.frombuffer(b, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n, offset=84)
    return a["v"].astype(np.float64) * 1000.0, a["n"].astype(np.float64)


def parse_urdf(path):
    root = ET.parse(path).getroot()
    links = {}
    for l in root.findall("link"):
        i = l.find("inertial")
        mesh = l.find("visual/geometry/mesh").get("filename").split("/")[-1]
        links[l.get("name")] = dict(mass=float(i.find("mass").get("value")),
                                    com=np.array([float(v) for v in i.find("origin").get("xyz").split()]) * 1000,
                                    mesh=mesh)
    joints = {}
    for j in root.findall("joint"):
        joints[j.find("child").get("link")] = dict(name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
                                                  xyz=np.array([float(v) for v in j.find("origin").get("xyz").split()]) * 1000,
                                                  rpy=[float(v) for v in j.find("origin").get("rpy").split()])
    return links, joints


def link_origin(joints, name):
    """零位（导出姿态）下 link 原点在 base 系的位置；导出的关节 rpy 全为 0，只累加平移"""
    o = np.zeros(3)
    while name in joints:
        j = joints[name]
        assert all(abs(r) < 1e-9 for r in j["rpy"]), "关节带旋转，需要完整变换"
        o += j["xyz"]
        name = j["parent"]
    return o


def components(tri):
    """按共享顶点把三角形分成连通件"""
    key = np.round(tri.reshape(-1, 3) * 100).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    f = inv.reshape(-1, 3)
    par = np.arange(inv.max() + 1)

    def find(a):
        while par[a] != a:
            par[a] = par[par[a]]
            a = par[a]
        return a
    for a, b, c in f:
        ra = find(a)
        par[find(b)] = ra
        par[find(c)] = ra
    return np.array([find(v) for v in f[:, 0]])


def find_lidar(tri, nrm):
    """在机身网格里找 XT-M60 外壳：宽约 65 mm、在最前面的连通件；正面朝向 = 面积加权法向里 y 分量为正的主峰"""
    comp = components(tri)
    best = None
    for c in np.unique(comp):
        v = tri[comp == c].reshape(-1, 3)
        w = v[:, 0].max() - v[:, 0].min()
        if abs(w - M60_W) < 2.5 and (best is None or v[:, 1].max() > best[1]):
            best = (c, v[:, 1].max())
    if best is None:
        raise SystemExit("机身网格里没找到宽 65 mm 的雷达外壳")
    m = comp == best[0]
    T, N = tri[m], nrm[m]
    area = 0.5 * np.linalg.norm(np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]), axis=1)
    ang = np.degrees(np.arctan2(N[:, 2], N[:, 1]))
    sel = (np.abs(N[:, 0]) < 0.3) & (np.abs(ang) < 60)
    h, e = np.histogram(ang[sel], bins=240, range=(-60, 60), weights=area[sel])
    k = h.argmax()
    near = sel & (np.abs(ang - (e[k] + e[k + 1]) / 2) < 2.0)
    tilt = float(np.average(ang[near], weights=area[near]))        # 正面法向相对水平前方的仰角（负 = 朝下）
    V = T.reshape(-1, 3)
    n = np.array([0.0, math.cos(math.radians(tilt)), math.sin(math.radians(tilt))])     # 光轴方向（正面法向）
    up = np.array([0.0, -n[2], n[1]])                                                  # 雷达自身的"上"
    d, u = V @ n, V @ up
    u_mid = (u.min() + u.max()) / 2
    # 正面中心（镜头位置按正面中心计，偏差在 ±10 mm 内）
    face_c = n * d.max() + up * u_mid
    face_c[0] = (V[:, 0].min() + V[:, 0].max()) / 2
    # 防水接插件：外壳正后方紧挨着的连通件，看它在外壳"上 / 下"哪一半（手册 4.3 节：正装时接插件靠下）
    conn = None
    for c in np.unique(comp):
        if c == best[0]:
            continue
        v = tri[comp == c].reshape(-1, 3)
        if v[:, 0].min() > V[:, 0].min() - 1 and v[:, 0].max() < V[:, 0].max() + 1 and len(v) > 300 and \
                V[:, 1].min() - 3 < v[:, 1].max() < V[:, 1].min() + 15:
            conn = dict(center=v.mean(0), up=float(v.mean(0) @ up - u_mid))
    return dict(tilt_deg=tilt, face_center=face_c, width=V[:, 0].max() - V[:, 0].min(),
                height=u.max() - u.min(), depth=d.max() - d.min(), connector=conn)


def calibrate(urdf, leg_mm):
    urdf = Path(urdf)
    mesh_dir = urdf.parent.parent / "meshes"
    links, joints = parse_urdf(urdf)
    P = {n: link_origin(joints, n) for n in links}
    tri_b, nrm_b = load_stl(mesh_dir / links["base"]["mesh"])
    lid = find_lidar(tri_b, nrm_b)
    # 髋关节、轮轴
    hips = {n: P[n] for n in links if n.endswith(("Front_link", "Back_link"))}
    hip_f = np.mean([P[n] for n in hips if "Front" in n], axis=0)
    hip_b = np.mean([P[n] for n in hips if "Back" in n], axis=0)
    hip_mid = (hip_f + hip_b) / 2
    x_c = float(np.mean([P[n][0] for n in hips]))                     # 左右对称面
    wheels = {n: P[n] for n in links if "Wheel" in n}
    # 轮子：半径、胎面宽度与中心（轮轴沿 X）
    wh = {}
    for n in wheels:
        t, _ = load_stl(mesh_dir / links[n]["mesh"])
        v = t.reshape(-1, 3)
        r = np.hypot(v[:, 1], v[:, 2])
        tread = v[r > r.max() - 3.0]
        wh[n] = dict(radius=float(r.max()), tread_x=float((tread[:, 0].min() + tread[:, 0].max()) / 2 + P[n][0]),
                     tread_w=float(tread[:, 0].max() - tread[:, 0].min()), axle=P[n])
    # 整车外形（导出姿态）
    allv = [tri_b.reshape(-1, 3)]
    for n in links:
        if n != "base":
            t, _ = load_stl(mesh_dir / links[n]["mesh"])
            allv.append(t.reshape(-1, 3) + P[n])
    A = np.concatenate(allv)
    # 质心
    mt = sum(l["mass"] for l in links.values())
    com = sum(l["mass"] * (l["com"] + P[n]) for n, l in links.items()) / mt
    legs = {}
    for side in ("Right", "Left"):
        ax = wheels[f"{side}Wheel_link"]
        d = ax - hip_mid
        legs[side] = dict(L0=float(math.hypot(d[1], d[2])), theta_deg=float(math.degrees(math.atan2(d[1], -d[2]))))
    R = float(np.mean([w["radius"] for w in wh.values()]))
    fc = lid["face_center"]
    # 导航机体系（原点：两轮轴线中点正下方地面；站姿：轮轴在髋关节中点正下方、机身水平）
    mount = dict(pitch_down_deg=-lid["tilt_deg"], x_mm=float(fc[1] - hip_mid[1]), y_mm=float(-(fc[0] - x_c)),
                 z_above_hip_mm=float(fc[2] - hip_mid[2]))
    mount["z_mm"] = leg_mm + R + mount["z_above_hip_mm"]
    track = abs(wh["RightWheel_link"]["tread_x"] - wh["LeftWheel_link"]["tread_x"])
    ext = dict(width=float(A[:, 0].max() - A[:, 0].min()), front=float(A[:, 1].max() - hip_mid[1]),
               rear=float(hip_mid[1] - A[:, 1].min()), half_w=float(max(A[:, 0].max() - x_c, x_c - A[:, 0].min())))
    ext["radius"] = float(math.hypot(max(ext["front"], ext["rear"]), ext["half_w"]))
    return dict(urdf=str(urdf), mass_kg=mt, com_rel_hip_mid=(com - hip_mid).tolist(),
                base_com_rel_hip_mid=(links["base"]["com"] - hip_mid).tolist(),
                hip_spacing_mm=float(np.linalg.norm(hip_f - hip_b)), legs_export=legs,
                wheel_radius_mm=R, tread_width_mm=float(np.mean([w["tread_w"] for w in wh.values()])),
                track_mm=float(track), extent_mm=ext, lidar=dict(
                    tilt_deg=lid["tilt_deg"], housing_mm=[lid["width"], lid["height"], lid["depth"]],
                    face_center_base_mm=fc.tolist(),
                    connector_up_mm=None if lid["connector"] is None else lid["connector"]["up"]),
                leg_mm=leg_mm, mount=mount)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("urdf", nargs="?", default=DEFAULT_URDF)
    ap.add_argument("--leg", type=float, default=130.0, help="站立腿长 L0 mm（髋关节中点到轮轴，底盘控制的设定值）")
    ap.add_argument("--json", help="结果另存为 JSON")
    a = ap.parse_args()
    c = calibrate(a.urdf, a.leg)
    L, m, e = c["lidar"], c["mount"], c["extent_mm"]
    print(f"URDF: {c['urdf']}")
    print(f"整车质量 {c['mass_kg'] * 1000:.0f} g（只含模型里有的零件）；质心相对髋关节中点（右 / 前 / 上）"
          f" {np.round(c['com_rel_hip_mid'], 1)} mm，机身 {np.round(c['base_com_rel_hip_mid'], 1)} mm")
    print(f"髋关节前后间距 {c['hip_spacing_mm']:.1f} mm；导出姿态："
          + "，".join(f"{s} 腿 L0 {v['L0']:.1f} mm / 前倾 {v['theta_deg']:+.1f}°" for s, v in c["legs_export"].items()))
    print(f"轮半径 {c['wheel_radius_mm']:.2f} mm，胎宽 {c['tread_width_mm']:.1f} mm，轮距（胎面中心）{c['track_mm']:.1f} mm")
    print(f"外形：宽 {e['width']:.0f} mm；轮轴中点前方 {e['front']:.0f} mm、后方 {e['rear']:.0f} mm（导出姿态）；"
          f"绕轮轴中点转动的外接圆半径 {e['radius']:.0f} mm")
    print(f"雷达外壳 {L['housing_mm'][0]:.1f} × {L['housing_mm'][1]:.1f} × {L['housing_mm'][2]:.1f} mm（宽 × 高 × 深），"
          f"光轴朝下 {-L['tilt_deg']:.1f}°；正面中心（base 系）{np.round(L['face_center_base_mm'], 1)}")
    cu = L["connector_up_mm"]
    if cu is None:
        print("防水接插件：没找到")
    elif abs(cu) < 0.15 * L["housing_mm"][1]:
        print(f"防水接插件在背面高度居中（{cu:+.1f} mm），从模型判断不了正装还是倒装，实物按 hardware_guide 6.5 节核对上下")
    else:
        print(f"防水接插件在外壳{'上' if cu > 0 else '下'}半部（{cu:+.1f} mm）"
              + ("——手册 4.3 节要求正装时接插件靠下，按模型是倒装，实物请确认" if cu > 0 else "，与手册的正装方向一致"))
    print(f"\n导航安装位姿（站姿：轮轴在髋关节中点正下方、机身水平；L0 = {c['leg_mm']:.0f} mm）：")
    print(f"  下俯 {m['pitch_down_deg']:.1f}°，前伸 x {m['x_mm']:.0f} mm，左右 y {m['y_mm']:+.0f} mm（+ 为左），"
          f"离地 z {m['z_mm']:.0f} mm（= L0 + 轮半径 {m['z_above_hip_mm'] + c['wheel_radius_mm']:+.1f} mm）")
    print("  L0 与离地高度：" + "  ".join(f"{l} -> {l + c['wheel_radius_mm'] + m['z_above_hip_mm']:.0f}" for l in range(100, 171, 10)))
    print(f"  固件 fw/nav/nav_task.c：pose_init(&nt->pose, ANGLE_FROM_CDEG({round(m['pitch_down_deg'] * 100)}), "
          f"{round(m['x_mm'])}, {round(m['z_mm'])});  board_init.py --mount-z {round(m['z_mm'])}")
    if a.json:
        Path(a.json).write_text(json.dumps(c, ensure_ascii=False, indent=1), encoding="utf-8")
        print(f"\n已写入 {a.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
