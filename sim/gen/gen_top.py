"""nav_subsystem 端到端测试激励：雷达以太网帧、每帧位姿、期望代价图、轨迹评估期望、主机报文。

用法: gen_top.py <输出目录>
"""
import math
import struct
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "model"))
import lidar_lut  # noqa: E402
import nav_fixed as nf  # noqa: E402
import netpkt as npk  # noqa: E402
import sim_world as sw  # noqa: E402
import xt_proto as xt  # noqa: E402

FPGA_L_MAC, FPGA_L_IP = 0x024E56000001, "192.168.1.100"
LIDAR_MAC, LIDAR_IP = 0x0CAFE0000060, "192.168.1.60"
FPGA_H_MAC, FPGA_H_IP = 0x024E56000002, "192.168.2.100"
RK_MAC, RK_IP = 0x0A1B2C3D4E5F, "192.168.2.10"
INV_RES = 1311
import trav_v3  # noqa: E402
MAPCFG = dict(trav_v3.DEFAULT_V3)      # DROP/FILL 寄存器用复位值（与此相同）
ZMIN, ZMAX = -500, 1500
TRAJ = [(0.0, 0.0, 0, 0), (0.5, 0.2, 10, 2), (1.0, 0.4, 20, 0)]


def w32(lines, v):
    lines.append(f"{v & 0xFFFFFFFF:08x}")


def main(out):
    out = Path(out)
    world = sw.default_scene()
    lid = sw.VirtualLidar(noise_mm=3.0, seed=5)
    Rbs, tbs = sw.mount_pose()
    lut = lidar_lut.lut_entries()
    lidar_lut.write_hex(out / "lut_default.hex", lut)

    eth = []           # 雷达口字节 {last, data}
    pose = []
    cost_lines = []

    def put(frame):
        for i, b in enumerate(frame):
            eth.append(f"{(int(i == len(frame) - 1) << 8) | b:03x}")

    put(npk.arp_request_frame(LIDAR_MAC, LIDAR_IP, FPGA_L_IP))
    em = nf.ElevMap()
    last_cost, last_c = None, None
    for k, (x, y, yaw, pitch) in enumerate(TRAJ):
        Rwb = sw.rotz(math.radians(yaw)) @ sw.roty(math.radians(pitch))
        Rws, tws = sw.compose(Rwb, np.array([x, y, 0.0]), Rbs, tbs)
        depth, amp = lid.capture(world, Rws, tws)
        fr = xt.build_frame(depth.tolist(), sn=100 + k, img_type=1)
        for i, p in enumerate(xt.packetize(fr, udp_skip=20, sn=100 + k)):
            put(npk.udp_frame(FPGA_L_MAC, LIDAR_MAC, LIDAR_IP, FPGA_L_IP, 7687, 7687, p, ident=k * 100 + i))
        Rq, tq = sw.to_fixed_pose(Rws, tws)
        cx = nf.world_cell(int(round(x * 1000)), INV_RES)
        cy = nf.world_cell(int(round(y * 1000)), INV_RES)
        for row in Rq:
            for v in row:
                w32(pose, v)
        for v in tq + [cx, cy]:
            w32(pose, v)
        pix = [(u, v, int(depth[v, u]), int(u == 0 and v == 0), int(u == 159 and v == 59))
               for v in range(60) for u in range(160)]
        upd = nf.pc_filter(nf.pc_transform(nf.pc_project(pix, lut), Rq, tq), INV_RES, cx, cy, ZMIN, ZMAX)
        em.update(upd)
        cost, _, _ = trav_v3.trav_v3(em, cx, cy, MAPCFG)
        cost_lines += [f"{int(c):02x}" for c in cost]
        last_cost, last_c = cost, (cx, cy, x, y, yaw)
        print(f"帧{k}: 更新 {sum(q[0] for q in upd)} 可通行 {int((cost <= 200).sum())}")

    # 轨迹评估：从最后一帧位置出发的一扇形候选
    cx, cy, x, y, yaw = last_c
    x0 = int(round(x * 1000 * INV_RES / 65536 * 256))
    y0 = int(round(y * 1000 * INV_RES / 65536 * 256))
    th0 = int(round(yaw / 360 * 65536)) & 0xFFFF
    T = 30
    cands = [(int(v), 0, int(w)) for v in (60, 120) for w in range(-800, 801, 200)]
    res = nf.traj_eval(last_cost, cx, cy, x0, y0, th0, cands, T, 253)
    tr = []
    for v in (x0, y0, th0, T, len(cands)):
        w32(tr, v)
    for dvx, dvy, dth in cands:
        w32(tr, ((dvy & 0xFFFF) << 16) | (dvx & 0xFFFF))
        w32(tr, dth & 0xFFFF)
    for mx, nun, cs, coll, left, csum, ex, ey in res:
        w32(tr, (left << 25) | (coll << 24) | (cs << 16) | (nun << 8) | mx)
        w32(tr, csum)
        w32(tr, ex)
        w32(tr, ey)
    print(f"轨迹: {len(cands)} 条，碰撞 {sum(r[3] for r in res)}")

    # 主机口：RK3588 发一个目标点报文（固件协议: 'G' + x,y,yaw）
    goal = b"NV" + bytes([1, 0x10]) + struct.pack("<iii", 2500, -800, 9000)
    hf = npk.udp_frame(FPGA_H_MAC, RK_MAC, RK_IP, FPGA_H_IP, 9001, 9000, goal)
    hl = [f"{(int(i == len(hf) - 1) << 8) | b:03x}" for i, b in enumerate(hf)]
    gw = []
    g = goal + b"\x00" * ((4 - len(goal) % 4) % 4)
    w32(gw, len(goal))
    for i in range(0, len(g), 4):
        w32(gw, struct.unpack("<I", g[i:i + 4])[0])

    (out / "eth_l.hex").write_text("\n".join(eth) + "\n")
    (out / "pose.hex").write_text("\n".join(pose) + "\n")
    (out / "cost_exp.hex").write_text("\n".join(cost_lines) + "\n")
    (out / "traj.hex").write_text("\n".join(tr) + "\n")
    (out / "eth_h.hex").write_text("\n".join(hl) + "\n")
    (out / "goal_exp.hex").write_text("\n".join(gw) + "\n")
    c = MAPCFG
    (out / "cfg.txt").write_text(" ".join(str(v) for v in [
        len(eth), len(TRAJ), len(hl), len(gw),
        c["age_max"] | (c["cnt_min"] << 8) | (c["keep_r"] << 16), c["step_max"] | (c["rough_max"] << 16),
        c["slope_rise"] | (c["jump"] << 16),
        c["k_slope"] | (c["k_step"] << 8) | (c["k_rough"] << 16), c["infl_r"],
        (ZMIN & 0xFFFF) | ((ZMAX & 0xFFFF) << 16)]) + "\n")


if __name__ == "__main__":
    main(sys.argv[1])
