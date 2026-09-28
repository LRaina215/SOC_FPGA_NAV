"""点云链路与地图子系统的激励/期望：虚拟雷达在默认场景中移动采集多帧。

用法: gen_map.py <输出目录>
输出:
  lut.hex       方向查找表
  pix.hex       像素流 {sof,eof,u,v,depth}
  pose.hex      每帧 9 个 R(Q14) + 3 个 t(mm) + cx + cy
  flt_exp.hex   pc_filter 期望输出 {upd,eof,addr,tag,z}（也是地图测试的输入）
  frm_cxy.hex   每帧窗口中心
  cost_exp.hex  每帧扫描后的前台代价图（按槽地址）
  mapcfg.hex    地图配置
  counts.txt    像素数 滤波输出数 帧数
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "model"))
import lidar_lut  # noqa: E402
import nav_fixed as nf  # noqa: E402
import sim_world as sw  # noqa: E402

INV_RES = 1311           # 50 mm
MAPCFG = dict(age_max=60, cnt_min=1, step_max=120, rough_max=120, slope_max=60,
              k_slope=64, k_step=32, k_rough=32, infl_r=3)
ZMIN, ZMAX = -500, 1500

# (x, y, yaw_deg, pitch_deg) 机身位姿序列：第 4 帧大跳跃，测试窗口滚动与过期清除
TRAJ = [(0.0, 0.0, 0, 0), (0.4, 0.1, 8, 0), (0.8, 0.3, 15, 3), (4.2, -0.2, 0, 0), (4.6, -0.2, -6, -4)]


def s32(v):
    return v & 0xFFFFFFFF


def main(out):
    out = Path(out)
    world = sw.default_scene()
    lid = sw.VirtualLidar(noise_mm=4.0, seed=1)
    Rbs, tbs = sw.mount_pose(height=0.2, pitch_down_deg=10.0)
    lut = lidar_lut.lut_entries()
    lidar_lut.write_hex(out / "lut.hex", lut)

    em = nf.ElevMap()
    pix_lines, pose_lines, flt_lines, cxy_lines, cost_lines = [], [], [], [], []
    for k, (x, y, yaw, pitch) in enumerate(TRAJ):
        Rwb = sw.rotz(math.radians(yaw)) @ sw.roty(math.radians(pitch))
        twb = np.array([x, y, 0.0])
        Rws, tws = sw.compose(Rwb, twb, Rbs, tbs)
        depth, _ = lid.capture(world, Rws, tws)
        pix = [(u, v, int(depth[v, u]), int(u == 0 and v == 0), int(u == 159 and v == 59))
               for v in range(60) for u in range(160)]
        for u, v, d, s, e in pix:
            pix_lines.append(f"{(s << 33) | (e << 32) | (u << 24) | (v << 16) | d:09x}")
        Rq, tq = sw.to_fixed_pose(Rws, tws)
        cx = nf.world_cell(int(round(x * 1000)), INV_RES)
        cy = nf.world_cell(int(round(y * 1000)), INV_RES)
        for row in Rq:
            for v in row:
                pose_lines.append(f"{v & 0xFFFF:08x}")
        for v in tq + [cx, cy]:
            pose_lines.append(f"{s32(v):08x}")
        cxy_lines += [f"{s32(cx):08x}", f"{s32(cy):08x}"]

        p = nf.pc_project(pix, lut)
        po = nf.pc_transform(p, Rq, tq)
        upd = nf.pc_filter(po, INV_RES, cx, cy, ZMIN, ZMAX)
        for u_, a, tg, z, e in upd:
            flt_lines.append(f"{(u_ << 39) | (e << 38) | (a << 24) | (tg << 16) | (z & 0xFFFF):010x}")
        em.update(upd)
        cost, _ = nf.trav_analysis(em, cx, cy, MAPCFG)
        cost_lines += [f"{int(c):02x}" for c in cost]
        hist = {n: int((cost == n).sum()) for n in (253, 254, 255)}
        print(f"帧{k}: 点 {sum(q[3] for q in p)} 更新 {sum(q[0] for q in upd)} 代价 {hist} "
              f"可通行 {int((cost <= 200).sum())}")

    (out / "pix.hex").write_text("\n".join(pix_lines) + "\n")
    (out / "pose.hex").write_text("\n".join(pose_lines) + "\n")
    (out / "flt_exp.hex").write_text("\n".join(flt_lines) + "\n")
    (out / "frm_cxy.hex").write_text("\n".join(cxy_lines) + "\n")
    (out / "cost_exp.hex").write_text("\n".join(cost_lines) + "\n")
    c = MAPCFG
    vals = [c["age_max"], c["cnt_min"], c["step_max"], c["rough_max"], c["slope_max"],
            c["k_slope"], c["k_step"], c["k_rough"], c["infl_r"], INV_RES, ZMIN & 0xFFFF, ZMAX & 0xFFFF]
    (out / "mapcfg.hex").write_text("\n".join(f"{v:04x}" for v in vals) + "\n")
    (out / "counts.txt").write_text(f"{len(pix_lines)} {len(flt_lines)} {len(TRAJ)}\n")


if __name__ == "__main__":
    main(sys.argv[1])
