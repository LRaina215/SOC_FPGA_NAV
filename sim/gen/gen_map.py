"""点云链路与地图子系统的激励/期望：虚拟雷达在默认场景中移动采集多帧。

用法: gen_map.py <输出目录>
输出:
  lut.hex       方向查找表
  pix.hex       像素流 {sof,eof,u,v,depth}
  pose.hex      每帧 9 个 R(Q14) + 3 个 t(mm) + cx + cy
  flt_exp.hex   pc_filter 期望输出 {upd,eof,addr,tag,z}（也是地图测试的输入）
  frm_cxy.hex   每帧窗口中心
  cost_exp.hex  每帧扫描后的前台代价图（按槽地址）
  mapcfg.hex    地图配置（顺序见 CFG_KEYS，之后 INV_RES ZMIN ZMAX）
  counts.txt    像素数 滤波输出数 帧数
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "model"))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sim" / "closed_loop"))
import lidar_lut  # noqa: E402
import nav_fixed as nf  # noqa: E402
import sim_world as sw  # noqa: E402
import trav_v3  # noqa: E402

INV_RES = 1311           # 50 mm
# 地形 v3 默认参数；年龄上限取 2 帧，让过期清除与近区保持在 5 帧内就被覆盖到
MAPCFG = dict(trav_v3.DEFAULT_V3, age_max=2, infl_r=3, infl_disk=1)
CFG_KEYS = ["age_max", "cnt_min", "keep_r", "step_max", "rough_max", "slope_rise", "jump", "drop_k", "drop_slope",
            "k_slope", "k_step", "k_rough", "fill_min", "fill_tol", "fill_cost", "infl_r"]
ZMIN, ZMAX = -500, 1500    # 高度带（相对世界 0；平台高 0.2 m 仍在带内）

# (x, y, yaw_deg) 闭环仿真 terrain 场景中的机身位姿（z/俯仰/横滚由轮下地形决定）：
# 坡道前、坡道上、平台上（看坡道侧沿落差）、大跳跃到陡坡前（窗口滚动、过期清除）、陡坡旁、原地转身
TRAJ = [(0.3, -0.3, 0), (1.5, -0.3, 5), (2.6, -0.2, -20), (7.3, -2.3, -10), (7.8, -1.2, -60),
        (7.8, -1.2, 150), (7.8, -1.2, 150), (7.8, -1.2, 150)]    # 原地转身：身后近区超过年龄上限，检验近区保持


def s32(v):
    return v & 0xFFFFFFFF


def main(out):
    out = Path(out)
    MAPCFG['step_high_only'] = int('--high-only' in sys.argv)
    import robot as rb
    import world as W
    scene = W.scene_terrain()
    world = scene.w
    bot = rb.RobotSim(scene, 0.0, 0.0, 0.0)
    lid = sw.VirtualLidar(noise_mm=4.0, seed=1)
    Rbs, tbs = sw.mount_pose(height=0.2, pitch_down_deg=10.0)
    lut = lidar_lut.lut_entries()
    lidar_lut.write_hex(out / "lut.hex", lut)

    em = nf.ElevMap()
    pix_lines, pose_lines, flt_lines, cxy_lines, cost_lines = [], [], [], [], []
    for k, (x, y, yaw) in enumerate(TRAJ):
        bot.x, bot.y, bot.yaw = x, y, math.radians(yaw)
        bot.update_attitude()
        Rwb, twb = bot.body_pose()
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
        # 覆盖率：分别关掉各项规则，统计代价图有差异的格数（确认测试真的用到了这些规则）
        import copy
        cov = {}
        for name, over in (("落差", {"drop_k": 1}), ("填补", {"fill_min": 9}), ("近区保持", {"keep_r": 0}),
                           ("基线坡度", {"slope_rise": 60000})):
            c2, _, _ = trav_v3.trav_v3(copy.deepcopy(em), cx, cy, dict(MAPCFG, **over))
            cov[name] = c2
        cost, _, _ = trav_v3.trav_v3(em, cx, cy, MAPCFG)
        print(f"帧{k} 规则覆盖（关掉后变化的格数）: " + " ".join(f"{n} {int((c2 != cost).sum())}" for n, c2 in cov.items()))
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
    vals = [c[k] for k in CFG_KEYS] + [INV_RES, ZMIN & 0xFFFF, ZMAX & 0xFFFF, c['step_high_only']]
    vals[CFG_KEYS.index("infl_r")] |= c.get("infl_disk", 1) << 4          # INFL.DISK
    (out / "mapcfg.hex").write_text("\n".join(f"{v:04x}" for v in vals) + "\n")
    (out / "counts.txt").write_text(f"{len(pix_lines)} {len(flt_lines)} {len(TRAJ)}\n")


if __name__ == "__main__":
    main(sys.argv[-1])
