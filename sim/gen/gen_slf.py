"""SLF 匹配硬件（rtl/match/slf_match.v）的激励与期望：terrain 场景中一段带转向的路线，逐帧
  点流（pc_transform 输出）-> 结构点提取 -> 245 个候选打分 -> 按给定修正量并图 -> 似然场建图。
期望全部来自 model/slf_match.py（闭环仿真所用的同一模型）。

用法: gen_slf.py <输出目录>
输出:
  pts.hex     每像素 4 个字：x, y, z, {eof, sof, pv}
  frm.hex     每帧：zlo zhi tx ty cx_mm cy_mm lf_cx lf_cy upd_dx upd_dy upd_dyaw n_pts
  cand.hex    候选（每个 2 字，所有帧相同）
  res.hex     每帧每候选 2 字：cost, {n_in, n_valid}（每帧补齐到 512 字）
  sm.hex      每帧并图后结构图 65536 个槽字
  smm.hex     对应比较掩码（窗口外的槽只比较前 15 位）
  有图模式（在线帧之后）：由各帧结构点离线生成先验定位图（窗口原点故意偏开，使部分点落在窗口外），
  写入结构图后，重放第 3、4 帧点流：第 3 帧不门控（对准）、第 4 帧门控，比较打分；UPDATE 只释放点缓冲。
  prior.hex   先验槽字：每条 2 字 {槽地址, 槽字}
  pres.hex    有图帧的期望打分（每帧 512 字）
  counts.txt  帧数 候选数 先验条数 lfo_x lfo_y
"""
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "model"))
sys.path.insert(0, str(ROOT / "sim" / "closed_loop"))
import lidar_lut  # noqa: E402
import nav_fast as fast  # noqa: E402
import nav_fixed as nf  # noqa: E402
import sim_world as sw  # noqa: E402
import slf_match as slf  # noqa: E402
import prior_map  # noqa: E402

INV_RES = 1311
MOUNT_Z = 200
# (x, y, yaw°)：原地转向建图后前进；UPD 为每帧并图时施加的修正量（任取非零值，检验变换路径）
TRAJ = [(3.0, -2.0, 0), (3.0, -2.0, 40), (3.0, -2.0, 80), (3.2, -1.9, 60), (3.4, -1.8, 45)]
UPD = [(0, 0, 0), (12, -7, 64), (-20, 9, -128), (5, 5, 0), (-9, 14, 128)]


def s32(v):
    return int(v) & 0xFFFFFFFF


def main(out):
    import robot as rb
    import world as W
    out = Path(out)
    scene = W.scene_terrain()
    lid = sw.VirtualLidar(noise_mm=4.0, seed=3, rng_max=8.0, step=0.015)
    Rbs, tbs = sw.mount_pose(height=0.20, pitch_down_deg=10.0, x=0.10)
    lut = np.array(lidar_lut.lut_entries(), dtype=np.int64)
    bot = rb.RobotSim(scene, 0.0, 0.0, 0.0)
    smap = slf.StructMap(INV_RES)
    cands = [(ix * 25, iy * 25, ia * 128) for ia in range(-2, 3) for iy in range(-3, 4) for ix in range(-3, 4)]
    pts, frm, res, smw, smm = [], [], [], [], []
    fr_sp = []
    for k, (x, y, yaw) in enumerate(TRAJ):
        bot.x, bot.y, bot.yaw = x, y, math.radians(yaw)
        bot.update_attitude()
        Rwb, twb = bot.body_pose()
        Rws, tws = sw.compose(Rwb, twb, Rbs, tbs)
        depth, _ = lid.capture(scene.w, Rws, tws)
        xyz, pv = fast.pc_project(depth, lut)
        Rq, tq = sw.to_fixed_pose(Rws, tws)
        p = fast.pc_transform(xyz, np.array(Rq), np.array(tq))
        n = len(pv)
        for i in range(n):
            pts += [f"{s32(p[i, 0]):08x}", f"{s32(p[i, 1]):08x}", f"{s32(p[i, 2]):08x}",
                    f"{(int(i == n - 1) << 2) | (int(i == 0) << 1) | int(pv[i]):08x}"]
        zref = int(tq[2]) - MOUNT_Z
        m = slf.structure_mask(p, pv, zref)
        allidx = np.nonzero(m)[0]
        spts = p[slf.subsample(allidx)]
        cxm, cym = int(round(x * 1000)), int(round(y * 1000))
        fr_sp.append((spts, cxm, cym))
        o = smap.score(spts[:, 0], spts[:, 1], np.array(cands, dtype=np.int64), cxm, cym)
        for c, n_in, nv in zip(o[:, 1], o[:, 2], o[:, 0]):
            res += [f"{s32(c):08x}", f"{((int(n_in) & 0xFFFF) << 16) | (int(nv) & 0xFFFF):08x}"]
        res += ["00000000"] * (512 - 2 * len(cands))          # 每帧 512 字对齐
        dx, dy, da = UPD[k]
        ux, uy = slf.apply_delta(spts[:, 0], spts[:, 1], dx, dy, da, cxm, cym)
        smap.update(np.stack([ux, uy], axis=1))
        lcx, lcy = nf.world_cell(cxm, INV_RES), nf.world_cell(cym, INV_RES)
        smap.build_lf(lcx, lcy)
        w, mask = smap.words()
        smw += [f"{int(v):06x}" for v in w]
        smm += [f"{int(v):06x}" for v in mask]
        frm += [f"{s32(v):08x}" for v in (zref + slf.Z_LO, zref + slf.Z_HI, tq[0], tq[1], cxm, cym, lcx, lcy,
                                           dx, dy, da & 0xFFFF, min(len(allidx), slf.PB_MAX))]
        print(f"帧{k}: 结构点 {len(allidx)}（参与 {len(spts)}），门控通过 {int(o[0, 0])}，结构格 {smap.n_struct}，"
              f"最优候选 {cands[int(np.argmin(o[:, 1]))]}")
    (out / "pts.hex").write_text("\n".join(pts) + "\n")
    (out / "frm.hex").write_text("\n".join(frm) + "\n")
    (out / "cand.hex").write_text("\n".join(
        f"{((c[1] & 0xFFFF) << 16) | (c[0] & 0xFFFF):08x}\n{c[2] & 0xFFFF:08x}" for c in cands) + "\n")
    (out / "res.hex").write_text("\n".join(res) + "\n")
    (out / "sm.hex").write_text("\n".join(smw) + "\n")
    (out / "smm.hex").write_text("\n".join(smm) + "\n")
    # ---------------- 有图模式
    allsp = np.concatenate([s for s, _, _ in fr_sp])[:, :2]
    ox, oy = int(nf.world_cell(4000, INV_RES)), int(nf.world_cell(-6000, INV_RES))
    occ, cxs, cys, _ = prior_map.struct_cells_from_points(allsp, ox, oy, min_count=1)
    q, has, gate = prior_map.prior_lf(occ, cxs, cys)
    slot, word = prior_map.struct_words(ox, oy, occ, cxs, cys, q, has, gate)
    pmap = slf.StructMap(INV_RES)
    pmap.load_prior(ox, oy, slot, word)
    pres = []
    for fi, ng in ((3, True), (4, False)):
        spts, cxm, cym = fr_sp[fi]
        pmap.nogate = ng
        o = pmap.score(spts[:, 0], spts[:, 1], np.array(cands, dtype=np.int64), cxm, cym)
        for c, n_in, nv in zip(o[:, 1], o[:, 2], o[:, 0]):
            pres += [f"{s32(c):08x}", f"{((int(n_in) & 0xFFFF) << 16) | (int(nv) & 0xFFFF):08x}"]
        pres += ["00000000"] * (512 - 2 * len(cands))
        print(f"有图帧{fi}（{'不门控' if ng else '门控'}）：参与 {int(o[0, 0])} / {len(spts)}，"
              f"最优 {cands[int(np.argmin(o[:, 1]))]}")
    (out / "prior.hex").write_text("\n".join(f"{int(s):08x}\n{int(w):08x}" for s, w in zip(slot, word)) + "\n")
    (out / "pres.hex").write_text("\n".join(pres) + "\n")
    (out / "counts.txt").write_text(f"{len(TRAJ)} {len(cands)} {len(slot)} {ox} {oy}\n")


if __name__ == "__main__":
    main(sys.argv[1])
