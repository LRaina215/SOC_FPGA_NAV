"""nav_fast（向量化）与 nav_fixed（逐位参考）的一致性测试：多帧、窗口跳变、噪声。"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "model"))
import lidar_lut  # noqa: E402
import nav_fast as fast  # noqa: E402
import nav_fixed as nf  # noqa: E402
import sim_world as sw  # noqa: E402

CFG = dict(age_max=60, cnt_min=1, step_max=120, rough_max=120, slope_max=60,
           k_slope=64, k_step=32, k_rough=32, infl_r=3)


def main():
    world = sw.default_scene()
    lid = sw.VirtualLidar(noise_mm=4.0, seed=3)
    Rbs, tbs = sw.mount_pose()
    lut_l = lidar_lut.lut_entries()
    lut = np.array(lut_l, dtype=np.int64)
    em_ref, em_fast = nf.ElevMap(), nf.ElevMap()
    bad = 0
    for k, (x, y, yaw, pitch) in enumerate([(0, 0, 0, 0), (0.6, 0.2, 20, 3), (4.3, -0.3, 5, -2), (4.8, -0.1, -15, 1)]):
        Rws, tws = sw.compose(sw.rotz(math.radians(yaw)) @ sw.roty(math.radians(pitch)), np.array([x, y, 0.0]), Rbs, tbs)
        depth, _ = lid.capture(world, Rws, tws)
        Rq, tq = sw.to_fixed_pose(Rws, tws)
        cx, cy = nf.world_cell(int(x * 1000), 1311), nf.world_cell(int(y * 1000), 1311)
        # 参考
        pix = [(u, v, int(depth[v, u]), int(u == 0 and v == 0), int(u == 159 and v == 59)) for v in range(60) for u in range(160)]
        upd = nf.pc_filter(nf.pc_transform(nf.pc_project(pix, lut_l), Rq, tq), 1311, cx, cy, -500, 1500)
        em_ref.update(upd)
        c_ref, _ = nf.trav_analysis(em_ref, cx, cy, CFG)
        # 向量化
        xyz, pv = fast.pc_project(depth, lut)
        p = fast.pc_transform(xyz, Rq, tq)
        a, tg, z = fast.pc_filter(p, pv, 1311, cx, cy, -500, 1500)
        fast.elev_update(em_fast, a, tg, z)
        c_fast, _, _ = fast.trav_v1(em_fast, cx, cy, CFG)
        n = int((c_ref != c_fast).sum())
        for f in ("zmax", "zmin", "cnt", "tag", "stamp"):
            n += int((getattr(em_ref, f) != getattr(em_fast, f)).sum())
        print(f"帧{k}: 不一致 {n}")
        bad += n
    print("NAV_FAST PASSED" if bad == 0 else "NAV_FAST FAILED")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
