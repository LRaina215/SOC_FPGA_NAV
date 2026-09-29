"""固件方向表生成（fw/nav/lidar_cal.c，定点）与 Python 模型（model/lidar_lut.py，双精度）对比。

检查：几组内参（M60 缺省、理想针孔、畸变更强、主点偏移）下，每个分量相差不超过 1 LSB，且 99.9% 以上完全一致；
切除像素完全一致；固件收到"雷达自带内参"后会重新生成整张表，内参不变不重算。
用法: python sim/closed_loop/test_lidar_cal.py [--no-build]
"""
import ctypes
import struct
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "model"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import lidar_lut as ll  # noqa: E402
import run_cl  # noqa: E402

NPIX = ll.W * ll.H


def bits(intr):
    return [struct.unpack("<I", struct.pack("<f", intr[k]))[0] for k in ll.LENS_KEYS]


def fw_lut(d, intr, cut=ll.CUT_R, flip=True):
    lens = (ctypes.c_uint32 * 9)(*bits(intr))
    out = (ctypes.c_int16 * (3 * NPIX))()
    d.py_lut_gen(lens, cut, int(flip), out)
    return np.array(out, dtype=np.int64).reshape(-1, 3)


def main():
    if "--no-build" not in sys.argv:
        run_cl.build_dll()
    d = ctypes.CDLL(str(run_cl.DLL))
    cases = {
        "M60 缺省": dict(ll.M60_DEFAULT),
        "理想针孔 120x45": dict(ll.IDEAL_120x45),
        "畸变更强": dict(ll.M60_DEFAULT, k1=-0.38, k2=0.11, p1=0.003, p2=-0.002),
        "主点偏移": dict(ll.M60_DEFAULT, cx=83.5, cy=31.2, fx=80.1, fy=77.4),
    }
    fails = 0
    for name, intr in cases.items():
        for cut, flip in ((ll.CUT_R, True), (0, False)):
            ref = np.array(ll.lut_entries(intr=intr, cut=cut, flip_u=flip), dtype=np.int64)
            got = fw_lut(d, intr, cut, flip)
            diff = np.abs(got - ref)
            zr, zg = (ref == 0).all(1), (got == 0).all(1)
            exact = (diff == 0).all(1).mean()
            ok = diff.max() <= 1 and exact >= 0.999 and (zr == zg).all()
            fails += not ok
            print(f"{name:14s} cut={cut:2d} flip={int(flip)}: 最大差 {diff.max()} LSB，完全一致 {exact * 100:.3f}%，"
                  f"切除 {int(zg.sum())}/{int(zr.sum())}  {'OK' if ok else '不通过'}")
    # 固件流程：上电按缺省生成；收到不同内参重算；再收到相同内参不重算
    d.py_init()
    d.py_poll()
    lut = (ctypes.c_int16 * (3 * NPIX))()
    st = (ctypes.c_int32 * 3)()
    n0 = d.py_get_lut(lut, st)
    ok0 = st[0] == 1 and n0 == NPIX
    ref = np.array(ll.lut_entries(), dtype=np.int64)
    ok0 = ok0 and np.abs(np.array(lut, dtype=np.int64).reshape(-1, 3) - ref).max() <= 1
    other = dict(ll.M60_DEFAULT, cx=83.5)
    d.py_set_lidar_info((ctypes.c_uint32 * 9)(*bits(other)))
    d.py_poll()
    n1 = d.py_get_lut(lut, st)
    ref1 = np.array(ll.lut_entries(intr=other), dtype=np.int64)
    ok1 = st[0] == 1 and st[2] == 1 and n1 == 2 * NPIX and \
        np.abs(np.array(lut, dtype=np.int64).reshape(-1, 3) - ref1).max() <= 1
    d.py_set_lidar_info((ctypes.c_uint32 * 9)(*bits(other)))
    d.py_poll()
    n2 = d.py_get_lut(lut, st)
    ok2 = n2 == n1
    print(f"固件流程：上电生成 {'OK' if ok0 else '不通过'}，新内参重算 {'OK' if ok1 else '不通过'}，"
          f"相同内参不重算 {'OK' if ok2 else '不通过'}")
    fails += (not ok0) + (not ok1) + (not ok2)
    print("LIDAR_CAL PASSED" if fails == 0 else f"LIDAR_CAL FAILED（{fails} 项）")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
