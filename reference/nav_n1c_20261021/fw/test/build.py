"""编译并运行固件主机测试（gcc，C99）。
用法: python fw/test/build.py [--no-run]
同时以 -DNAV_BASE 编译一遍上板 HAL（hal_fpga.c），检查它至少能通过编译。
"""
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / "fw"
OUT = ROOT / "sim" / "build" / "fw_test"
CC = "gcc"
CFLAGS = ["-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
          f"-I{FW / 'include'}", f"-I{FW / 'hal'}", f"-I{FW / 'nav'}", f"-I{FW / 'app'}", f"-I{FW / 'test'}"]


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    nav = sorted(str(p) for p in (FW / "nav").glob("*.c"))
    srcs = nav + [str(FW / "app" / "nav_app.c"), str(FW / "test" / "hal_sim.c"), str(FW / "test" / "test_main.c")]
    exe = OUT / "fw_test.exe"
    r = subprocess.run([CC] + CFLAGS + ["-D_USE_MATH_DEFINES", "-o", str(exe)] + srcs + ["-lm"])
    if r.returncode:
        return r.returncode
    # 上板 HAL 编译检查（不链接）
    r = subprocess.run([CC] + CFLAGS + ["-c", "-o", str(OUT / "hal_fpga.o"), str(FW / "drv" / "hal_fpga.c")])
    if r.returncode:
        return r.returncode
    r = subprocess.run([CC] + CFLAGS + [f"-I{FW / 'bsp'}", "-c", "-o", str(OUT / "main.o"), str(FW / "app" / "main.c")])
    if r.returncode:
        return r.returncode
    if "--no-run" in sys.argv:
        return 0
    return subprocess.run([str(exe), str(ROOT / "sim" / "build" / "tb_traj_eval")]).returncode


if __name__ == "__main__":
    sys.exit(main())
