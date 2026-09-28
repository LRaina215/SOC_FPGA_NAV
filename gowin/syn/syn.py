"""用 Gowin 命令行综合（GW5AT-LV138PG484AC1/I0，版本 B）。

用法:
  python gowin/syn/syn.py            只综合 nav_subsystem（可综合性、资源、Fmax 估计）
  python gowin/syn/syn.py --board    板级顶层 nav_board_top：综合 + 布局布线 + 生成比特流（50 MHz，无 IP）
结果: gowin/syn/build[_board]/impl/...
"""
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
GW_SH = Path(os.environ.get("GW_SH", r"D:\Gowin2\Gowin_V1.9.12.04_x64\IDE\bin\gw_sh.exe"))
BUILD = ROOT / "gowin" / "syn" / "build"

sys.path.insert(0, str(ROOT / "sim"))
from tests import ALL_RTL  # noqa: E402


def main():
    board = "--board" in sys.argv
    global BUILD
    if board:
        BUILD = ROOT / "gowin" / "syn" / "build_board"
    BUILD.mkdir(parents=True, exist_ok=True)
    for f in ("rtl/plan/sin_lut.hex", "rtl/lidar/lut_default.hex"):
        shutil.copy(ROOT / f, BUILD / Path(f).name)
    tcl = ["set_device -device_version B GW5AT-LV138PG484AC1/I0"]
    top = "nav_board_top" if board else "nav_subsystem"
    files = list(ALL_RTL) + (["gowin/nav_board_top.v"] if board else [])
    for s in files:
        tcl.append(f"add_file -type verilog {{{(ROOT / s).as_posix()}}}")
    if board:
        tcl.append(f"add_file -type cst {{{(ROOT / 'gowin/nav_board.cst').as_posix()}}}")
        (BUILD / "nav_board.sdc").write_text("create_clock -name clk50 -period 20 [get_ports {clk50}]\n")
        tcl.append(f"add_file -type sdc {{{(BUILD / 'nav_board.sdc').as_posix()}}}")
    inc = (ROOT / "rtl/regs").as_posix()
    tcl += [
        f"set_option -include_path {{{inc}}}",
        f"set_option -top_module {top}",
        f"set_option -output_base_name {top}",
        "set_option -verilog_std v2001",
        "set_option -print_all_synthesis_warning 1",
        "set_option -looplimit 100000",
        "set_option -use_sspi_as_gpio 1",
        "set_option -use_mspi_as_gpio 1",
        "set_option -use_ready_as_gpio 1",
        "set_option -use_done_as_gpio 1",
        "set_option -use_i2c_as_gpio 1",
        "set_option -use_cpu_as_gpio 1",
        "run all" if board else "run syn",
    ]
    (BUILD / "run.tcl").write_text("\n".join(tcl) + "\n", encoding="utf-8")
    with open(BUILD / "gw_sh.log", "w", encoding="utf-8", errors="replace") as log:
        rc = subprocess.run([str(GW_SH), "run.tcl"], cwd=BUILD, stdout=log, stderr=subprocess.STDOUT).returncode
    text = (BUILD / "gw_sh.log").read_text(encoding="utf-8", errors="replace")
    errs = [l for l in text.splitlines() if "ERROR" in l]
    for l in errs[:20]:
        print(l)
    rpt = BUILD / "impl" / "gwsynthesis" / f"{top}_syn.rpt.html"
    if rc or errs or not rpt.exists():
        print(f"综合失败，见 {BUILD / 'gw_sh.log'}")
        return 1
    h = re.sub(r"<[^>]+>", " | ", rpt.read_text(encoding="utf-8", errors="replace"))
    h = re.sub(r"(\s*\|\s*)+", " | ", h)
    m = h.rfind("Resource Utilization Summary")
    print(h[m:m + 330])
    m = h.find("Max Frequency Summary:")
    print(h[m:m + 200])
    if board:
        for f in sorted((BUILD / "impl" / "pnr").glob("*")):
            print("pnr:", f.name)
    return 0


if __name__ == "__main__":
    sys.exit(main())
