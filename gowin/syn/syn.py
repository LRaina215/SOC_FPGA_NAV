"""用 Gowin 命令行综合（GW5AT-LV138PG484AC1/I0，版本 B）。

用法:
  python gowin/syn/syn.py            只综合 nav_subsystem（可综合性、资源、Fmax 估计）
  python gowin/syn/syn.py --top=slf_lf  只综合某个模块
  python gowin/syn/syn.py --board    板级顶层 nav_board_top：综合 + 布局布线 + 生成比特流（50 MHz，无 IP）
  板级选项:
    --app=nav|selftest --ram-kb=128   软核 RAM 初始镜像与大小
    --define=NAV_USE_SFP              打开顶层中的 IP（可重复：NAV_USE_PLL），写入 gowin/soc_cfg.vh；雷达口 MAC 已内置，不用 define
    IP 源文件列在 gowin/ip/ip_files.txt（每行一个路径，相对 nav/ 或绝对路径；# 开头为注释），板级构建时自动加入
    --tcl-only                        只生成 run.tcl，不调用 Gowin（检查文件列表用）
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
from tests import ALL_RTL, SOC  # noqa: E402


def main():
    board = "--board" in sys.argv
    global BUILD
    if board:
        BUILD = ROOT / "gowin" / "syn" / "build_board"
    for a in sys.argv[1:]:
        if a.startswith('--output-dir='): BUILD=Path(a.split('=',1)[1]).resolve()
    BUILD.mkdir(parents=True, exist_ok=True)
    for f in ("rtl/plan/sin_lut.hex",):
        shutil.copy(ROOT / f, BUILD / Path(f).name)
    ram_kb, app = "128", "nav"
    firmware=ROOT/'fw'
    for a in sys.argv[1:]:
        if a.startswith("--ram-kb="):
            ram_kb = a.split("=", 1)[1]
        if a.startswith("--app="):
            app = a.split("=", 1)[1]
        if a.startswith('--firmware-dir='): firmware=Path(a.split('=',1)[1]).resolve()
    if board:                                   # 软核 RAM 初始镜像（缺省导航固件；--app=selftest --ram-kb=32 为上板自检用的小镜像）
        rc = subprocess.run([sys.executable, str(firmware / "bsp/build_rv.py"), "--app", app,
                             "--ram-kb", ram_kb, "--output-dir",str(BUILD/'firmware'),"--copy-to", str(BUILD)])
        if rc.returncode:
            return 1
        defines = [a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--define=")]
        (ROOT / "gowin" / "soc_cfg.vh").write_text(
            "".join(f"`define {d}" + chr(10) for d in [f"NAV_SOC_RAM_KB {ram_kb}"] + defines))
    tcl = ["set_device -device_version B GW5AT-LV138PG484AC1/I0"]
    top = "nav_board_top" if board else "nav_subsystem"
    for a in sys.argv[1:]:
        if a.startswith("--top="):              # 单独综合某个模块（排查用）
            top = a.split("=", 1)[1]
    files = list(ALL_RTL) + (SOC + ["rtl/net/rgmii_mac100.v", "gowin/nav_board_top.v"] if board else [])
    for s in files:
        tcl.append(f"add_file -type verilog {{{(ROOT / s).as_posix()}}}")
    ip_list = ROOT / "gowin" / "ip" / "ip_files.txt"
    if board and ip_list.exists():
        for ln in ip_list.read_text(encoding="utf-8").splitlines():
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            p = Path(ln) if Path(ln).is_absolute() else ROOT / ln
            if not p.exists():
                print(f"ip_files.txt 中的文件不存在: {p}")
                return 1
            kind = {".sdc": "sdc", ".cst": "cst"}.get(p.suffix.lower(), "verilog")
            tcl.append(f"add_file -type {kind} {{{p.as_posix()}}}")
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
    ] + ([
        # 软核 RAM 160 KB 时 BSRAM 86%：缺省算法布线阶段长时间不收敛；改为可布性优先的布局 + 快速布线
        "set_option -place_option 1",
        "set_option -route_option 0",
    ] if board else []) + [
        "run all" if board else "run syn",
    ]
    (BUILD / "run.tcl").write_text("\n".join(tcl) + "\n", encoding="utf-8")
    if "--tcl-only" in sys.argv:
        print(f"已生成 {BUILD / 'run.tcl'}")
        return 0
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
