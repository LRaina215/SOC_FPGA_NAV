"""软核固件交叉编译（xPack riscv-none-elf-gcc，rv32imc）。

用法: python fw/bsp/build_rv.py [--ram-kb 128]
输出: fw/build/rv/<app>/nav_fw.elf / .bin / .map，ram_b0..3.hex（按字节通道拆分，随比特流初始化片上 RAM），ram_w.hex（32 位字）
"""
import argparse
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
GCC_DIR = Path(os.environ.get("RISCV_GCC", r"D:\tools\xpack-riscv-none-elf-gcc-15.2.0-1\bin"))
PREFIX = str(GCC_DIR / "riscv-none-elf-")
OUT = ROOT / "build" / "rv"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ram-kb", type=int, default=128)
    ap.add_argument("-O", default="s", help="优化级别（缺省 s：128 KB RAM 要靠 -Os 省出约 7 KB 代码）")
    ap.add_argument("--app", default="nav", choices=["nav", "selftest"], help="nav = 导航固件；selftest = 自检程序")
    ap.add_argument("--copy-to", help="把 RAM 初始化文件复制到该目录（xsim 测试用）")
    ap.add_argument("--grid", default="64,2", help="记忆栅格边长,每格含几个 5 cm 局部格（5 m 场地：64,2 = 10 cm 格、6.4 m）")
    a = ap.parse_args()
    bsp = [str(ROOT / "bsp" / "console.c"), str(ROOT / "bsp" / "start.S")]
    if a.app == "nav":
        srcs = sorted(str(p) for p in (ROOT / "nav").glob("*.c")) + [
            str(ROOT / "app" / "nav_app.c"), str(ROOT / "app" / "main.c"), str(ROOT / "drv" / "hal_fpga.c")] + bsp
    else:
        srcs = [str(ROOT / "bsp" / "selftest.c")] + bsp
    out = OUT / a.app
    out.mkdir(parents=True, exist_ok=True)
    elf = out / "nav_fw.elf"
    cmd = [PREFIX + "gcc", "-march=rv32imc_zicsr", "-mabi=ilp32", f"-O{a.O}", "-g", "-std=c99",
           "-Wall", "-Wno-unused-function", "-ffunction-sections", "-fdata-sections", "-fno-common",
           "-nostartfiles", "-specs=nano.specs", "-DNAV_BASE=0x40000000u",
           f"-DGM_N={a.grid.split(',')[0]}", f"-DASTAR_MAXN={a.grid.split(',')[0]}", f"-DGM_DS={a.grid.split(',')[1]}",
           f"-Wl,--defsym=RAM_SIZE={a.ram_kb * 1024}", "-Wl,--gc-sections", f"-Wl,-Map={out / 'nav_fw.map'}",
           "-T", str(ROOT / "bsp" / "link.ld")] + \
        [f"-I{ROOT / d}" for d in ("include", "hal", "nav", "app", "bsp")] + srcs + ["-o", str(elf), "-lgcc"]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout + r.stderr)
        return 1
    warn = [ln for ln in r.stderr.splitlines() if "warning" in ln]
    if warn:
        print("\n".join(warn[:20]))
    subprocess.run([PREFIX + "objcopy", "-O", "binary", str(elf), str(out / "nav_fw.bin")], check=True)
    sz = subprocess.run([PREFIX + "size", "-A", str(elf)], capture_output=True, text=True).stdout
    print(subprocess.run([PREFIX + "size", str(elf)], capture_output=True, text=True).stdout.strip())
    img = (out / "nav_fw.bin").read_bytes()
    nw = a.ram_kb * 256
    img = img + bytes(4 * nw - len(img))
    for b in range(4):
        (out / f"ram_b{b}.hex").write_text("\n".join(f"{img[4 * i + b]:02x}" for i in range(nw)) + "\n")
    (out / "ram_w.hex").write_text("\n".join(img[4 * i:4 * i + 4][::-1].hex() for i in range(nw)) + "\n")
    used = len((out / "nav_fw.bin").read_bytes())
    bss = [ln for ln in sz.splitlines() if ln.startswith(".bss")]
    print(f"镜像 {used} 字节，.bss {bss[0].split()[1] if bss else 0} 字节，RAM {a.ram_kb} KB")
    if a.copy_to:
        import shutil
        for f in ["ram_w.hex"] + [f"ram_b{b}.hex" for b in range(4)]:
            shutil.copy(out / f, Path(a.copy_to) / f)
    return 0


if __name__ == "__main__":
    sys.exit(main())
