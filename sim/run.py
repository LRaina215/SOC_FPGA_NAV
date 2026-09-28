"""xsim 回归测试运行器。

用法:
  python sim/run.py            # 运行全部测试
  python sim/run.py tb_fifo    # 运行指定测试（可多个）
  python sim/run.py -l         # 列出测试

每个测试: 可选 pre（生成激励/期望，Python 模型），xvlog/xelab/xsim，可选 post（比对）。
测试台以打印 "TEST PASSED" 表示通过；出现 "TEST FAILED"、$fatal 或 ERROR 视为失败。
"""
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / "sim" / "build"
VIVADO_BIN = Path(os.environ.get("VIVADO_BIN", r"E:\2026.1\Vivado\bin"))
PY = sys.executable

sys.path.insert(0, str(ROOT / "sim"))
from tests import TESTS  # noqa: E402


def sh(cmd, cwd, log):
    with open(log, "w", encoding="utf-8", errors="replace") as f:
        p = subprocess.run(cmd, cwd=cwd, stdout=f, stderr=subprocess.STDOUT, shell=False)
    return p.returncode


def bat(name):
    return str(VIVADO_BIN / f"{name}.bat")


def run_test(name):
    t = TESTS[name]
    wd = BUILD / name
    wd.mkdir(parents=True, exist_ok=True)
    env_args = [str(wd)]
    t0 = time.time()

    for step in ("pre",):
        if t.get(step):
            rc = sh([PY, str(ROOT / t[step][0])] + t[step][1:] + env_args, ROOT, wd / f"{step}.log")
            if rc:
                return False, f"{step} 失败，见 {wd / (step + '.log')}"

    import shutil
    for f in t.get("copy", []):
        shutil.copy(ROOT / f, wd / Path(f).name)

    defs = []
    for d in t.get("defines", []):
        defs += ["-d", d]
    inc = ["-i", str(ROOT / "rtl" / "regs"), "-i", str(ROOT / "rtl")]
    # 可综合代码（.v）按 Verilog-2001 编译，防止混入综合器不支持的 SV 写法；测试台（.sv）按 SV 编译
    v_srcs = [str(ROOT / s) for s in t["src"] if s.endswith(".v")]
    sv_srcs = [str(ROOT / s) for s in t["src"] if s.endswith(".sv")]
    for tag, extra, srcs in (("v", [], v_srcs), ("sv", ["-sv"], sv_srcs)):
        if not srcs:
            continue
        cmd = ["cmd", "/c", bat("xvlog")] + extra + inc + defs + srcs
        if sh(cmd, wd, wd / f"xvlog_{tag}.log"):
            return False, f"xvlog 失败，见 {wd / f'xvlog_{tag}.log'}"
    cmd = ["cmd", "/c", bat("xelab"), "-debug", "off", "-timescale", "1ns/1ps", t["top"], "-s", "snap"]
    for k, v in t.get("generics", {}).items():
        cmd += ["-generic_top", f"{k}={v}"]
    if sh(cmd, wd, wd / "xelab.log"):
        return False, f"xelab 失败，见 {wd / 'xelab.log'}"
    sh(["cmd", "/c", bat("xsim"), "snap", "-R"], wd, wd / "xsim.log")
    out = (wd / "xsim.log").read_text(encoding="utf-8", errors="replace")
    if "TEST FAILED" in out or "Fatal" in out or "FATAL" in out or "ERROR:" in out:
        return False, f"仿真失败，见 {wd / 'xsim.log'}"
    if "TEST PASSED" not in out:
        return False, f"未见 TEST PASSED，见 {wd / 'xsim.log'}"

    if t.get("post"):
        rc = sh([PY, str(ROOT / t["post"][0])] + t["post"][1:] + env_args, ROOT, wd / "post.log")
        if rc:
            return False, f"post 比对失败，见 {wd / 'post.log'}"
    return True, f"{time.time() - t0:.1f}s"


def main(argv):
    if argv and argv[0] == "-l":
        for k in TESTS:
            print(k)
        return 0
    names = argv or list(TESTS)
    bad = 0
    for n in names:
        if n not in TESTS:
            print(f"[??] {n}: 未定义")
            bad += 1
            continue
        ok, msg = run_test(n)
        print(f"[{'PASS' if ok else 'FAIL'}] {n}: {msg}", flush=True)
        bad += not ok
    print(f"\n{len(names) - bad}/{len(names)} passed")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
