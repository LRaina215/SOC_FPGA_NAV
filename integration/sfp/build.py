"""Freeze and build the SFP diagnostic image; does not program the board."""
import argparse
import json
from pathlib import Path
import shutil
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "integration/g2"))
from bounded_tool import run_tool
from generate_ip import generate, DEFAULT_IDE, DEFAULT_CACHE
from inspect_build import inspect

SOURCES = ["integration/sfp/" + n for n in
           ("sfp_bringup_top.v", "sfp_host_mac.v", "sfp_udp_echo.v")]
SOURCES += ["rtl/common/" + n for n in
            ("nav_cdc.v", "nav_ram_sdp.v", "nav_async_fifo.v", "nav_fifo.v", "nav_fifo_ob.v")]
SOURCES += ["rtl/net/" + n for n in ("net_port.v", "udp_rx.v", "udp_tx.v")]
SOURCES += ["rtl/io/uart_bus_bridge.v"]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--ide", type=Path, default=DEFAULT_IDE)
    p.add_argument("--mac-export", type=Path, default=DEFAULT_CACHE)
    p.add_argument("--stage", choices=("prepare", "synth", "all"), default="all")
    p.add_argument("--max-seconds", type=int, default=600)
    a = p.parse_args()
    out = a.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    for name in SOURCES:
        shutil.copyfile(ROOT / name, out / Path(name).name)
    for ext in ("cst", "sdc"):
        shutil.copyfile(HERE / f"sfp_bringup.{ext}", out / f"sfp_bringup.{ext}")
    ip = generate(out / "ip", a.ide, a.mac_export)
    sources = [out / Path(n).name for n in SOURCES] + [ip / "serdes.v", ip / "giga_serial_ethernet.v"]
    tcl = ["set_device -device_version C GW5AT-LV138PG484AC1/I0",
           f"set_csr {{{(ip / 'serdes.csr').as_posix()}}}"]
    tcl += [f"add_file -type verilog {{{s.as_posix()}}}" for s in sources]
    tcl += [f"add_file -type {ext} {{{(out / ('sfp_bringup.' + ext)).as_posix()}}}" for ext in ("cst", "sdc")]
    tcl += ["set_option -top_module sfp_bringup_top", "set_option -output_base_name sfp_bringup",
            "set_option -verilog_std v2001", "set_option -looplimit 100000", "set_option -global_freq 50",
            "set_option -gen_verilog_sim_netlist 1", "set_option -place_option 1",
            "set_option -route_option 2", "run " + ("syn" if a.stage == "synth" else "all"), "exit"]
    (out / "run.tcl").write_text("\n".join(tcl) + "\n", encoding="utf-8")
    if a.stage == "prepare":
        print(f"Prepared {out}")
        return 0
    state = run_tool([str(a.ide / "bin/gw_sh.exe"), "run.tcl"], out,
                     max_runtime_s=a.max_seconds, max_no_progress_s=180)
    state.update(stage=a.stage, programmed=False,
                 scope="Standalone diagnostic build; inspect logs/timing before using bitstream")
    if a.stage == "all" and state["tool_exit"] == 0 and not state["cancelled"]:
        try:
            state["inspection"] = inspect(out)
        except Exception as error:
            state["inspection_error"] = str(error)
    (out / "build_result.json").write_text(json.dumps(state, indent=2), encoding="utf-8")
    print(json.dumps(state, indent=2))
    return 0 if state["tool_exit"] == 0 and not state["cancelled"] and "inspection_error" not in state else 2


if __name__ == "__main__":
    raise SystemExit(main())
