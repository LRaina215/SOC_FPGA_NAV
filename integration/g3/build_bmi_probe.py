"""Freeze and build a small diagnostic image; never program hardware."""
import argparse
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "integration/g2"))
from bounded_tool import run_tool  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    sources = [ROOT / "integration/g3/bmi_probe_top.v"] + [
        ROOT / "control/fpga/src" / name for name in (
            "bmi088.v", "oled_driver.v", "board_uart_tx.v", "uart_telemetry.v")]
    for source in sources:
        shutil.copyfile(source, out / source.name)
    shutil.copyfile(ROOT / "integration/g3/bmi_probe.cst", out / "bmi_probe.cst")
    (out / "bmi_probe.sdc").write_text(
        "create_clock -name core_clk -period 20.000 [get_ports {clk}]\n", encoding="utf-8")
    tcl = ["set_device -device_version C GW5AT-LV138PG484AC1/I0"]
    tcl += [f"add_file -type verilog {{{(out / source.name).as_posix()}}}" for source in sources]
    tcl += [f"add_file -type {kind} {{{(out / ('bmi_probe.' + kind)).as_posix()}}}"
            for kind in ("cst", "sdc")]
    tcl += ["set_option -top_module bmi_probe_top", "set_option -output_base_name bmi_probe",
            "set_option -verilog_std v2001", "set_option -global_freq 50",
            "set_option -gen_verilog_sim_netlist 1", "set_option -place_option 1",
            "set_option -route_option 2", "run all", "exit"]
    (out / "run.tcl").write_text("\n".join(tcl) + "\n", encoding="utf-8")
    state = run_tool(["D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe", "run.tcl"],
                     out, max_runtime_s=300, max_no_progress_s=180)
    state.update(scope="diagnostic build only; inspect timing, constant actuator outputs and UART contract before SRAM test",
                 original_driver_copied_without_edits=True, programmed=False)
    (out / "build_result.json").write_text(json.dumps(state, indent=2), encoding="utf-8")
    print(json.dumps(state, indent=2))
    return 0 if state["tool_exit"] == 0 and not state["cancelled"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
