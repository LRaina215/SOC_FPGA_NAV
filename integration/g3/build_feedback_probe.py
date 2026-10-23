"""Freeze and build B1 diagnostics without programming hardware."""
import argparse,json,re,shutil,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'integration/g2'))
from bounded_tool import run_tool


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--prepare-only',action='store_true');a=p.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    files=[ROOT/'integration/g3'/n for n in ('feedback_probe_top.v','i2c_id_observer.v')]
    files += [ROOT/'control/fpga/src'/n for n in ('bmi088.v','oled_driver.v','motor520_speed.v','board_uart_tx.v','uart_telemetry.v')]
    for file in files:shutil.copyfile(file,out/file.name)
    # Reuse the reviewed B0 actuator/BMI/UART pins and original encoder pins.
    encoders=re.findall(r'^IO_(?:LOC|PORT) "M[12]_ENC[AB]"[^\r\n]+', (ROOT/'control/fpga/src/main.cst').read_text(),re.M)
    assert len(encoders)==8,encoders
    (out/'feedback_probe.cst').write_text((ROOT/'integration/g3/bmi_probe.cst').read_text()+'\n'+'\n'.join(encoders)+'\n',encoding='utf-8')
    (out/'feedback_probe.sdc').write_text('create_clock -name core_clk -period 20.000 [get_ports {clk}]\n',encoding='utf-8')
    lines=['set_device -device_version C GW5AT-LV138PG484AC1/I0']
    lines += [f'add_file -type verilog {{{(out/f.name).as_posix()}}}'for f in files]
    lines += [f'add_file -type {kind} {{{(out/("feedback_probe."+kind)).as_posix()}}}'for kind in ('cst','sdc')]
    lines += ['set_option -top_module feedback_probe_top','set_option -output_base_name feedback_probe',
        'set_option -verilog_std v2001','set_option -global_freq 50','set_option -gen_verilog_sim_netlist 1',
        'set_option -place_option 1','set_option -route_option 2','run all','exit']
    (out/'run.tcl').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    manifest=dict(sources=[str(f)for f in files],programmed=False,original_drivers_copied_without_edits=True,
        scope='B1 diagnostic only; independent timing, physical constant outputs and current actuator power must be checked before SRAM program')
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    if a.prepare_only:print(json.dumps(dict(prepared=True,build=str(out),programmed=False)));return
    result=run_tool(['D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe','run.tcl'],out,max_runtime_s=300,max_no_progress_s=180)
    (out/'build_result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result))
    raise SystemExit(0 if result['tool_exit']==0 and not result['cancelled']else 2)


if __name__=='__main__':main()
