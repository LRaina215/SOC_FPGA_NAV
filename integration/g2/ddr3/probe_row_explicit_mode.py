"""Prove explicit row RAM behavior and observe its actual synthesized mode."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import sys
import xml.etree.ElementTree as ET

import probe_row_update as row
sys.path.insert(0,str(row.HERE.parent))
from audit_netlist_memories import MODULE, PARAM
from measure_slf_row_banks import TOOL, run_syn
from preflight import validate


def run(out):
    row.prepare(out)
    hardware=(out/'nav_ddr_row_stream.v').read_text()
    (out/'hardware_row_stream.v').write_text(hardware)
    library=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
    (out/'nav_ddr_row_stream.v').write_text('`define GOWIN_ROW_PRIMITIVES\n'+hardware+'\n'+library.read_text())
    bench=(out/'tb_slf_row_update.sv').read_text()
    assert bench.count('module tb_slf_row_update;')==1
    bench=bench.replace('module tb_slf_row_update;','module tb_slf_row_update;\n    GSR GSR(.GSRI(1\'b1));')
    (out/'tb_slf_row_update.sv').write_text(bench)
    row.run(out)
    result=json.loads((out/'functional_result.json').read_text())
    original=json.loads((out.parent/'ddr_row_normalmode_probe1/functional_result.json').read_text())
    assert result['timing']==original['timing'],'primitive-model cycle counts differ'
    return finish_synthesis(out)


def finish_synthesis(out):
    result=json.loads((out/'functional_result.json').read_text())
    original=json.loads((out.parent/'ddr_row_normalmode_probe1/functional_result.json').read_text())
    assert result['passed'] and result['timing']==original['timing']
    raw=(out/'hardware_row_stream.v').read_text()
    assert raw==(row.HERE/'nav_ddr_row_stream.v').read_text()
    # Gowin does not define SYNTHESIS in this project. Bind the exact same
    # explicit primitive branch exercised by the passed simulator run.
    hardware='`define GOWIN_ROW_PRIMITIVES\n'+raw
    (out/'hardware_explicit_row_stream.v').write_text(hardware)
    syn=out/'synthesis_explicit';syn.mkdir()
    names=('nav_ddr_native.v','nav_ddr_row_stream.v','nav_slf_row_update.v','slf_lf_ce.v')
    for name in names:
        text=hardware if name=='nav_ddr_row_stream.v' else (out/name).read_text()
        if name=='nav_slf_row_update.v':text=text.replace('module nav_slf_row_update (','module g2_top (')
        (syn/name).write_text(text)
    (syn/'clock.sdc').write_text('create_clock -name core -period 20 [get_ports {clk}]\ncreate_clock -name ui -period 10 [get_ports {ui_clk}]\n')
    settings=['set_device -device_version C GW5AT-LV138PG484AC1/I0']
    settings += [f'add_file -type verilog {{{(syn/n).as_posix()}}}' for n in names]
    settings += [f'add_file -type sdc {{{(syn/"clock.sdc").as_posix()}}}',
                 'set_option -top_module g2_top','set_option -output_base_name g2_top',
                 'set_option -global_freq 50','set_option -convert_sdp32_36_to_sdp16_18 0',
                 'set_option -looplimit 100000']
    validate(TOOL,syn,settings,50,0)
    (syn/'run.tcl').write_text('\n'.join(settings+['run syn','exit'])+'\n')
    runtime=run_syn(syn,scope='row RAM mode fix only; observe actual primitive mode before another joint SYN')
    modules=dict(MODULE.findall((syn/'impl/gwsynthesis/g2_top.vg').read_text()))
    parameters={}
    for body in modules.values():
        for instance,key,value in PARAM.findall(body):
            if 'input_row_primitive' in instance:
                parameters.setdefault(instance.strip(),{})[key]=value.strip()
    assert len(parameters)==2 and all(p['WRITE_MODE0']==p['WRITE_MODE1']=="2'b00" for p in parameters.values()),parameters
    resources=ET.parse(syn/'impl/gwsynthesis/g2_top_syn_rsc.xml').getroot().attrib
    result.update(tested_at=datetime.now().astimezone().isoformat(),actual_vendor_DPX9B_model=True,
        vendor_library='D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v',
        hardware_source=str(out/'hardware_explicit_row_stream.v'),
        original_8_case_cycle_counts_unchanged=True,actual_SYN_input_RAM_parameters=parameters,
        actual_module_resources=resources,actual_module_SYN_runtime=runtime,
        PNR_mode_acceptance_verified=False,whole_joint_implementation_verified=False)
    (out/'explicit_mode_result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='timing'},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True)
    p.add_argument('--finish-synthesis-only',action='store_true')
    a=p.parse_args()
    (finish_synthesis if a.finish_synthesis_only else run)(a.out.resolve())
