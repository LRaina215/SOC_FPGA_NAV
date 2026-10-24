"""Small standalone map-cache model/SYN. Never launches a joint build or PNR."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from measure_slf_row_banks import run_syn, inventory, TOOL
from retry_frozen_synthesis import memory_status
from preflight import validate


def functional(out):
    assert not out.exists()
    out.mkdir(parents=True)
    for f in ('nav_ddr_map_cache.v', 'nav_ddr_native.v', 'tb_ddr_maps.sv'):
        shutil.copyfile(HERE/f, out/f)
    model = (HERE/'ddr_native_model.sv').read_text()
    # Only increase the modeled address range and supply initialized backing
    # data. The model is not a DDR training/power-on initialization test.
    changes = [('module ddr_native_model', 'module ddr_map_native_model'),
        ('[0:4095]', '[0:16383]'), ('[11:0] held_addr', '[13:0] held_addr'),
        ('i<4096', 'i<16384'), ("mem[i]='x", 'mem[i]=0'),
        ('addr[27:15]', 'addr[27:17]'), ('addr[14:3]', 'addr[16:3]')]
    for old, new in changes:
        assert old in model, old
        model = model.replace(old, new)
    (out/'ddr_map_native_model.sv').write_text(model)
    viv = Path('E:/2026.1/Vivado/bin')
    commands = [('xvlog',['nav_ddr_native.v','nav_ddr_map_cache.v']),
        ('xvlog',['-sv','ddr_map_native_model.sv','tb_ddr_maps.sv']),
        ('xelab',['-debug','off','-timescale','1ns/1ps','tb_ddr_maps','-s','snap']),
        ('xsim',['snap','-runall'])]
    for i, (tool, args) in enumerate(commands):
        with (out/f'{i}_{tool}_stdout.log').open('w') as log:
            subprocess.run(['cmd','/c',str(viv/(tool+'.bat')),*args],cwd=out,
                stdout=log,stderr=subprocess.STDOUT,timeout=180,check=True,
                creationflags=subprocess.CREATE_NO_WINDOW)
    s=(out/'3_xsim_stdout.log').read_text(errors='replace')
    assert 'TEST PASSED' in s and not re.search(r'FATAL|Fatal|ERROR:',s),s[-3000:]
    line=re.search(r'TEST PASSED[^\r\n]*',s)[0]
    metrics={k:int(v) for k,v in re.findall(r'(\w+)=(\d+)',line)}
    result=dict(tested_at=datetime.now().astimezone().isoformat(),passed=True,
        metrics=metrics,programmed=False,
        scope='full map capacities, cache/native RTL plus behavioral initialized DDR; not PHY, integration or timing admission',
        backing_initialization_tested=False,algorithm_adapters_implemented=False,
        hardware_latency_bound_verified=False)
    (out/'functional_result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))


def synthesize(tested,out):
    proof=json.loads((tested/'functional_result.json').read_text())
    assert proof['passed']
    for f in ('nav_ddr_map_cache.v','nav_ddr_native.v'):
        assert (tested/f).read_bytes()==(HERE/f).read_bytes(),'untested source changed'
    m=memory_status()
    assert m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3,'2/4 GiB startup gate'
    assert not out.exists()
    out.mkdir(parents=True)
    for f in ('nav_ddr_map_cache.v','nav_ddr_native.v'):
        s=(HERE/f).read_text().replace('module nav_ddr_maps_probe #(', 'module g2_top #(')
        (out/f).write_text(s)
    (out/'clock.sdc').write_text('create_clock -name core -period 20 [get_ports {clk}]\ncreate_clock -name ui -period 10 [get_ports {ui_clk}]\n')
    settings=['set_device -device_version C GW5AT-LV138PG484AC1/I0']
    settings += [f'add_file -type verilog {{{(out/f).as_posix()}}}' for f in ('nav_ddr_native.v','nav_ddr_map_cache.v')]
    settings += [f'add_file -type sdc {{{(out/"clock.sdc").as_posix()}}}',
        'set_option -top_module g2_top','set_option -output_base_name g2_top',
        'set_option -global_freq 50','set_option -convert_sdp32_36_to_sdp16_18 0',
        'set_option -looplimit 100000']
    effective=validate(TOOL,out,settings,50,0)
    (out/'run.tcl').write_text('\n'.join(settings+['run syn','exit'])+'\n')
    runtime=run_syn(out,scope='two map caches, arbitration and native mailbox only; no joint build or PNR')
    syn=out/'impl/gwsynthesis'
    r=dict(tested=str(tested),build=str(out),runtime=runtime,effective_options=effective,
        startup_available_physical=m.avail_phys,startup_available_commit=m.avail_page,
        resources=ET.parse(syn/'g2_top_syn_rsc.xml').getroot().attrib,
        inventory=inventory(syn/'g2_top.vg'),programmed=False,
        full_joint_implementation=False,physical_resources_verified=False)
    (out/'result.json').write_text(json.dumps(r,indent=2))
    print(json.dumps({k:v for k,v in r.items() if k!='inventory'},indent=2))


def mapped(synthesis,out):
    r=json.loads((synthesis/'result.json').read_text())
    tested=Path(r['tested'])
    assert r['runtime']['tool_exit']==0 and not r['runtime']['cancelled']
    for f in ('nav_ddr_map_cache.v','nav_ddr_native.v'):
        assert (tested/f).read_bytes()==(HERE/f).read_bytes()
    assert not out.exists()
    out.mkdir(parents=True)
    text=(synthesis/'impl/gwsynthesis/g2_top.vg').read_text()
    assert text.count('module g2_top (')==1
    (out/'mapped_maps.v').write_text(text.replace('module g2_top (','module nav_ddr_maps_probe ('))
    shutil.copyfile(tested/'ddr_map_native_model.sv',out/'ddr_map_native_model.sv')
    bench=(HERE/'tb_ddr_maps.sv').read_text()
    (out/'tb_ddr_maps.sv').write_text('`define MAPPED_MAPS\n'+bench)
    library=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
    shutil.copyfile(library,out/'vendor_primitives.v')
    viv=Path('E:/2026.1/Vivado/bin')
    commands=[('xvlog',['vendor_primitives.v','mapped_maps.v']),
        ('xvlog',['-sv','ddr_map_native_model.sv','tb_ddr_maps.sv']),
        ('xelab',['-debug','off','-timescale','1ns/1ps','tb_ddr_maps','-s','snap']),
        ('xsim',['snap','-runall'])]
    for i,(tool,args) in enumerate(commands):
        with (out/f'{i}_{tool}_stdout.log').open('w') as log:
            subprocess.run(['cmd','/c',str(viv/(tool+'.bat')),*args],cwd=out,
                stdout=log,stderr=subprocess.STDOUT,timeout=180,check=True,
                creationflags=subprocess.CREATE_NO_WINDOW)
    s=(out/'3_xsim_stdout.log').read_text(errors='replace')
    assert 'TEST PASSED' in s and not re.search(r'FATAL|Fatal|ERROR:',s),s[-3000:]
    line=re.search(r'TEST PASSED[^\r\n]*',s)[0]
    proof=dict(tested_at=datetime.now().astimezone().isoformat(),passed=True,
        synthesis=str(synthesis),primitive_library=str(library),
        metrics={k:int(v) for k,v in re.findall(r'(\w+)=(\d+)',line)
                 if not k.startswith('native_')},
        native_cycle_monitor_omitted=True,programmed=False,
        scope='actual synthesized cache/native/mailbox netlist and vendor primitives; behavioral DDR backing, no PHY/timing acceptance')
    (out/'mapped_result.json').write_text(json.dumps(proof,indent=2))
    print(json.dumps(proof,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True)
    p.add_argument('--synthesis-after',type=Path);p.add_argument('--mapped-after',type=Path);a=p.parse_args()
    if a.mapped_after:mapped(a.mapped_after.resolve(),a.out.resolve())
    elif a.synthesis_after:synthesize(a.synthesis_after.resolve(),a.out.resolve())
    else:functional(a.out.resolve())
