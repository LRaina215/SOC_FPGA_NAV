"""Synthesize/map-check the optional four-bank parallel cache, no joint build."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET
from interleave_struct_cache import derive
from probe_row_update import HERE, run_syn, TOOL, validate, memory_status
from audit_netlist_memories import inventory


def synthesize(tested, bound, out):
    for proof in (tested, bound):
        p = json.loads((proof/'result.json').read_text())
        assert p['passed'] and p['interleaved_cache_banks']
        assert (proof/'nav_ddr_struct_cache_fast.v').read_text() == derive((HERE/'nav_ddr_struct_cache.v').read_text())
    m = memory_status()
    assert m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3, '2/4GiB startup gate'
    assert not out.exists();out.mkdir(parents=True)
    s=(tested/'nav_ddr_struct_cache_fast.v').read_text().replace('module nav_ddr_struct_cache_fast(', 'module g2_top(')
    (out/'cache.v').write_text(s)
    (out/'clock.sdc').write_text('create_clock -name core -period 20 [get_ports {clk}]\n')
    settings=['set_device -device_version C GW5AT-LV138PG484AC1/I0',
        f'add_file -type verilog {{{(out/"cache.v").as_posix()}}}',
        f'add_file -type sdc {{{(out/"clock.sdc").as_posix()}}}',
        'set_option -top_module g2_top','set_option -output_base_name g2_top',
        'set_option -global_freq 50','set_option -convert_sdp32_36_to_sdp16_18 0',
        'set_option -looplimit 100000']
    options=validate(TOOL,out,settings,50,0)
    (out/'run.tcl').write_text('\n'.join(settings+['run syn','exit'])+'\n')
    runtime=run_syn(out,scope='isolated optional interleaved structure cache only, no G2 SYN/PNR')
    inv=inventory(out)
    assert len(inv['leaves'])==4 and all(x['parameters']['BIT_WIDTH_0']=='18' for x in inv['leaves'])
    r=dict(tested=str(tested),bound=str(bound),runtime=runtime,effective_options=options,
        startup_available_physical=m.avail_phys,startup_available_commit=m.avail_page,
        resources=ET.parse(out/'impl/gwsynthesis/g2_top_syn_rsc.xml').getroot().attrib,
        inventory=inv,source_bsram_leaves=4,wide_leaves=0,
        physical_resources_verified=False,full_joint_implementation=False,programmed=False)
    (out/'result.json').write_text(json.dumps(r,indent=2))
    print(json.dumps({k:v for k,v in r.items() if k!='inventory'},indent=2))


def check(out, synthesis=None):
    assert not out.exists();out.mkdir(parents=True)
    bench=(HERE/'tb_interleaved_cache.sv').read_text()
    if synthesis:
        result=json.loads((synthesis/'result.json').read_text())
        expected=derive((HERE/'nav_ddr_struct_cache.v').read_text()).replace('module nav_ddr_struct_cache_fast(', 'module g2_top(')
        assert (synthesis/'cache.v').read_text()==expected
        assert result['runtime']['tool_exit']==0 and not result['runtime']['cancelled']
        shutil.copyfile(synthesis/'impl/gwsynthesis/g2_top.vg',out/'cache.v')
        shutil.copyfile(Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v'),out/'vendor_primitives.v')
        names=['vendor_primitives.v','cache.v'];bench='`define MAPPED_CACHE\n'+bench
    else:
        (out/'cache.v').write_text(derive((HERE/'nav_ddr_struct_cache.v').read_text()))
        names=['cache.v']
    (out/'tb.sv').write_text(bench)
    commands=[('xvlog',names),('xvlog',['-sv','tb.sv']),
        ('xelab',['-debug','off','-timescale','1ns/1ps','tb_interleaved_cache','-s','snap']),
        ('xsim',['snap','-runall'])]
    for i,(tool,args) in enumerate(commands):
        with (out/f'{i}_{tool}.log').open('w') as log:
            subprocess.run(['cmd','/c',str(Path('E:/2026.1/Vivado/bin')/(tool+'.bat')),*args],cwd=out,
                stdout=log,stderr=subprocess.STDOUT,check=True,timeout=1200,creationflags=subprocess.CREATE_NO_WINDOW)
    text=(out/'3_xsim.log').read_text(errors='replace')
    assert 'TEST PASSED' in text and not re.search(r'FATAL|Fatal|ERROR:',text),text[-4000:]
    r=dict(tested_at=datetime.now().astimezone().isoformat(),passed=True,mapped=bool(synthesis),
        synthesis=str(synthesis) if synthesis else None,
        metrics={k:int(v) for k,v in re.findall(r'(\w+)=(\d+)',re.search(r'TEST PASSED[^\r\n]*',text)[0])},
        real_worst_update_under_5ms_verified=False,physical_resources_verified=False,programmed=False)
    (out/'result.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True)
    p.add_argument('--synthesis-after',type=Path);p.add_argument('--bound-proof',type=Path)
    p.add_argument('--mapped-after',type=Path);a=p.parse_args()
    if a.synthesis_after:
        assert a.bound_proof;synthesize(a.synthesis_after.resolve(),a.bound_proof.resolve(),a.out.resolve())
    else:check(a.out.resolve(),a.mapped_after.resolve() if a.mapped_after else None)
