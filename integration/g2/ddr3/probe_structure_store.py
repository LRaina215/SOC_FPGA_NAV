"""Packed-cache/exclusive-UPDATE coherence proof, not DDR physical admission."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from probe_row_update import HERE,run_syn,inventory,TOOL,memory_status,validate
sys.path.insert(0,str(HERE.parent))
from synthesis_connectivity import read
from audit_routing_topology import primitive_directions
from audit_netlist_memories import inventory as memory_inventory


def run(out,row_probe):
    assert not out.exists();out.mkdir(parents=True)
    old=json.loads((row_probe/'functional_result.json').read_text());assert old['passed']
    files=('nav_ddr_native.v','nav_ddr_row_stream.v','nav_slf_row_update.v',
        'nav_ddr_struct_cache.v','nav_ddr_structure_store.v')
    for f in files:
        if (row_probe/f).exists():assert (row_probe/f).read_bytes()==(HERE/f).read_bytes()
        shutil.copyfile(HERE/f,out/f)
    shutil.copyfile(row_probe/'slf_lf_ce.v',out/'slf_lf_ce.v')
    for f in row_probe.glob('case*.hex'):shutil.copyfile(f,out/f.name)
    shutil.copyfile(row_probe/'cases.txt',out/'cases.txt')
    shutil.copyfile(HERE/'tb_structure_store.sv',out/'tb_structure_store.sv')
    model=(HERE/'tb_slf_row_update.sv').read_text().split('module tb_slf_row_update;')[0]
    (out/'row_service_model.sv').write_text(model)
    viv=Path('E:/2026.1/Vivado/bin')
    commands=[('xvlog',[*files,'slf_lf_ce.v']),
        ('xvlog',['-sv','row_service_model.sv','tb_structure_store.sv']),
        ('xelab',['-debug','off','-timescale','1ns/1ps','tb_structure_store','-s','snap']),
        ('xsim',['snap','-runall'])]
    for i,(tool,args) in enumerate(commands):
        with (out/f'{i}_{tool}_stdout.log').open('w') as log:
            subprocess.run(['cmd','/c',str(viv/(tool+'.bat')),*args],cwd=out,
                stdout=log,stderr=subprocess.STDOUT,timeout=180,check=True,
                creationflags=subprocess.CREATE_NO_WINDOW)
    s=(out/'3_xsim_stdout.log').read_text(errors='replace')
    assert 'TEST PASSED' in s and not re.search(r'FATAL|Fatal|ERROR:',s),s[-4000:]
    result=dict(tested_at=datetime.now().astimezone().isoformat(),passed=True,
        timing=[{k:float(v) if '.' in v else int(v) for k,v in re.findall(r'(\w+)=([\d.]+)',line)}
            for line in s.splitlines() if line.startswith('COHERENCE_TIMING')],
        metrics={k:int(v) for k,v in re.findall(r'(\w+)=(\d+)',re.search(r'TEST PASSED[^\r\n]*',s)[0])},
        packed18_cache_and_stream_same_layout=True,all_resident_lines_dirty_before_update=True,
        stalled_client_reads_new_data=True,inflight_client_drained_before_update=True,
        calibration_fault_sticky=True,
        scope='complete cache/UPDATE ownership RTL plus bounded native-service model; not whole matcher, PHY or CDC timing',
        real_worst_update_under_5ms_verified=False,full_joint_implementation=False,programmed=False)
    (out/'result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))


def synthesize(tested,out):
    proof=json.loads((tested/'result.json').read_text());assert proof['passed']
    names=('nav_ddr_native.v','nav_ddr_row_stream.v','nav_slf_row_update.v',
        'nav_ddr_struct_cache.v','nav_ddr_structure_store.v')
    for f in names:assert (tested/f).read_bytes()==(HERE/f).read_bytes()
    m=memory_status()
    assert m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3,'2/4GiB startup gate'
    assert not out.exists();out.mkdir(parents=True)
    for f in (*names,'slf_lf_ce.v'):
        s=(tested/f).read_text(encoding='utf-8')
        if f=='nav_ddr_structure_store.v':s=s.replace('module nav_ddr_structure_store(','module g2_top(')
        (out/f).write_text(s,encoding='utf-8')
    (out/'clock.sdc').write_text('create_clock -name core -period 20 [get_ports {clk}]\ncreate_clock -name ui -period 10 [get_ports {ui_clk}]\n')
    settings=['set_device -device_version C GW5AT-LV138PG484AC1/I0']
    settings += [f'add_file -type verilog {{{(out/f).as_posix()}}}' for f in (*names,'slf_lf_ce.v')]
    settings += [f'add_file -type sdc {{{(out/"clock.sdc").as_posix()}}}',
        'set_option -top_module g2_top','set_option -output_base_name g2_top',
        'set_option -global_freq 50','set_option -convert_sdp32_36_to_sdp16_18 0',
        'set_option -looplimit 100000']
    effective=validate(TOOL,out,settings,50,0)
    (out/'run.tcl').write_text('\n'.join(settings+['run syn','exit'])+'\n')
    runtime=run_syn(out,scope='isolated coherent packed cache/UPDATE store only; no G2 SYN/PNR')
    syn=out/'impl/gwsynthesis'
    result=dict(tested=str(tested),runtime=runtime,effective_options=effective,
        startup_available_physical=m.avail_phys,startup_available_commit=m.avail_page,
        resources=ET.parse(syn/'g2_top_syn_rsc.xml').getroot().attrib,
        inventory=inventory(syn/'g2_top.vg'),programmed=False,
        full_joint_implementation=False,physical_resources_verified=False)
    result['ownership_sync_audit']=audit_control(out)
    (out/'result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='inventory'},indent=2))


def audit_control(out):
    cells=read(out/'impl/gwsynthesis/g2_top.vg')
    directions=primitive_directions(Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v'))
    def driver(net):
        found=[c for c in cells.values() if c['kind'].startswith('DFF') and c['ports'].get('Q')==[net]]
        assert len(found)==1,(net,found)
        return found[0]
    checks=[]
    for first,second,src_clock,dst_clock in (
        ('own1','own2','clk_d','ui_clk_d'),
        ('own_ack1','own_ack2','ui_clk_d','clk_d'),
        ('cf1','cf2','ui_clk_d','clk_d')):
        a,b=driver(first),driver(second)
        source=driver(a['ports']['D'][0])
        loads=[dict(cell=n,pin=p,kind=c['kind']) for n,c in cells.items()
            for p,nets in c['ports'].items()
            if directions[c['kind']][p] in ('input','inout') and first in nets]
        assert b['ports']['D']==[first]
        assert source['ports']['CLK']==[src_clock]
        assert a['ports']['CLK']==b['ports']['CLK']==[dst_clock]
        assert a['ports']['CLEAR']==b['ports']['CLEAR']
        assert a['ports']['CE']==b['ports']['CE']==['VCC']
        assert loads==[dict(cell=b['name'],pin='D',kind=b['kind'])]
        checks.append(dict(source=source,first=a,second=b,first_Q_loads=loads,passed=True))
    proof=dict(passed=True,scope='three two-FF controls only, not payload or physical CDC timing',checks=checks)
    (out/'ownership_sync_audit.json').write_text(json.dumps(proof,indent=2))
    ram=memory_inventory(out)
    cache=[v for v in ram['leaves'] if '/u_cache/' in v['path']]
    assert len(cache)==4 and all(v['parameters']['BIT_WIDTH_0']=='18' for v in cache)
    (out/'memory_inventory.json').write_text(json.dumps(ram,indent=2))
    return proof


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True)
    p.add_argument('--row-probe',type=Path);p.add_argument('--synthesis-after',type=Path)
    p.add_argument('--audit-existing',action='store_true');a=p.parse_args()
    if a.audit_existing:print(json.dumps(audit_control(a.out.resolve()),indent=2))
    elif a.synthesis_after:synthesize(a.synthesis_after.resolve(),a.out.resolve())
    else:
        assert a.row_probe is not None
        run(a.out.resolve(),a.row_probe.resolve())
