"""Full-capacity streaming update experiment, not a vendor DDR WCET claim."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sys.path.insert(0,str(ROOT/'model'))
import slf_match as slf
sys.path.insert(0,str(HERE.parent))
from measure_slf_row_banks import run_syn,inventory,TOOL
from retry_frozen_synthesis import memory_status
from preflight import validate


def prepare(out):
    assert not out.exists()
    out.mkdir(parents=True)
    for name in ('nav_ddr_native.v','nav_ddr_row_stream.v','nav_slf_row_update.v','tb_slf_row_update.sv'):
        shutil.copyfile(HERE/name,out/name)
    original=(ROOT/'rtl/match/slf_lf.v').read_text(encoding='utf-8-sig')
    assert original.count('module slf_lf #(')==1
    assert original.count('    input  wire        rst,')==1
    assert original.count('        end else begin\n            // ---------------- 扫描')==1
    assert original.count("        sm_we <= 1'b0; done <= 1'b0;")==1
    assert original.count("            run <= 1'b0; busy <= 1'b0;")==1
    modified=original.replace('module slf_lf #(','module slf_lf_ce #(').replace(
        '    input  wire        rst,','    input  wire        rst,\n    input wire ce,').replace(
        '        end else begin\n            // ---------------- 扫描',
        '        end else if(ce) begin\n            // ---------------- 扫描')
    modified=modified.replace("        sm_we <= 1'b0; done <= 1'b0;",
        "        if(ce)begin sm_we <= 1'b0; done <= 1'b0;end").replace(
        "            run <= 1'b0; busy <= 1'b0;",
        "            sm_we <= 1'b0; done <= 1'b0;\n            run <= 1'b0; busy <= 1'b0;")
    (out/'slf_lf_original.v').write_text(original,encoding='utf-8')
    (out/'slf_lf_ce.v').write_text(modified,encoding='utf-8')
    rng=np.random.default_rng(4500125)
    cases=[]
    for case,(cx,cy,mode,offset,period,latency,expect_5ms) in enumerate([
        (128,128,'spread',0,8,12,1),(-17,241,'spread',389,8,12,1),
        (511,-260,'row',200,8,12,1),(0,0,'duplicates',27,8,12,1),
        (1,-1,'empty',0,8,12,1),(128,128,'spread',101,16,20,1),
        (128,128,'four_rows',389,16,20,1),
        (128,128,'spread',200,32,80,0)]):
        m=slf.StructMap()
        m.tag=rng.integers(0,4,65536,dtype=np.int64)
        m.hit=rng.integers(0,4,65536,dtype=np.int64)
        m.cxs=2*rng.integers(0,8,65536,dtype=np.int64)+1
        m.cys=2*rng.integers(0,8,65536,dtype=np.int64)+1
        low=rng.integers(0,256,65536,dtype=np.int64)
        before=slf.pack_hi(m.tag,m.hit,m.cxs,m.cys)|low
        points=rng.integers(-100000,100001,(1024,2),dtype=np.int64)
        if mode=='row':points[:,1]=25
        if mode=='duplicates':points[1::2]=points[::2]
        if mode=='four_rows':
            slots=np.arange(1024,dtype=np.int64)
            points=np.column_stack(((16*(slots&255)*4096+1310)//1311,
                (16*(slots>>8)*4096+1310)//1311))
        if mode=='empty':points=points[:0]
        fx,fy=m.cell16(points[:,0]),m.cell16(points[:,1])
        ix,iy=fx>>4,fy>>4
        a=((iy&255)<<8)|(ix&255)
        meta=(slf.tag_of(ix,iy)<<6)|(((fx&15)>>1)<<3)|((fy&15)>>1)
        records=(a<<8)|meta
        m.update(points);m.build_lf(cx,cy)
        gold,mask=m.words()
        assert np.all(mask==(1<<18)-1)
        names=[f'case{case}_{s}.hex' for s in ('initial','gold','records')]
        for name,values in zip(names,(before,gold,records)):
            (out/name).write_text(''.join(f'{int(v):06x}\n' for v in values))
        cases.append(' '.join(names)+f' {len(records)} {cx} {cy} {offset} {period} {latency} {expect_5ms}')
    (out/'cases.txt').write_text(str(len(cases))+'\n'+'\n'.join(cases)+'\n')


def run(out):
    viv=Path('E:/2026.1/Vivado/bin')
    commands=[('xvlog',['nav_ddr_native.v','nav_ddr_row_stream.v','slf_lf_ce.v','nav_slf_row_update.v']),
        ('xvlog',['-sv','tb_slf_row_update.sv']),
        ('xelab',['-debug','off','-timescale','1ns/1ps','tb_slf_row_update','-s','snap']),
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
            for line in s.splitlines() if line.startswith('ROW_TIMING')],
        metrics={k:int(v) for k,v in re.findall(r'(\w+)=(\d+)',re.search(r'TEST PASSED[^\r\n]*',s)[0])},
        scope='full-size update/LF RTL with original LF arithmetic and service-envelope native model',
        actual_point_transform_integrated=False,vendor_service_bound_verified=False,
        all_legal_native_service_schedules_meet_5ms=False,
        complete_joint_candidate=False,programmed=False)
    (out/'functional_result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))


def synthesize(tested,out):
    proof=json.loads((tested/'functional_result.json').read_text())
    assert proof['passed']
    names=('nav_ddr_native.v','nav_ddr_row_stream.v','nav_slf_row_update.v')
    for name in names:assert (tested/name).read_bytes()==(HERE/name).read_bytes()
    assert (tested/'slf_lf_original.v').read_text(encoding='utf-8')==\
        (ROOT/'rtl/match/slf_lf.v').read_text(encoding='utf-8-sig')
    m=memory_status()
    assert m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3,'2/4GiB startup gate'
    assert not out.exists();out.mkdir(parents=True)
    for name in (*names,'slf_lf_ce.v'):
        s=(tested/name).read_text(encoding='utf-8')
        if name=='nav_slf_row_update.v':s=s.replace('module nav_slf_row_update (','module g2_top (')
        (out/name).write_text(s,encoding='utf-8')
    (out/'clock.sdc').write_text('create_clock -name core -period 20 [get_ports {clk}]\ncreate_clock -name ui -period 10 [get_ports {ui_clk}]\n')
    settings=['set_device -device_version C GW5AT-LV138PG484AC1/I0']
    settings += [f'add_file -type verilog {{{(out/name).as_posix()}}}' for name in (*names,'slf_lf_ce.v')]
    settings += [f'add_file -type sdc {{{(out/"clock.sdc").as_posix()}}}',
        'set_option -top_module g2_top','set_option -output_base_name g2_top',
        'set_option -global_freq 50','set_option -convert_sdp32_36_to_sdp16_18 0',
        'set_option -looplimit 100000']
    effective=validate(TOOL,out,settings,50,0)
    (out/'run.tcl').write_text('\n'.join(settings+['run syn','exit'])+'\n')
    runtime=run_syn(out,scope='isolated full-size streaming update/LF only; no G2 SYN/PNR')
    syn=out/'impl/gwsynthesis'
    result=dict(tested=str(tested),runtime=runtime,effective_options=effective,
        startup_available_physical=m.avail_phys,startup_available_commit=m.avail_page,
        resources=ET.parse(syn/'g2_top_syn_rsc.xml').getroot().attrib,
        inventory=inventory(syn/'g2_top.vg'),programmed=False,
        full_joint_implementation=False,physical_resources_verified=False)
    (out/'result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='inventory'},indent=2))


def mapped(synthesis,out):
    result=json.loads((synthesis/'result.json').read_text())
    assert result['runtime']['tool_exit']==0 and not result['runtime']['cancelled']
    tested=Path(result['tested'])
    for name in ('nav_ddr_native.v','nav_ddr_row_stream.v','nav_slf_row_update.v'):
        assert (tested/name).read_bytes()==(HERE/name).read_bytes()
    assert not out.exists();out.mkdir(parents=True)
    text=(synthesis/'impl/gwsynthesis/g2_top.vg').read_text()
    assert text.count('module g2_top (')==1
    (out/'mapped_row_update.v').write_text(text.replace('module g2_top (','module nav_slf_row_update ('))
    shutil.copyfile(Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v'),out/'vendor_primitives.v')
    for p in tested.glob('case*.hex'):shutil.copyfile(p,out/p.name)
    shutil.copyfile(tested/'cases.txt',out/'cases.txt')
    (out/'tb_slf_row_update.sv').write_text('`define MAPPED_ROW_UPDATE\n'+(HERE/'tb_slf_row_update.sv').read_text())
    viv=Path('E:/2026.1/Vivado/bin')
    commands=[('xvlog',['vendor_primitives.v','mapped_row_update.v']),
        ('xvlog',['-sv','tb_slf_row_update.sv']),
        ('xelab',['-debug','off','-timescale','1ns/1ps','tb_slf_row_update','-s','snap']),
        ('xsim',['snap','-runall'])]
    for i,(tool,args) in enumerate(commands):
        with (out/f'{i}_{tool}_stdout.log').open('w') as log:
            subprocess.run(['cmd','/c',str(viv/(tool+'.bat')),*args],cwd=out,
                stdout=log,stderr=subprocess.STDOUT,timeout=180,check=True,
                creationflags=subprocess.CREATE_NO_WINDOW)
    s=(out/'3_xsim_stdout.log').read_text(errors='replace')
    assert 'TEST PASSED' in s and not re.search(r'FATAL|Fatal|ERROR:',s),s[-4000:]
    proof=dict(tested_at=datetime.now().astimezone().isoformat(),passed=True,
        synthesis=str(synthesis),timing=[{k:float(v) if '.' in v else int(v)
            for k,v in re.findall(r'(\w+)=([\d.]+)',line)}
            for line in s.splitlines() if line.startswith('ROW_TIMING')],
        metrics={k:int(v) for k,v in re.findall(r'(\w+)=(\d+)',re.search(r'TEST PASSED[^\r\n]*',s)[0])},
        scope='actual synthesized pipeline/RAM and vendor primitives; native service model, not PHY/WCET acceptance',
        vendor_service_bound_verified=False,programmed=False)
    (out/'mapped_result.json').write_text(json.dumps(proof,indent=2))
    print(json.dumps(proof,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True)
    p.add_argument('--synthesis-after',type=Path)
    p.add_argument('--mapped-after',type=Path)
    a=p.parse_args();out=a.out.resolve()
    if a.mapped_after:mapped(a.mapped_after.resolve(),out)
    elif a.synthesis_after:synthesize(a.synthesis_after.resolve(),out)
    else:prepare(out);run(out)
