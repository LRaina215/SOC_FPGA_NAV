"""Unique PNR85-derived SYN -> actual-netlist review -> unlimited PNR86. No programming."""
import argparse
from collections import Counter
import ctypes
from datetime import datetime
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

import psutil

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE));sys.path.insert(0,str(HERE.parent))
from prepare_candidate import prepare
from audit_netlist_memories import inventory,MODULE,PARAM,BSRAM
from audit_video_cdc import load as load_cells,bits
from bounded_tool import run_tool

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'K_terrain19_follow1';ANCHOR=BASE/'K_pnr85_terrain19p2r0'
SYN=BASE/'K_ultrasonic_manual_hold_syn1';PNR=BASE/'K_pnr86_ultrasonicp2r0'
OUT=BASE/'ultrasonic_joint_one_shot'
COMPACT_PROFILE=False
GW=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
GIB=1024**3

class Memory(ctypes.Structure):
    _fields_=[('length',ctypes.c_uint32),('load',ctypes.c_uint32)]+[(k,ctypes.c_uint64) for k in
        ('total_phys','avail_phys','total_page','avail_page','total_virtual','avail_virtual','avail_extended')]

def memory():
    m=Memory();m.length=ctypes.sizeof(m)
    if not ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(m)):raise ctypes.WinError()
    return dict(available_physical_bytes=m.avail_phys,available_commit_bytes=m.avail_page)

def write(path,data):path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf8')

def memory_parameters(build):
    text=(build/'impl/gwsynthesis/g2_top.vg').read_text(encoding='utf8')
    params={name:{} for name,_ in MODULE.findall(text)}
    for name,body in MODULE.findall(text):
        for inst,key,value in PARAM.findall(body):params[name].setdefault(inst.strip(),{})[key]=value.strip()
    inv=inventory(build);result={}
    for row in inv['leaves']:
        leaf=row['path'].rsplit('/',1)[-1]
        # Inventory path retains escaped names and hierarchy; module parameters use leaf names.
        values=params[row['module_type']].get(leaf,{})
        if not values:values=params[row['module_type']].get('\\'+leaf,{})
        assert values,('memory parameter resolution',row['path'])
        result[row['path']]=dict(primitive=row['primitive'],parameters=values)
    return inv,result

def hierarchy_cells(netlist):
    """Expand actual SYN hierarchy; flat physical-VO readers cannot parse this VG."""
    text=Path(netlist).read_text(encoding='utf8')
    mods={name:body for name,body in MODULE.findall(text)}
    cell_re=re.compile(r'^  (\w+)\s+(\\\S+|\w+)\s*\(\s*\n(.*?)\);',re.M|re.S)
    port_re=re.compile(r'\.([A-Za-z_][A-Za-z0-9_]*)\(([^()]*)\)')
    contents={name:[(k,n.lstrip('\\'),dict(port_re.findall(b))) for k,n,b in cell_re.findall(body)] for name,body in mods.items()}
    leaves={}
    def resolve(expr,bindings,prefix):
        found=bits(expr)
        return [bindings.get(n,(prefix+'/'+n).lstrip('/')) for n in found]
    def walk(kind,prefix,bindings):
        for child,name,ports in contents[kind]:
            path=(prefix+'/'+name).lstrip('/')
            if child in mods:
                bound={}
                for pin,expr in ports.items():
                    nets=resolve(expr,bindings,prefix)
                    if len(nets)==1:bound[pin]=nets[0]
                walk(child,path,bound)
            else:leaves[path]=(child,ports,resolve(ports.get('CLK',''),bindings,prefix))
    walk('g2_top','',{})
    return leaves

def review_syn():
    old,old_ram=memory_parameters(SOURCE);new,new_ram=memory_parameters(SYN)
    differences=[n for n in set(old_ram)|set(new_ram) if old_ram.get(n)!=new_ram.get(n)]
    cells=hierarchy_cells(SYN/'impl/gwsynthesis/g2_top.vg')
    sdc=(PNR/'g2.sdc').read_text(encoding='utf8')
    unresolved=[]
    for path in re.findall(r'\[get_pins \{([^}]+)\}\]',sdc):
        name,pin=path.rsplit('/',1)
        if name not in cells or pin not in cells[name][1]:unresolved.append(path)
    new_flops={n:c for n,c in cells.items() if c[0].startswith('DFF') and
               (n.startswith('u_control/u_ultrasonic_guard/') or n.startswith('u_sonar_led/'))}
    clocks={n:c[2] for n,c in new_flops.items()}
    wrong_clock={n:c for n,c in clocks.items() if c!=['clk50_d']}
    guard_cells={n:c for n,c in cells.items() if n.startswith('u_control/u_ultrasonic_guard/')}
    guard_luts=sum(c[0].startswith('LUT') for c in guard_cells.values())
    guard_dsps={n:c[0] for n,c in guard_cells.items() if 'MULT' in c[0]}
    review=dict(passed=not differences and not unresolved and not wrong_clock and bool(new_flops),
        old_BSRAM_source=old['expanded_bsram_count'],new_BSRAM_source=new['expanded_bsram_count'],
        predicted_BSRAM_with_wide_split=new['predicted_split_bsram'],
        all_memory_geometry_and_explicit_INIT_differences=differences,
        unresolved_original_SDC_pins=unresolved,new_guard_LED_flops=len(new_flops),wrong_new_clocks=wrong_clock,
        primitive_resources=new['primitive_types'],
        guard_LUT_count=guard_luts,guard_DSP_cells=guard_dsps,
        firmware_and_model_assets='byte-preserved source snapshot; review original full RAM INIT parameters',
        scope='SYN netlist admission, not physical timing or hardware acceptance')
    review['passed'] &= new['predicted_split_bsram']<=336
    if COMPACT_PROFILE:
        review['compact_mapping_target_met']=guard_luts<1000 and not guard_dsps
        review['passed'] &= review['compact_mapping_target_met']
    write(OUT/'pre_pnr_review.json',review)
    if not review['passed']:raise RuntimeError('Actual SYN review blocked PNR; read pre_pnr_review.json')
    return review

def final_review():
    folder=PNR/'impl/pnr'
    log='\n'.join(p.read_text(encoding='utf8',errors='replace') for p in (PNR/'gw_sh.log',folder/'g2_top.log') if p.exists())
    unrouted=re.findall(r'(\d+)\s+(?:nets?\s+)?(?:remain\s+)?unrouted|unrouted[^\r\n]*?(\d+)',log,re.I)
    rpt=(folder/'g2_top.rpt.txt').read_text(encoding='utf8',errors='replace') if (folder/'g2_top.rpt.txt').exists() else ''
    ram=re.search(r'^\s*BSRAM\s*\|\s*(\d+)/(\d+)',rpt,re.M)
    result=dict(BSRAM=int(ram[1]) if ram else None,unrouted_log_matches=unrouted,
                PR0004='PR0004' in log,setup=None,hold=None,deployment_accepted=False)
    runtime=json.loads((PNR/'tool_runtime.json').read_text())
    result.update(tool_exit=runtime.get('tool_exit'),cancelled=runtime.get('cancelled'),
                  bitstream_exists=(folder/'g2_top.fs').is_file())
    tr=folder/'g2_top.tr'
    if tr.exists():
        head=tr.read_text(encoding='utf8',errors='replace')[:65536]
        summary=head.split('Note:Core Timing Report',1)[-1].split('\n3. Timing Details')[0]
        for key,label in [('setup','Setup'),('hold','Hold')]:
            m=re.search(r'<Numbers of '+label+r' Violated Endpoints>:\s*(\d+)',summary)
            result[key]=int(m[1]) if m else None
        result['frequencies']=[dict(clock=n,target_MHz=float(c),Fmax_MHz=float(v)) for n,c,v in
            re.findall(r'^\s*\d+\s+(\S+)\s+([\d.]+)\(MHz\)\s+([\d.]+)\(MHz\)',summary,re.M)]
        result['domain_TNS']=[dict(clock=n,type=t,tns_ns=float(v),violations=int(e)) for n,t,v,e in
            re.findall(r'^\s*(\S+)\s+(setup|hold|recovery|removal)\s+(-?[\d.]+)\s+(\d+)\s*$',summary,re.M)]
    result['report_timing_gate']=(result['tool_exit']==0 and not result['cancelled'] and result['bitstream_exists'] and
        not result['PR0004'] and result['setup']==0 and result['hold']==0 and
        result['BSRAM'] is not None and result['BSRAM']<=336 and len(result.get('frequencies',[]))==4 and
        {r['clock'] for r in result.get('frequencies',[])}=={'clk50','rgmii_rxc','camera_pclk','lcd9'} and
        len(result.get('domain_TNS',[]))>=8 and
        all(r['Fmax_MHz']>=r['target_MHz'] for r in result.get('frequencies',[])) and
        all(r['tns_ns']==0 and r['violations']==0 for r in result.get('domain_TNS',[])))
    write(OUT/'final_review.json',result);return result

def main():
    global SYN,PNR,OUT,COMPACT_PROFILE
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--launch',action='store_true');p.add_argument('--start',action='store_true')
    p.add_argument('--launch-review-continuation',action='store_true')
    p.add_argument('--review-continuation',action='store_true')
    p.add_argument('--profile',choices=['original','compact','minimal'],default='original');a=p.parse_args()
    if a.profile=='compact':
        COMPACT_PROFILE=True
        SYN=BASE/'K_ultrasonic_compact_syn2';PNR=BASE/'K_pnr87_ultrasonic_compactp2r0';OUT=BASE/'ultrasonic_compact_joint_one_shot'
    if a.profile=='minimal':
        COMPACT_PROFILE=True
        SYN=BASE/'K_ultrasonic_minimal_syn3';PNR=BASE/'K_pnr88_ultrasonic_minimalp2r0';OUT=BASE/'ultrasonic_minimal_joint_one_shot'
    if a.launch_review_continuation:
        assert (OUT/'job.json').exists() and not (OUT/'review_continuation_launch.json').exists()
        with (OUT/'review_continuation.log').open('w') as log:
            child=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'--review-continuation','--profile',a.profile],
                stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW|subprocess.CREATE_NEW_PROCESS_GROUP)
        data=dict(pid=child.pid,created_epoch=psutil.Process(child.pid).create_time(),
                  scope='wait existing SYN supervisor; only repair its rejected flat-netlist reader; never repeat SYN')
        write(OUT/'review_continuation_launch.json',data);print(json.dumps(data));return
    if a.review_continuation:
        while True:
            prior=json.loads((OUT/'job.json').read_text())
            if prior['phase'] in ('routing','PNR_finished_review_pending'):
                print('Existing continuation already reached PNR; no duplicate action');return
            if prior['phase']=='failed_preserved':
                if 'Actual SYN review blocked PNR' not in prior.get('error',''):
                    raise RuntimeError('Existing SYN failed for another reason; preserved without retry')
                break
            time.sleep(5)
        # Original controller has completed its SYN and rejected only its old flat reader.
        while True:
            try:
                owner=psutil.Process(prior['controller_pid'])
                if abs(owner.create_time()-prior['controller_created_epoch'])>.01:break
                if not owner.is_running():break
            except psutil.NoSuchProcess:break
            time.sleep(1)
        assert not (PNR/'impl').exists(),'A PNR already exists; do not repeat it'
        shutil.copyfile(OUT/'job.json',OUT/'initial_review_supervisor.json')
        shutil.copyfile(OUT/'pre_pnr_review.json',OUT/'first_flat_reader_rejection.json')
        state=dict(controller_pid=os.getpid(),controller_created_epoch=psutil.Process().create_time(),
            phase='reviewing_actual_SYN',SYN=str(SYN),PNR=str(PNR),programmed=False,
            PNR_total_limit_s=0,PNR_silent_limit_s=0,review_fix='expand actual SYN hierarchy instead of reading it as a flat physical VO')
        def record(phase,**kw):
            state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw);write(OUT/'job.json',state)
        try:
            record('reviewing_actual_SYN');review=review_syn();record('SYN_review_passed',review=review)
            shutil.copyfile(SYN/'impl/gwsynthesis/g2_top.vg',PNR/'g2_top.vg')
            while True:
                m=memory();record('waiting_memory_PNR',**m)
                if m['available_physical_bytes']>=2*GIB and m['available_commit_bytes']>=4*GIB:break
                time.sleep(10)
            record('routing',command=[str(GW),'run.tcl'],runtime_file=str(PNR/'tool_runtime.json'))
            runtime=run_tool([str(GW),'run.tcl'],PNR,max_runtime_s=0,max_no_progress_s=0,capture_system_telemetry=True)
            record('PNR_finished_review_pending',runtime=runtime,final=final_review())
        except Exception as exc:record('failed_preserved',error=repr(exc));raise
        return
    if a.launch:
        OUT.mkdir(exist_ok=False)
        with (OUT/'controller.log').open('w') as log:
            child=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'--start','--profile',a.profile],stdout=log,stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW|subprocess.CREATE_NEW_PROCESS_GROUP)
        launched=dict(controller_pid=child.pid,controller_created_epoch=psutil.Process(child.pid).create_time(),
                      output=str(OUT),syn=str(SYN),pnr=str(PNR),programmed=False)
        write(OUT/'launch.json',launched);print(json.dumps(launched,indent=2));return
    assert a.start and OUT.exists() and not SYN.exists() and not PNR.exists()
    state=dict(controller_pid=os.getpid(),controller_created_epoch=psutil.Process().create_time(),
        SYN=str(SYN),PNR=str(PNR),source=str(SOURCE),programmed=False,PNR_total_limit_s=0,PNR_silent_limit_s=0)
    def save(phase,**kw):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw);write(OUT/'job.json',state)
    def admit(stage):
        while True:
            m=memory();save('waiting_memory_'+stage,**m)
            if m['available_physical_bytes']>=2*GIB and m['available_commit_bytes']>=4*GIB:return m
            time.sleep(10)
    try:
        save('freezing_inputs')
        shutil.copytree(SOURCE,SYN,ignore=shutil.ignore_patterns('impl','sim','isolated_read_decode_probe','*.log','*.err','__pycache__'))
        overlay=OUT/'overlay';prepare(SOURCE,overlay)
        for name in ('control4500_core.v','g2_top.v','ultrasonic_drive_guard.v','ultrasonic_mode_led.v'):
            shutil.copyfile(overlay/name,SYN/name)
        shutil.copyfile(overlay/'g2_range_servo.v',SYN/'video/g2_range_servo.v')
        PNR.mkdir()
        for name in ('g2.cst','g2.sdc'):
            shutil.copyfile(ANCHOR/name,PNR/name);shutil.copyfile(ANCHOR/name,SYN/name)
        tcl=(SOURCE/'run.tcl').read_text().replace(SOURCE.as_posix(),SYN.as_posix())
        tcl=tcl.replace('run syn','add_file -type verilog {'+str(SYN.as_posix())+'/ultrasonic_drive_guard.v}\n'+
            'add_file -type verilog {'+str(SYN.as_posix())+'/ultrasonic_mode_led.v}\nrun syn')
        (SYN/'run.tcl').write_text(tcl,encoding='utf8')
        (PNR/'run.tcl').write_text((ANCHOR/'run.tcl').read_text().replace(ANCHOR.as_posix(),PNR.as_posix()),encoding='utf8')
        write(OUT/'input_manifest.json',dict(source=str(SOURCE),SYN=str(SYN),PNR=str(PNR),
            changed_RTL=['g2_top.v','control4500_core.v','video/g2_range_servo.v'],
            added_RTL=['ultrasonic_drive_guard.v','ultrasonic_mode_led.v'],
            unchanged='all other hardware, full firmware, weights and assets; PNR85 CST/SDC and PNR recipe',
            remote_forward_q12=1229,SG90_scan=False,slow_mm=600,stop_mm=250,parking_hold=True,
            profile=a.profile,compact_mapping_target='guard DSP0 and LUT<1000' if COMPACT_PROFILE else None,
            added_fault_timeouts=False if a.profile=='minimal' else True,
            SYN_runtime_limit_seconds=2400,PNR_total_limit_seconds=0,PNR_silent_limit_seconds=0))
        admit('SYN')
        with (SYN/'gw_sh.log').open('w') as log:
            proc=subprocess.Popen([str(GW),'run.tcl'],cwd=SYN,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
            start=time.monotonic();low=0
            save('synthesizing',tool_pid=proc.pid,tool_created_epoch=psutil.Process(proc.pid).create_time(),command=[str(GW),'run.tcl'])
            while proc.poll() is None:
                m=memory();danger=m['available_commit_bytes']<512*1024**2 or (m['available_physical_bytes']<256*1024**2 and m['available_commit_bytes']<GIB)
                low=low+1 if danger else 0
                if low>=2 or time.monotonic()-start>=2400:
                    for q in reversed(psutil.Process(proc.pid).children(recursive=True)):
                        try:q.kill()
                        except psutil.NoSuchProcess:pass
                    proc.kill();proc.wait();raise RuntimeError('Only owned SYN stopped by original40min/memory protection')
                save('synthesizing',elapsed_s=time.monotonic()-start,**m);time.sleep(3)
        assert proc.returncode==0,('SYN failed',proc.returncode)
        save('reviewing_actual_SYN');review=review_syn();save('SYN_review_passed',review=review)
        shutil.copyfile(SYN/'impl/gwsynthesis/g2_top.vg',PNR/'g2_top.vg')
        admit('PNR');save('routing',command=[str(GW),'run.tcl'],PNR_total_limit_s=0,PNR_silent_limit_s=0)
        runtime=run_tool([str(GW),'run.tcl'],PNR,max_runtime_s=0,max_no_progress_s=0,capture_system_telemetry=True)
        final=final_review();save('PNR_finished_review_pending',runtime=runtime,final=final)
    except Exception as exc:
        save('failed_preserved',error=repr(exc));raise

if __name__=='__main__':main()
