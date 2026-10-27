"""One independent second-place run of the timing-clean SYN49/PNR61 netlist.

Only place_option2->1 differs. No SYN, new timing exceptions, SRAM or Flash.
Uses the existing bounded_tool supervisor with total/silent limits both zero.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import sys
import time

import psutil
from bounded_tool import run_tool
from retry_frozen_synthesis import memory_status

BASE=Path('F:/nav_integration_g2_20261023')
REF=BASE/'K_pnr61_s49p2r0'
OUT=BASE/'K_pnr62_s49p1r0'
CONTROL=BASE/'PNR61_second_place_one_shot'
GW='D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe'


def write(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf8')


def prepare():
    assert not OUT.exists() and not CONTROL.exists(),'An existing second-place run must be read, not duplicated'
    summary=json.loads((BASE/'PNR61_independent_signoff_20261027/summary.json').read_text())
    assert summary['memories']['passed'] and summary['memories']['distributed_memories']['passed']
    assert summary['CDC26_structure_passed'] and summary['clock_inventory']['passed']
    assert summary['timing_summary']['all_target_clock_setup_hold_report_passed']
    network=json.loads((BASE/'PNR61_MAC_UDP_XT_pipeline_probe2/result.json').read_text());assert network['passed']
    OUT.mkdir();CONTROL.mkdir()
    for name in ('g2_top.vg','g2.cst','g2.sdc'):shutil.copyfile(REF/name,OUT/name)
    script=(REF/'run.tcl').read_text().replace(REF.as_posix(),OUT.as_posix())
    assert script.count('set_option -place_option 2')==1
    script=script.replace('set_option -place_option 2','set_option -place_option 1')
    (OUT/'run.tcl').write_text(script,encoding='utf8')
    assert script.replace(OUT.as_posix(),REF.as_posix()).replace('set_option -place_option 1','set_option -place_option 2')==(REF/'run.tcl').read_text()
    assert all((REF/n).read_bytes()==(OUT/n).read_bytes() for n in ('g2_top.vg','g2.cst','g2.sdc'))
    state=dict(prepared_at=datetime.now().astimezone().isoformat(),phase='prepared',source=str(REF),output=str(OUT),
        sole_factor='place_option2->1; route0, VG/CST/SDC/device/options otherwise unchanged',
        total_limit_s=0,silent_limit_s=0,SYN_started=False,programmed=False,Flash_written=False,
        open_external_IO_CDC_reset_and_vehicle_gates_preserved=True)
    write(CONTROL/'job.json',state)
    print(json.dumps(state))


def run():
    state=json.loads((CONTROL/'job.json').read_text());assert state['phase']=='prepared'
    me=psutil.Process();me.nice(psutil.NORMAL_PRIORITY_CLASS)
    state.update(controller_pid=me.pid,controller_created_epoch=me.create_time())
    def update(phase,**kw):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw)
        write(CONTROL/'job.json',state)
    update('waiting_for_actual_memory_admission')
    while True:
        m=memory_status()
        update('waiting_for_actual_memory_admission',available_physical_bytes=m.avail_phys,available_commit_bytes=m.avail_page)
        if m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3:break
        time.sleep(15)
    assert all((REF/n).read_bytes()==(OUT/n).read_bytes() for n in ('g2_top.vg','g2.cst','g2.sdc'))
    update('PNR_running',actual_start_requested_at=datetime.now().astimezone().isoformat(),admission_physical=m.avail_phys,admission_commit=m.avail_page)
    r=run_tool([GW,str(OUT/'run.tcl')],OUT,max_runtime_s=0,max_no_progress_s=0,capture_system_telemetry=True)
    log=(OUT/'gw_sh.log').read_text(errors='replace')
    tr=OUT/'impl/pnr/g2_top.tr'
    result=dict(completed_at=datetime.now().astimezone().isoformat(),runtime=r,
        routing_completed='Placement and routing completed' in log,
        unrouted_numeric=[int(x) for x in re.findall(r'(\d+) unrouted nets',log)],
        timing_report_present=tr.exists(),complete_signoff=False,programmed=False)
    if tr.exists():
        with tr.open(encoding='utf8') as f:head=''.join(next(f) for _ in range(85))
        result['setup_hold']={k:int(re.search(r'<Numbers of '+k+r' Violated Endpoints>:(\d+)',head)[1]) for k in ('Setup','Hold')}
    cmd=OUT/'impl/pnr/cmd.do';dev=OUT/'impl/pnr/device.cfg'
    if cmd.exists():
        actual=cmd.read_text().replace(str(OUT),str(REF)).replace('-place_option 1','-place_option 2')
        result['actual_cmd_only_expected_path_place_change']=actual==(REF/'impl/pnr/cmd.do').read_text()
    if dev.exists():result['actual_device_cfg_equal']=dev.read_bytes()==(REF/'impl/pnr/device.cfg').read_bytes()
    write(OUT/'second_place_result.json',result)
    update('PNR_complete_requires_independent_physical_review',actual_result=str(OUT/'second_place_result.json'),tool_exit=r['tool_exit'])


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--prepare',action='store_true');a=p.parse_args()
    if a.prepare:prepare()
    else:run()
