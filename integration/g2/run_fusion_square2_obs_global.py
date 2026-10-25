"""Prepare one evidenced global-routing request contrast; start only by --start.

Reuses complete SYN26 and PNR36 input bytes/options. The only routing change
requests global routing for the former4001-load observation LW control and
ordinary routing for a newly promoted357-load control. GenericBUFG does not
guaranteeLW. An extra timing-report request changes no20/10ns requirement.
"""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import re
import shutil
import time

import psutil

from bounded_tool import run_tool
from collect_pnr_result import finalize_gate
from retry_frozen_synthesis import memory_status
from run_baseline_single_factor_pnr import equal_bytes,metadata,physical_connections

ROOT=Path('F:/nav_integration_g2_20261023')
REFERENCE=ROOT/'baseline_fusion_square2_pnr36'
SOURCE=ROOT/'baseline_fusion_square2_syn26'
OUTPUT=ROOT/'baseline_fusion_square2_obs_global_pnr38'
CONTROL=ROOT/'baseline_fusion_square2_obs_global_one_shot'
AUDIT=ROOT/'pnr36_vs_pnr32_final_review1/observation_global_request_preflight.json'
AUDIT_BINDING=ROOT/'pnr36_vs_pnr32_final_review1/observation_global_request_proof_binding.json'
SYN_REVIEW=ROOT/'baseline_fusion_square2_one_shot/actual_SYN26_netlist_review.json'
TOOL=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
OBS='u_control/model_thread/observation/n15679_10'
LOCAL='u_nav/u_map/u_tv/n25284_7'
CST_COMMANDS=[f'CLOCK_LOC "{OBS}" BUFG = CE|LOGIC;',f'CLOCK_LOC "{LOCAL}" LOCAL_CLOCK;']
REPORT='report_timing -setup -max_paths 5000 -max_common_paths 1'
PROFILE=dict(output=OUTPUT,interval=13)
GIB=1024**3

def load(path):return json.loads(Path(path).read_text(encoding='utf-8-sig'))

def save(path,value):
    path=Path(path);temporary=path.with_name(path.name+'.'+str(os.getpid())+'.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');temporary.replace(path)

def source_check():
    proof=load(AUDIT);actual=load(SYN_REVIEW);job=load(SOURCE/'job.json');binding=load(AUDIT_BINDING)
    assert binding['passed']and metadata(AUDIT)==binding['proof']
    for row in binding['actual_VO_bindings']+[binding['evidence']]:assert metadata(Path(row['path']))==row
    assert proof['passed']and proof['no_guessed_numeric_aliases']
    assert proof['requested_CST']==CST_COMMANDS
    assert len(proof['driver_input_cone_proofs'])==2 and all(p['passed']for p in proof['driver_input_cone_proofs'])
    assert len(proof['state_boundary_exact_checks'])==5
    assert proof['observation_loads']['loads']==4001 and proof['local_loads']['loads']==357
    assert job['phase']=='synthesis_complete'and job['tool_exit']==0
    assert actual['passed']and actual['actual_RAM_INIT_clock_26CDC_preflight_passed']
    assert Path(actual['source']).resolve()==SOURCE.resolve()
    assert [actual[k]for k in ('source_BSRAM','wide_leaves','CPU_SP_count','true_clocks','known_CDC_paths')]==[314,18,64,2,26]
    for row in actual['reviewed_input_metadata']:assert metadata(Path(row['path']))==row
    assert equal_bytes(REFERENCE/'g2_top.vg',SOURCE/'impl/gwsynthesis/g2_top.vg')
    assert equal_bytes(REFERENCE/'g2_top.vg',Path(actual['reviewed_netlist_copy']))
    assert equal_bytes(REFERENCE/'g2.cst',Path(actual['reviewed_CST_copy']))and equal_bytes(REFERENCE/'g2.sdc',Path(actual['reviewed_SDC_copy']))
    assert 'CLOCK_LOC'not in (REFERENCE/'g2.cst').read_text()
    assert (REFERENCE/'g2.sdc').read_text().splitlines()==[
        'create_clock -name clk50 -period 20 [get_ports {clk50}]',
        'create_clock -name rgmii_rxc -period 10 [get_ports {rgmii_rxc}]']
    runtime=load(REFERENCE/'tool_runtime.json');gate=load(REFERENCE/'g2_gate.json')
    assert runtime['tool_exit']==1 and not runtime['cancelled']and gate['unrouted_nets']==1256
    try:
        process=psutil.Process(runtime['pid'])
        # The exact old creation time is independently recorded by the start
        # review; a reused PID belongs to a different process and is harmless.
        expected=load(ROOT/'baseline_fusion_square2_one_shot/actual_PNR36_start_review.json')['actual_tool']['created_epoch']
        assert abs(process.create_time()-expected)>.01,'PNR36 has not exited'
    except psutil.NoSuchProcess:pass
    return proof,actual

def texts():
    cst=(REFERENCE/'g2.cst').read_bytes()+('\n// Evidence-bound global request contrast; actual resource type must be reported.\n'+'\n'.join(CST_COMMANDS)+'\n').encode()
    sdc=(REFERENCE/'g2.sdc').read_bytes()+('\n# Additional report request only; clock constraints unchanged.\n'+REPORT+'\n').encode()
    original=(REFERENCE/'run.tcl').read_bytes()
    assert b'set_option -route_option 0'in original
    tcl=original.replace(REFERENCE.as_posix().encode(),OUTPUT.as_posix().encode()).replace(str(REFERENCE).encode(),str(OUTPUT).encode())
    assert tcl.replace(OUTPUT.as_posix().encode(),REFERENCE.as_posix().encode()).replace(str(OUTPUT).encode(),str(REFERENCE).encode())==original
    return cst,sdc,tcl

def prepare():
    proof,actual=source_check();assert not OUTPUT.exists(),'preserve existing directory, no automatic repeat'
    OUTPUT.mkdir();cst,sdc,tcl=texts()
    shutil.copyfile(REFERENCE/'g2_top.vg',OUTPUT/'g2_top.vg')
    for name,data in [('g2.cst',cst),('g2.sdc',sdc),('run.tcl',tcl)]:(OUTPUT/name).write_bytes(data)
    shutil.copyfile(AUDIT,OUTPUT/'actual_driver_cone_preflight.json')
    shutil.copyfile(AUDIT_BINDING,OUTPUT/'actual_driver_cone_input_binding.json')
    shutil.copyfile(SYN_REVIEW,OUTPUT/'actual_SYN_preflight_review.json')
    paths=actual['paths'];assert len(paths)==26 and all(p['passed']for p in paths)
    save(OUTPUT/'pure_CDC_connection_reference.json',dict(source_review=str(SYN_REVIEW),synthesis_connection_preflight_passed=True,paths=paths,added_timing_exceptions=0))
    manifest=dict(source=str(SOURCE),source_manifest=load(SOURCE/'manifest.json'),frozen_netlist=True,reused_completed_synthesis=True,
        reference=str(REFERENCE),actual_SYN_review=str(SYN_REVIEW),global_request_preflight=str(AUDIT),
        source_netlist_bytes_unchanged=True,only_CST_routing_changes=CST_COMMANDS,SDC_additional_report_only=REPORT,
        expected_observation_resource='generic_global_request_not_guaranteed_LW',LOCAL_control_loads=357,observation_loads=4001,
        clock_periods_ns=dict(clk50=20,rgmii_rxc=10),added_timing_exceptions=0,route_option=0,
        max_runtime_minutes=0,max_no_progress_minutes=0,diagnostic_only=True,programmed=False,deployment_accepted=False)
    save(OUTPUT/'manifest.json',manifest)
    source_files=[REFERENCE/n for n in ('g2_top.vg','g2.cst','g2.sdc','run.tcl','impl/pnr/cmd.do','impl/pnr/device.cfg')]+[AUDIT,AUDIT_BINDING,SYN_REVIEW]
    result=dict(passed=True,preparation_only=True,recorded_at=datetime.now().astimezone().isoformat(),output=str(OUTPUT),reference=str(REFERENCE),
        source_review_passed=True,driver_cones_exact=True,one_source_global_assignment_intervention=True,
        source_VG_bytes_unchanged=True,clock_periods_unchanged=True,SDC_only_adds_report_not_exception=True,
        CST_only_adds_two_commands=CST_COMMANDS,generic_BUFG_not_guaranteed_LW=True,actual_role_restoration_not_yet_verified=True,
        new_tool_or_build_started=False,bindings=[metadata(p)for p in source_files],prepared_bindings=[metadata(OUTPUT/n)for n in ('g2_top.vg','g2.cst','g2.sdc','run.tcl')])
    save(OUTPUT/'prepared_request_review.json',result);validate();return result

def validate():
    source_check();proof=load(OUTPUT/'prepared_request_review.json');assert proof['passed']
    for row in proof['bindings']+proof['prepared_bindings']:assert metadata(Path(row['path']))==row
    assert equal_bytes(REFERENCE/'g2_top.vg',OUTPUT/'g2_top.vg')
    assert load(OUTPUT/'actual_driver_cone_preflight.json')==load(AUDIT)
    assert load(OUTPUT/'actual_driver_cone_input_binding.json')==load(AUDIT_BINDING)
    assert load(OUTPUT/'actual_SYN_preflight_review.json')==load(SYN_REVIEW)
    for name,data in zip(('g2.cst','g2.sdc','run.tcl'),texts()):assert (OUTPUT/name).read_bytes()==data
    assert not (OUTPUT/'tool_runtime.json').exists()and not (OUTPUT/'impl').exists(),'physical attempt already exists; do not repeat'

def run():
    validate();CONTROL.mkdir(exist_ok=False);me=psutil.Process();me.nice(psutil.NORMAL_PRIORITY_CLASS);me.cpu_affinity(list(range(psutil.cpu_count())))
    state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),output=str(OUTPUT),reference=str(REFERENCE),
        total_limit_s=0,silent_limit_s=0,priority=int(me.nice()),affinity=me.cpu_affinity(),programmed=False,deployment_accepted=False)
    with (CONTROL/'launch_claim.json').open('x')as file:json.dump(state,file,indent=2)
    def status(phase,**extra):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**extra);save(CONTROL/'job.json',state)
    def memory_gate(phase):
        while True:
            memory=memory_status();status(phase,free_bytes=memory.avail_phys,available_commit_bytes=memory.avail_page,minimum_physical_bytes=2*GIB,minimum_commit_bytes=4*GIB)
            if memory.avail_phys>=2*GIB and memory.avail_page>=4*GIB:return
            time.sleep(13)
    try:
        memory_gate('waiting_for_memory_admission');validate();memory_gate('rechecking_memory_immediately_before_launch')
        status('starting_unique_PNR38')
        runtime=run_tool([str(TOOL),'run.tcl'],OUTPUT,max_runtime_s=0,max_no_progress_s=0,capture_system_telemetry=True)
        status('tool_exited_collecting_actual_result',runtime=runtime)
        normalize=lambda path,root:path.read_text().replace(str(root),'<BUILD>').replace(root.as_posix(),'<BUILD>').split()
        same_options=(OUTPUT/'impl/pnr/cmd.do').exists()and normalize(OUTPUT/'impl/pnr/cmd.do',OUTPUT)==normalize(REFERENCE/'impl/pnr/cmd.do',REFERENCE)
        same_device=(OUTPUT/'impl/pnr/device.cfg').exists()and equal_bytes(OUTPUT/'impl/pnr/device.cfg',REFERENCE/'impl/pnr/device.cfg')
        finalize_gate(OUTPUT);gate=load(OUTPUT/'g2_gate.json')
        cdc=physical_connections(PROFILE,status)
        report=(OUTPUT/'impl/pnr/g2_top.rpt.txt').read_text();rows={}
        for line in report.split('6. Global Clock Signals',1)[1].split('7. Pinout',1)[0].splitlines():
            if '| PRIMARY'in line or'| LW 'in line:
                name,kind,location=[x.strip()for x in line.split('|')];rows[name]=dict(resource=kind,location=location)
        effect=OBS in rows and LOCAL not in rows
        requested_LW=rows.get(OBS,{}).get('resource')=='LW'and LOCAL not in rows
        tr=OUTPUT/'impl/pnr/g2_top.tr';reported5000=tr.exists()and REPORT in tr.read_text()
        status('completed_requires_actual_routing_and_timing_review'if gate['routing_passed']else'physical_failure_preserved_no_retry',
            runtime=runtime,routing_passed=gate['routing_passed'],actual_options_equal_to_PNR36=same_options,actual_device_cfg_equal=same_device,
            physical26_connections_passed=cdc,actual_global_request_effective=effect,original_observation_LW_role_restored=requested_LW,
            actual_requested_globals={OBS:rows.get(OBS),LOCAL:rows.get(LOCAL)},report5000_request_verified=reported5000,actual_timing_report_available=tr.exists(),unrouted=gate['unrouted_nets'])
        return int(runtime['cancelled']or runtime['tool_exit']!=0 or not gate['routing_passed']or not same_options or not same_device or not cdc or not effect)
    except Exception as error:
        import traceback
        status('failed_preserved_without_retry',error=str(error),traceback=traceback.format_exc());return 1

def main():
    parser=argparse.ArgumentParser(description=__doc__);group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--prepare-only',action='store_true');group.add_argument('--start',action='store_true')
    args=parser.parse_args()
    if args.prepare_only:
        result=prepare();print(json.dumps({k:result[k]for k in ('passed','preparation_only','output','CST_only_adds_two_commands','new_tool_or_build_started')}));return 0
    return run()

if __name__=='__main__':raise SystemExit(main())
