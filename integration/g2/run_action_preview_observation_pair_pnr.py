"""PNR40: one paired-observation global-role contrast on failed PNR39 VG."""
from datetime import datetime
import json
import os
from pathlib import Path
import time
import psutil
import run_action_preview_pnr as origin

BASE=origin.BASE
PHYSICAL=BASE/'baseline_fusion_action_preview_pnr39'
OUTPUT=BASE/'baseline_fusion_action_preview_obs_pair_pnr40'
CONTROL=BASE/'baseline_fusion_action_preview_obs_pair_one_shot'
PROOF=BASE/'pnr39_final_actual_review1/observation_pair_request_preflight.json'
origin.OUTPUT=OUTPUT
load=origin.load
same=origin.same
meta=origin.meta
save=origin.save

def bound_request():
    proof=load(PROOF);assert proof['passed']and proof['no_numeric_aliases']
    assert len(proof['actual_four_cone_checks'])==4 and all(x['passed']for x in proof['actual_four_cone_checks'])
    assert len(proof['actual_FF_boundary_cells'])==55 and proof['all_boundary_D_CLK_CE_reset_INIT_byte_identical']
    for row in proof['evidence_bindings']:assert meta(Path(row['path']))==row
    gate=load(PHYSICAL/'g2_gate.json');rt=load(PHYSICAL/'tool_runtime.json')
    assert gate['unrouted_nets']==293 and rt['tool_exit']==1 and not rt['cancelled']
    assert same(PHYSICAL/'g2_top.vg',origin.SOURCE/'impl/gwsynthesis/g2_top.vg')
    assert same(PHYSICAL/'g2.cst',origin.REFERENCE/'g2.cst')
    requests=proof['requested_CST'];assert len(requests)==4
    cst=(PHYSICAL/'g2.cst').read_bytes()+b'\n// Actual PNR39 paired observation global-role contrast.\n'+('\n'.join(requests)+'\n').encode()
    return proof,cst

def prepare():
    proof,cst=bound_request();origin.prepare()
    (OUTPUT/'g2.cst').write_bytes(cst)
    old=load(OUTPUT/'prepared_request_review.json');save(OUTPUT/'prepared_before_global_request.json',old)
    old.update(paired_observation_request_bound=True,paired_request_proof=str(PROOF),actual_requested_CST=proof['requested_CST'],same_completed_SYN28_VG_and_SDC_as_failed39=True)
    old['bindings']+=[meta(PROOF),meta(PHYSICAL/'g2_top.vg'),meta(PHYSICAL/'g2.cst'),meta(PHYSICAL/'g2.sdc'),meta(PHYSICAL/'g2_gate.json')]
    old['prepared_bindings']=[meta(OUTPUT/n)for n in('g2_top.vg','g2.cst','g2.sdc','run.tcl')]
    save(OUTPUT/'prepared_request_review.json',old)
    manifest=load(OUTPUT/'manifest.json');manifest.update(reference_physical=str(PHYSICAL),only_additional_CST=proof['requested_CST'],paired_global_request_preflight=str(PROOF),no_new_synthesis=True,global_requests=4,original_observation_LW_roles_restored_not_yet_verified=True)
    save(OUTPUT/'manifest.json',manifest);validate()

def validate():
    _,sdc,tcl=origin.source_check();proof,cst=bound_request();ready=load(OUTPUT/'prepared_request_review.json')
    assert ready['passed']and ready['paired_observation_request_bound']
    for row in ready['bindings']+ready['prepared_bindings']:assert meta(Path(row['path']))==row
    assert same(OUTPUT/'g2_top.vg',PHYSICAL/'g2_top.vg')and same(OUTPUT/'g2.sdc',PHYSICAL/'g2.sdc')
    assert (OUTPUT/'g2.sdc').read_bytes()==sdc and(OUTPUT/'run.tcl').read_bytes()==tcl
    assert (OUTPUT/'g2.cst').read_bytes()==cst
    assert not(OUTPUT/'impl').exists()and not(OUTPUT/'tool_runtime.json').exists(),'refuse repeated physical input'

def main():
    prepare();CONTROL.mkdir(exist_ok=False);me=psutil.Process();me.nice(psutil.NORMAL_PRIORITY_CLASS);me.cpu_affinity(list(range(psutil.cpu_count())))
    state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),source=str(origin.SOURCE),output=str(OUTPUT),reference=str(PHYSICAL),total_limit_s=0,silent_limit_s=0,programmed=False,deployment_accepted=False)
    with(CONTROL/'launch_claim.json').open('x')as file:json.dump(state,file)
    def status(phase,**data):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**data);save(CONTROL/'job.json',state)
    def wait_memory():
        while True:
            m=origin.memory_status();status('waiting_for_memory_admission',free_bytes=m.avail_phys,available_commit_bytes=m.avail_page,minimum_physical_bytes=2*origin.GIB,minimum_commit_bytes=4*origin.GIB)
            if m.avail_phys>=2*origin.GIB and m.avail_page>=4*origin.GIB:return
            time.sleep(13)
    try:
        wait_memory();validate();wait_memory();status('starting_unique_PNR40')
        runtime=origin.run_tool([str(origin.engine.TOOL),'run.tcl'],OUTPUT,max_runtime_s=0,max_no_progress_s=0,capture_system_telemetry=True)
        status('tool_exited_collecting_actual_result',runtime=runtime);wait_memory();origin.finalize_gate(OUTPUT);gate=load(OUTPUT/'g2_gate.json')
        normal=lambda p,r:p.read_text().replace(str(r),'<BUILD>').replace(r.as_posix(),'<BUILD>').split()
        options=normal(OUTPUT/'impl/pnr/cmd.do',OUTPUT)==normal(PHYSICAL/'impl/pnr/cmd.do',PHYSICAL)
        report=(OUTPUT/'impl/pnr/g2_top.rpt.txt').read_text();rows={}
        for line in report.split('6. Global Clock Signals',1)[1].split('7. Pinout',1)[0].splitlines():
            if '| PRIMARY'in line or '| LW 'in line:
                name,kind,location=[s.strip()for s in line.split('|')];rows[name]=dict(resource=kind,location=location)
        ce='u_control/model_thread/observation/n15679_10';reset='u_control/model_thread/observation/n331_7'
        targets=[ce,reset,'u_nav/u_pf/n3967_6','u_nav/u_map/u_tv/n25284_7']
        effect=all(rows.get(n,{}).get('resource')=='LW'for n in(ce,reset))and all(n not in rows for n in targets[2:])
        tr=OUTPUT/'impl/pnr/g2_top.tr'
        if tr.exists():save(OUTPUT/'actual_setup_endpoint_coverage.json',origin.scan_report(tr))
        status('routed_requires_actual_timing_and_full_signoff_review'if gate['routing_passed']else'physical_failure_preserved_no_retry',runtime=runtime,routing_passed=gate['routing_passed'],unrouted=gate['unrouted_nets'],actual_options_same_as39=options,actual_requested_globals={n:rows.get(n)for n in targets},both_original_observation_LW_roles_restored=effect,TR_available=tr.exists(),gate=str(OUTPUT/'g2_gate.json'))
        return int(runtime['tool_exit']!=0 or not gate['routing_passed']or not options)
    except Exception as error:
        import traceback
        status('failed_preserved_no_automatic_retry',error=str(error),traceback=traceback.format_exc());return 1

if __name__=='__main__':raise SystemExit(main())
