"""One PNR39 from the completed, reviewed private-product SYN28.

Use the routed PNR35 clock/26-path CDC contract unchanged and request expanded
setup reports in that same SDC. No new synthesis, global requests or deadlines.
"""
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import time
import psutil
from audit_cdc_first_stages import sdc_fragment
from bounded_tool import run_tool
from collect_pnr_result import finalize_gate
from check_setup_endpoint_coverage import scan_report
from retry_frozen_synthesis import memory_status
import run_baseline_single_factor_pnr as engine
from review_baseline_action_preview_syn import bind

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'baseline_fusion_action_preview_syn28'
PREPARED=BASE/'baseline_fusion_action_preview_prepared1'
REVIEW=BASE/'baseline_fusion_action_preview_one_shot/actual_SYN28_review.json'
MAPPING=BASE/'baseline_fusion_action_preview_one_shot/actual_preview_product_mapping.json'
REFERENCE=BASE/'baseline_fusion32_CDC52_pnr35'
OUTPUT=BASE/'baseline_fusion_action_preview_pnr39'
CONTROL=BASE/'baseline_fusion_action_preview_pnr_one_shot'
PNR_LABEL='PNR39'
SYN_LABEL='SYN28'
CHANGE_CONCEPT='private preview product pipeline on existing publication edge'
CHANGED=sorted(['control/mem_policy.v','control/policy_action_map.v','control/rl_policy_tick_v5.v'])
REPORT='report_timing -setup -max_paths 5000 -max_common_paths 1'
GIB=1024**3
engine.BASELINE=BASE/'baseline_fusion_only_pnr32'
PROFILE=dict(source=SOURCE,prepared=PREPARED,output=OUTPUT,interval=13)
load=engine.load
meta=engine.metadata
same=engine.equal_bytes

def mapping_check(mapped):
    assert mapped['passed']and mapped['action_DSP_count']==6
    assert mapped['all_six_original_core_actions_are_actual_sources']and mapped['actual_products_absorbed_in_DSP_output_registers']
    return mapped['evidence_bindings']

def save(path,value):
    tmp=path.with_name(path.name+'.'+str(os.getpid())+'.tmp');tmp.write_text(json.dumps(value,indent=2)+'\n');tmp.replace(path)

def source_check():
    actual=engine.check_review(PROFILE,REVIEW);proof=load(PREPARED/'prepared_source_review.json')
    assert sorted(actual['changed_HW_inputs'])==CHANGED
    bind(SOURCE,PREPARED,proof)
    job=load(SOURCE/'job.json');assert job['phase']=='synthesis_complete'and job['tool_exit']==0
    assert Path(job['source']).resolve()==PREPARED.resolve()
    a,b=engine.hardware(SOURCE),engine.hardware(PREPARED);assert a.keys()==b.keys()
    assert all(same(a[n],b[n])for n in a)
    origin=BASE/'baseline_fusion_only_syn23'
    for suffixes,count in[(engine.ASSETS,69),({'.h','.vh'},31)]:
        old,new=engine.inputs(origin,suffixes),engine.inputs(SOURCE,suffixes)
        assert old.keys()==new.keys()and len(new)==count
        assert all(same(old[n],new[n])for n in old)
    mapped=load(MAPPING)
    for row in mapping_check(mapped):assert meta(Path(row['path']))==row
    final35=load(BASE/'fusion32_CDC52_final_review1/review_result.json');assert final35['passed']
    assert load(REFERENCE/'g2_gate.json')['routing_passed']
    assert same(REFERENCE/'g2.cst',engine.BASELINE/'g2.cst')
    original=(SOURCE/'g2.sdc').read_bytes()
    sdc=original+b'\n'+sdc_fragment(dict(passed=True,paths=actual['paths'])).encode()
    assert sdc==(REFERENCE/'g2.sdc').read_bytes(),'new actual26 clocks/endpoints/budgets differ from routed35'
    sdc+=b'\n# Additional setup report only.\n'+REPORT.encode()+b'\n'
    active=[s for s in sdc.decode().splitlines()if s and not s.startswith('#')]
    assert len(active)==55 and sum(s.startswith('set_false_path')and s.endswith(' -hold')for s in active)==26
    assert sum(s.startswith('set_max_delay')for s in active)==26
    run=(REFERENCE/'run.tcl').read_bytes()
    tcl=run.replace(str(REFERENCE).encode(),str(OUTPUT).encode()).replace(REFERENCE.as_posix().encode(),OUTPUT.as_posix().encode())
    assert b'set_option -route_option 0'in tcl
    assert len([s for s in tcl.decode().splitlines()if 'add_file'in s and '.sdc'in s])==1
    return actual,sdc,tcl

def prepare():
    actual,sdc,tcl=source_check();assert not OUTPUT.exists();OUTPUT.mkdir()
    shutil.copyfile(SOURCE/'impl/gwsynthesis/g2_top.vg',OUTPUT/'g2_top.vg')
    shutil.copyfile(REFERENCE/'g2.cst',OUTPUT/'g2.cst')
    (OUTPUT/'g2.sdc').write_bytes(sdc);(OUTPUT/'run.tcl').write_bytes(tcl)
    shutil.copyfile(REVIEW,OUTPUT/'actual_SYN_preflight_review.json');shutil.copyfile(MAPPING,OUTPUT/'actual_product_mapping_review.json')
    save(OUTPUT/'cdc_reference_audit.json',dict(netlist=str(OUTPUT/'g2_top.vg'),passed=True,paths=actual['paths'],expected_paths=26,errors=[],scope='Actual '+SYN_LABEL+' 78-register geometry/INIT/clock/CE/reset/unique-first-load audit; recheck actual physical'+PNR_LABEL+' and52 effective commands after PNR.'))
    save(OUTPUT/'manifest.json',dict(source=str(SOURCE),source_manifest=load(SOURCE/'manifest.json'),prepared=str(PREPARED),frozen_netlist=True,reused_completed_synthesis=True,reference=str(REFERENCE),only_RTL_change_concept=CHANGE_CONCEPT,modified_RTL=CHANGED,clock_periods_ns=dict(clk50=20,rgmii_rxc=10),hold_only_false_paths=26,explicit_max_delays=26,SDC_same_as_routed35_except_report=REPORT,global_requests=0,max_runtime_minutes=0,max_no_progress_minutes=0,diagnostic_only=True,programmed=False,deployment_accepted=False))
    save(OUTPUT/'prepared_request_review.json',dict(passed=True,source=str(SOURCE),reference=str(REFERENCE),actual_SYN_and_product_mapping_passed=True,same26_CDC_contract_as_routed35=True,report_added_to_same_single_SDC=True,new_tool_started=False,bindings=[meta(p)for p in(REVIEW,MAPPING,REFERENCE/'run.tcl',REFERENCE/'g2.cst',REFERENCE/'g2.sdc',SOURCE/'impl/gwsynthesis/g2_top.vg')],prepared_bindings=[meta(OUTPUT/n)for n in('g2_top.vg','g2.cst','g2.sdc','run.tcl')]))

def validate():
    actual,sdc,tcl=source_check();proof=load(OUTPUT/'prepared_request_review.json');assert proof['passed']
    for row in proof['bindings']+proof['prepared_bindings']:assert meta(Path(row['path']))==row
    assert same(OUTPUT/'g2_top.vg',SOURCE/'impl/gwsynthesis/g2_top.vg')and same(OUTPUT/'g2.cst',REFERENCE/'g2.cst')
    assert (OUTPUT/'g2.sdc').read_bytes()==sdc and(OUTPUT/'run.tcl').read_bytes()==tcl
    assert load(OUTPUT/'cdc_reference_audit.json')['paths']==actual['paths']
    assert not(OUTPUT/'tool_runtime.json').exists()and not(OUTPUT/'impl').exists(),'refuse repeated physical attempt'

def main():
    prepare();CONTROL.mkdir(exist_ok=False);me=psutil.Process();me.nice(psutil.NORMAL_PRIORITY_CLASS);me.cpu_affinity(list(range(psutil.cpu_count())))
    state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),source=str(SOURCE),output=str(OUTPUT),total_limit_s=0,silent_limit_s=0,programmed=False,deployment_accepted=False)
    with(CONTROL/'launch_claim.json').open('x')as file:json.dump(state,file)
    def status(phase,**fields):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**fields);save(CONTROL/'job.json',state)
    def wait_memory():
        while True:
            m=memory_status();status('waiting_for_memory_admission',free_bytes=m.avail_phys,available_commit_bytes=m.avail_page,minimum_physical_bytes=2*GIB,minimum_commit_bytes=4*GIB)
            if m.avail_phys>=2*GIB and m.avail_page>=4*GIB:return
            time.sleep(13)
    try:
        wait_memory();validate();wait_memory();status('starting_unique_'+PNR_LABEL)
        runtime=run_tool([str(engine.TOOL),'run.tcl'],OUTPUT,max_runtime_s=0,max_no_progress_s=0,capture_system_telemetry=True)
        status('tool_exited_collecting',runtime=runtime);wait_memory();finalize_gate(OUTPUT);gate=load(OUTPUT/'g2_gate.json')
        normal=lambda p,r:p.read_text().replace(str(r),'<BUILD>').replace(r.as_posix(),'<BUILD>').split()
        options=normal(OUTPUT/'impl/pnr/cmd.do',OUTPUT)==normal(REFERENCE/'impl/pnr/cmd.do',REFERENCE)
        same_device=same(OUTPUT/'impl/pnr/device.cfg',REFERENCE/'impl/pnr/device.cfg')
        tr=OUTPUT/'impl/pnr/g2_top.tr'
        if tr.exists():save(OUTPUT/'actual_setup_endpoint_coverage.json',scan_report(tr))
        status('routed_requires_actual_timing_and_full_signoff_review'if gate['routing_passed']else'physical_failure_preserved_no_retry',runtime=runtime,routing_passed=gate['routing_passed'],unrouted=gate['unrouted_nets'],actual_options_same_as_PNR35=options,actual_device_cfg_same=same_device,TR_available=tr.exists(),gate=str(OUTPUT/'g2_gate.json'))
        return int(runtime['tool_exit']!=0 or not gate['routing_passed']or not options or not same_device)
    except Exception as error:
        import traceback
        status('failed_preserved_no_automatic_retry',error=str(error),traceback=traceback.format_exc());return 1

if __name__=='__main__':raise SystemExit(main())
