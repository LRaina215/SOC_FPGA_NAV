"""Prepare/start only the explicitMAX SYN37 PNR50 after root review.

Retains all three physical clock targets and original audited radar constraints.
Preparation is read-only to tools; --start authorizes just one PNR, never programming.
"""
from datetime import datetime
import json
import argparse
import os
from pathlib import Path
import shutil
import time
import psutil
from audit_cdc_first_stages import sdc_fragment
from bounded_tool import run_tool
from check_setup_endpoint_coverage import scan_report
from collect_pnr_result import finalize_gate
from prepare_baseline_fusion_retime import metadata, same_bytes
from retry_frozen_synthesis import memory_status
from revalidate_lidar_cdc import definitions
from review_model25_joint_syn import module_bindings

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'baseline_fusion_model25_bounded_MAX_syn37'
REFERENCE=BASE/'baseline_fusion32_CDC52_pnr35'
REVIEW=BASE/'baseline_fusion_model25_bounded_MAX_actual_review_one_shot'
MAPPING=REVIEW/'actual_mapping_retry2/final_review.json'
MEMORY=REVIEW/'actual_memories_clocks_retry1/result.json'
PNR49=BASE/'baseline_fusion_model25_NAV50_pnr49'
RADAR=BASE/'model25_SYN34_actual_review1/actual_review4/actual_original_radar26.json'
OUTPUT=BASE/'baseline_fusion_model25_bounded_MAX_pnr50'
CONTROL=BASE/'baseline_fusion_model25_bounded_MAX_pnr_one_shot'
TOOL=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
REPORT='report_timing -setup -max_paths 5000 -max_common_paths 1'
GIB=1024**3

def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,a):
    t=p.with_name(p.name+'.'+str(os.getpid())+'.tmp')
    t.write_text(json.dumps(a,ensure_ascii=False,indent=2)+'\n',encoding='utf-8',newline='\n');t.replace(p)

def prepare():
    job=load(SOURCE/'job.json');assert job['phase']=='synthesis_complete'and job['tool_exit']==0
    actual=load(MAPPING);memory=load(MEMORY);assert actual['passed']and memory['passed']
    assert actual['mapping_goal_achieved']and actual['selected_RTL_delta']==['command_lease_guard.v']
    assert actual['actual_explicit_tree']['actual_explicit_LUT_count']==11
    assert actual['actual_explicit_tree']['actual_age_Q_to_nonmax_LUT_levels']==3
    assert memory['BSRAM']==314 and memory['wide_leaves']==18 and memory['CPU_reconstructed_bytes']==131072 and memory['original_FW_BIN_bytes']==51712
    assert memory['all_memory_parameters_INIT_CPU_and_clock_checks_passed']
    assert memory['true_clock_roots']==['clk50','model25','rgmii_rxc']and memory['PLL_all_ports_parameters_identical']
    assert memory['original_radar26_strict_checker_passed']and memory['original_radar26_changed_endpoint_cells']==0
    # These26 exact endpoints were individually checked in the actual37 VG,
    # not selected by old automatic IDs or assumed from source identity.
    radar=load(RADAR);assert radar['passed']and len(radar['paths'])==26
    strict=load(REVIEW/'actual_memories_clocks_retry1/v2_initial_strict_radar26_revalidation.json')
    assert strict['passed']and strict['paths']==26 and strict['source_first_second_data_clock_enable_and_single_fanout_rechecked']
    endpoints=load(REVIEW/'actual_memories_clocks_retry1/v2_actual_radar26_endpoint_differences.json')
    assert endpoints['endpoint_differences']==[]and endpoints['strict_checker_passed']
    for row in memory['metadata_bindings']:assert metadata(Path(row['path']))==row
    clocks=load(REVIEW/'actual_memories_clocks_retry1/v2_actual_three_clocks_PLL.json')
    assert clocks['passed']and len(clocks['PLLs'])==1 and clocks['PLLs'][0]['CLKOUT0_mhz']==25
    assert clocks['PLLs'][0]['actual_cell']['ports']['CLKOUT0']==['clk_model25']
    old_syn=BASE/'baseline_fusion_model25_NAV50_syn34'
    oldmods,newmods=definitions(old_syn/'impl/gwsynthesis/g2_top.vg'),definitions(SOURCE/'impl/gwsynthesis/g2_top.vg')
    assert all(oldmods[n]==newmods[n]for n in('g2_model25_guard','control4500_core','safety_guard'))
    guarded=['u_control','u_control/model_clock_guard','u_model_pll']
    a,b=module_bindings(oldmods,guarded),module_bindings(newmods,guarded)
    assert all(a[n]['ports']==b[n]['ports']for n in guarded)
    cold=BASE/'model25_SYN34_actual_review1/actual_review4/actual_guard_cold_start_and_INIT1_proof.json'
    assert load(cold)['passed']
    sdc=(SOURCE/'g2.sdc').read_text()
    commands=lambda t:[x.strip()for x in t.splitlines()if x.strip()and not x.strip().startswith('#')]
    lines=commands(sdc)
    assert 'create_clock -name clk50 -period 20 [get_ports {clk50}]'in lines
    assert 'create_clock -name rgmii_rxc -period 10 [get_ports {rgmii_rxc}]'in lines
    generated='create_generated_clock -name model25 -source [get_ports {clk50}] -divide_by 2 [get_pins {u_model_pll/PLL_inst/CLKOUT0}]'
    assert lines.count(generated)==1
    timing=lambda rows:[x for x in rows if x.startswith(('set_false_path','set_max_delay'))]
    assert timing(lines)==timing(commands(sdc_fragment(radar)))
    assert len(timing(lines))==52 and sum(x.endswith(' -hold')for x in timing(lines))==26
    assert not any(x.startswith(('set_clock_groups','set_multicycle_path'))for x in lines)
    assert len(lines)==55,lines
    assert same_bytes(SOURCE/'g2.cst',REFERENCE/'g2.cst')
    assert not any(x in (SOURCE/'g2.cst').read_text()for x in ('GROUP ','GRP_LOC ','BUFG '))
    assert not OUTPUT.exists();OUTPUT.mkdir()
    shutil.copyfile(SOURCE/'impl/gwsynthesis/g2_top.vg',OUTPUT/'g2_top.vg')
    shutil.copyfile(SOURCE/'g2.cst',OUTPUT/'g2.cst')
    # Expected references are explicitly labelled, never presented as newtool outputs.
    shutil.copyfile(PNR49/'impl/pnr/device.cfg',OUTPUT/'expected_from49_device.cfg')
    expected_cmd=(PNR49/'impl/pnr/cmd.do').read_text().replace(str(PNR49),str(OUTPUT)).replace(PNR49.as_posix(),OUTPUT.as_posix())
    (OUTPUT/'expected_from49_cmd.do').write_text(expected_cmd,encoding='utf-8',newline='\n')
    (OUTPUT/'g2.sdc').write_text(sdc+'\n# Additional setup report only.\n'+REPORT+'\n',encoding='utf-8',newline='\n')
    tcl=(REFERENCE/'run.tcl').read_text().replace(str(REFERENCE),str(OUTPUT)).replace(REFERENCE.as_posix(),OUTPUT.as_posix())
    assert tcl.count('run pnr')==1 and tcl.count('add_file -type sdc')==1
    (OUTPUT/'run.tcl').write_text(tcl,encoding='utf-8',newline='\n')
    assert same_bytes(OUTPUT/'g2.cst',PNR49/'g2.cst')
    assert same_bytes(OUTPUT/'g2.sdc',PNR49/'g2.sdc')
    prior=(PNR49/'run.tcl').read_text().replace(str(PNR49),str(OUTPUT)).replace(PNR49.as_posix(),OUTPUT.as_posix())
    assert tcl==prior
    assert '-inc_pnr'not in tcl and 'max_runtime'not in tcl
    save(OUTPUT/'cdc_reference_audit.json',dict(passed=True,paths=radar['paths'],expected_paths=26,netlist=str(OUTPUT/'g2_top.vg'),scope='Audited original radar26; five related-clock model channels remain fully timed, no new exceptions'))
    proof=dict(passed=True,source=str(SOURCE),review=str(REVIEW),reference=str(REFERENCE),
      clock_targets_mhz=dict(clk50=50,model25=25,rgmii_rxc=100),source_BSRAM=314,predicted_physical_BSRAM=332,
      same_VG_as_completed_SYN37=same_bytes(SOURCE/'impl/gwsynthesis/g2_top.vg',OUTPUT/'g2_top.vg'),
      same_options_as_routed35=True,new_global_requests=0,new_timing_exceptions=0,
      original26_hold_and_max_delay_preserved=True,actual_INIT1_cold_clear_proved=True,
      INIT1_proof_scope='OriginalactualSYN34 coldproof carried only after exactactual guard/control/safetydefinitions andu_control/model_clock_guard/PLL connection equality toSYN37',
      full4500_finite_production_proof_passed=True,source_private_reset_candidate36_not_merged=True,
      root_actual_tree_state_and_memory_clock_reviews=[str(MAPPING),str(MEMORY)],
      actual37_original26_endpoints_revalidated=True,
      expected_cmd_and_device_only_actual_tool_generation_not_yet_verified=True,
      region35_location_dependency=False,automatic_programming=False,deployment_accepted=False,
      bindings=[metadata(p)for p in [SOURCE/'impl/gwsynthesis/g2_top.vg',MAPPING,MEMORY,RADAR,cold,old_syn/'impl/gwsynthesis/g2_top.vg',REVIEW/'actual_memories_clocks_retry1/v2_initial_strict_radar26_revalidation.json',REVIEW/'actual_memories_clocks_retry1/v2_actual_radar26_endpoint_differences.json',REVIEW/'actual_memories_clocks_retry1/v2_actual_three_clocks_PLL.json']],
      prepared_bindings=[metadata(OUTPUT/n)for n in ['g2_top.vg','g2.cst','g2.sdc','run.tcl']])
    save(OUTPUT/'prepared_request_review.json',proof)
    save(OUTPUT/'manifest.json',dict(source=str(SOURCE),source_manifest=load(SOURCE/'manifest.json'),
      frozen_netlist=True,reused_completed_synthesis=True,real_model25_NAV50=True,
      clock_periods_ns=dict(clk50=20,model25=40,rgmii_rxc=10),
      max_runtime_minutes=0,max_no_progress_minutes=0,diagnostic_only=True,programmed=False,deployment_accepted=False))
    return proof

def main(start):
    if not start:
        proof=prepare();print(json.dumps(proof,indent=2));return 0
    assert not CONTROL.exists();CONTROL.mkdir()
    me=psutil.Process();me.nice(psutil.NORMAL_PRIORITY_CLASS);me.cpu_affinity(list(range(psutil.cpu_count())))
    state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),output=str(OUTPUT),source=str(SOURCE),total_limit_s=0,silent_limit_s=0,programmed=False,deployment_accepted=False)
    def status(phase,**fields):state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**fields);save(CONTROL/'job.json',state)
    with(CONTROL/'launch_claim.json').open('x')as f:json.dump(state,f)
    try:
        status('checking_frozen_root_reviewed_PNR50_request');proof=load(OUTPUT/'prepared_request_review.json');assert proof['passed']
        while True:
            m=memory_status();status('waiting_for_actual_memory_admission',free_bytes=m.avail_phys,available_commit_bytes=m.avail_page)
            if m.avail_phys>=2*GIB and m.avail_page>=4*GIB:break
            time.sleep(13)
        for row in proof['bindings']+proof['prepared_bindings']:assert metadata(Path(row['path']))==row
        assert not(OUTPUT/'impl').exists()and not(OUTPUT/'tool_runtime.json').exists()
        status('starting_unique_PNR50')
        runtime=run_tool([str(TOOL),'run.tcl'],OUTPUT,max_runtime_s=0,max_no_progress_s=0,capture_system_telemetry=True)
        status('tool_exited_collecting',runtime=runtime)
        finalize_gate(OUTPUT);gate=load(OUTPUT/'g2_gate.json')
        tr=OUTPUT/'impl/pnr/g2_top.tr'
        if tr.exists():save(OUTPUT/'actual_setup_endpoint_coverage.json',scan_report(tr))
        status('routed_requires_independent_all3clock_signoff'if gate['routing_passed']else'physical_failure_preserved_no_retry',runtime=runtime,gate=str(OUTPUT/'g2_gate.json'),routing_passed=gate['routing_passed'],unrouted=gate['unrouted_nets'],TR_available=tr.exists())
        return int(runtime['tool_exit']!=0 or not gate['routing_passed'])
    except Exception as error:
        import traceback
        status('failed_preserved_no_automatic_retry',error=str(error),traceback=traceback.format_exc());return 1

if __name__=='__main__':
    ap=argparse.ArgumentParser();mode=ap.add_mutually_exclusive_group(required=True);mode.add_argument('--prepare',action='store_true');mode.add_argument('--start',action='store_true');args=ap.parse_args();raise SystemExit(main(args.start))
