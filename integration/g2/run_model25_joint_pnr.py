"""One exploratory real model25/NAV50 PNR, independent of region-GUI/RTL sim.

Retains all three physical clock targets and original audited radar constraints.
Remaining whole-word numeric proofs run in parallel and remain deployment gates.
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
from check_setup_endpoint_coverage import scan_report
from collect_pnr_result import finalize_gate
from prepare_baseline_fusion_retime import metadata, same_bytes
from retry_frozen_synthesis import memory_status

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'baseline_fusion_model25_NAV50_syn34'
REFERENCE=BASE/'baseline_fusion32_CDC52_pnr35'
REVIEW=BASE/'model25_SYN34_actual_review1/actual_review4'
OUTPUT=BASE/'baseline_fusion_model25_NAV50_pnr49'
CONTROL=BASE/'baseline_fusion_model25_NAV50_pnr_one_shot'
TOOL=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
REPORT='report_timing -setup -max_paths 5000 -max_common_paths 1'
GIB=1024**3

def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,a):
    t=p.with_name(p.name+'.'+str(os.getpid())+'.tmp')
    t.write_text(json.dumps(a,ensure_ascii=False,indent=2)+'\n',encoding='utf-8',newline='\n');t.replace(p)

def prepare():
    job=load(SOURCE/'job.json');assert job['phase']=='synthesis_complete'and job['tool_exit']==0
    actual=load(REVIEW/'final_review.json');assert actual['passed']
    base=actual['base'];assert base['actual_BSRAM']==314 and base['wide_leaves']==18
    assert base['CPU_image_bytes']==131072 and base['original_FW_BIN_bytes']==51712
    assert base['all_memory_geometry_INIT_preserved']
    assert base['true_clock_roots']==['clk50','model25','rgmii_rxc']
    radar=load(REVIEW/'actual_original_radar26.json');assert radar['passed']and len(radar['paths'])==26
    assert load(REVIEW/'actual_guard_cold_start_and_INIT1_proof.json')['passed']
    clocks=load(REVIEW/'actual_clock_PLL_inventory.json')
    assert clocks['PLL']['name']=='u_model_pll/PLL_inst'
    assert clocks['model_output_mhz']==25 and clocks['VCO_mhz']==800
    assert clocks['PLL']['ports']['CLKOUT0']==['clk_model25']
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
    (OUTPUT/'g2.sdc').write_text(sdc+'\n# Additional setup report only.\n'+REPORT+'\n',encoding='utf-8',newline='\n')
    tcl=(REFERENCE/'run.tcl').read_text().replace(str(REFERENCE),str(OUTPUT)).replace(REFERENCE.as_posix(),OUTPUT.as_posix())
    assert tcl.count('run pnr')==1 and tcl.count('add_file -type sdc')==1
    (OUTPUT/'run.tcl').write_text(tcl,encoding='utf-8',newline='\n')
    save(OUTPUT/'cdc_reference_audit.json',dict(passed=True,paths=radar['paths'],expected_paths=26,netlist=str(OUTPUT/'g2_top.vg'),scope='Audited original radar26; five related-clock model channels remain fully timed, no new exceptions'))
    proof=dict(passed=True,source=str(SOURCE),review=str(REVIEW),reference=str(REFERENCE),
      clock_targets_mhz=dict(clk50=50,model25=25,rgmii_rxc=100),source_BSRAM=314,predicted_physical_BSRAM=332,
      same_VG_as_completed_SYN34=same_bytes(SOURCE/'impl/gwsynthesis/g2_top.vg',OUTPUT/'g2_top.vg'),
      same_options_as_routed35=True,new_global_requests=0,new_timing_exceptions=0,
      original26_hold_and_max_delay_preserved=True,actual_INIT1_cold_clear_proved=True,
      whole_word_alias_numeric_proof_and_full4500_simulation_pending=True,
      region35_location_dependency=False,automatic_programming=False,deployment_accepted=False,
      bindings=[metadata(p)for p in [SOURCE/'impl/gwsynthesis/g2_top.vg',REVIEW/'final_review.json',REVIEW/'actual_original_radar26.json',REVIEW/'actual_guard_cold_start_and_INIT1_proof.json',REVIEW/'actual_clock_PLL_inventory.json']],
      prepared_bindings=[metadata(OUTPUT/n)for n in ['g2_top.vg','g2.cst','g2.sdc','run.tcl']])
    save(OUTPUT/'prepared_request_review.json',proof)
    save(OUTPUT/'manifest.json',dict(source=str(SOURCE),source_manifest=load(SOURCE/'manifest.json'),
      frozen_netlist=True,reused_completed_synthesis=True,real_model25_NAV50=True,
      clock_periods_ns=dict(clk50=20,model25=40,rgmii_rxc=10),
      max_runtime_minutes=0,max_no_progress_minutes=0,diagnostic_only=True,programmed=False,deployment_accepted=False))
    return proof

def main():
    assert not CONTROL.exists();CONTROL.mkdir()
    me=psutil.Process();me.nice(psutil.NORMAL_PRIORITY_CLASS);me.cpu_affinity(list(range(psutil.cpu_count())))
    state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),output=str(OUTPUT),source=str(SOURCE),total_limit_s=0,silent_limit_s=0,programmed=False,deployment_accepted=False)
    def status(phase,**fields):state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**fields);save(CONTROL/'job.json',state)
    with(CONTROL/'launch_claim.json').open('x')as f:json.dump(state,f)
    try:
        status('preparing_exploratory_three_clock_PNR49');proof=prepare()
        while True:
            m=memory_status();status('waiting_for_actual_memory_admission',free_bytes=m.avail_phys,available_commit_bytes=m.avail_page)
            if m.avail_phys>=2*GIB and m.avail_page>=4*GIB:break
            time.sleep(13)
        for row in proof['bindings']+proof['prepared_bindings']:assert metadata(Path(row['path']))==row
        assert not(OUTPUT/'impl').exists()and not(OUTPUT/'tool_runtime.json').exists()
        status('starting_unique_PNR49')
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

if __name__=='__main__':raise SystemExit(main())
