"""One frozen ring-storage PNR51 after actual SYN39 and root review.

The new ring is not a same-input retry of PNR50. All three clock targets,
original52 radar constraints, IO and routed35 implementation options remain.
No programming or automatic source changes. Both PNR deadlines are zero.
"""
from pathlib import Path
from datetime import datetime
import argparse, json, shutil, os, time, psutil
import run_lease_bounded_MAX_pnr50 as runner
from audit_cdc_first_stages import sdc_fragment
from prepare_baseline_fusion_retime import metadata, same_bytes

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'baseline_fusion_model25_observation_ring_syn39'
REVIEW=BASE/'SYN39_ring_root_preflight1/result.json'
REFERENCE=BASE/'baseline_fusion32_CDC52_pnr35'
PNR50=BASE/'baseline_fusion_model25_bounded_MAX_pnr50'
OUTPUT=BASE/'baseline_fusion_model25_observation_ring_pnr51'
CONTROL=BASE/'baseline_fusion_model25_observation_ring_pnr_one_shot'
RADAR=BASE/'model25_SYN34_actual_review1/actual_review4/actual_original_radar26.json'

def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))

def prepare():
    job=load(SOURCE/'job.json')
    assert job['phase']=='synthesis_complete' and job['tool_exit']==0
    root=load(REVIEW)
    assert root['passed'] and root['actual_source']==str(SOURCE)
    for key in ('actual_resources_storage_clocks_passed',
                'nonring_RAM_INIT_CPU_preserved','guard_and_stop_boundary_preserved',
                'original26_strict_rechecked','real4500_20_publications_passed'):
        assert root[key],key
    assert root['actual_ring_semantics_passed'] or root['pending_prefetch_CE_review_parallel_exploration_only']
    for row in root['metadata_bindings']:assert metadata(Path(row['path']))==row
    radar=load(RADAR);assert radar['passed'] and len(radar['paths'])==26
    sdc=(SOURCE/'g2.sdc').read_text()
    commands=lambda t:[x.strip()for x in t.splitlines()if x.strip()and not x.strip().startswith('#')]
    lines=commands(sdc)
    assert 'create_clock -name clk50 -period 20 [get_ports {clk50}]' in lines
    assert 'create_clock -name rgmii_rxc -period 10 [get_ports {rgmii_rxc}]' in lines
    generated='create_generated_clock -name model25 -source [get_ports {clk50}] -divide_by 2 [get_pins {u_model_pll/PLL_inst/CLKOUT0}]'
    assert lines.count(generated)==1
    timing=lambda rows:[x for x in rows if x.startswith(('set_false_path','set_max_delay'))]
    assert timing(lines)==timing(commands(sdc_fragment(radar)))
    assert len(timing(lines))==52 and sum(x.endswith(' -hold')for x in timing(lines))==26
    assert len(lines)==55 and not any(x.startswith(('set_clock_groups','set_multicycle_path'))for x in lines)
    assert same_bytes(SOURCE/'g2.cst',REFERENCE/'g2.cst')
    assert not any(x in (SOURCE/'g2.cst').read_text()for x in ('GROUP ','GRP_LOC ','BUFG '))
    assert not OUTPUT.exists();OUTPUT.mkdir()
    shutil.copyfile(SOURCE/'impl/gwsynthesis/g2_top.vg',OUTPUT/'g2_top.vg')
    shutil.copyfile(SOURCE/'g2.cst',OUTPUT/'g2.cst')
    (OUTPUT/'g2.sdc').write_text(sdc+'\n# Additional setup report only.\n'+runner.REPORT+'\n',encoding='utf-8',newline='\n')
    tcl=(REFERENCE/'run.tcl').read_text().replace(str(REFERENCE),str(OUTPUT)).replace(REFERENCE.as_posix(),OUTPUT.as_posix())
    assert tcl.count('run pnr')==1 and tcl.count('add_file -type sdc')==1
    assert '-inc_pnr'not in tcl
    (OUTPUT/'run.tcl').write_text(tcl,encoding='utf-8',newline='\n')
    assert same_bytes(OUTPUT/'g2.sdc',PNR50/'g2.sdc')
    expected=(PNR50/'run.tcl').read_text().replace(str(PNR50),str(OUTPUT)).replace(PNR50.as_posix(),OUTPUT.as_posix())
    assert tcl==expected
    shutil.copyfile(PNR50/'impl/pnr/device.cfg',OUTPUT/'expected_from50_device.cfg')
    cmd=(PNR50/'impl/pnr/cmd.do').read_text().replace(str(PNR50),str(OUTPUT)).replace(PNR50.as_posix(),OUTPUT.as_posix())
    (OUTPUT/'expected_from50_cmd.do').write_text(cmd,encoding='utf-8',newline='\n')
    runner.save(OUTPUT/'cdc_reference_audit.json',dict(passed=True,paths=radar['paths'],expected_paths=26,netlist=str(OUTPUT/'g2_top.vg'),scope='Original26 strict rechecked in actual39. Five related-clock channels remain fully timed.'))
    proof=dict(passed=True,source=str(SOURCE),review=str(REVIEW),reference=str(REFERENCE),
      actual_SYN39_FF=31056,actual_SYN39_LUT=46188,actual_SYN39_BSRAM=314,
      predicted_split_physical_BSRAM=332,physical_CLS_gain_routing_timing_unknown=True,
      clocks_mhz=dict(clk50=50,model25=25,rgmii_rxc=100),new_timing_exceptions=0,new_global_requests=0,
      original26_hold_and_max_delay_preserved=True,real4500_20_publications_passed=True,
      actual_ring_semantics_passed=root['actual_ring_semantics_passed'],
      prefetch_CE_checker_difference_pending=not root['actual_ring_semantics_passed'],
      OLED_and_IR_stop_preserved=True,all_weights_and_CPU_capacity_preserved=True,
      same_VG_as_actual39=same_bytes(SOURCE/'impl/gwsynthesis/g2_top.vg',OUTPUT/'g2_top.vg'),
      same_options_as_routed35=True,automatic_programming=False,deployment_accepted=False,
      bindings=root['metadata_bindings']+[metadata(REVIEW),metadata(RADAR)],
      prepared_bindings=[metadata(OUTPUT/n)for n in ('g2_top.vg','g2.cst','g2.sdc','run.tcl')])
    runner.save(OUTPUT/'prepared_request_review.json',proof)
    runner.save(OUTPUT/'manifest.json',dict(source=str(SOURCE),source_manifest=load(SOURCE/'manifest.json'),
      frozen_netlist=True,reused_completed_synthesis=True,real_model25_NAV50=True,
      clock_periods_ns=dict(clk50=20,model25=40,rgmii_rxc=10),
      max_runtime_minutes=0,max_no_progress_minutes=0,diagnostic_only=True,programmed=False,deployment_accepted=False))
    return proof

def main(start):
    if not start:
        proof=prepare();print(json.dumps({k:v for k,v in proof.items()if k not in ('bindings','prepared_bindings')},indent=2));return 0
    assert not CONTROL.exists();CONTROL.mkdir()
    me=psutil.Process();me.nice(psutil.NORMAL_PRIORITY_CLASS);me.cpu_affinity(list(range(psutil.cpu_count())))
    state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),output=str(OUTPUT),source=str(SOURCE),total_limit_s=0,silent_limit_s=0,programmed=False,deployment_accepted=False)
    def status(phase,**fields):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**fields)
        runner.save(CONTROL/'job.json',state)
    with(CONTROL/'launch_claim.json').open('x')as f:json.dump(state,f)
    try:
        status('checking_frozen_root_reviewed_PNR51_request')
        proof=load(OUTPUT/'prepared_request_review.json');assert proof['passed']
        while True:
            m=runner.memory_status();status('waiting_for_actual_memory_admission',free_bytes=m.avail_phys,available_commit_bytes=m.avail_page)
            if m.avail_phys>=2*runner.GIB and m.avail_page>=4*runner.GIB:break
            time.sleep(13)
        for row in proof['bindings']+proof['prepared_bindings']:assert metadata(Path(row['path']))==row
        assert not(OUTPUT/'impl').exists()and not(OUTPUT/'tool_runtime.json').exists()
        status('starting_unique_PNR51')
        runtime=runner.run_tool([str(runner.TOOL),'run.tcl'],OUTPUT,max_runtime_s=0,max_no_progress_s=0,capture_system_telemetry=True)
        status('tool_exited_collecting',runtime=runtime)
        runner.finalize_gate(OUTPUT);gate=load(OUTPUT/'g2_gate.json')
        tr=OUTPUT/'impl/pnr/g2_top.tr'
        if tr.exists():runner.save(OUTPUT/'actual_setup_endpoint_coverage.json',runner.scan_report(tr))
        status('routed_requires_independent_all3clock_signoff'if gate['routing_passed']else'physical_failure_preserved_no_retry',runtime=runtime,gate=str(OUTPUT/'g2_gate.json'),routing_passed=gate['routing_passed'],unrouted=gate['unrouted_nets'],TR_available=tr.exists())
        return int(runtime['tool_exit']!=0 or not gate['routing_passed'])
    except Exception as error:
        import traceback
        status('failed_preserved_no_automatic_retry',error=str(error),traceback=traceback.format_exc());return 1

if __name__=='__main__':
    ap=argparse.ArgumentParser();g=ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--prepare',action='store_true');g.add_argument('--start',action='store_true')
    a=ap.parse_args();raise SystemExit(main(a.start))
