"""One exploratory PNR52 of the actual ring/MAX original50MHz netlist.

Reuses SYN40 after root actual-netlist review. No new synthesis, new clocks,
global requests, timing exceptions, retries or programming. Both limits zero.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,os,shutil,time,psutil
from audit_cdc_first_stages import sdc_fragment
from bounded_tool import run_tool
from check_setup_endpoint_coverage import scan_report
from collect_pnr_result import finalize_gate
from prepare_baseline_fusion_retime import metadata,same_bytes
from retry_frozen_synthesis import memory_status

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'baseline_fusion50_ring_MAX_syn40'
REFERENCE=BASE/'baseline_fusion32_CDC52_pnr35'
REVIEW=BASE/'SYN40_root_PNR52_preflight1/result.json'
OUTPUT=BASE/'baseline_fusion50_ring_MAX_pnr52'
CONTROL=BASE/'baseline_fusion50_ring_MAX_pnr_one_shot'
TOOL=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
REPORT='report_timing -setup -max_paths 5000 -max_common_paths 1'
GIB=1024**3
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,a):
    q=p.with_name(p.name+'.'+str(os.getpid())+'.tmp')
    q.write_text(json.dumps(a,ensure_ascii=False,indent=2)+'\n',encoding='utf-8',newline='\n');q.replace(p)
def prepare():
    job=load(SOURCE/'job.json');assert job['phase']=='synthesis_complete'and job['tool_exit']==0
    root=load(REVIEW);assert root['passed']and root['actual_source']==str(SOURCE)
    for key in ('actual_resources_storage_clocks_passed','nonring_RAM_INIT_CPU_preserved',
                'guard_and_stop_boundary_preserved','original26_strict_rechecked',
                'selected_ring_mode_correct','original50MHz_period400000_gate500000'):
        assert root[key],key
    assert root['real4500_result_passed']or root['parallel_numerical_proof_pending_exploration_only']
    assert root['actual_guard_nextstate_proved']or root['mapped_guard_input_roles_review_pending_exploration_only']
    for row in root['metadata_bindings']:assert metadata(Path(row['path']))==row
    radar=load(Path(root['actual_radar26_file']));assert radar['passed']and len(radar['paths'])==26
    sdc=(SOURCE/'g2.sdc').read_text()
    commands=lambda t:[x.strip()for x in t.splitlines()if x.strip()and not x.strip().startswith('#')]
    lines=commands(sdc)
    assert 'create_clock -name clk50 -period 20 [get_ports {clk50}]'in lines
    assert 'create_clock -name rgmii_rxc -period 10 [get_ports {rgmii_rxc}]'in lines
    assert len(lines)==54 and not any(x.startswith(('create_generated_clock','set_clock_groups','set_multicycle_path'))for x in lines)
    timing=lambda rows:[x for x in rows if x.startswith(('set_false_path','set_max_delay'))]
    assert timing(lines)==timing(commands(sdc_fragment(radar)))
    assert len(timing(lines))==52 and sum(x.endswith(' -hold')for x in timing(lines))==26
    assert same_bytes(SOURCE/'g2.cst',REFERENCE/'g2.cst')
    assert same_bytes(SOURCE/'g2.sdc',REFERENCE/'g2.sdc')
    assert not any(x in (SOURCE/'g2.cst').read_text()for x in ('GROUP ','GRP_LOC ','BUFG '))
    assert not OUTPUT.exists();OUTPUT.mkdir()
    shutil.copyfile(SOURCE/'impl/gwsynthesis/g2_top.vg',OUTPUT/'g2_top.vg')
    shutil.copyfile(SOURCE/'g2.cst',OUTPUT/'g2.cst')
    (OUTPUT/'g2.sdc').write_bytes((SOURCE/'g2.sdc').read_bytes()+b'\n# Additional setup report only.\n'+REPORT.encode()+b'\n')
    expected=(REFERENCE/'g2.sdc').read_bytes()+b'\n# Additional setup report only.\n'+REPORT.encode()+b'\n'
    assert (OUTPUT/'g2.sdc').read_bytes()==expected
    tcl=(REFERENCE/'run.tcl').read_text().replace(str(REFERENCE),str(OUTPUT)).replace(REFERENCE.as_posix(),OUTPUT.as_posix())
    assert tcl.count('run pnr')==1 and tcl.count('add_file -type sdc')==1 and '-inc_pnr'not in tcl
    (OUTPUT/'run.tcl').write_text(tcl,encoding='utf-8',newline='\n')
    shutil.copyfile(REFERENCE/'impl/pnr/device.cfg',OUTPUT/'expected_from35_device.cfg')
    cmd=(REFERENCE/'impl/pnr/cmd.do').read_text().replace(str(REFERENCE),str(OUTPUT)).replace(REFERENCE.as_posix(),OUTPUT.as_posix())
    (OUTPUT/'expected_from35_cmd.do').write_text(cmd,encoding='utf-8',newline='\n')
    save(OUTPUT/'cdc_reference_audit.json',dict(passed=True,paths=radar['paths'],expected_paths=26,netlist=str(OUTPUT/'g2_top.vg')))
    proof=dict(passed=True,source=str(SOURCE),review=str(REVIEW),actual_SYN40_resources=root['actual_resources'],
      original50_100MHz_architecture=True,ring_and_MAX_only=True,PLL25_and_five_related_bridges_absent=True,
      clock_targets_mhz=dict(clk50=50,rgmii_rxc=100),new_global_requests=0,new_timing_exceptions=0,
      physical_CLS_routing_and_timing_unknown=True,real4500_result_passed=root['real4500_result_passed'],
      actual_guard_nextstate_proved=root['actual_guard_nextstate_proved'],
      mapped_guard_input_roles_review_pending=not root['actual_guard_nextstate_proved'],
      same_VG_as_completed_SYN40=same_bytes(SOURCE/'impl/gwsynthesis/g2_top.vg',OUTPUT/'g2_top.vg'),
      bindings=root['metadata_bindings']+[metadata(REVIEW)],
      prepared_bindings=[metadata(OUTPUT/n)for n in ('g2_top.vg','g2.cst','g2.sdc','run.tcl')],
      automatic_programming=False,deployment_accepted=False)
    save(OUTPUT/'prepared_request_review.json',proof)
    save(OUTPUT/'manifest.json',dict(source=str(SOURCE),source_manifest=load(SOURCE/'manifest.json'),
      frozen_netlist=True,reused_completed_synthesis=True,clock_periods_ns=dict(clk50=20,rgmii_rxc=10),
      max_runtime_minutes=0,max_no_progress_minutes=0,diagnostic_only=True,programmed=False,deployment_accepted=False))
    return proof
def main(start):
    if not start:
        proof=prepare();print(json.dumps({k:v for k,v in proof.items()if k not in ('bindings','prepared_bindings')},indent=2));return 0
    assert not CONTROL.exists();CONTROL.mkdir()
    me=psutil.Process();me.nice(psutil.NORMAL_PRIORITY_CLASS);me.cpu_affinity(list(range(psutil.cpu_count())))
    state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),source=str(SOURCE),output=str(OUTPUT),total_limit_s=0,silent_limit_s=0,programmed=False,deployment_accepted=False)
    def status(phase,**fields):state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**fields);save(CONTROL/'job.json',state)
    with(CONTROL/'launch_claim.json').open('x')as f:json.dump(state,f)
    try:
        proof=load(OUTPUT/'prepared_request_review.json');assert proof['passed']
        while True:
            m=memory_status();status('waiting_for_actual_memory_admission',free_bytes=m.avail_phys,available_commit_bytes=m.avail_page)
            if m.avail_phys>=2*GIB and m.avail_page>=4*GIB:break
            time.sleep(13)
        for row in proof['bindings']+proof['prepared_bindings']:assert metadata(Path(row['path']))==row
        assert not(OUTPUT/'impl').exists()and not(OUTPUT/'tool_runtime.json').exists()
        status('starting_unique_PNR52')
        runtime=run_tool([str(TOOL),'run.tcl'],OUTPUT,max_runtime_s=0,max_no_progress_s=0,capture_system_telemetry=True)
        status('tool_exited_collecting',runtime=runtime);finalize_gate(OUTPUT);gate=load(OUTPUT/'g2_gate.json')
        tr=OUTPUT/'impl/pnr/g2_top.tr'
        if tr.exists():save(OUTPUT/'actual_setup_endpoint_coverage.json',scan_report(tr))
        status('routed_requires_independent_signoff'if gate['routing_passed']else'physical_failure_preserved_no_retry',runtime=runtime,routing_passed=gate['routing_passed'],unrouted=gate['unrouted_nets'],TR_available=tr.exists())
        return int(runtime['tool_exit']!=0 or not gate['routing_passed'])
    except Exception as error:
        import traceback
        status('failed_preserved_no_automatic_retry',error=str(error),traceback=traceback.format_exc());return 1
if __name__=='__main__':
    ap=argparse.ArgumentParser();g=ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--prepare',action='store_true');g.add_argument('--start',action='store_true')
    a=ap.parse_args();raise SystemExit(main(a.start))
