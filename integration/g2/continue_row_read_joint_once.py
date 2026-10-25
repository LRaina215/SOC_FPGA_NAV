"""Wait for PNR19's real result; conditionally evaluate one frozen row candidate.

Never interrupts the prior build. No unconditional adoption, retry or hardware
action. New physical construction requires a fresh full netlist/clock/CDC audit.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
import traceback

import psutil

from collect_routing_diagnostic import collect
from measure_row_read_context import reference_snapshot
from retry_frozen_synthesis import memory_status

ROOT=Path(__file__).resolve().parents[2]
GIB=1024**3


def decision_from_result(build):
    result=collect(build)
    runtime=result['runtime'];assert runtime and 'tool_exit' in runtime, 'prior tool has not produced a final result'
    if runtime['cancelled']:
        return dict(action='review_required',reason='Prior tool was cancelled; do not treat as a natural routing outcome.',evidence=result)
    report=build/'impl/pnr/g2_top.rpt.txt';text=report.read_text() if report.exists() else ''
    memory=re.search(r'^\s*BSRAM\s*\|\s*(\d+)/(\d+)',text,re.M)
    if not memory or int(memory[1])>336:
        return dict(action='review_required',reason='Actual prior BSRAM is missing or outside the336-block work gate; shared reads do not reduce BSRAM.',evidence=result)
    result['actual_BSRAM']=int(memory[1])
    errors=re.findall(r'^.*ERROR\s*\([^\n]+',text,re.M)
    if result['reported_unrouted_nets'] and errors and all('PR0004' in e for e in errors):
        return dict(action='evaluate_frozen_shared_candidate',
            reason='Natural PR0004 with actual unresolved routes and RAM within budget. Evaluate one source-only row-read alternative with completed local function/50MHz evidence; no failed-net identity inferred.',evidence=result)
    if runtime['tool_exit']==0 and result['timing_report_available'] and not errors:
        timing=(build/'impl/pnr/g2_top.tr').read_text()
        counts={k:int(re.search(r'<Numbers of '+v+r' Violated Endpoints>:\s*(\d+)',timing)[1]) for k,v in [('setup','Setup'),('hold','Hold')]}
        result['global_timing_violations']=counts
        if counts==dict(setup=0,hold=0):
            return dict(action='retain_prior_candidate',reason='Prior routing and reported global setup/hold are clean; preserve it and continue remaining joint gate review instead of introducing an optional change.',evidence=result)
        relevant=[line for line in timing.splitlines() if re.match(r'^\s*\d+\s+-[\d.]+\s+',line) and 'u_nav/u_match/u_ex/' in line]
        if relevant:
            return dict(action='evaluate_frozen_shared_candidate',reason='Actual negative timing rows intersect the extractor affected by the measured alternative. Evaluate once; local pass does not guarantee joint repair.',relevant_reported_rows=relevant,evidence=result)
        return dict(action='review_required',reason='Prior timing failed, but available negative rows do not justify this extractor change. Preserve candidate and analyze the actual paths.',evidence=result)
    return dict(action='review_required',reason='Final result is neither a supported natural routing failure nor a completed timing result; inspect the concrete tool error.',evidence=result)


def main():
    p=argparse.ArgumentParser()
    for key in ('prepared','prior-pnr','prior-state','syn-out','pnr-out','state-dir'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--check-only',action='store_true')
    p.add_argument('--resume-settings-only',action='store_true')
    a=p.parse_args()
    prepared,prior,prior_state,syn,out,state_dir=[x.resolve() for x in (a.prepared,a.prior_pnr,a.prior_state,a.syn_out,a.pnr_out,a.state_dir)]
    baseline=prior.parent/'sfp_lane2_observation_lw_pnr15'
    if a.check_only:
        from review_small_block_stores import review
        proof=review(syn,baseline)
        (state_dir/'pre_pnr_checks.json').write_text(json.dumps(proof,indent=2)+'\n',encoding='utf-8')
        return 0
    assert not out.exists()
    if a.resume_settings_only:
        previous=json.loads((syn/'job.json').read_text())
        assert previous['phase']=='validating_frozen_settings' and 'tool_pid' not in previous
        assert not (syn/'impl').exists() and not (syn/'gw_sh.log').exists()
    else:assert not syn.exists()
    preparation=json.loads((prepared/'prepared_source_review.json').read_text())
    assert preparation['phase']=='prepared_only_waiting_for_PNR19_result_review'
    assert preparation['unchanged_assets']==104 and preparation['unchanged_hardware_inputs']==67
    state_dir.mkdir(exist_ok=False)
    me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    state=dict(prepared=str(prepared),prior_PNR=str(prior),synthesis=str(syn),PNR=str(out),
        controller_pid=me.pid,controller_created_epoch=me.create_time(),programmed=False,
        deployment_accepted=False,maximum_new_SYN_runs=1,maximum_new_PNR_runs=1,
        PNR_total_seconds=0,PNR_silent_seconds=0)
    def save(phase,**extra):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**extra)
        (state_dir/'job.json').write_text(json.dumps(state,indent=2)+'\n',encoding='utf-8')
    try:
        protected=preparation['protected_joint_build'];latest=reference_snapshot(prior)
        assert protected['input_metadata']==latest['input_metadata']
        assert latest['pid']==protected['pid'] and latest['total_limit_s']==latest['idle_limit_s']==0
        prior_job=json.loads((prior_state/'job.json').read_text())
        identities=[]
        for pid,needle in [(latest['pid'],'gw_sh.exe'),(prior_job['pnr_wrapper_pid'],'pnr_from_netlist.py'),(prior_job['controller_pid'],'continue_smallblocks_once.py')]:
            try:
                process=psutil.Process(pid);command=process.cmdline()
                assert any(needle in part for part in command), ('prior process identity changed',pid,command)
                if pid==latest['pid']:
                    assert abs(process.create_time()-protected['created_epoch'])<0.1, 'prior tool PID was reused'
                if pid==prior_job['controller_pid']:
                    assert abs(process.create_time()-prior_job['controller_created_epoch'])<0.1, 'prior controller PID was reused'
                identities.append(dict(pid=pid,created_epoch=process.create_time(),command=command))
            except psutil.NoSuchProcess:pass
        save('waiting_for_PNR19_natural_completion_and_result',protected_inputs=protected['input_metadata'],prior_processes=identities)
        while True:
            running=[]
            for identity in identities:
                try:
                    process=psutil.Process(identity['pid'])
                    if abs(process.create_time()-identity['created_epoch'])<0.1:running.append(identity['pid'])
                except psutil.NoSuchProcess:pass
            if not running:break
            time.sleep(15)
        latest=reference_snapshot(prior)
        assert latest['input_metadata']==protected['input_metadata'], 'prior inputs changed while waiting'
        save('reviewing_actual_PNR19_result')
        choice=decision_from_result(prior)
        (state_dir/'prior_result_decision.json').write_text(json.dumps(choice,indent=2)+'\n',encoding='utf-8')
        if choice['action']!='evaluate_frozen_shared_candidate':
            save('prior_result_requires_full_gate_review' if choice['action']=='retain_prior_candidate' else 'prior_result_requires_targeted_review',decision=choice['action'],reason=choice['reason'])
            return 0
        save('waiting_for_serial_SYN_memory',decision=choice['action'],reason=choice['reason'])
        while True:
            memory=memory_status()
            if memory.avail_phys>=2*GIB and memory.avail_page>=4*GIB:break
            time.sleep(15)
        command=[sys.executable,'-X','utf8',str(ROOT/'integration/g2/retry_frozen_synthesis.py'),str(prepared),'--out',str(syn),'--start-now']
        if a.resume_settings_only:command.append('--resume-settings-only')
        with (state_dir/'synthesis_runner.log').open('w',encoding='utf-8') as stream:
            child=subprocess.Popen(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
            save('one_frozen_joint_SYN_running',synthesis_supervisor_pid=child.pid,command=command)
            child.wait()
        assert child.returncode==0, 'new joint SYN failed; inspect synthesis_runner.log and actual job'
        assert json.loads((syn/'job.json').read_text())['phase']=='synthesis_complete'
        save('reviewing_new_netlist_clocks_and_CDC')
        shutil.copyfile(ROOT/'integration/g2/review_small_block_stores.py',state_dir/'reviewer_snapshot.py')
        command=[sys.executable,'-X','utf8',str(Path(__file__).resolve()),
                 '--prepared',str(prepared),'--prior-pnr',str(prior),'--prior-state',str(prior_state),
                 '--syn-out',str(syn),'--pnr-out',str(out),'--state-dir',str(state_dir),'--check-only']
        with (state_dir/'validation.log').open('w',encoding='utf-8') as stream:
            subprocess.run(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,check=True,timeout=300,creationflags=subprocess.CREATE_NO_WINDOW)
        assert json.loads((state_dir/'pre_pnr_checks.json').read_text())['passed']
        inputs=state_dir/'pnr_inputs';(inputs/'impl/gwsynthesis').mkdir(parents=True);(inputs/'ip').mkdir()
        for name in ('impl/gwsynthesis/g2_top.vg','impl/gwsynthesis/g2_top.prj','effective_options.tcl','manifest.json'):
            shutil.copyfile(syn/name,inputs/name)
        for name in ('g2.cst','g2.sdc','ip/serdes.csr','cdc_reference_audit.json','sfp_cdc_reference.json'):
            shutil.copyfile(baseline/name,inputs/name)
        save('waiting_for_serial_PNR_memory',pre_pnr_checks_passed=True)
        while True:
            memory=memory_status()
            if memory.avail_phys>=2*GIB and memory.avail_page>=4*GIB:break
            time.sleep(15)
        command=[sys.executable,'-X','utf8',str(ROOT/'integration/g2/pnr_from_netlist.py'),str(inputs),'--out',str(out),
            '--wide-split','1','--replicate-resources','1','--place-option','1','--route-option','2','--timing-driven','1',
            '--max-runtime-minutes','0','--max-no-progress-minutes','0','--routing-evidence','--capture-system-telemetry']
        with (state_dir/'pnr_runner.log').open('w',encoding='utf-8') as stream:
            child=subprocess.Popen(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
            save('one_full_shared_candidate_PNR_running',pnr_wrapper_pid=child.pid,command=command)
            child.wait()
        result=collect(out)
        (state_dir/'routing_result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
        if result['pnr_report_available'] and (out/'impl/pnr/g2_top.vo').exists():
            from audit_global_muxes import audit
            requested=json.loads((ROOT/'integration/g2/clock_resource_plan_lane2_observation_lw.json').read_text())
            names=[r['net'] for r in requested['promote']+requested['prefer_global']]+requested['local']
            actual=audit(out,requested_nets=names+['clk50_d','rgmii_rxc_d','h_rx_clk','h_tx_clk'])
            (state_dir/'actual_global_assignments.json').write_text(json.dumps(actual,indent=2)+'\n',encoding='utf-8')
        save('shared_candidate_PNR_collected_full_joint_review_required',wrapper_exit=child.returncode,
             reported_unrouted_nets=result['reported_unrouted_nets'],timing_report_available=result['timing_report_available'])
        return 0
    except Exception as exc:
        (state_dir/'failure.log').write_text(traceback.format_exc(),encoding='utf-8')
        save('specific_review_or_execution_failure',failure=str(exc),error_type=type(exc).__name__)
        return 1


if __name__=='__main__':raise SystemExit(main())
