"""One lossless LF-storage alternative after the current full PNR completes.

Never interrupts another build. Every new PNR remains unlimited in total and
silent time, and source/netlist/clock/CDC checks precede physical construction.
No result from this controller authorizes programming by itself.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback

import psutil

from collect_routing_diagnostic import collect
from continue_row_read_joint_once import decision_from_result
from measure_row_read_context import reference_snapshot
from retry_frozen_synthesis import memory_status

ROOT=Path(__file__).resolve().parents[2]
GIB=1024**3


def main():
    parser=argparse.ArgumentParser()
    for name in ('prepared','prior-pnr','prior-state','baseline','syn-out','pnr-out','state-dir'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--shared-prepared',type=Path,help='LF-only alternative on the running shared-row origin; choose it only if actual prior unrouted improves on184')
    parser.add_argument('--check-only',action='store_true')
    args=parser.parse_args()
    prepared,prior,prior_state,baseline,syn,out,folder=[p.resolve() for p in (
        args.prepared,args.prior_pnr,args.prior_state,args.baseline,args.syn_out,args.pnr_out,args.state_dir)]
    if args.check_only:
        from review_lf_line_joint import review
        result=review(syn,baseline)
        (folder/'pre_pnr_checks.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
        return 0
    assert not syn.exists() and not out.exists()
    preparation=json.loads((prepared/'prepared_source_review.json').read_text())
    assert preparation['passed'] and preparation['unchanged_hardware_inputs']==67 and preparation['unchanged_assets']==104
    assert preparation['modified_rtl_inputs']==['nav/rtl/match/slf_lf.v']
    shared=args.shared_prepared.resolve() if args.shared_prepared else None
    if shared:
        shared_review=json.loads((shared/'prepared_source_review.json').read_text())
        assert shared_review['passed'] and shared_review['unchanged_hardware_inputs']==67 and shared_review['unchanged_assets']==104
        assert shared_review['modified_rtl_inputs']==['nav/rtl/match/slf_lf.v']
        assert Path(shared_review['source']).name=='sfp_lane2_row_read_shared_syn17'
    folder.mkdir(exist_ok=False)
    me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    state=dict(prepared=str(prepared),prior_PNR=str(prior),baseline=str(baseline),synthesis=str(syn),PNR=str(out),
        controller_pid=me.pid,controller_created_epoch=me.create_time(),maximum_new_SYN_runs=1,maximum_new_PNR_runs=1,
        PNR_total_seconds=0,PNR_silent_seconds=0,programmed=False,deployment_accepted=False,
        all_navigation_control_SFP_functions_retained=True)
    def save(phase,**extra):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**extra)
        (folder/'job.json').write_text(json.dumps(state,indent=2)+'\n',encoding='utf-8')
    def memory_wait(phase):
        save(phase)
        while True:
            memory=memory_status()
            if memory.avail_phys>=2*GIB and memory.avail_page>=4*GIB:return memory
            time.sleep(15)
    try:
        snapshot=reference_snapshot(prior)
        assert snapshot['total_limit_s']==snapshot['idle_limit_s']==0
        current=json.loads((prior_state/'job.json').read_text())
        identities=[]
        for pid,needle in [(snapshot['pid'],'gw_sh.exe'),(current['pnr_wrapper_pid'],'pnr_from_netlist.py'),
                           (current['controller_pid'],'continue_row_read_joint_once.py')]:
            try:
                process=psutil.Process(pid);command=process.cmdline()
                assert any(needle in item for item in command), ('prior identity mismatch',pid,command)
                if pid==snapshot['pid']:
                    assert abs(process.create_time()-snapshot['created_epoch'])<0.1
                    assert process.ppid()==current['pnr_wrapper_pid']
                if pid==current['controller_pid']:
                    assert abs(process.create_time()-current['controller_created_epoch'])<0.1
                identities.append(dict(pid=pid,created_epoch=process.create_time(),command=command))
            except psutil.NoSuchProcess:pass
        save('waiting_for_current_PNR_natural_completion_and_collection',protected_inputs=snapshot['input_metadata'],prior_processes=identities)
        while True:
            alive=[]
            for identity in identities:
                try:
                    process=psutil.Process(identity['pid'])
                    if abs(process.create_time()-identity['created_epoch'])<0.1:alive.append(process.pid)
                except psutil.NoSuchProcess:pass
            if not alive:break
            time.sleep(15)
        assert reference_snapshot(prior)['input_metadata']==snapshot['input_metadata'], 'prior inputs changed'
        choice=decision_from_result(prior)
        supported=choice['action']=='evaluate_frozen_shared_candidate' and bool(choice['evidence']['reported_unrouted_nets'])
        if supported:
            if shared and choice['evidence']['reported_unrouted_nets']<184:
                prepared=shared
            choice.update(action='evaluate_lossless_LF_candidate',
                prepared=str(prepared),
                reason='Natural unresolved routing with actual RAM within budget. Apply the proved LF-only change once to the shared origin if its actual unresolved count improved on184; otherwise use the retained184-net origin. This bounds one implementation comparison, not identification of failed-net sources.')
            state['prepared']=str(prepared)
        (folder/'prior_result_decision.json').write_text(json.dumps(choice,indent=2)+'\n',encoding='utf-8')
        # Only a natural unresolved-routing result justifies this prearranged
        # alternative. Other failures need the actual path/error to be reviewed.
        if not supported:
            save('prior_result_requires_full_or_targeted_review',decision=choice['action'],reason=choice['reason'])
            return 0
        memory=memory_wait('waiting_for_serial_joint_SYN_memory')
        command=[sys.executable,'-X','utf8',str(ROOT/'integration/g2/retry_frozen_synthesis.py'),str(prepared),'--out',str(syn),'--start-now']
        with (folder/'synthesis_runner.log').open('w',encoding='utf-8') as stream:
            child=subprocess.Popen(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
            save('one_lossless_LF_joint_SYN_preparation_and_execution',synthesis_supervisor_pid=child.pid,command=command,
                 startup_free_bytes=memory.avail_phys,startup_commit_reserve_bytes=memory.avail_page)
            child.wait()
        if child.returncode:
            syn_job=json.loads((syn/'job.json').read_text()) if (syn/'job.json').exists() else {}
            failure=(folder/'synthesis_runner.log').read_text(encoding='utf-8')
            preflight=(syn/'preflight.log').read_text(encoding='utf-8') if (syn/'preflight.log').exists() else ''
            settings_timeout=(syn_job.get('phase')=='validating_frozen_settings' and 'tool_pid' not in syn_job
                and 'TimeoutExpired' in failure and 'ERROR' not in preflight
                and (syn/'effective_options.tcl').exists() and not (syn/'impl').exists() and not (syn/'gw_sh.log').exists())
            if settings_timeout:
                # The known settings-export exit timeout is not a SYN run.
                # Preserve it and permit exactly one fresh readback attempt.
                command=command+['--resume-settings-only']
                with (folder/'synthesis_runner_settings_retry.log').open('w',encoding='utf-8') as stream:
                    child=subprocess.Popen(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
                    save('one_preserved_settings_only_retry_no_prior_SYN',synthesis_supervisor_pid=child.pid,command=command)
                    child.wait()
        assert child.returncode==0, 'joint SYN did not complete; inspect synthesis_runner.log and actual job'
        assert json.loads((syn/'job.json').read_text())['phase']=='synthesis_complete'
        save('reviewing_complete_new_netlist_clocks_CDC_and_RAM')
        shutil.copyfile(ROOT/'integration/g2/review_lf_line_joint.py',folder/'reviewer_snapshot.py')
        command=[sys.executable,'-X','utf8',str(Path(__file__).resolve()),'--prepared',str(prepared),
            '--prior-pnr',str(prior),'--prior-state',str(prior_state),'--baseline',str(baseline),
            '--syn-out',str(syn),'--pnr-out',str(out),'--state-dir',str(folder),'--check-only']
        with (folder/'validation.log').open('w',encoding='utf-8') as stream:
            subprocess.run(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,check=True,timeout=300,creationflags=subprocess.CREATE_NO_WINDOW)
        assert json.loads((folder/'pre_pnr_checks.json').read_text())['passed']
        inputs=folder/'pnr_inputs';(inputs/'impl/gwsynthesis').mkdir(parents=True);(inputs/'ip').mkdir()
        for name in ('impl/gwsynthesis/g2_top.vg','impl/gwsynthesis/g2_top.prj','effective_options.tcl','manifest.json'):
            shutil.copyfile(syn/name,inputs/name)
        for name in ('g2.cst','g2.sdc','ip/serdes.csr','cdc_reference_audit.json','sfp_cdc_reference.json'):
            shutil.copyfile(baseline/name,inputs/name)
        memory_wait('waiting_for_serial_joint_PNR_memory')
        command=[sys.executable,'-X','utf8',str(ROOT/'integration/g2/pnr_from_netlist.py'),str(inputs),'--out',str(out),
            '--wide-split','1','--replicate-resources','1','--place-option','1','--route-option','2','--timing-driven','1',
            '--max-runtime-minutes','0','--max-no-progress-minutes','0','--routing-evidence','--capture-system-telemetry']
        with (folder/'pnr_runner.log').open('w',encoding='utf-8') as stream:
            child=subprocess.Popen(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
            save('one_lossless_LF_joint_PNR_running',pnr_wrapper_pid=child.pid,command=command,pre_pnr_checks_passed=True)
            child.wait()
        result=collect(out)
        (folder/'routing_result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
        save('lossless_LF_joint_PNR_collected_full_signoff_required',wrapper_exit=child.returncode,
             reported_unrouted_nets=result['reported_unrouted_nets'],timing_report_available=result['timing_report_available'])
        return 0
    except Exception as error:
        (folder/'failure.log').write_text(traceback.format_exc(),encoding='utf-8')
        save('specific_validation_or_execution_failure',error=str(error),error_type=type(error).__name__)
        return 1


if __name__=='__main__':raise SystemExit(main())
