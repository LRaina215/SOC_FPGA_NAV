"""Wait for the owned SYN, review exact inputs, then run one unlimited PNR.

The original PNR15 inputs and current synthesis are never modified. No retries
or hardware action. A failed review is retained, not treated as permission.
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
from retry_frozen_synthesis import memory_status

ROOT=Path(__file__).resolve().parents[2]


def main():
    p=argparse.ArgumentParser()
    for name in ('synthesis','baseline','pnr-out','state-dir'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--wait-pid',type=int)
    p.add_argument('--check-only',action='store_true')
    a=p.parse_args()
    source,baseline,out,state_dir=map(lambda x:x.resolve(),(a.synthesis,a.baseline,a.pnr_out,a.state_dir))
    if a.check_only:
        from review_small_block_stores import review
        evidence=review(source,baseline)
        (state_dir/'pre_pnr_checks.json').write_text(json.dumps(evidence,indent=2)+'\n')
        return 0
    state_dir.mkdir(parents=True,exist_ok=False)
    assert not out.exists()
    state=dict(synthesis=str(source),baseline=str(baseline),pnr_out=str(out),
               controller_pid=psutil.Process().pid,controller_created_epoch=psutil.Process().create_time(),
               programmed=False,deployment_accepted=False,maximum_pnr_runs=1,
               max_runtime_s=0,max_no_progress_s=0)
    def save(phase,**kw):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw)
        (state_dir/'job.json').write_text(json.dumps(state,indent=2)+'\n',encoding='utf-8')
    try:
        job=json.loads((source/'job.json').read_text())
        if job['phase']!='synthesis_complete':
            assert a.wait_pid
            proc=psutil.Process(a.wait_pid);cmd=proc.cmdline()
            assert any('retry_frozen_synthesis.py' in c for c in cmd)
            assert source.as_posix() in [c.replace('\\','/') for c in cmd]
            save('waiting_for_owned_synthesis',wait_pid=a.wait_pid,wait_created_epoch=proc.create_time(),wait_command=cmd)
            proc.wait()
        save('reviewing_full_candidate')
        shutil.copyfile(ROOT/'integration/g2/review_small_block_stores.py',state_dir/'reviewer_snapshot.py')
        command=[sys.executable,'-X','utf8',str(Path(__file__).resolve()),'--synthesis',str(source),
                 '--baseline',str(baseline),'--pnr-out',str(out),'--state-dir',str(state_dir),'--check-only']
        with (state_dir/'validation.log').open('w',encoding='utf-8') as log:
            subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True,
                           cwd=ROOT,creationflags=subprocess.CREATE_NO_WINDOW,timeout=300)
        assert json.loads((state_dir/'pre_pnr_checks.json').read_text())['passed']
        # Constraint reuse is authorized only after proving every named driver.
        prepared=state_dir/'pnr_inputs';(prepared/'impl/gwsynthesis').mkdir(parents=True)
        (prepared/'ip').mkdir()
        for name in ('impl/gwsynthesis/g2_top.vg','impl/gwsynthesis/g2_top.prj','effective_options.tcl','manifest.json'):
            shutil.copyfile(source/name,prepared/name)
        for name in ('g2.cst','g2.sdc','ip/serdes.csr','cdc_reference_audit.json','sfp_cdc_reference.json'):
            shutil.copyfile(baseline/name,prepared/name)
        save('waiting_for_memory',pre_pnr_checks_passed=True,prepared_inputs=str(prepared))
        for _ in range(24):
            m=memory_status()
            if m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3:break
            time.sleep(5)
        else:
            save('not_started_host_memory',free_bytes=m.avail_phys,available_commit_bytes=m.avail_page);return 1
        command=[sys.executable,'-X','utf8',str(ROOT/'integration/g2/pnr_from_netlist.py'),str(prepared),
                 '--out',str(out),'--wide-split','1','--replicate-resources','1','--place-option','1',
                 '--route-option','2','--timing-driven','1','--max-runtime-minutes','0','--max-no-progress-minutes','0',
                 '--routing-evidence','--capture-system-telemetry']
        with (state_dir/'pnr_runner.log').open('w',encoding='utf-8') as log:
            child=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,
                                   creationflags=subprocess.CREATE_NO_WINDOW)
            save('one_full_pnr_running',command=command,pnr_wrapper_pid=child.pid,
                 startup_free_bytes=m.avail_phys,startup_commit_reserve_bytes=m.avail_page)
            child.wait()
        from collect_routing_diagnostic import collect
        result=collect(out)
        (state_dir/'routing_result.json').write_text(json.dumps(result,indent=2)+'\n')
        save('pnr_collected_full_review_required',wrapper_exit=child.returncode,
             reported_unrouted_nets=result['reported_unrouted_nets'],runtime=result['runtime'],
             timing_report_available=result['timing_report_available'])
        return 0
    except Exception as error:
        (state_dir/'failure.log').write_text(traceback.format_exc(),encoding='utf-8')
        save('prerequisite_or_execution_failed',error_type=type(error).__name__,error=str(error))
        return 1


if __name__=='__main__':raise SystemExit(main())
