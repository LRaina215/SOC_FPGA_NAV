"""Wait for one existing SYN, review its outputs, then launch one unlimited PNR."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import psutil

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from retry_frozen_synthesis import memory_status


def run(a):
    assert not a.job_dir.exists() and not a.inputs.exists() and not a.out.exists()
    a.job_dir.mkdir(parents=True)
    owner=psutil.Process(a.watch_pid)
    assert abs(owner.create_time()-datetime.fromisoformat(a.watch_created).timestamp())<0.01
    command=' '.join(owner.cmdline())
    assert 'retry_frozen_synthesis.py' in command and str(a.source).replace('\\','/') in command.replace('\\','/')
    state=dict(controller_pid=os.getpid(),controller_created_at=datetime.now().astimezone().isoformat(),
        source=str(a.source),watched_SYN_pid=a.watch_pid,watched_SYN_created_at=a.watch_created,
        watched_SYN_command=owner.cmdline(),inputs=str(a.inputs),pnr=str(a.out),
        programmed=False,total_PNR_limit_s=0,idle_PNR_limit_s=0,starts_new_synthesis=False)
    def status(phase,**extra):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**extra)
        (a.job_dir/'job.json').write_text(json.dumps(state,indent=2))
        print(json.dumps(dict(phase=phase,**extra)),flush=True)
    def checked_run(arguments,log_name):
        with (a.job_dir/log_name).open('w') as log:
            child=subprocess.Popen(arguments,cwd=HERE.parents[2],stdout=log,stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW)
            status('validating_new_netlist' if log_name=='validation.log' else 'pnr_running',
                   child_pid=child.pid,child_command=arguments)
            code=child.wait()
        return code
    status('waiting_for_owned_SYN')
    try:
        while owner.is_running() and abs(owner.create_time()-datetime.fromisoformat(a.watch_created).timestamp())<0.01:
            time.sleep(2)
    except psutil.NoSuchProcess:pass
    job=json.loads((a.source/'job.json').read_text())
    if job['phase']!='synthesis_complete' or job.get('tool_exit')!=0:
        status('source_SYN_not_passed',source_job=job);return 1
    arguments=[sys.executable,'-X','utf8',str(HERE/'review_map_joint.py')]
    for name in ('source','baseline','reference','phy','inputs'):
        arguments+=['--out' if name=='inputs' else '--'+name,str(getattr(a,name))]
    code=checked_run(arguments,'validation.log')
    if code:
        status('pre_pnr_validation_failed',validation_exit=code,validation_log=str(a.job_dir/'validation.log'));return code
    review=json.loads((a.inputs/'pre_pnr_review.json').read_text())
    assert review['passed'] and review['new_DDR_timing_exceptions']==0
    status('waiting_for_start_memory',pre_pnr_checks=str(a.inputs/'pre_pnr_review.json'))
    while True:
        m=memory_status()
        if m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3:break
        time.sleep(5)
    state['start_memory_gate']=dict(available_physical_bytes=m.avail_phys,available_commit_bytes=m.avail_page)
    arguments=[sys.executable,'-X','utf8',str(HERE.parent/'pnr_from_netlist.py'),str(a.inputs),
        '--out',str(a.out),'--wide-split','1','--replicate-resources','1',
        '--place-option','1','--route-option','2','--timing-driven','1',
        '--max-runtime-minutes','0','--max-no-progress-minutes','0',
        '--routing-evidence','--capture-system-telemetry']
    code=checked_run(arguments,'pnr_wrapper.log')
    status('pnr_collected_requires_full_review',pnr_wrapper_exit=code,
           runtime=str(a.out/'tool_runtime.json'),acceptance_not_inferred=True)
    return code


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for key in ('source','baseline','reference','phy','inputs','out','job-dir'):
        p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--watch-pid',type=int,required=True)
    p.add_argument('--watch-created',required=True)
    a=p.parse_args()
    for key in ('source','baseline','reference','phy','inputs','out','job_dir'):
        setattr(a,key,getattr(a,key).resolve())
    raise SystemExit(run(a))
