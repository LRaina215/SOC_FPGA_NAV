"""After an owned runtime exits naturally, diagnose its unpublished-in-log phase."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
import time
import os

import psutil


def run(args):
    assert not args.job.exists() and not args.out.exists()
    args.job.mkdir(parents=True)
    owner=psutil.Process(args.wait_pid)
    assert abs(owner.create_time()-datetime.fromisoformat(args.wait_created).timestamp())<0.01
    assert 'probe_actual_phy.py' in ' '.join(owner.cmdline())
    identities=[dict(pid=p.pid,created=p.create_time()) for p in [owner,*owner.children(recursive=True)]]
    state=dict(pid=os.getpid(),created=datetime.now().astimezone().isoformat(),
               prior=str(args.prior),tracked_original_processes=identities,out=str(args.out),
               stops_no_process=True,starts_no_joint_build=True,programmed=False)
    def status(phase,**extra):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**extra)
        (args.job/'job.json').write_text(json.dumps(state,indent=2))
        print(json.dumps(dict(phase=phase,**extra)),flush=True)
    status('waiting_for_original_runtime_natural_exit')
    while True:
        alive=[]
        for row in identities:
            try:
                p=psutil.Process(row['pid'])
                if abs(p.create_time()-row['created'])<0.01:alive.append(p.pid)
            except psutil.NoSuchProcess:pass
        if not alive:break
        time.sleep(5)
    result=json.loads((args.prior/'result.json').read_text())
    assert result.get('completed_at'),'original final report absent'
    if result.get('calibration_observed') and result.get('functional_readback_passed'):
        status('diagnostic_skipped_original_calibrated_and_readback_passed');return
    remaining=args.probe_args
    if remaining and remaining[0]=='--':remaining=remaining[1:]
    command=[sys.executable,'-X','utf8',str(Path(__file__).with_name('probe_actual_phy.py')),
             '--out',str(args.out),'--observe-public-stages','--calibration-snapshot-samples','16',*remaining]
    with (args.job/'diagnostic_wrapper.log').open('w') as log:
        p=subprocess.Popen(command,cwd=Path(__file__).resolve().parents[3],
            stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
        status('public_stage_snapshot_running',wrapper_pid=p.pid,command=command)
        code=p.wait()
    status('public_stage_snapshot_completed_requires_analysis',wrapper_exit=code,
           result=str(args.out/'result.json'),is_not_service_or_5ms_acceptance=True)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for n in ('prior','job','out'):p.add_argument('--'+n,type=Path,required=True)
    p.add_argument('--wait-pid',type=int,required=True);p.add_argument('--wait-created',required=True)
    p.add_argument('probe_args',nargs=argparse.REMAINDER)
    run(p.parse_args())
