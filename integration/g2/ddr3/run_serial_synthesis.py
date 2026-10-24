"""Wait for the identified PNR collector, then run exactly one DDR full SYN.

No PNR, no programming, no cancellation or modification of the waited job.
The existing SYN runner retains its 40-minute and 2/4 GiB resource gates.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
import time
import psutil


def main():
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--state-dir',type=Path,required=True);p.add_argument('--wait-pid',type=int,required=True)
    p.add_argument('--wait-pnr',type=Path,required=True);a=p.parse_args()
    out=a.out.resolve();state_dir=a.state_dir.resolve();source=a.source.resolve();prior=a.wait_pnr.resolve()
    assert not out.exists() and not state_dir.exists()
    prepared=json.loads((source/'ddr_preparation.json').read_text())
    assert prepared['all_control_sources_unchanged'] and not prepared['automatic_pnr']
    process=psutil.Process(a.wait_pid)
    identity=dict(pid=process.pid,created_at=process.create_time(),command=process.cmdline())
    assert any(prior.name in x for x in identity['command']),identity
    assert (prior/'tool_runtime.json').is_file()
    state_dir.mkdir(parents=True)
    state=dict(controller_pid=psutil.Process().pid,started_at=datetime.now().astimezone().isoformat(),
        source=str(source),synthesis=str(out),wait_pnr=str(prior),wait_identity=identity,
        maximum_synthesis_runs=1,maximum_pnr_runs=0,automatic_pnr=False,programmed=False,
        waited_pnr_termination_deadline=None,waited_pnr_untouched=True)
    def save(phase,**kw):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw)
        (state_dir/'job.json').write_text(json.dumps(state,indent=2),encoding='utf-8')
    save('waiting_for_pnr15_and_collection')
    while True:
        try:
            q=psutil.Process(a.wait_pid)
            if q.create_time()!=identity['created_at'] or not q.is_running():break
        except psutil.NoSuchProcess:break
        save('waiting_for_pnr15_and_collection');time.sleep(10)
    runtime=json.loads((prior/'tool_runtime.json').read_text())
    if 'tool_exit' not in runtime:
        save('prior_runtime_needs_review',runtime=runtime);return 1
    runner=Path(__file__).resolve().parent.parent/'retry_frozen_synthesis.py'
    command=[sys.executable,'-X','utf8',str(runner),str(source),'--out',str(out),'--start-now']
    save('starting_unique_serial_synthesis',command=command)
    with (state_dir/'synthesis_wrapper.log').open('w',encoding='utf-8') as log:
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
        save('serial_synthesis_wrapper_running',synthesis_wrapper_pid=child.pid)
        rc=child.wait()
    actual=json.loads((out/'job.json').read_text()) if (out/'job.json').is_file() else None
    save('synthesis_finished_pending_review' if rc==0 else 'synthesis_failed_needs_review',
        wrapper_exit=rc,synthesis_job=actual,automatic_pnr=False)
    return rc


if __name__=='__main__':raise SystemExit(main())
