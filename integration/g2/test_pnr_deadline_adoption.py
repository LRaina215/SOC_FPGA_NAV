"""Exercise supervisor handoff with disposable sleeping processes, never FPGA."""
import argparse,json,subprocess,sys,time
from datetime import datetime,timedelta
from pathlib import Path
from adopt_pnr_deadline import supervise
WRAPPER='''import json,subprocess,sys,time
from pathlib import Path
from datetime import datetime
root=Path(sys.argv[1]);p=subprocess.Popen([sys.executable,'-c','import time;time.sleep('+sys.argv[2]+')'])
state=dict(pid=p.pid,started_at=datetime.now().astimezone().isoformat(),max_runtime_s=2,max_no_observable_progress_s=2,cancelled=False,programmed=False,elapsed_s=0,no_observable_progress_s=0,observed_files=[])
(root/'tool_runtime.json').write_text(json.dumps(state))
started=time.monotonic()
while p.poll() is None:
 if time.monotonic()-started>=2:p.terminate();p.wait();break
 time.sleep(.02)
(root/'wrapper_finished').write_text(str(p.returncode))
'''


def main():
    a=argparse.ArgumentParser();a.add_argument('--out',type=Path,required=True);args=a.parse_args()
    root=args.out.resolve();root.mkdir(parents=True,exist_ok=False);results=[]
    for name,duration,deadline_s,cancelled,expected_exit in [('natural',1.5,8,False,0),('deadline',60,1.5,True,124),('no_deadline',3.2,None,False,0)]:
        out=root/name;out.mkdir();script=out/'wrapper.py';script.write_text(WRAPPER)
        wrapper=subprocess.Popen([sys.executable,str(script),str(out),str(duration)],creationflags=subprocess.CREATE_NO_WINDOW)
        try:
            until=time.monotonic()+10
            while not(out/'tool_runtime.json').exists():
                assert time.monotonic()<until,'fixture startup timeout';time.sleep(.02)
            previous=json.loads((out/'tool_runtime.json').read_text())
            deadline=None if deadline_s is None else datetime.now().astimezone()+timedelta(seconds=deadline_s)
            state=supervise(out,deadline,require_gowin=False,poll_s=.02,record_s=.1)
            assert state['pid']==previous['pid']and state['cancelled']==cancelled and state['tool_exit']==expected_exit,state
            assert(out/'wrapper_finished').is_file(),'original wrapper did not resume'
            assert wrapper.wait(timeout=5)==0
            if deadline is None:
                assert state['max_runtime_s']==state['max_no_observable_progress_s']==0 and state['absolute_deadline'] is None
                assert state['elapsed_s']>2, 'fixture must exceed its original wrapper deadline'
            results.append(dict(case=name,pid_preserved=True,wrapper_resumed=True,cancelled=state['cancelled'],exit=state['tool_exit']))
        finally:
            if wrapper.poll()is None:wrapper.kill();wrapper.wait()
    result=dict(passed=True,cases=results,scope='disposable process lifecycle contract; no Gowin run, app-close guarantee or hardware test')
    (root/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))


if __name__=='__main__':main()
