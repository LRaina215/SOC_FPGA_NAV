"""Bound one owned tool process; CPU activity is never progress evidence."""
from datetime import datetime
import json
from pathlib import Path
import subprocess
import time


def run_tool(argv,cwd,log_name='gw_sh.log',max_runtime_s=0,max_no_progress_s=0,poll_s=2):
    cwd=Path(cwd);log=cwd/log_name
    observed=[log]+[cwd/'impl/pnr'/('g2_top'+suffix)for suffix in ('.log','.db','.p','.pr','.tr','.rpt.txt','.vo')]
    def signature():
        result=[]
        for path in observed:
            try:stat=path.stat()
            except FileNotFoundError:continue
            result.append((str(path.relative_to(cwd)),stat.st_size,stat.st_mtime_ns))
        return result
    started=time.monotonic();last_progress=started;last_signature=None;last_status=0
    state=dict(command=argv,started_at=datetime.now().astimezone().isoformat(),
        max_runtime_s=max_runtime_s,max_no_observable_progress_s=max_no_progress_s,
        cancelled=False,programmed=False,scope='process runtime/file-progress budget, not proof of algorithm deadlock or hardware acceptance')
    with log.open('w',encoding='utf-8')as stream:
        process=subprocess.Popen(argv,cwd=cwd,stdout=stream,stderr=subprocess.STDOUT,
            creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        state['pid']=process.pid
        try:
            while process.poll()is None:
                now=time.monotonic();current=signature()
                if current!=last_signature:last_progress=now;last_signature=current
                elapsed=now-started;idle=now-last_progress
                reason='maximum runtime exceeded'if max_runtime_s and elapsed>=max_runtime_s else (
                    'no observable log/artifact progress within budget'if max_no_progress_s and idle>=max_no_progress_s else None)
                state.update(elapsed_s=elapsed,no_observable_progress_s=idle,observed_files=current)
                if reason:
                    state.update(cancelled=True,reason=reason)
                    process.terminate()
                    try:process.wait(timeout=10)
                    except subprocess.TimeoutExpired:process.kill();process.wait()
                    break
                if now-last_status>=30:
                    (cwd/'tool_runtime.json').write_text(json.dumps(state,indent=2),encoding='utf-8');last_status=now
                time.sleep(poll_s)
        except BaseException:
            if process.poll()is None:process.kill();process.wait()
            state.update(cancelled=True,reason='supervisor interrupted')
            raise
        finally:
            state.update(tool_exit=process.poll(),elapsed_s=time.monotonic()-started,
                ended_at=datetime.now().astimezone().isoformat())
            (cwd/'tool_runtime.json').write_text(json.dumps(state,indent=2),encoding='utf-8')
            if state['cancelled']:
                (cwd/'resource_cancellation.json').write_text(json.dumps(state,indent=2),encoding='utf-8')
    return state
