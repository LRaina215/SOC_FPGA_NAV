"""Bound one owned tool process; CPU activity is never progress evidence."""
from datetime import datetime
import json
from pathlib import Path
import subprocess
import time


def run_tool(argv,cwd,log_name='gw_sh.log',max_runtime_s=0,max_no_progress_s=0,poll_s=2,capture_system_telemetry=False):
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
    if capture_system_telemetry:
        import psutil
        import ctypes
        class MemoryStatus(ctypes.Structure):
            _fields_=[('length',ctypes.c_uint32),('load',ctypes.c_uint32)]+[(n,ctypes.c_uint64)for n in
                ('total_phys','avail_phys','total_page','avail_page','total_virtual','avail_virtual','avail_extended')]
        state['system_telemetry_file']='system_telemetry.jsonl'
        state['telemetry_is_progress_evidence']=False
    def record_system():
        if not capture_system_telemetry:return
        sample=dict(recorded_at=datetime.now().astimezone().isoformat(),pid=process.pid,
            elapsed_s=time.monotonic()-started,no_observable_progress_s=state.get('no_observable_progress_s'))
        try:
            owned=psutil.Process(process.pid)
            sample.update(cpu_seconds=owned.cpu_times()._asdict(),threads=owned.num_threads(),
                memory=owned.memory_info()._asdict(),io=owned.io_counters()._asdict())
            memory=MemoryStatus();memory.length=ctypes.sizeof(memory)
            if not ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(memory)):raise ctypes.WinError()
            sample['host_memory']=dict(available_physical_bytes=memory.avail_phys,total_physical_bytes=memory.total_phys,
                available_commit_bytes=memory.avail_page,commit_limit_bytes=memory.total_page)
        except Exception as error:
            sample['collection_error']=str(error)
        # This diagnostic file is deliberately absent from signature(). CPU,
        # I/O and memory samples never extend the observable-progress budget.
        with (cwd/'system_telemetry.jsonl').open('a',encoding='utf-8')as file:
            file.write(json.dumps(sample)+'\n')
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
                    record_system()
                    (cwd/'tool_runtime.json').write_text(json.dumps(state,indent=2),encoding='utf-8');last_status=now
                time.sleep(poll_s)
        except BaseException:
            if process.poll()is None:process.kill();process.wait()
            state.update(cancelled=True,reason='supervisor interrupted')
            raise
        finally:
            record_system()
            state.update(tool_exit=process.poll(),elapsed_s=time.monotonic()-started,
                ended_at=datetime.now().astimezone().isoformat())
            (cwd/'tool_runtime.json').write_text(json.dumps(state,indent=2),encoding='utf-8')
            if state['cancelled']:
                (cwd/'resource_cancellation.json').write_text(json.dumps(state,indent=2),encoding='utf-8')
    return state
