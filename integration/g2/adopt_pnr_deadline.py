"""Extend one running PNR without restarting it; retain an absolute deadline.

The original Python wrapper is suspended while this supervisor owns the budget,
then resumed after the tool exits so its original report collector can finish.
"""
import argparse,contextlib,ctypes,json,os,time,traceback
from ctypes import wintypes
from datetime import datetime
from pathlib import Path
import psutil


class MemoryStatus(ctypes.Structure):
    _fields_=[('length',ctypes.c_uint32),('load',ctypes.c_uint32)]+[(n,ctypes.c_uint64)for n in
        ('total_phys','avail_phys','total_page','avail_page','total_virtual','avail_virtual','avail_extended')]


def supervise(build,deadline,*,require_gowin=True,poll_s=2,record_s=30):
    build=Path(build).resolve();runtime=build/'tool_runtime.json'
    previous=json.loads(runtime.read_text());started=datetime.fromisoformat(previous['started_at'])
    if deadline.tzinfo is None or deadline<=datetime.now().astimezone():raise ValueError('future timezone-aware deadline required')
    if previous.get('ended_at'):raise ValueError('tool already finished; no restart permitted')
    tool=psutil.Process(previous['pid']);wrapper=tool.parent()
    if abs(tool.create_time()-started.timestamp())>30:raise ValueError('tool PID identity changed')
    wrapper_created=wrapper.create_time();tool_created=tool.create_time()
    if require_gowin:
        if tool.name().lower()!='gw_sh.exe':raise ValueError('not the recorded Gowin process')
        command=wrapper.cmdline()
        if not any(x.replace('\\','/').endswith('integration/g2/pnr_from_netlist.py')for x in command):raise ValueError('unexpected wrapper')
        if '--out'not in command or Path(command[command.index('--out')+1]).resolve()!=build:raise ValueError('wrapper belongs to another build')
        if Path(tool.cwd()).resolve()!=build:raise ValueError('tool belongs to another build directory')
    k=ctypes.WinDLL('kernel32',use_last_error=True)
    k.OpenProcess.argtypes=[wintypes.DWORD,wintypes.BOOL,wintypes.DWORD];k.OpenProcess.restype=wintypes.HANDLE
    k.WaitForSingleObject.argtypes=[wintypes.HANDLE,wintypes.DWORD];k.WaitForSingleObject.restype=wintypes.DWORD
    k.GetExitCodeProcess.argtypes=[wintypes.HANDLE,ctypes.POINTER(wintypes.DWORD)]
    k.TerminateProcess.argtypes=[wintypes.HANDLE,wintypes.UINT];k.CloseHandle.argtypes=[wintypes.HANDLE]
    handle=k.OpenProcess(0x100000|0x1000|0x0001,False,tool.pid)
    if not handle:raise ctypes.WinError(ctypes.get_last_error())
    observed=[build/'gw_sh.log']+[build/'impl/pnr'/('g2_top'+s)for s in('.log','.db','.p','.pr','.tr','.rpt.txt','.vo')]
    def signature():
        result=[]
        for file in observed:
            try:s=file.stat()
            except FileNotFoundError:continue
            result.append([str(file.relative_to(build)),s.st_size,s.st_mtime_ns])
        return result
    state=previous.copy();last_signature=previous.get('observed_files',[])
    last_progress=started.timestamp()+previous.get('elapsed_s',0)-previous.get('no_observable_progress_s',0)
    extension=dict(recorded_at=datetime.now().astimezone().isoformat(),deadline=deadline.isoformat(),
        authorized_reason='human explicitly extended this same running PNR to the recorded absolute deadline and disabled the idle timeout',
        original_wrapper_pid=wrapper.pid,original_wrapper_created=wrapper_created,tool_pid=tool.pid,tool_created=tool_created,
        supervisor_pid=os.getpid(),previous_runtime=previous,tool_restarted=False,automatic_programming=False,
        scope='one absolute-deadline extension; CPU/telemetry never count as routing progress')
    extension_file=build/'deadline_extension.json'
    if extension_file.exists():raise ValueError('extension already recorded; inspect existing supervisor')
    state.update(max_runtime_s=(deadline-started).total_seconds(),max_no_observable_progress_s=0,
        absolute_deadline=deadline.isoformat(),deadline_supervisor_pid=os.getpid(),cancelled=False,
        runtime_budget_source='deadline_extension.json; original wrapper suspended until tool exits')
    paused=False;last_record=0;tool_exit=None;failure=None
    def save():
        state.update(elapsed_s=time.time()-started.timestamp(),no_observable_progress_s=max(0,time.time()-last_progress),observed_files=last_signature)
        runtime.write_text(json.dumps(state,indent=2),encoding='utf-8')
        sample=dict(recorded_at=datetime.now().astimezone().isoformat(),pid=tool.pid,elapsed_s=state['elapsed_s'],
            no_observable_progress_s=state['no_observable_progress_s'],deadline_supervisor_pid=os.getpid())
        try:
            sample.update(cpu_seconds=tool.cpu_times()._asdict(),threads=tool.num_threads(),memory=tool.memory_info()._asdict(),io=tool.io_counters()._asdict())
            m=MemoryStatus();m.length=ctypes.sizeof(m)
            if not k.GlobalMemoryStatusEx(ctypes.byref(m)):raise ctypes.WinError()
            sample['host_memory']=dict(available_physical_bytes=m.avail_phys,total_physical_bytes=m.total_phys,
                available_commit_bytes=m.avail_page,commit_limit_bytes=m.total_page)
        except Exception as error:sample['collection_error']=str(error)
        with(build/'system_telemetry.jsonl').open('a',encoding='utf-8')as file:file.write(json.dumps(sample)+'\n')
    try:
        if k.WaitForSingleObject(handle,0)==0:raise ValueError('tool finished before takeover')
        wrapper.suspend();paused=True
        extension.update(phase='supervising_same_tool',wrapper_suspended=True)
        extension_file.write_text(json.dumps(extension,indent=2),encoding='utf-8')
        while k.WaitForSingleObject(handle,0)==258:
            now=time.time();current=signature()
            if current!=last_signature:last_signature=current;last_progress=now
            if now>=deadline.timestamp():
                state.update(cancelled=True,reason='human-authorized absolute deadline reached')
                if not k.TerminateProcess(handle,124):raise ctypes.WinError(ctypes.get_last_error())
                k.WaitForSingleObject(handle,10000)
                break
            if time.monotonic()-last_record>=record_s:save();last_record=time.monotonic()
            time.sleep(poll_s)
        exit_code=wintypes.DWORD()
        if not k.GetExitCodeProcess(handle,ctypes.byref(exit_code)):raise ctypes.WinError(ctypes.get_last_error())
        tool_exit=exit_code.value
    except BaseException as error:
        failure=repr(error);raise
    finally:
        if paused:
            try:
                if wrapper.is_running()and wrapper.create_time()==wrapper_created:wrapper.resume()
            except psutil.NoSuchProcess:pass
        k.CloseHandle(handle)
        if tool_exit is not None:
            # Let the original wrapper write its vendor exit/report result first.
            try:wrapper.wait(timeout=120)
            except psutil.TimeoutExpired:state['original_wrapper_collection_pending']=True
            state.update(tool_exit=tool_exit,ended_at=datetime.now().astimezone().isoformat())
            save()
            if state['cancelled']:(build/'resource_cancellation.json').write_text(json.dumps(state,indent=2),encoding='utf-8')
        extension.update(phase='failed_original_wrapper_resumed'if failure else 'tool_finished',wrapper_suspended=False,
            finished_at=datetime.now().astimezone().isoformat(),tool_exit=tool_exit,error=failure)
        extension_file.write_text(json.dumps(extension,indent=2),encoding='utf-8')
    return state


def main():
    p=argparse.ArgumentParser();p.add_argument('build',type=Path);p.add_argument('--deadline',required=True);a=p.parse_args()
    with(a.build/'deadline_supervisor_stdout.log').open('a',encoding='utf-8')as log,contextlib.redirect_stdout(log),contextlib.redirect_stderr(log):
        try:
            state=supervise(a.build,datetime.fromisoformat(a.deadline));print(json.dumps(state))
            return 0 if state['tool_exit']==0 and not state['cancelled']else 2
        except Exception:traceback.print_exc();return 2


if __name__=='__main__':raise SystemExit(main())
