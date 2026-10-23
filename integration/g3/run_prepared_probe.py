"""One dependent diagnostic build, with finite wait and no hardware access."""
import argparse,ctypes,json,sys,time
from datetime import datetime
from pathlib import Path
import psutil
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'integration/g2'))
from bounded_tool import run_tool


class MemoryStatus(ctypes.Structure):
    _fields_=[('length',ctypes.c_uint32),('load',ctypes.c_uint32)]+[(n,ctypes.c_uint64)for n in
        ('total_phys','avail_phys','total_page','avail_page','total_virtual','avail_virtual','avail_extended')]


def main():
    p=argparse.ArgumentParser();p.add_argument('build',type=Path);p.add_argument('--after-runtime',type=Path,required=True)
    p.add_argument('--max-wait-minutes',type=float,default=95);a=p.parse_args()
    if not 0<a.max_wait_minutes<=100:p.error('finite wait must be in (0,100] minutes')
    out=a.build.resolve();dependency=a.after_runtime.resolve()
    if(out/'impl').exists()or(out/'job.json').exists():p.error('existing job/implementation; preserve it and prepare a fresh build')
    manifest=json.loads((out/'manifest.json').read_text())
    for source in manifest['sources']:assert(out/Path(source).name).is_file(),source
    assert(out/'run.tcl').is_file()
    state=dict(build=str(out),dependency=str(dependency),programmed=False,automatic_programming=False,
        source_frozen_before_wait=True,scope='one bounded diagnostic build after existing G2 exits, no recurring supervision')
    def save(phase,**values):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**values)
        (out/'job.json').write_text(json.dumps(state,indent=2),encoding='utf-8')
    started=time.monotonic()
    while True:
        prior=json.loads(dependency.read_text())
        pid=prior.get('pid');ended='ended_at'in prior and 'tool_exit'in prior
        still_owned=False
        if pid and psutil.pid_exists(pid):
            try:
                created=psutil.Process(pid).create_time();expected=datetime.fromisoformat(prior['started_at']).timestamp()
                still_owned=abs(created-expected)<30
            except psutil.NoSuchProcess:pass
        memory=MemoryStatus();memory.length=ctypes.sizeof(memory)
        if not ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(memory)):raise ctypes.WinError()
        if ended and not still_owned and memory.avail_phys>=2*2**30 and memory.avail_page>=4*2**30:break
        if time.monotonic()-started>=a.max_wait_minutes*60:
            save('wait_budget_exceeded',dependency_ended=ended,dependency_process_alive=still_owned);return 2
        save('waiting_for_dependency_or_memory',dependency_ended=ended,dependency_process_alive=still_owned,
            available_physical_bytes=memory.avail_phys,available_commit_bytes=memory.avail_page)
        time.sleep(15)
    save('building',available_physical_bytes=memory.avail_phys,available_commit_bytes=memory.avail_page)
    result=run_tool(['D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe','run.tcl'],out,
        max_runtime_s=300,max_no_progress_s=180,capture_system_telemetry=True)
    (out/'build_result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    success=result['tool_exit']==0 and not result['cancelled']
    save('tool_complete_needs_report_review'if success else 'build_failed',tool_exit=result['tool_exit'],cancelled=result['cancelled'])
    return 0 if success else 2


if __name__=='__main__':raise SystemExit(main())
