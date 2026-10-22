"""One-shot serial retry of frozen SYN sources after an owned PNR finishes.

Copies source inputs into a new directory before waiting. Never programs,
changes running builds, reruns physics or automatically launches another PNR.
"""
import argparse
import ctypes
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
from preflight import validate


class MemoryStatus(ctypes.Structure):
    _fields_=[('length',ctypes.c_uint32),('load',ctypes.c_uint32)]+[(n,ctypes.c_uint64)for n in
        ('total_phys','avail_phys','total_page','avail_page','total_virtual','avail_virtual','avail_extended')]


def available_memory():
    value=MemoryStatus();value.length=ctypes.sizeof(value)
    if not ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(value)):raise ctypes.WinError()
    return value.avail_phys


def main():
    ap=argparse.ArgumentParser();ap.add_argument('source',type=Path)
    ap.add_argument('--out',type=Path,required=True);ap.add_argument('--wait-pid',type=int,required=True)
    a=ap.parse_args();src=a.source.resolve();out=a.out.resolve()
    assert a.wait_pid>0 and out!=src and src not in out.parents and out not in src.parents
    manifest=json.loads((src/'manifest.json').read_text(encoding='utf-8'))
    assert manifest['stage']=='syn' and not manifest['control_weights_changed'] and not manifest['programmed']
    original=(src/'run.tcl').read_text(encoding='utf-8')
    assert [s.strip()for s in original.splitlines()if s.strip().startswith('run ')]==['run syn']
    # Preserve the cancelled directory, including partial implementation files.
    shutil.copytree(src,out,ignore=shutil.ignore_patterns('impl','gw_sh.log','preflight.log',
        'resource_cancellation.json','__pycache__','job.json'))
    def rewrite(value):
        if isinstance(value,str):return value.replace(src.as_posix(),out.as_posix()).replace(str(src),str(out))
        if isinstance(value,list):return [rewrite(v)for v in value]
        if isinstance(value,dict):return {k:rewrite(v)for k,v in value.items()}
        return value
    manifest=rewrite(manifest);manifest['frozen_source_build']=str(src)
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    settings=[rewrite(line)for line in original.splitlines()if line.strip()not in ('run syn','exit')]
    state=dict(source=str(src),build=str(out),wait_pid=a.wait_pid,programmed=False,
        source_frozen_before_wait=True,automatic_pnr=False,
        scope='one frozen SYN retry only; SYN completion is not G2/vehicle acceptance')
    def status(phase,**extra):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**extra)
        (out/'job.json').write_text(json.dumps(state,indent=2),encoding='utf-8')
        print(json.dumps(dict(phase=phase,**extra)),flush=True)
    tool=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
    status('validating_frozen_settings')
    effective=validate(tool,out,settings,50,manifest['wide_split'])
    (out/'run.tcl').write_text('\n'.join(settings+['run syn','exit'])+'\n',encoding='utf-8')
    manifest['effective_options']=effective
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    kernel.OpenProcess.argtypes=[ctypes.c_uint32,ctypes.c_int,ctypes.c_uint32];kernel.OpenProcess.restype=ctypes.c_void_p
    kernel.WaitForSingleObject.argtypes=[ctypes.c_void_p,ctypes.c_uint32]
    kernel.CloseHandle.argtypes=[ctypes.c_void_p]
    handle=kernel.OpenProcess(0x100000,False,a.wait_pid)
    if not handle:
        status('wait_process_unavailable',windows_error=ctypes.get_last_error());return 1
    status('waiting_for_prior_pnr')
    try:
        while True:
            result=kernel.WaitForSingleObject(handle,2000)
            if result==0:break
            if result!=0x102:raise ctypes.WinError(ctypes.get_last_error())
    finally:kernel.CloseHandle(handle)
    status('waiting_for_memory',minimum_start_free_bytes=2*1024**3)
    for _ in range(24):
        if available_memory()>=2*1024**3:break
        time.sleep(5)
    else:
        status('not_started_host_memory',free_bytes=available_memory());return 1
    status('synthesizing')
    with (out/'gw_sh.log').open('w',encoding='utf-8') as log:
        process=subprocess.Popen([str(tool),'run.tcl'],cwd=out,stdout=log,stderr=subprocess.STDOUT,
            creationflags=subprocess.CREATE_NO_WINDOW)
        low=0
        while process.poll()is None:
            free=available_memory();low=low+1 if free<1024**3 else 0
            if low>=2:
                process.kill();process.wait()
                status('cancelled_host_memory',free_bytes=free,tool_exit=process.returncode);return 1
            time.sleep(2)
    text=(out/'gw_sh.log').read_text(encoding='utf-8',errors='replace')
    errors=re.findall(r'^.*ERROR\s*\(.*$',text,re.M)
    if process.returncode or errors:
        status('synthesis_failed',tool_exit=process.returncode,errors=errors);return 1
    here=Path(__file__).resolve().parent
    for script,name in [('collect_evidence.py','synthesis_summary.json'),('audit_netlist_memories.py','memory_inventory.json')]:
        subprocess.run([sys.executable,'-X','utf8',str(here/script),str(out),'--out',str(out/name)],check=True)
    status('synthesis_complete',tool_exit=process.returncode,joint_pnr_measured=False)
    return 0


if __name__=='__main__':raise SystemExit(main())
