"""One admitted, bounded joint SYN; never auto-start PNR or change board image."""
import argparse,json,re,shutil,subprocess,sys,time
from datetime import datetime
from pathlib import Path
import psutil
from retry_frozen_synthesis import memory_status
from preflight import validate

HERE=Path(__file__).resolve().parent
TOOL=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')

def run(source,out,proof):
    assert not out.exists() and source.is_dir()
    p=json.loads((proof/'result.json').read_text(encoding='utf-8'))
    assert p['passed'] and Path(p['source']).resolve()==source.resolve()
    assert [c['unit']for c in p['checks']]==['framebuffer','hud','pid']
    # The proof is bound to the actual tested source files; do not silently use
    # an edited RTL file after the probe. These are raw byte comparisons, no hash.
    for rel,probe_file in (
        ('video/g2_video_framebuffer.v',proof/'source/g2_video_framebuffer.v'),
        ('video/g2_video_hud.v',proof/'source/g2_video_hud.v'),
        ('control/motor520_closed_loop.v',proof/'source/motor520_closed_loop.v')):
        assert (source/rel).read_bytes()==probe_file.read_bytes(),rel
    shutil.copytree(source,out)
    tcl=(out/'run.tcl').read_text(encoding='utf-8').replace(source.as_posix(),out.as_posix())
    assert [s.strip()for s in tcl.splitlines()if s.strip().startswith('run ')]==['run syn']
    for file in out.rglob('*.v'):
        text=file.read_text(encoding='utf-8-sig')
        if source.as_posix()in text:file.write_text(text.replace(source.as_posix(),out.as_posix()),encoding='utf-8',newline='\n')
    (out/'run.tcl').write_text(tcl,encoding='utf-8',newline='\n')
    manifest=json.loads((out/'video_candidate_manifest.json').read_text(encoding='utf-8'))
    manifest.update(output=str(out),stage='syn',source_frozen_from=str(source),unit_probe=str(proof),
        control_weights_changed=False,programmed=False,wide_split=1)
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    me=psutil.Process()
    state=dict(source=str(source),build=str(out),controller_pid=me.pid,controller_created_epoch=me.create_time(),
        automatic_PNR=False,programmed=False,maximum_SYN_runtime_s=2400,
        minimum_start_physical_bytes=2*1024**3,minimum_start_commit_bytes=4*1024**3)
    def status(phase,**values):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**values)
        (out/'job.json').write_text(json.dumps(state,indent=2),encoding='utf-8')
        print(json.dumps(dict(phase=phase,**values)),flush=True)
    status('waiting_for_actual_memory_admission')
    while True:
        m=memory_status()
        if m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3:break
        status('waiting_for_actual_memory_admission',physical_available=m.avail_phys,commit_available=m.avail_page)
        time.sleep(30)
    status('settings_preflight',physical_available=m.avail_phys,commit_available=m.avail_page)
    settings=[line for line in tcl.splitlines()if line.strip()not in ('run syn','exit')]
    state['effective_options']=validate(TOOL,out,settings,50,1)
    m=memory_status()
    if m.avail_phys<2*1024**3 or m.avail_page<4*1024**3:
        status('not_started_memory_changed');return 1
    started=time.monotonic();low=0
    with(out/'gw_sh.log').open('w',encoding='utf-8')as log:
        process=subprocess.Popen([str(TOOL),'run.tcl'],cwd=out,stdout=log,stderr=subprocess.STDOUT,
            creationflags=subprocess.CREATE_NO_WINDOW)
        identity=psutil.Process(process.pid);identity.nice(psutil.NORMAL_PRIORITY_CLASS)
        status('synthesizing',tool_pid=process.pid,tool_created_epoch=identity.create_time(),
            tool_command=identity.cmdline(),actual_affinity=identity.cpu_affinity(),
            synthesis_started_at=datetime.now().astimezone().isoformat())
        while process.poll()is None:
            m=memory_status();elapsed=time.monotonic()-started
            low=low+1 if m.avail_page<512*1024**2 or (m.avail_phys<256*1024**2 and m.avail_page<1024**3) else 0
            if low>=2 or elapsed>=2400:
                for child in reversed(identity.children(recursive=True)):
                    try:child.kill()
                    except psutil.NoSuchProcess:pass
                process.kill();process.wait()
                status('cancelled_host_memory'if low>=2 else 'cancelled_SYN_runtime',tool_exit=process.returncode)
                return 1
            if int(elapsed)%30<2:
                status('synthesizing',elapsed_s=elapsed,physical_available=m.avail_phys,commit_available=m.avail_page)
            time.sleep(2)
    text=(out/'gw_sh.log').read_text(encoding='utf-8',errors='replace')
    errors=re.findall(r'^.*ERROR\s*\(.*$',text,re.M)
    if process.returncode or errors:
        status('synthesis_failed',tool_exit=process.returncode,errors=errors);return 1
    status('collecting_actual_synthesis_reports',tool_exit=process.returncode)
    for script,name in [('collect_evidence.py','synthesis_summary.json'),('audit_netlist_memories.py','memory_inventory.json')]:
        with(out/(name+'.stdout.log')).open('w',encoding='utf-8')as log:
            subprocess.run([sys.executable,'-B','-X','utf8',str(HERE/script),str(out),'--out',str(out/name)],
                check=True,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
    status('synthesis_complete_actual_netlist_review_pending',tool_exit=process.returncode)
    return 0

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--source',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True);ap.add_argument('--proof',type=Path,required=True)
    a=ap.parse_args();raise SystemExit(run(a.source.resolve(),a.out.resolve(),a.proof.resolve()))
