"""Explicit finite RTL contracts for frozen stage2B BSRAM migrations; no SYN/PNR."""
from pathlib import Path
from datetime import datetime
import argparse,json,shutil,subprocess,time,psutil
from prepare_baseline_fusion_retime import metadata
from retry_frozen_synthesis import memory_status

VIV=Path('E:/2026.1/Vivado/bin')
GIB=1024**3
def read(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf8')

def run(candidate,out):
    entry=read(candidate/'validation/entry.json')
    bindings=read(candidate/'validation/input_bindings.json')['inputs']
    assert entry['Gowin_SYN_or_PNR_commands']==[] and not out.exists()
    for row in bindings:assert metadata(Path(row['path']))==row,('frozen input changed',row)
    shutil.copytree(candidate/'validation',out)
    proc=psutil.Process();proc.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    state=dict(controller_pid=proc.pid,controller_created_epoch=proc.create_time(),candidate=str(candidate),output=str(out),
               synthesis_started=False,PNR_started=False,programmed=False,automatic_retry=False,
               frozen_inputs=bindings,finite_watchdog_is_stimulus_only=True)
    claim=out/'run_claim.json'
    with claim.open('x',encoding='utf8')as f:json.dump(state,f,ensure_ascii=False)
    def status(phase,**kw):state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw);save(out/'job.json',state)
    runtimes=[]
    for index,(tool,args)in enumerate(entry['commands']):
        assert tool in ('xvlog','xelab','xsim')
        while True:
            m=memory_status();status('waiting_memory_before_'+tool,available_physical_bytes=m.avail_phys,available_commit_bytes=m.avail_page)
            if m.avail_phys>=2*GIB and m.avail_page>=4*GIB:break
            time.sleep(17)
        for row in bindings:assert metadata(Path(row['path']))==row,('frozen input changed',row)
        cmd=['cmd','/c',str(VIV/(tool+'.bat')),*args]
        with(out/f'{index}_{tool}_stdout.log').open('w',encoding='utf8')as log:
            child=subprocess.Popen(cmd,cwd=out,stdout=log,stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW|subprocess.BELOW_NORMAL_PRIORITY_CLASS)
            runtime=dict(tool=tool,command=cmd,wrapper_pid=child.pid,wrapper_created_epoch=psutil.Process(child.pid).create_time(),
                started_at=datetime.now().astimezone().isoformat(),admission_physical_bytes=m.avail_phys,admission_commit_bytes=m.avail_page)
            status('running_'+tool,tool_runtime=runtime)
            while child.poll()is None:
                try:runtime['processes']=[dict(pid=p.pid,created_epoch=p.create_time(),name=p.name(),command=p.cmdline())for p in psutil.Process(child.pid).children(recursive=True)]
                except(psutil.NoSuchProcess,psutil.AccessDenied):pass
                save(out/'tool_runtime.json',runtime);time.sleep(2)
            runtime.update(exit_code=child.returncode,ended_at=datetime.now().astimezone().isoformat());runtimes.append(runtime)
            save(out/f'{index}_{tool}_runtime.json',runtime)
        if child.returncode:
            status('tool_failed_preserved_no_retry',runtimes=runtimes)
            return 1
    text=(out/'xsim.log').read_text(encoding='utf8',errors='replace')
    passed='STAGE2B_RTL_CONTRACTS_PASS cases=13'in text and 'Fatal:'not in text
    for row in bindings:assert metadata(Path(row['path']))==row,('frozen input changed',row)
    result=dict(state,phase='finite_RTL_contracts_complete',passed=passed,runtimes=runtimes,
        contracts_13_cases=True if passed else None,mapped_vendor_primitives_verified=False,
        full_consumer_or_CPU_handshake_verified=False,full4500_verified=False,
        actual_resources_measured=False,deployment_accepted=False)
    save(out/'result.json',result);save(out/'job.json',result)
    return 0 if passed else 1

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True);p.add_argument('--out',type=Path);p.add_argument('--run',action='store_true');a=p.parse_args()
    if a.run:
        assert a.out
        raise SystemExit(run(a.candidate.resolve(),a.out.resolve()))
    print(json.dumps(read(a.candidate/'validation/entry.json'),ensure_ascii=False,indent=2))
