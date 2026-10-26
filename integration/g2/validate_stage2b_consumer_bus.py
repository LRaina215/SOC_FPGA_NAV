"""Explicit finite bus miters of already frozen TX-shadow/console candidates."""
from pathlib import Path
from datetime import datetime
import argparse,json,re,shutil,subprocess,time,traceback,psutil
from prepare_baseline_fusion_retime import metadata
from retry_frozen_synthesis import memory_status

VIV=Path('E:/2026.1/Vivado/bin');GIB=1024**3
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf8')

def run(candidate,out,validation=None):
 val=validation or candidate/'validation'
 entry=load(val/'entry.json');review=load(candidate/'prepared_source_review.json')
 assert review['prepared']and not out.exists()
 bindings=review['hardware_bindings']+review['validation_bindings']
 if validation:bindings+=load(val/'input_bindings.json')['inputs']
 def frozen():
  for row in bindings:assert metadata(Path(row['path']))==row,('frozen input changed',row)
 frozen();shutil.copytree(val,out)
 me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
 state=dict(profile=review['profile'],candidate=str(candidate),validation=str(val),output=str(out),controller_pid=me.pid,controller_created_epoch=me.create_time(),frozen_input_count=len(bindings),finite_watchdog_is_stimulus_only=True,synthesis_started=False,PNR_started=False,programmed=False,automatic_retry=False)
 with(out/'launch_claim.json').open('x')as f:json.dump(state,f,ensure_ascii=False)
 runtimes=[]
 def status(phase,**kw):state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw);save(out/'job.json',state)
 try:
  for i,(tool,args)in enumerate(entry['commands']):
   assert tool in ['xvlog','xelab','xsim']
   while True:
    m=memory_status();status('waiting_memory_before_'+tool,available_physical_bytes=m.avail_phys,available_commit_bytes=m.avail_page)
    if m.avail_phys>=2*GIB and m.avail_page>=4*GIB:break
    time.sleep(17)
   frozen();cmd=['cmd','/c',str(VIV/(tool+'.bat')),*args]
   with(out/f'{i}_{tool}_stdout.log').open('w')as log:
    p=subprocess.Popen(cmd,cwd=out,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW|subprocess.BELOW_NORMAL_PRIORITY_CLASS)
    runtime=dict(tool=tool,command=cmd,wrapper_pid=p.pid,wrapper_created_epoch=psutil.Process(p.pid).create_time(),started_at=datetime.now().astimezone().isoformat(),admission_physical_bytes=m.avail_phys,admission_commit_bytes=m.avail_page)
    status('running_'+tool,tool_runtime=runtime)
    while p.poll()is None:
     try:runtime['processes']=[dict(pid=q.pid,created_epoch=q.create_time(),name=q.name(),command=q.cmdline())for q in psutil.Process(p.pid).children(recursive=True)]
     except(psutil.NoSuchProcess,psutil.AccessDenied):pass
     save(out/'tool_runtime.json',runtime);time.sleep(2)
    runtime.update(exit_code=p.returncode,ended_at=datetime.now().astimezone().isoformat());runtimes.append(runtime);save(out/f'{i}_{tool}_runtime.json',runtime)
   assert p.returncode==0,(tool,p.returncode)
  text=(out/'xsim.log').read_text(errors='replace')
  marker='TEST PASSED TXSHADOW BUS'if review['profile']=='txshadow'else'TEST PASSED CONSOLE BUS'
  assert marker in text and not re.search(r'\bFatal:',text)
  line=next(x for x in text.splitlines()if x.startswith(marker));counts={k:int(v)for k,v in re.findall(r'(\w+)=(\d+)',line)}
  frozen();result=dict(state,phase='finite_bus_miter_complete',passed=True,counts=counts,actual_PASS_line=line,runtimes=runtimes,full_Pico_firmware_or_wholeNAV_verified=False,cut=entry['cut'],actual_mapped_primitives_or_physical_timing_verified=False)
  save(out/'result.json',result);save(out/'job.json',result);return 0
 except Exception as e:
  result=dict(state,phase='failed_preserved_no_retry',passed=False,error=str(e),traceback=traceback.format_exc(),runtimes=runtimes)
  save(out/'result.json',result);save(out/'job.json',result);return 1

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True);p.add_argument('--validation',type=Path);p.add_argument('--out',type=Path);p.add_argument('--run',action='store_true');a=p.parse_args()
 if a.run:assert a.out;raise SystemExit(run(a.candidate.resolve(),a.out.resolve(),a.validation.resolve()if a.validation else None))
 print(json.dumps(load((a.validation or a.candidate/'validation')/'entry.json'),ensure_ascii=False,indent=2))
