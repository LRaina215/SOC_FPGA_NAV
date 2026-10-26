"""Explicit-only RTL validation of an already frozen stage2B storage candidate.

Default merely displays the prepared request. --run is required and never
invokes Gowin, PNR, firmware rebuilding or board programming.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,shutil,subprocess,time,psutil
from prepare_baseline_fusion_retime import metadata
from retry_frozen_synthesis import memory_status

VIV=Path('E:/2026.1/Vivado/bin')
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf8')

def run(candidate,out):
 spec=load(candidate/'validation/validation_request.json');review=load(candidate/'prepared_source_review.json')
 assert review['prepared']and review['validation_not_run']and not out.exists()
 for row in review['bindings']+review['validation_bindings']:assert metadata(Path(row['path']))==row,('frozen input changed',row)
 m=memory_status();assert m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3,('memory admission',m)
 shutil.copytree(candidate/'validation',out)
 me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
 state=dict(profile=spec['profile'],controller_pid=me.pid,controller_created_epoch=me.create_time(),candidate=str(candidate),out=str(out),simulation_started=True,synthesis_started=False,PNR_started=False,programmed=False,finite_watchdog_is_stimulus_only=True)
 save(out/'job.json',dict(state,phase='starting_RTL_validation'))
 commands=[('xvlog',['-sv',*spec['files']]),('xelab',['-debug','off','-timescale','1ns/1ps',spec['top'],'-s','snapshot']),('xsim',['snapshot','-runall'])]
 runtimes=[]
 for i,(tool,args)in enumerate(commands):
  m=memory_status();assert m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3,('memory admission',tool,m)
  cmd=['cmd','/c',str(VIV/(tool+'.bat')),*args]
  with(out/f'{i}_{tool}_stdout.log').open('w')as log:
   p=subprocess.Popen(cmd,cwd=out,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW|subprocess.BELOW_NORMAL_PRIORITY_CLASS)
   runtime=dict(tool=tool,command=cmd,wrapper_pid=p.pid,wrapper_created_epoch=psutil.Process(p.pid).create_time(),started_at=datetime.now().astimezone().isoformat(),admission_physical_bytes=m.avail_phys,admission_commit_bytes=m.avail_page)
   save(out/'job.json',dict(state,phase='running_'+tool,tool_runtime=runtime))
   while p.poll()is None:
    try:runtime['processes']=[dict(pid=q.pid,created_epoch=q.create_time(),name=q.name(),command=q.cmdline())for q in psutil.Process(p.pid).children(recursive=True)]
    except(psutil.NoSuchProcess,psutil.AccessDenied):pass
    save(out/'tool_runtime.json',runtime);time.sleep(2)
   runtime.update(exit_code=p.returncode,ended_at=datetime.now().astimezone().isoformat());save(out/f'{i}_{tool}_runtime.json',runtime);runtimes.append(runtime)
  if p.returncode:
   save(out/'job.json',dict(state,phase='failed_preserve_no_retry',runtimes=runtimes));raise AssertionError((tool,p.returncode))
 text=(out/'xsim.log').read_text(errors='replace')
 assert 'TEST PASSED'in text and 'Fatal:'not in text
 if spec['profile']=='weight9':assert 'WEIGHT9 SCAN reads=40960'in text
 result=dict(state,phase='RTL_complete',passed=True,runtimes=runtimes,RTL_cycle_contract_only=True,full_G2_or_physical_acceptance=False,all_logs_preserved=True)
 save(out/'result.json',result);save(out/'job.json',result);return result

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True);p.add_argument('--out',type=Path);p.add_argument('--run',action='store_true');a=p.parse_args()
 if a.run:assert a.out;print(json.dumps(run(a.candidate.resolve(),a.out.resolve()),ensure_ascii=False))
 else:print(json.dumps(load(a.candidate/'validation/validation_request.json'),ensure_ascii=False,indent=2))
