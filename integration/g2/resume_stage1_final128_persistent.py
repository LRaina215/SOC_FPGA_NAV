"""Resume only incomplete frozen cells; Windows service-parent launch, one worker."""
from pathlib import Path
from datetime import datetime
import argparse,json,os,subprocess,sys,time,traceback
import psutil
import stage1_final_nav3x5_multiseed as S
import prepare_stage1_final_long_patrol as P
import run_stage1_final_long_patrol as L
import stage1a_nav3x5_window_ablation as A
from prepare_baseline_fusion_retime import metadata

ROOT=S.OUT
RESUME=ROOT/'resume1'

def now():return datetime.now().astimezone().isoformat()

def prepare():
 assert not RESUME.exists(),'Do not duplicate resume'
 old=A.read(ROOT/'manifest.json');state=A.read(ROOT/'job.json');long=A.read(P.OUT/'job.json')
 for pid,created in((state['controller_pid'],state['controller_created_epoch']),(long['controller_pid'],long['controller_created_epoch'])):
  if psutil.pid_exists(pid):assert abs(psutil.Process(pid).create_time()-created)>.01,'Owned controller still active'
 for b in old['bindings']:assert metadata(Path(b['path']))==b,b['path']
 RESUME.mkdir();A.write(RESUME/'previous_job.json',state);A.write(RESUME/'previous_long_job.json',long)
 completed=[];interrupted=[]
 for job in old['jobs']:
  directory=Path(job['out'])
  if(directory/'metrics.json').exists():
   result=A.read(directory/'metrics.json');assert result['seed']==job['seed']and result['scene']=='field3x5_v1'
   assert(directory/'timeseries.csv').stat().st_size>0 and(directory/'final_maps.npz').stat().st_size>0
   completed.append(job['name']);continue
  assert job['result_source']=='new_run'
  if directory.exists():
   attempt=directory.with_name(directory.name+'_attempt2');assert not attempt.exists()
   interrupted.append(dict(name=job['name'],original=str(directory),next_attempt=str(attempt),contents=[p.name for p in directory.iterdir()],reason='Parent and worker absent; no metrics/checkpoint. Preserve original, restart incomplete cell only.'))
   job['interrupted_original_out']=job['out'];job['out']=str(attempt);job['command'][job['command'].index('--out')+1]=str(attempt);job['attempt']=2
 assert len(completed)==20 and len(interrupted)==1
 old.update(active_resume=str(RESUME),completed_at_resume=completed,interrupted_preserved=interrupted)
 A.write(RESUME/'manifest.json',old)
 record=dict(recorded_at=now(),previous_control_absent=True,previous_worker_absent=True,completed_reused=20,completed_new=6,pending_new=52,
  interrupted=interrupted,frozen_model_DLL_scene_unmodified=True,command_delta='Only incomplete cell --out is changed to attempt2; other arguments/env same',
  cause_confirmed=False,observed='Both exec-session control trees vanished near agent final; no Python crash event found in Application log.',
  inference='Tool/agent session cleanup is likely but not proven by an OS termination event.',
  repair='Launch resume via local Windows WMI service parent; own heartbeat and sequential worker, then long patrol in same control. Not attached to unified exec session.',
  automatic_EDA=False)
 A.write(RESUME/'recovery_preflight.json',record);return record

def run():
 me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS);me.cpu_affinity([18,19])
 with(RESUME/'run_claim.json').open('x')as f:json.dump(dict(pid=me.pid,created=me.create_time(),parent_pid=me.ppid()),f)
 sys.stdout=(RESUME/'controller_stdout.log').open('a',encoding='utf8',buffering=1);sys.stderr=(RESUME/'controller_stderr.log').open('a',encoding='utf8',buffering=1)
 manifest=A.read(RESUME/'manifest.json');long_manifest=A.read(P.OUT/'manifest.json')
 for b in manifest['bindings']:assert metadata(Path(b['path']))==b,b['path']
 for b in long_manifest['bindings']:assert metadata(Path(b['path']))==b,b['path']
 phase='multiseed';current=None;child_info={};done=20;newdone=6;longdone=0
 identity=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),controller_parent_pid=me.ppid(),launch='Windows WMI service-parent; hidden window',one_worker=True,automatic_EDA=False)
 def status(label,**kw):
  nonlocal child_info
  base=dict(identity,phase=label,updated_at=now(),active_manifest=str(RESUME/'manifest.json'),planned=72,reused=14,new_completed=newdone,completed=done,pending_new=72-done,current=current,
   minimum_start_physical_bytes=2*A.GIB,minimum_start_commit_bytes=3*A.GIB,**child_info)
  base.update(kw)
  if phase=='multiseed':
   A.write(ROOT/'job.json',base)
   A.write(P.OUT/'job.json',dict(identity,phase='waiting_same_persistent_controller_multiseed',updated_at=now(),planned=2,completed=0,stage1_completed=done,stage1_phase=label))
  else:A.write(P.OUT/'job.json',dict(base,planned=2,completed=longdone,pending=2-longdone))
  A.write(RESUME/'heartbeat.json',dict(base,active_phase=phase))
 def execute(job):
  nonlocal current,child_info
  current=job['name'];child_info={}
  directory=Path(job['out']);assert not directory.exists(),'Never overwrite a prior attempt'
  while True:
   memory=A.memory_status();status('waiting_memory',free_bytes=memory.avail_phys,commit_bytes=memory.avail_page)
   if memory.avail_phys>=2*A.GIB and memory.avail_page>=3*A.GIB:break
   time.sleep(10)
  directory.mkdir(parents=True);env=os.environ.copy();env.update(job['environment'])
  with(directory/'stdout.log').open('w',encoding='utf8')as log:
   child=subprocess.Popen(job['command'],env=env,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
   p=psutil.Process(child.pid);p.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS);p.cpu_affinity([18,19]);peak_rss=peak_private=0;low_since=None
   child_info=dict(child_pid=child.pid,child_created_epoch=p.create_time(),command=job['command'],environment=job['environment'],admission_physical_bytes=memory.avail_phys,admission_commit_bytes=memory.avail_page)
   while child.poll()is None:
    memory=A.memory_status()
    try:
     info=p.memory_info();peak_rss=max(peak_rss,info.rss);peak_private=max(peak_private,getattr(info,'private',info.vms));cpu=p.cpu_times();cpu_seconds=cpu.user+cpu.system
    except psutil.NoSuchProcess:cpu_seconds=None
    status('running_case'if phase=='multiseed'else'running',free_bytes=memory.avail_phys,commit_bytes=memory.avail_page,peak_rss=peak_rss,peak_private=peak_private,child_CPU_seconds=cpu_seconds)
    low=memory.avail_phys<256*1024**2 and memory.avail_page<A.GIB
    low_since=time.monotonic()if low and low_since is None else low_since if low else None
    if low_since is not None and time.monotonic()-low_since>=30:child.terminate();child.wait();raise RuntimeError('Own new worker stopped by original runtime memory guard')
    time.sleep(5)
  A.write(directory/'runtime.json',dict(exit=child.returncode,peak_rss=peak_rss,peak_private=peak_private,child_pid=child.pid,child_creation=child_info['child_created_epoch'],persistent_controller=me.pid))
  assert child.returncode==0,(directory/'stdout.log').read_text(encoding='utf8')
  assert(directory/'metrics.json').exists()
  print(now(),job['name'],'complete',flush=True)
 try:
  S.summarize(manifest)
  for job in manifest['jobs']:
   if(Path(job['out'])/'metrics.json').exists():continue
   execute(job);done+=1;newdone+=1;S.summarize(manifest);status('case_complete')
  status('multiseed_complete_long_patrol_pending',summary=str(ROOT/'summary.json'))
  summary=S.summarize(manifest);assert summary['completed']==72
  selected=L.select(summary);A.write(P.OUT/'selected_best_distance.json',selected)
  phase='long_patrol';jobs=[];results=[]
  for profile,sn,mm in(('control256',256,0),('best128',128,selected['selected_offset_mm'])):
   name=f'patrol330s_{profile}_s1';out=P.OUT/'results'/name;command=list(long_manifest['base_command']);command[command.index('--out')+1]=str(out)
   env=dict(long_manifest['environment_template'],SLF_SN_LOG2=str(sn.bit_length()-1),SLF_LF_N=str(sn),SLF_WINDOW_MODE='forward'if mm else'rolling',SLF_FORWARD_MM=str(mm))
   jobs.append(dict(name=name,profile=profile,SLF_SN=sn,forward_mm=mm,prior=False,case='long_patrol',seed=1,command=command,environment=env,out=str(out)))
  A.write(P.OUT/'selected_jobs.json',dict(selection=selected,jobs=jobs))
  for job in jobs:
   execute(job);results.append(L.analyze_result(job));longdone+=1
   A.write(P.OUT/'summary.json',dict(completed=longdone,planned=2,selection=selected,results=results));status('patrol_complete')
  status('complete',summary=str(P.OUT/'summary.json'));return 0
 except Exception:status('failed',error=traceback.format_exc());traceback.print_exc();raise

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--prepare',action='store_true');p.add_argument('--run',action='store_true');a=p.parse_args()
 if a.prepare:print(json.dumps(prepare(),ensure_ascii=False))
 if a.run:raise SystemExit(run())
