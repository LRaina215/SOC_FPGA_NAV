"""One lightweight handoff: after the single multiseed worker, run two native loops."""
from pathlib import Path
from datetime import datetime
import argparse,json,os,subprocess,sys,time,traceback
import psutil
import prepare_stage1_final_long_patrol as L
import stage1_final_nav3x5_multiseed as S
import stage1a_nav3x5_window_ablation as A
from prepare_baseline_fusion_retime import metadata

def select(summary):
 rows=summary['rows'];ranking=[]
 for mm in(1000,1250,1500):
  group=[r for r in rows if r['profile']==f'forward{mm}'];assert len(group)==12
  key=(sum(r['result']['collisions']for r in group),sum(not r['result']['P95_le50']for r in group),
       max(r['result']['XY_valid']['P95_mm']for r in group),sum(r['result']['XY_valid']['P95_mm']for r in group)/12)
  ranking.append(dict(offset_mm=mm,descriptive_rank=list(key),P95_pass_k_n=[sum(r['result']['P95_le50']for r in group),12]))
 ranking.sort(key=lambda r:r['descriptive_rank'])
 return dict(selected_offset_mm=ranking[0]['offset_mm'],ranking=ranking,
  selection_rule='lexicographic fewer observed collisions, fewer P95>50 cases, lower worst P95, lower mean P95; minimax choice is not acceptance',
  no_map_lock_status='Not established by constant runner loc_valid; use separate observability analysis',production128_fixed=True)

def analyze_result(job):
 import numpy as np
 import csv,math
 root=Path(job['out']);metric=A.read(root/'metrics.json');audit=A.read(root/'patrol_protocol_audit.json')
 with(root/'timeseries.csv').open(encoding='utf8')as f:rows=list(csv.DictReader(f))
 t=np.array([float(r['t'])for r in rows]);error=np.array([1000*math.hypot(float(r['ex'])-float(r['x']),float(r['ey'])-float(r['y']))for r in rows])
 transitions=[v for v in audit['state_goal_transitions']if v['mode']==1] # Origin[0,0] is the legitimate fourth odometry waypoint.
 goals=[]
 for event in transitions:
  if not goals or event['goal_mm']!=goals[-1]['goal_mm']:goals.append(event)
 route=audit['packets'][0]['waypoints_mm'];expected=0;advances=0;bad=[]
 for event in goals:
  if event['goal_mm']!=route[expected]:bad.append(event)
  else:advances+=1;expected=(expected+1)%len(route)
 # Initial first goal is a start, every later return to first proves a completed cycle.
 cycles=max(0,(advances-1)//len(route));windows=[]
 for begin in np.arange(0,t[-1],10):
  mask=(t>=begin)&(t<begin+60)
  if mask.any():windows.append(dict(begin_s=float(begin),end_s=float(min(t[-1],begin+60)),samples=int(mask.sum()),P95_mm=float(np.percentile(error[mask],95)),
    RMSE_mm=float(np.sqrt(np.mean(error[mask]**2))),endpoint_mm=float(error[mask][-1]),max_mm=float(error[mask].max())))
 result=dict(name=job['name'],profile=job['profile'],forward_mm=job['forward_mm'],actual_simulated_duration_s=metric['simulation_end_s'],
  at_least300s=metric['simulation_end_s']>=300,ordered_native_goal_advances=advances,completed_cycles=cycles,unexpected_goal_events=bad,
  at_least2_cycles=cycles>=2,valid_XY_P95_mm=float(np.percentile(error,95)),P95_le50=float(np.percentile(error,95))<=50,
  collisions=metric['collisions'],n_lost=metric['n_lost'],no_map_lock_proved=None,rolling60s=windows,
  full_error_linear_trend_mm_per_min=float(np.polyfit(t,error,1)[0]*60),sustained_drift_pass=None,
  drift_limit='No numeric sustained-growth criterion specified; report curve/windows/slope for review, not invented threshold',
  terminal_outcome_reported_not_acceptance=metric['outcome'],trace=str(root/'timeseries.csv'),protocol_audit=str(root/'patrol_protocol_audit.json'))
 with(root/'error_curve.csv').open('w',encoding='utf8')as f:
  f.write('t_s,error_xy_mm\n')
  for tt,ee in zip(t,error):f.write(f'{tt:.4f},{ee:.9f}\n')
 A.write(root/'long_patrol_analysis.json',result);return result

def run():
 root=L.OUT;manifest=A.read(root/'manifest.json')
 for b in manifest['bindings']:assert metadata(Path(b['path']))==b,b['path']
 me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS);me.cpu_affinity([18,19])
 with(root/'run_claim.json').open('x')as f:json.dump(dict(pid=me.pid,created=me.create_time()),f)
 completed=0
 def status(phase,**kw):A.write(root/'job.json',dict(phase=phase,updated_at=datetime.now().astimezone().isoformat(),controller_pid=me.pid,controller_created_epoch=me.create_time(),planned=2,completed=completed,one_worker=True,automatic_EDA=False,**kw))
 try:
  while True:
   state=A.read(S.OUT/'job.json');status('waiting_existing_single_worker_and_best_distance',stage1_phase=state['phase'],stage1_completed=state['completed'])
   if state['phase']=='multiseed_complete_long_patrol_pending':break
   assert state['phase']!='failed','Existing multiseed failed; preserve and do not start patrol'
   time.sleep(30)
  summary=S.summarize();assert summary['completed']==72
  chosen=select(summary);A.write(root/'selected_best_distance.json',chosen)
  jobs=[]
  for profile,sn,mm in(('control256',256,0),('best128',128,chosen['selected_offset_mm'])):
   name=f'patrol330s_{profile}_s1';out=root/'results'/name;command=list(manifest['base_command']);command[command.index('--out')+1]=str(out)
   env=dict(manifest['environment_template'],SLF_SN_LOG2=str(sn.bit_length()-1),SLF_LF_N=str(sn),SLF_WINDOW_MODE='forward'if mm else'rolling',SLF_FORWARD_MM=str(mm))
   jobs.append(dict(name=name,profile=profile,SLF_SN=sn,forward_mm=mm,prior=False,case='long_patrol',seed=1,command=command,environment=env,out=str(out)))
  A.write(root/'selected_jobs.json',dict(selection=chosen,jobs=jobs))
  results=[]
  for job in jobs:
   assert not Path(job['out']).exists(),'No duplicate long patrol'
   while True:
    memory=A.memory_status();status('waiting_memory',next=job['name'],physical=memory.avail_phys,commit=memory.avail_page)
    if memory.avail_phys>=2*A.GIB and memory.avail_page>=3*A.GIB:break
    time.sleep(10)
   directory=Path(job['out']);directory.mkdir(parents=True);env=os.environ.copy();env.update(job['environment'])
   with(directory/'stdout.log').open('w',encoding='utf8')as f:
    p=subprocess.Popen(job['command'],env=env,stdout=f,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
    child=psutil.Process(p.pid);child.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS);child.cpu_affinity([18,19]);peak=0;low_since=None
    status('running',current=job['name'],child_pid=p.pid,child_created_epoch=child.create_time(),command=job['command'],environment=job['environment'],selected_offset_mm=chosen['selected_offset_mm'],admission_physical=memory.avail_phys,admission_commit=memory.avail_page)
    while p.poll()is None:
     try:peak=max(peak,child.memory_info().rss)
     except psutil.NoSuchProcess:pass
     memory=A.memory_status();low=memory.avail_phys<256*1024**2 and memory.avail_page<A.GIB
     low_since=time.monotonic()if low and low_since is None else low_since if low else None
     if low_since is not None and time.monotonic()-low_since>=30:p.terminate();p.wait();raise RuntimeError('Only own long worker stopped by sustained low memory')
     time.sleep(5)
   A.write(directory/'runtime.json',dict(exit=p.returncode,peak_rss=peak));assert p.returncode==0,(directory/'stdout.log').read_text(encoding='utf8')
   results.append(analyze_result(job));completed+=1;A.write(root/'summary.json',dict(completed=completed,planned=2,selection=chosen,results=results))
  status('complete',summary=str(root/'summary.json'));return 0
 except Exception:status('failed',error=traceback.format_exc());raise

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--start',action='store_true');a=p.parse_args()
 if a.start:raise SystemExit(run())
