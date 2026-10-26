"""Isolated 3x5m frozen-N1c SLF128/256 closed-loop comparison.

No RTL/PNR mutation. Freeze scene, runner/model/DLL and commands first; one
low-priority worker waits for actual memory admission independently of PNR52.
"""
from pathlib import Path
from datetime import datetime
from collections import deque
import argparse,json,os,re,shutil,subprocess,sys,time,traceback
import psutil
from prepare_baseline_fusion_retime import metadata,same_bytes
from retry_frozen_synthesis import memory_status

REPO=Path('D:/Desktop/SOC FPGA/nav')
N1C=REPO/'sim/build/batch/nav_opt_n1c_goto_20261021'
BASE=Path('F:/nav_integration_g2_20261023')
OUT=BASE/'nav3x5_N1c_SLF128_256_stage1_20261026'
GIB=1024**3
SCENE=dict(id='field3x5_v1',size_m=[5.0,3.0],origin_m=[0.0,-1.5],bounds_m=[0.0,-1.5,5.0,1.5],world_res_m=0.01,walls=dict(thickness_m=.05,height_m=.5),primitives=[
 dict(kind='box',args=[1.60,-1.30,1.95,-.95,.30],name='box_A'),
 dict(kind='box',args=[3.60,.65,3.95,1.00,.25],name='box_B'),
 dict(kind='box',args=[2.50,-.15,2.80,.20,.30],name='box_C'),
 dict(kind='box',args=[2.20,1.35,2.50,1.50,.50],name='asymmetric_wall_bump'),
 dict(kind='ramp',args=[.70,1.30,.40,1.20,0,.084],name='ramp8_up'),
 dict(kind='box',args=[1.30,.40,1.65,1.20,.084],name='ramp_platform'),
 dict(kind='ramp',args=[2.25,1.65,.40,1.20,0,.084],name='ramp8_down'),
 dict(kind='box',args=[3.15,-1.00,3.25,-.45,.015],name='crossable_15mm_sill'),
 dict(kind='box',args=[4.00,-1.50,4.10,-.50,.060],name='noncrossable_60mm_curb')],
 assumptions=dict(vehicle_footprint_radius_m=.21,step_climb_m=.02,slope_limit_deg=12,all_vehicle_operation_within_rectangle=True,physical_terrain_vehicle_capabilities_not_measured=True))
CASES={
 'goal':dict(start=[.45,-.95,0],goal=[4.45,.95],time_s=90),
 'ramp':dict(start=[.45,.85,0],goal=[1.45,.85],time_s=60),
 'step':dict(start=[2.90,-.80,0],goal=[3.65,-.80],time_s=50),
 'turn_loop':dict(start=[.45,-.95,0],waypoints=[[4.45,-.05],[4.45,.98],[.45,0.00],[.45,-.95]],time_s=150)}

CASE_ENTRY=r'''from pathlib import Path
import sys,json,os
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import run_cl as R
ROOT=R.ROOT
import slf_match
scene_spec=json.loads((ROOT/'scene.json').read_text(encoding='utf8'))
def make_scene():
 s=R.W.Scene(size=tuple(scene_spec['size_m']),origin=tuple(scene_spec['origin_m']),res=scene_spec['world_res_m'])
 s.walls(t=scene_spec['walls']['thickness_m'],h=scene_spec['walls']['height_m'])
 for p in scene_spec['primitives']:getattr(s,p['kind'])(*p['args'],p['name'])
 return s
R.W.SCENES[scene_spec['id']]=make_scene
def get_prior(scene_name,spec):
 if spec!='auto':return Path(spec).read_bytes()
 assert scene_name==scene_spec['id']
 ray=getattr(R.sw.World,'raycast_version','legacy_march')
 key=f"{scene_name}_SN{slf_match.SN}_N1c_noise4_lidarSeed11_surveySeed3_spacing1_yaws6_{ray}"
 cache=ROOT.parent/'prior_cache'/scene_name/f'sn{slf_match.SN}'
 cache.mkdir(parents=True,exist_ok=True);p=cache/(key+'.nvmp')
 if p.exists():return p.read_bytes()
 sc=make_scene();lid=R.sw.VirtualLidar(noise_mm=4.0,seed=11,rng_max=8.0,step=.015)
 sp,ap,_=R.prior_map.survey_scan(sc,lid,R.sw.mount_pose())
 x0,y0,x1,y1=sc.bounds
 pkg,info=R.prior_map.build_package(sp,ap,bounds_mm=(x0*1000-200,y0*1000-200,x1*1000+200,y1*1000+200))
 tmp=p.with_suffix('.tmp');tmp.write_bytes(pkg);tmp.replace(p)
 (cache/(key+'.json')).write_text(json.dumps(dict(key=key,SLF_SN_LOG2=slf_match.SN_LOG2,SLF_SN=slf_match.SN,scene=scene_spec,package_bytes=len(pkg),mapping_info=info),ensure_ascii=False,indent=2,default=lambda x:x.tolist()if hasattr(x,'tolist')else str(x)),encoding='utf8')
 return pkg
R.get_prior=get_prior
original_check_output=R.subprocess.check_output
def frozen_manifest_query(cmd,*args,**kw):
 if list(cmd[:2])==['git','rev-parse']:return 'frozen_N1c_20261021\n'
 if list(cmd[:2])==['git','status']:return ''
 return original_check_output(cmd,*args,**kw)
R.subprocess.check_output=frozen_manifest_query
if __name__=='__main__':
 assert slf_match.SN_LOG2 in(7,8)
 raise SystemExit(R.main())
'''

def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
def prepare():
 assert not OUT.exists(),'Preserve existing stage1 run rather than repeat it'
 assert (N1C/'nav_fw.dll').is_file()and load(N1C/'manifest.json')['evaluation']=='nav-opt-v2'
 OUT.mkdir();tree=OUT/'execution_tree'
 shutil.copytree(N1C/'runner',tree/'sim/closed_loop',ignore=shutil.ignore_patterns('__pycache__'))
 shutil.copytree(N1C/'reference_model',tree/'model',ignore=shutil.ignore_patterns('__pycache__'))
 shutil.copytree(N1C/'config',tree/'config',ignore=shutil.ignore_patterns('__pycache__'))
 shutil.copytree(N1C/'firmware_source',OUT/'firmware_source',ignore=shutil.ignore_patterns('__pycache__','build'))
 (tree/'tools').mkdir();shutil.copyfile(REPO/'tools/nav_host.py',tree/'tools/nav_host.py')
 shutil.copyfile(N1C/'nav_fw.dll',OUT/'nav_fw.dll')
 save(tree/'scene.json',SCENE);save(OUT/'scene.json',SCENE);save(OUT/'cases.json',CASES)
 (tree/'sim/closed_loop/stage1_case.py').write_text(CASE_ENTRY,encoding='utf8')
 # This does not replace the frozen model or runner: register only a new
 # scene and explicit cache function in an isolated wrapper process.
 for directory,dest in [('runner',tree/'sim/closed_loop'),('reference_model',tree/'model'),('config',tree/'config')]:
  for p in(N1C/directory).rglob('*'):
   if p.is_file()and'__pycache__'not in p.parts:assert same_bytes(p,dest/p.relative_to(N1C/directory))
 assert same_bytes(N1C/'nav_fw.dll',OUT/'nav_fw.dll')
 jobs=[]
 # Put the four directly comparable goal cells first. Each subsequent
 # geometry stress case uses the same seed/mode/SN matrix, not retuned values.
 for case in CASES:
  for prior in(False,True):
   for sn in(8,7):
    c=CASES[case];name=f'{case}_sn{1<<sn}_{"prior"if prior else"nomap"}_s1';result=OUT/'results'/name
    cmd=[sys.executable,'-X','utf8',str(tree/'sim/closed_loop/stage1_case.py'),'--scene',SCENE['id'],'--seed','1','--no-build','--firmware-dll',str(OUT/'nav_fw.dll'),'--no-plots','--start',','.join(map(str,c['start'])),'--time',str(c['time_s']),'--out',str(result)]
    if'goal'in c:cmd+=['--goal',','.join(map(str,c['goal']))]
    else:cmd+=['--waypoints',','.join(str(v)for xy in c['waypoints']for v in xy),'--laps','1']
    if prior:cmd+=['--prior','auto','--init-err','rand']
    jobs.append(dict(name=name,case=case,SLF_SN_LOG2=sn,SLF_SN=1<<sn,prior=prior,seed=1,command=cmd,out=str(result)))
 bindings=[metadata(p)for p in tree.rglob('*')if p.is_file()and'__pycache__'not in p.parts]+[metadata(OUT/'nav_fw.dll')]
 manifest=dict(prepared=True,recorded_at=datetime.now().astimezone().isoformat(),N1c_source=str(N1C),frozen_FW_DLL_reused=True,N1c_runner_reference_model_config_byte_identical=True,host_NV_packet_encoder_frozen_from=str(REPO/'tools/nav_host.py'),new_scene=SCENE,cases=CASES,seed_list=[1],jobs=jobs,matching_parameters_changed_only_by_environment_SLF_SN_LOG2=True,SLF_NBR='3x3',SLF_W18=1,original_evaluation='nav-opt-v2',arrival_criteria=dict(position_m=.150,speed_mps=.020,yaw_rate_rps=.050,dwell_s=.500,observe_s=2.000),subcentimeter_accuracy_accepted=False,hardware_RTL_PNR_unchanged=True,cache_key_includes_scene_revision_SLF_size_and_survey_parameters=True,bindings=bindings)
 save(OUT/'manifest.json',manifest);save(OUT/'job.json',dict(phase='prepared_not_started',jobs=len(jobs),automatic_PNR=False));return manifest

def validate_scene():
 import numpy as np
 tree=OUT/'execution_tree';sys.path.insert(0,str(tree/'sim/closed_loop'));sys.path.insert(0,str(tree/'model'))
 import world as W
 s=W.Scene(size=tuple(SCENE['size_m']),origin=tuple(SCENE['origin_m']),res=SCENE['world_res_m']);s.walls(t=.05,h=.5)
 for p in SCENE['primitives']:getattr(s,p['kind'])(*p['args'],p['name'])
 x0,y0,x1,y1=s.bounds;res=.05;xs=np.arange(x0+res/2,x1,res);ys=np.arange(y0+res/2,y1,res);X,Y=np.meshgrid(xs,ys)
 # A circle-clearance grid conservatively excludes labels1/2 around vehicle
 # centers. Geometry validation is independent of the navigation output.
 lab=s.gt_label(X,Y,cell=res);blocked=lab!=0
 from scipy.ndimage import distance_transform_edt,label
 clearance=distance_transform_edt(~blocked)*res
 free=clearance>=SCENE['assumptions']['vehicle_footprint_radius_m']
 free&=(X-x0>=.21)&(x1-X>=.21)&(Y-y0>=.21)&(y1-Y>=.21)
 comp,_=label(free);checked=[]
 def at(xy):return int((xy[1]-y0)/res),int((xy[0]-x0)/res)
 for name,c in CASES.items():
  points=[c['start'][:2]]+([c['goal']]if'goal'in c else c['waypoints']);ids=[]
  for xy in points:
   iy,ix=at(xy);assert x0+.21<=xy[0]<=x1-.21 and y0+.21<=xy[1]<=y1-.21,(name,xy,'boundary');assert free[iy,ix],(name,xy,'not circle-clear in conservative GT grid',float(clearance[iy,ix]));ids.append(int(comp[iy,ix]))
  assert ids[0]>0 and len(set(ids))==1,(name,ids,'no connected free route')
  checked.append(dict(case=name,all_start_goal_waypoints_inside_bounds_with_vehicle_radius=True,all_connected_in_conservative_5cm_clearance_grid=True,component=ids[0]))
 np.savez(OUT/'scene_geometry.npz',x=xs,y=ys,height=s.height(X,Y),labels=lab,vehicle_clear=free,component=comp)
 result=dict(passed=True,bounds_m=list(s.bounds),area_m2=15,grid_cell_m=res,primitive_count=len(s.prims),cases=checked,scope='geometry and path-existence check, not navigation success or real terrain capability')
 save(OUT/'scene_validation.json',result)
 import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
 fig,ax=plt.subplots(figsize=(10,6));ax.imshow(s.height(X,Y),origin='lower',extent=(x0,x1,y0,y1),cmap='terrain',vmin=0,vmax=.5);ax.contour(X,Y,free.astype(int),levels=[.5],colors=['black'],linewidths=.4)
 for name,c in CASES.items():
  pts=np.array([c['start'][:2]]+([c['goal']]if'goal'in c else c['waypoints']));ax.plot(pts[:,0],pts[:,1],'o--',label=name,alpha=.75)
 ax.set(xlim=(x0,x1),ylim=(y0,y1),xlabel='x / m',ylabel='y / m',title='Independent 5m x 3m scene; dashed lines show tasks, not planned routes');ax.set_aspect('equal');ax.legend();fig.tight_layout();fig.savefig(OUT/'scene.png',dpi=130);plt.close(fig);return result

def summarize():
 manifest=load(OUT/'manifest.json');rows=[]
 for job in manifest['jobs']:
  p=Path(job['out'])/'metrics.json'
  if not p.exists():continue
  m=load(p);rows.append(dict(case=job['case'],seed=job['seed'],SLF_SN=job['SLF_SN'],prior=job['prior'],arrived=m['arrived'],outcome=m['outcome'],collisions=m['collisions'],firmware_arrived=m['firmware_arrived'],false_arrival=m['outcome']=='false_arrival',terminal_drift=m['outcome']=='terminal_drift',goal_err_m=m['goal_err_m'],loc_err_xy_rms_m=m['loc_err_xy_rms_m'],loc_err_xy_final_m=m['loc_err_xy_final_m'],loc_err_xy_max_m=m['loc_err_xy_max_m'],loc_err_xy_full_rms_m=m.get('loc_err_xy_full_rms_m'),align_s=m['align_s'],n_align_fail=m['n_align_fail'],n_lost=m['n_lost'],t_lost_s=m['t_lost_s'],loc_valid_fraction=m.get('loc_valid_fraction'),time_s=m['time_s'],wall_s=m['wall_s'],metrics=str(p)))
 save(OUT/'summary.json',dict(recorded_at=datetime.now().astimezone().isoformat(),completed=len(rows),planned=len(manifest['jobs']),all_success=False if not rows else all(x['arrived']and x['collisions']==0 for x in rows),subcentimeter_accuracy_accepted=False,rows=rows));return rows

def run():
 m=load(OUT/'manifest.json');assert load(OUT/'scene_validation.json')['passed']
 for row in m['bindings']:assert metadata(Path(row['path']))==row,row['path']
 me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS);me.cpu_affinity(list(range(max(0,psutil.cpu_count()-2),psutil.cpu_count())))
 with(OUT/'run_claim.json').open('x')as f:json.dump(dict(pid=me.pid,created_epoch=me.create_time(),priority='BELOW_NORMAL',logical_CPUs=me.cpu_affinity()),f)
 state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),one_worker=True,PNR52_inputs_unchanged=True,minimum_start_physical_bytes=int(2.5*GIB),minimum_start_commit_bytes=int(4.5*GIB),completed=0)
 def status(phase,**kw):state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw);save(OUT/'job.json',state)
 for index,job in enumerate(m['jobs']):
  assert not(Path(job['out'])/'metrics.json').exists(),'Do not repeat an existing completed case'
  while True:
   mem=memory_status();status('waiting_memory_before_case',next=job['name'],free_bytes=mem.avail_phys,available_commit_bytes=mem.avail_page)
   if mem.avail_phys>=int(2.5*GIB)and mem.avail_page>=int(4.5*GIB):break
   time.sleep(17)
  env=os.environ.copy();env.update(SLF_SN_LOG2=str(job['SLF_SN_LOG2']),SLF_NBR='3x3',SLF_W18='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1')
  Path(job['out']).mkdir(parents=True,exist_ok=False)
  with(Path(job['out'])/'stdout.log').open('w',encoding='utf8')as log:
   child=subprocess.Popen(job['command'],stdout=log,stderr=subprocess.STDOUT,env=env,creationflags=subprocess.CREATE_NO_WINDOW)
   cp=psutil.Process(child.pid);cp.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS);cp.cpu_affinity(me.cpu_affinity());status('running_case',current=job['name'],child_pid=child.pid,child_created_epoch=cp.create_time(),command=job['command'],admission_physical_bytes=mem.avail_phys,admission_commit_bytes=mem.avail_page)
   low_since=None
   while child.poll()is None:
    mem=memory_status();low=mem.avail_phys<256*1024**2 and mem.avail_page<GIB
    low_since=time.monotonic()if low and low_since is None else low_since if low else None
    if low_since is not None and time.monotonic()-low_since>=30:
     child.terminate();child.wait();status('own_simulation_stopped_for_memory_preserved',PNR_untouched=True);return 2
    time.sleep(5)
  assert child.returncode==0,('case failed; preserve stdout, no automatic retry',job['name'],child.returncode)
  rows=summarize();status('case_complete',completed=len(rows),last=job['name'],last_outcome=rows[-1]['outcome'])
 status('all_stage1_cases_complete',completed=len(summarize()));return 0

if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--prepare',action='store_true');a.add_argument('--run',action='store_true');a.add_argument('--summary',action='store_true');v=a.parse_args();assert sum((v.prepare,v.run,v.summary))==1
 if v.prepare:
  p=prepare();print(json.dumps(dict(prepared=p['prepared'],out=str(OUT),jobs=len(p['jobs']),scene_validation=validate_scene()),ensure_ascii=False))
 elif v.summary:print(json.dumps(summarize(),ensure_ascii=False))
 else:
  try:raise SystemExit(run())
  except Exception:
   save(OUT/'failure.json',dict(error=traceback.format_exc(),hardware_PNR_inputs_unchanged=True));raise
