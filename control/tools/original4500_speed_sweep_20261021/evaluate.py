"""Five fixed-command cases; first-failure statistics only."""
import sys,json,time,shutil,zipfile,hashlib
from pathlib import Path
import numpy as np
for k,v in {'float':float,'int':int,'bool':bool}.items():
 if k not in np.__dict__:setattr(np,k,v)
import isaacgym
from isaacgym.torch_utils import quat_rotate
import torch
from wheel_legged_gym.envs import task_registry
from wheel_legged_gym.envs.car_robot.car_robot import CarRobot
from wheel_legged_gym.utils import get_args
from wheel_legged_gym.utils.helpers import class_to_dict,set_seed
from wheel_legged_gym.rsl_rl.runners.on_policy_runner import OnPolicyRunner
root=Path('/home/cyd/fpga/tools/original4500_speed_sweep_20261021')
out=root/'evaluations'/time.strftime('%Y%m%d_%H%M%S');out.mkdir(parents=True)
cp=Path('/home/cyd/fpga/model/wheel_velocity_original/model_4500.pt')
assert hashlib.sha256(cp.read_bytes()).hexdigest()=='a0c0024e132b71d10fc42cc92081129e2738e8148b0191f56acf7eecfc73d99b'
shutil.copy2(cp,out/'checkpoint.pt')
saved=json.loads(Path('/home/cyd/fpga/model/mem_4500/training_config.json').read_text());sys.argv=['eval','--task=eval_motion','--headless'];args=get_args();cfg,train=task_registry.get_cfgs('car_robot')
def apply(obj,d):
 for k,v in d.items():
  if isinstance(v,dict) and hasattr(obj,k) and not isinstance(getattr(obj,k),dict):apply(getattr(obj,k),v)
  else:setattr(obj,k,v)
apply(cfg,saved['env']);apply(train,saved['train']);cfg.env.num_envs=832;args.num_envs=832;cfg.env.spawn_radius=0.;cfg.env.reset_velocity_scale=0.;cfg.env.episode_length_s=21
cfg.terrain.mesh_type='plane';cfg.terrain.curriculum=False;cfg.noise.add_noise=False;cfg.domain_rand.push_robots=False;cfg.commands.heading_command=False;cfg.seed=17;train.seed=17;set_seed(17);torch.set_num_threads(2)
for name in ['parking_heading','parking_position','parking_velocity','parking_yaw']:setattr(cfg.rewards.scales,name,0.)
cases=[('stand',0.,0.)]+[('v%+.1f'%v,v,0.) for v in [.1,.2,.3,-.1,-.2,-.3]]+[('yaw%+.1f'%y,0.,y) for y in [.5,1.,2.,-.5,-1.,-2.]]
class EvalRobot(CarRobot):
 def _resample_commands(self,ids):
  super()._resample_commands(ids)
  table=torch.tensor([[v,y,.1305,0.] for _,v,y in cases],device=self.device)
  self.commands[ids]=table[ids//64]
 def center(self):
  offset=torch.tensor([-.080000002,-.006656542,0.],device=self.device).repeat(self.num_envs,1)
  return (self.root_states[:,:3]+quat_rotate(self.root_states[:,3:7],offset))[:,:2]
 def check_termination(self):
  super().check_termination()
  if not getattr(self,'ev_ready',False):return
  alive=self.ev_alive;tilt=torch.rad2deg(torch.acos((-self.projected_gravity[:,2]).clamp(-1,1)));v=self.task_forward_velocity();y=self.base_ang_vel[:,2]
  self.ev_peak=torch.where(alive,torch.maximum(self.ev_peak,tilt),self.ev_peak)
  self.ev_tilt+=torch.where(alive,tilt*tilt,0.);self.ev_count+=alive.float()
  valid=alive & (self.ev_time>=2.)
  for name,value in [('speed',v),('speed_abs',v.abs()),('speed_error',(v-self.commands[:,0]).abs()),('yaw',y),('yaw_error',(y-self.commands[:,1]).abs()),('speed_sq',v*v),('yaw_sq',y*y)]:
   setattr(self,'ev_'+name,getattr(self,'ev_'+name)+torch.where(valid,value,0.))
  self.ev_sample+=valid.float();self.ev_drift=torch.where(alive,torch.linalg.norm(self.center()-self.ev_origin,dim=1),self.ev_drift)
  failed=alive & self.reset_buf.bool() & ~self.time_out_buf.bool();self.ev_first=torch.where(failed,torch.full_like(self.ev_first,self.ev_time),self.ev_first);self.ev_alive &= ~failed

task_registry.register('eval_motion',EvalRobot,cfg,train);env,_=task_registry.make_env('eval_motion',args=args,env_cfg=cfg)
runner=OnPolicyRunner(env,class_to_dict(train),None,device=args.rl_device);runner.load(str(out/'checkpoint.pt'),load_optimizer=False);actor=runner.get_inference_policy(env.device)
env.ev_ready=False;env.reset();obs,hist=env.get_observations();env.ev_alive=torch.ones(832,dtype=torch.bool,device=env.device);env.ev_first=torch.full((832,),20.,device=env.device);env.ev_origin=env.center().clone()
for n in ['peak','tilt','count','speed','speed_abs','speed_error','yaw','yaw_error','speed_sq','yaw_sq','sample','drift']:setattr(env,'ev_'+n,torch.zeros(832,device=env.device))
env.ev_ready=True;print('EVALUATION_START',cp.name,flush=True)
for step in range(round(20/env.dt)):
 env.ev_time=(step+1)*env.dt
 with torch.no_grad():
  act=actor(obs,hist);act=act[0] if isinstance(act,(tuple,list)) else act;obs,_,_,_,_,hist=env.step(act)
 if step%625==624:print('PROGRESS',env.ev_time,int(env.ev_alive.sum()),flush=True)
results=[]
for i,(name,v,y) in enumerate(cases):
 sl=slice(i*64,(i+1)*64);n=env.ev_sample[sl].sum().clamp_min(1)
 row={'case':name,'target_mps':v,'target_yaw_radps':y,'survived':int(env.ev_alive[sl].sum()),'environments':64,'mean_first_failure_or_horizon_s':float(env.ev_first[sl].mean()),'tilt_rms_deg':float((env.ev_tilt[sl].sum()/env.ev_count[sl].sum().clamp_min(1)).sqrt()),'mean_peak_tilt_deg':float(env.ev_peak[sl].mean()),'final_drift_m':float(env.ev_drift[sl].mean())}
 for key in ['speed','speed_abs','speed_error','yaw','yaw_error','speed_sq','yaw_sq']:row[key+'_after2s']=float(getattr(env,'ev_'+key)[sl].sum()/n)
 row['speed_std_after2s']=max(0.,row['speed_sq_after2s']-row['speed_after2s']**2)**.5
 row['yaw_std_after2s']=max(0.,row['yaw_sq_after2s']-row['yaw_after2s']**2)**.5
 results.append(row)
result={'checkpoint':cp.name,'sha256':hashlib.sha256((out/'checkpoint.pt').read_bytes()).hexdigest(),'seed':17,'seconds':20,'height_m':.1305,'terrain':'plane','noise':False,'pushes':False,'reset_velocity_scale':0,'exclude_after_first_failure':True,'results':results}
(out/'result.json').write_text(json.dumps(result,indent=2));(root/'result.json').write_text(json.dumps(result,indent=2));print('EVALUATION_RESULT',json.dumps(result),flush=True)
