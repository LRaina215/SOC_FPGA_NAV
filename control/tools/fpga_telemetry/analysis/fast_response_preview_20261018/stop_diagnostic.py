"""Same checkpoint, same nominal dynamics; vary timing/interfaces and PID proxy."""
import sys,json,csv,math,hashlib,time
from pathlib import Path
import isaacgym
import torch
torch.set_num_threads(2)
sys.path.insert(0,'/home/cyd/fpga/model/wheel_velocity_original/deploy')
from hardware_env import HardwareRobot,MODES,CASES,PER_CASE
from wheel_legged_gym.envs import task_registry
from wheel_legged_gym.utils import get_args
from wheel_legged_gym.utils.helpers import class_to_dict
from wheel_legged_gym.rsl_rl.runners.on_policy_runner import OnPolicyRunner
root=Path(__file__).parent
saved_path=Path('/home/cyd/fpga/model/wheel_velocity_original/parking_config.json')
checkpoint=Path('/home/cyd/fpga/model/wheel_velocity_original/model_4500.pt')
saved=json.loads(saved_path.read_text())
n=6
sys.argv=['eval','--task=car_robot','--headless',f'--num_envs={n}']
args=get_args();cfg,train=task_registry.get_cfgs('car_robot')
def apply(obj,values):
 for k,v in values.items():
  if isinstance(v,dict) and hasattr(obj,k) and not isinstance(getattr(obj,k),dict):apply(getattr(obj,k),v)
  else:setattr(obj,k,v)
apply(cfg,saved['env']);apply(train,saved['train'])
cfg.env.num_envs=n;cfg.env.spawn_radius=0.;cfg.env.reset_velocity_scale=0.;cfg.env.episode_length_s=30.
cfg.terrain.mesh_type='plane';cfg.terrain.curriculum=False;cfg.noise.add_noise=False;cfg.domain_rand.push_robots=False
cfg.commands.curriculum=False;cfg.commands.heading_command=False;cfg.commands.parking_extra_fraction=0.
task_registry.register('car_robot',HardwareRobot,cfg,train)
env,_=task_registry.make_env('car_robot',args=args,env_cfg=cfg)
runner=OnPolicyRunner(env,class_to_dict(train),None,device=args.rl_device);runner.load(str(checkpoint),load_optimizer=False)
actor=runner.get_inference_policy(env.device)

env.mode=torch.tensor([0,0,3,3,6,6],device=env.device);env.phase[:]=0
commands=torch.zeros(n,4,device=env.device);commands[:,2]=535/4096
env._resample_commands=lambda ids:env.commands.__setitem__(ids,commands[ids])
env.reset();obs,hist=env.get_observations();env.obs_history.copy_(obs.repeat(1,env.obs_history_length));obs,hist=env.get_observations()
names=['blue_forward','blue_reverse','orange_forward','orange_reverse','pink_forward','pink_reverse']
rows=[];start=time.monotonic()
with torch.no_grad():
 for step in range(round(16/env.dt)):
  t=step*env.dt
  commands[:,0]=torch.tensor([.2,-.2,.2,-.2,.2,-.2],device=env.device) if t>=3 else 0
  env.commands.copy_(commands)
  if step==round(3/env.dt):env.compute_observations();obs,hist=env.get_observations()
  action=actor(obs,hist);action=action[0] if isinstance(action,(tuple,list)) else action
  obs,_,_,done,_,hist=env.step(action)
  g=env.projected_gravity
  tilt=torch.rad2deg(torch.acos((-g[:,2]).clamp(-1,1)))
  physical=env.dof_vel[:,env.wheel_ids]*torch.tensor([-.0325,.0325],device=env.device)
  raw=action[:,env.wheel_ids]*torch.tensor([-.6,.6],device=env.device)
  target=torch.where((env.mode>=2)[:,None],env.last_hw_target.float()/1000,raw)
  body=env.task_forward_velocity()
  data=torch.cat([commands[:,0:1],body[:,None],raw,target,physical,env.pid['reference'].float()/1000,env.pid['duty'].float()/2500,tilt[:,None],env.base_ang_vel[:,0:1],env.ever_fault.float(),done[:,None].float(),env.episode_length_buf[:,None].float()],1).cpu().tolist()
  for i,row in enumerate(data):rows.append([round((step+1)*env.dt,6),names[i],*row])
  if step%250==0:print('DIAG_PROGRESS',round(t,2),'wall',round(time.monotonic()-start,1),flush=True)
fields=['time_s','group','command','body_speed','raw_left','raw_right','target_left','target_right','actual_left','actual_right','ref_left','ref_right','pwm_left','pwm_right','tilt_deg','gyro_x','fault_left','fault_right','done','episode_step']
with (root/'stop_diagnostic.csv').open('w') as f:
 w=csv.writer(f);w.writerow(fields);w.writerows(rows)
print('DIAGNOSTIC_DONE',flush=True)
