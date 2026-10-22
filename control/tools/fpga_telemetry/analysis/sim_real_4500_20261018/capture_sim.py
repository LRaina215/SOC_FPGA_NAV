import sys,json,csv,math,hashlib
from pathlib import Path
import isaacgym
import torch
torch.set_num_threads(2)
sys.path.insert(0,'/home/cyd/fpga/model/wheel_velocity_original/deploy')
from parking_env import ParkingRobot
from wheel_legged_gym.envs import task_registry
from wheel_legged_gym.utils import get_args
from wheel_legged_gym.utils.helpers import class_to_dict,set_seed
from wheel_legged_gym.rsl_rl.runners.on_policy_runner import OnPolicyRunner
out=Path(__file__).parent
config_path=Path('/home/cyd/fpga/model/wheel_velocity_original/parking_config.json')
ckpt=Path('/home/cyd/fpga/model/wheel_velocity_original/model_4500.pt')
saved=json.loads(config_path.read_text())
sys.argv=['capture','--task=car_robot','--headless','--num_envs=96']
args=get_args();cfg,train=task_registry.get_cfgs('car_robot')
def apply(obj,values):
 for k,v in values.items():
  if isinstance(v,dict) and hasattr(obj,k) and not isinstance(getattr(obj,k),dict):apply(getattr(obj,k),v)
  else:setattr(obj,k,v)
apply(cfg,saved['env']);apply(train,saved['train'])
cfg.env.num_envs=96;cfg.env.spawn_radius=0.;cfg.env.reset_velocity_scale=0.;cfg.env.episode_length_s=30.
cfg.terrain.mesh_type='plane';cfg.terrain.curriculum=False;cfg.noise.add_noise=False;cfg.domain_rand.push_robots=False
cfg.commands.curriculum=False;cfg.commands.heading_command=False;cfg.commands.parking_extra_fraction=0.
set_seed(71)
task_registry.register('car_robot',ParkingRobot,cfg,train)
env,_=task_registry.make_env('car_robot',args=args,env_cfg=cfg)
runner=OnPolicyRunner(env,class_to_dict(train),None,device=args.rl_device);runner.load(str(ckpt),load_optimizer=False)
actor=runner.get_inference_policy(env.device)
commands=torch.zeros(96,4,device=env.device);commands[:,2]=.1305
case_v=torch.tensor([0.]*32+[.3]*32+[-.3]*32,device=env.device)
env._resample_commands=lambda ids:env.commands.__setitem__(ids,commands[ids])
env.reset();obs,hist=env.get_observations();env.obs_history.copy_(obs.repeat(1,env.obs_history_length));obs,hist=env.get_observations()
fields=['time_s','env_id','case','forward_command','yaw_command','height_command','left_target','right_target','left_speed','right_speed','body_forward','roll_deg','pitch_deg','gyro_x','gyro_y','gyro_z','gravity_x','gravity_y','gravity_z','lh_rad','lk_rad','rh_rad','rk_rad','lh_action','lk_action','rh_action','rk_action','left_torque','right_torque','done','alive']
f=(out/'simulation.csv').open('w');w=csv.writer(f);w.writerow(fields)
left,right=env.left_indices[2],env.right_indices[2]
legs=[env.left_indices[0],env.left_indices[1],env.right_indices[0],env.right_indices[1]]
alive=torch.ones(96,dtype=torch.bool,device=env.device)
stride=5
for step in range(round(15/env.dt)):
 t=step*env.dt
 commands[:,0]=case_v if 3<=t<13 else 0.
 env.commands.copy_(commands)
 # Reflect new commands in the current observation before inferring.
 if step==round(3/env.dt) or step==round(13/env.dt):env.compute_observations();obs,hist=env.get_observations()
 with torch.no_grad():
  action=actor(obs,hist);action=action[0] if isinstance(action,(tuple,list)) else action
  # Match env.step's clipping, compare the applied action with post-step state.
  action=action.clamp(-cfg.normalization.clip_actions,cfg.normalization.clip_actions)
  obs,_,_,done,_,hist=env.step(action)
  failed=done.bool()&~env.time_out_buf.bool();alive &=~failed
  if step%stride==0:
   g=env.projected_gravity
   roll=torch.rad2deg(torch.atan2(-g[:,1],-g[:,2]));pitch=torch.rad2deg(torch.atan2(g[:,0],torch.sqrt(g[:,1]**2+g[:,2]**2)))
   data=torch.cat([commands[:,:3],(-action[:,left]*.6)[:,None],(action[:,right]*.6)[:,None],(-env.dof_vel[:,left]*.0325)[:,None],(env.dof_vel[:,right]*.0325)[:,None],env.task_forward_velocity()[:,None],roll[:,None],pitch[:,None],env.base_ang_vel,g,env.dof_pos[:,legs],action[:,legs],env.torques[:,[left,right]],done[:,None].float(),alive[:,None].float()],dim=1).cpu().tolist()
   for i,row in enumerate(data):w.writerow([round((step+1)*env.dt,6),i,['zero','forward','reverse'][i//32],*row])
 if step%250==0:print('STEP',step,'t',round(t,2),'alive',int(alive.sum()),flush=True)
f.close()
manifest={'checkpoint':str(ckpt),'sha256':hashlib.sha256(ckpt.read_bytes()).hexdigest(),'saved_config':str(config_path),'effective_config':{'env':class_to_dict(cfg),'train':class_to_dict(train)},'seconds':15,'command_step_on_s':3,'command_step_off_s':13,'output_multiplier':1,'noise':False,'pushes':False,'sample_interval_s':env.dt*stride,'survivors':int(alive.sum()),'notes':'Simulation uses true body angular velocity and projected gravity, no BMI088 sensor emulation. No hardware programming or source changes.'}
(out/'simulation_manifest.json').write_text(json.dumps(manifest,indent=2))
print('CAPTURE_DONE',flush=True)
