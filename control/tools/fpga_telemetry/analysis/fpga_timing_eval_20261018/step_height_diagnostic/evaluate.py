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
n=len(MODES)*len(CASES)*PER_CASE
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
cases=[(0.,0.,.1305),(.3,0.,.1305),(-.3,0.,.1305),(0.,1.,.1305),(0.,-1.,.1305),(0.,0.,.085),(0.,0.,.185)]
case_commands=torch.tensor(cases,device=env.device).repeat_interleave(PER_CASE,0).repeat(len(MODES),1)
commands=torch.zeros(n,4,device=env.device);commands[:,2]=.1305
env._resample_commands=lambda ids:env.commands.__setitem__(ids,commands[ids])
env.reset();obs,hist=env.get_observations();env.obs_history.copy_(obs.repeat(1,env.obs_history_length));obs,hist=env.get_observations()
alive=torch.ones(n,dtype=torch.bool,device=env.device);first_fail=torch.full((n,),18.,device=env.device)
no_stop=torch.ones_like(alive);first_stop=first_fail.clone()
# Accumulate only active-command stable interval, excluding after first physical failure/stop.
metrics=torch.zeros(n,9,device=env.device);count=torch.zeros(n,device=env.device)
phase_samples=(torch.arange(n,device=env.device)%PER_CASE)==0
selected=phase_samples.nonzero(as_tuple=False).flatten()
fields=['time_s','mode','case','forward_command','yaw_command','height_command','left_target','right_target','left_speed','right_speed','body_forward','yaw_actual','roll_deg','pitch_deg','gyro_x','tilt_deg','left_duty','right_duty','left_fault','right_fault','motor_stop','done','alive']
f=(root/'telemetry.csv').open('w',buffering=1);writer=csv.writer(f);writer.writerow(fields)
start=time.monotonic()
for step in range(round(18/env.dt)):
 t=step*env.dt
 if t>=3:commands[:,2]=case_commands[:,2]
 commands[:,:2]=case_commands[:,:2] if 3<=t<13 else 0.
 env.commands.copy_(commands)
 if step in [round(3/env.dt),round(13/env.dt)]:env.compute_observations();obs,hist=env.get_observations()
 with torch.no_grad():
  action=actor(obs,hist);action=action[0] if isinstance(action,(tuple,list)) else action
  obs,_,_,done,_,hist=env.step(action)
  failed=done.bool()&~env.time_out_buf.bool()
  first_fail=torch.where(alive&failed,torch.full_like(first_fail,(step+1)*env.dt),first_fail);alive &=~failed
  stopped=(env.ever_fault!=0).any(1)
  first_stop=torch.where(no_stop&stopped,torch.full_like(first_stop,(step+1)*env.dt),first_stop);no_stop &=~stopped
  valid=alive&no_stop&~done.bool()
  g=env.projected_gravity;roll=torch.rad2deg(torch.atan2(-g[:,1],-g[:,2]));pitch=torch.rad2deg(torch.atan2(g[:,0],torch.sqrt(g[:,1]**2+g[:,2]**2)))
  tilt=torch.rad2deg(torch.acos((-g[:,2]).clamp(-1,1)))
  physical=env.dof_vel[:,env.wheel_ids]*torch.tensor([-.0325,.0325],device=env.device)
  target=env.actions[:,env.wheel_ids]*torch.tensor([-.6,.6],device=env.device)
  target=torch.where((env.mode>=2)[:,None],env.last_hw_target.float()/1000,target)
  body=env.task_forward_velocity()
  if 5<=t<13:
   vals=torch.stack([(body-commands[:,0]).abs(),(env.base_ang_vel[:,2]-commands[:,1]).abs(),tilt**2,(env.base_height-commands[:,2]).abs(),body,env.base_ang_vel[:,2],(target-physical).abs().mean(1),env.base_ang_vel[:,0]**2,(env.pid['duty'].abs()>=2375).float().mean(1)],1)
   metrics+=vals*valid[:,None];count+=valid
  if step%5==0:
   data=torch.cat([commands[:,:3],target,physical,body[:,None],env.base_ang_vel[:,2:3],roll[:,None],pitch[:,None],env.base_ang_vel[:,0:1],tilt[:,None],env.pid['duty'].float(),env.ever_fault.float(),stopped[:,None].float(),done[:,None].float(),alive[:,None].float()],1)[selected].cpu().tolist()
   for idx,row in enumerate(data):writer.writerow([round((step+1)*env.dt,6),MODES[idx//len(CASES)],CASES[idx%len(CASES)],*row])
 if step%250==0:print('PROGRESS',step,'sim_s',round(t,2),'alive',int(alive.sum()),'not_stopped',int(no_stop.sum()),'wall_s',round(time.monotonic()-start,1),flush=True)
f.close()
results=[]
for m,mode in enumerate(MODES):
 for j,case in enumerate(CASES):
  sl=slice((m*len(CASES)+j)*PER_CASE,(m*len(CASES)+j+1)*PER_CASE)
  total=float(count[sl].sum());mean=(metrics[sl].sum(0)/max(total,1)).cpu().tolist()
  r=dict(mode=mode,case=case,physical_survival=float(alive[sl].float().mean()),no_motor_stop=float(no_stop[sl].float().mean()),first_failure_s=first_fail[sl].cpu().tolist(),first_motor_stop_s=first_stop[sl].cpu().tolist(),valid_samples=int(total),forward_mae=mean[0] if total else None,yaw_mae=mean[1] if total else None,tilt_rms_deg=math.sqrt(mean[2]) if total else None,height_mae_m=mean[3] if total else None,mean_forward=mean[4] if total else None,mean_yaw=mean[5] if total else None,wheel_mae=mean[6] if total else None,foreaft_angular_rms=math.sqrt(mean[7]) if total else None,pwm_saturated_fraction=mean[8] if total else None,motor_faults=env.ever_fault[sl].cpu().tolist())
  results.append(r);print('RESULT',json.dumps(r),flush=True)
manifest={'checkpoint':str(checkpoint),'checkpoint_sha256':hashlib.sha256(checkpoint.read_bytes()).hexdigest(),'saved_config':str(saved_path),'effective_config':{'env':class_to_dict(cfg),'train':class_to_dict(train)},'modes':MODES,'cases':CASES,'environments_per_mode_case':PER_CASE,'phase_offsets_ms':list(range(0,20,2)),'seconds':18,'command_on_s':3,'command_off_s':13,'score_interval_s':[5,13],'physics_dt_s':.002,'policy_period_s':.008,'model_latency_approx_s':.002,'hardware_sample_s':.01,'servo_hold_s':.02,'pid_test':json.loads((root/'pid_verification.json').read_text()),'motor_proxy':{'formula':'tau_physical=stall_torque*(duty/2500 - wheel_linear_speed/no_load_mps); coast at zero target or stopped','stall_torque_sweep_Nm':[.15,.3,.5],'no_load_mps':1.,'measured':False},'not_emulated':['real mass/COM changes','real servo internal dynamics/backlash/deadzone','motor current/battery voltage','real sensor bias/noise/I2C driver and fusion dynamics','RTL full fixed-point network; uses original FP32 checkpoint','sub-2ms PWM switching/deadtime and exact sampling phase'], 'source_hashes':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path('/home/cyd/fpga/fpga/src/main.v'),Path('/home/cyd/fpga/fpga/src/motor520_closed_loop.v'),Path('/home/cyd/fpga/fpga/src/motor520_speed.v'),Path('/home/cyd/fpga/fpga/src/servo_driver.v'),Path(__file__).with_name('hardware_env.py'),Path(__file__).with_name('fpga_pid.py')]},'results':results,'wall_seconds':time.monotonic()-start}
(root/'results.json').write_text(json.dumps(manifest,indent=2));print('EVALUATION_DONE',flush=True)
