"""Interactive same-policy comparison. Keys control all three simulated robots."""
import sys,json,time
from pathlib import Path
import isaacgym
from isaacgym import gymapi
import torch
torch.set_num_threads(2)
sys.path.insert(0,'/home/cyd/fpga/model/wheel_velocity_original/deploy')
sys.path.insert(0,'/home/cyd/fpga/rl_ws/_car_tools')
from hardware_env import HardwareRobot
from wheel_legged_gym.envs import task_registry
from wheel_legged_gym.utils import get_args
from wheel_legged_gym.utils.helpers import class_to_dict
from wheel_legged_gym.rsl_rl.runners.on_policy_runner import OnPolicyRunner
from viewer_time_hud import draw_time_hud
root=Path(__file__).parent
class PreviewRobot(HardwareRobot):
 def __init__(self,*a,**kw):
  super().__init__(*a,**kw)
  self.mode=torch.tensor([0,3,5],device=self.device)
  self.phase[:]=0
  self.env_origins[:]=torch.tensor([[-.8,0.,0.],[0.,0.,0.],[.8,0.,0.]],device=self.device)
  for i,color in enumerate([(0.15,.45,.95),(.95,.45,.10),(.85,.15,.45)]):
   for body in range(self.num_bodies):self.gym.set_rigid_body_color(self.envs[i],self.actor_handles[i],body,gymapi.MESH_VISUAL,gymapi.Vec3(*color))
saved=json.loads(Path('/home/cyd/fpga/model/wheel_velocity_original/parking_config.json').read_text())
sys.argv=['preview','--task=car_robot','--num_envs=3']
args=get_args();cfg,train=task_registry.get_cfgs('car_robot')
def apply(obj,values):
 for k,v in values.items():
  if isinstance(v,dict) and hasattr(obj,k) and not isinstance(getattr(obj,k),dict):apply(getattr(obj,k),v)
  else:setattr(obj,k,v)
apply(cfg,saved['env']);apply(train,saved['train'])
cfg.env.num_envs=3;cfg.env.spawn_radius=0.;cfg.env.reset_velocity_scale=0.;cfg.env.episode_length_s=3600.
cfg.noise.add_noise=False;cfg.domain_rand.push_robots=False;cfg.terrain.mesh_type='plane';cfg.terrain.curriculum=False
cfg.commands.curriculum=False;cfg.commands.heading_command=False;cfg.commands.parking_extra_fraction=0.
task_registry.register('car_robot',PreviewRobot,cfg,train)
env,_=task_registry.make_env('car_robot',args=args,env_cfg=cfg)
runner=OnPolicyRunner(env,class_to_dict(train),None,device=args.rl_device)
checkpoint='/home/cyd/fpga/model/wheel_velocity_original/model_4500.pt'
runner.load(checkpoint,load_optimizer=False);policy=runner.get_inference_policy(env.device)
command=torch.tensor([0.,0.,535/4096],device=env.device)
height_target=535;applied_height=535;height_timer=0.
def commands(ids):env.commands[ids,:3]=command;env.commands[ids,3]=0.
env._resample_commands=commands

def reset():
 global obs,hist,height_target,applied_height,height_timer
 command[:2]=0;command[2]=535/4096;height_target=535;applied_height=535;height_timer=0
 env.ever_fault[:]=0;env.reset();env.held_timing_torque[:]=0
 obs,hist=env.get_observations();env.obs_history.copy_(obs.repeat(1,env.obs_history_length));obs,hist=env.get_observations()
reset()
keys={gymapi.KEY_W:'forward',gymapi.KEY_S:'backward',gymapi.KEY_A:'left',gymapi.KEY_D:'right',gymapi.KEY_U:'raise',gymapi.KEY_J:'lower',gymapi.KEY_SPACE:'stop',gymapi.KEY_R:'reset',gymapi.KEY_V:'camera',gymapi.KEY_P:'pause',gymapi.KEY_1:'low',gymapi.KEY_2:'high',gymapi.KEY_ESCAPE:'quit'}
for key,act in keys.items():env.gym.subscribe_viewer_keyboard_event(env.viewer,key,act)
start=time.monotonic();simtime=0.;render_deadline=0.;status_deadline=0.;pause=False;snapshot=False
names=['BLUE ORIGINAL','ORANGE HW 1.0','PINK HW 1.8']

def set_camera():
 env.gym.viewer_camera_look_at(env.viewer,None,gymapi.Vec3(1.4,-2.5,1.65),gymapi.Vec3(0.,.1,.12))
set_camera()
# This viewer handles rendering independently; do not let env.step render twice.
env.render=lambda *a,**kw:None
print('HARDWARE_PREVIEW_READY: 3 robots / original4500 / HW1x / HW1.8x',flush=True)
print('W/S +/-0.1m/s, A/D +/-0.5rad/s, U/J height +/-1cm, SPACE zero motion, R reset all, V restore camera, P pause, ESC exit',flush=True)
with torch.no_grad():
 while not env.gym.query_viewer_has_closed(env.viewer):
  now=time.monotonic()
  if now>=render_deadline:
   for event in env.gym.query_viewer_action_events(env.viewer):
    if event.value<=0:continue
    act=event.action
    if act=='quit':raise SystemExit
    if act=='forward':command[0]=min(.6,float(command[0])+.1)
    if act=='backward':command[0]=max(-.6,float(command[0])-.1)
    if act=='left':command[1]=min(2.9,float(command[1])+.5)
    if act=='right':command[1]=max(-2.9,float(command[1])-.5)
    if act=='raise':height_target=min(778,height_target+41)
    if act=='lower':height_target=max(348,height_target-41)
    if act=='low':height_target=348
    if act=='high':height_target=758
    if act=='stop':command[:2]=0
    if act=='reset':reset()
    if act=='camera':set_camera()
    if act=='pause':pause=not pause
   env.gym.fetch_results(env.sim,True);env.gym.step_graphics(env.sim)
   pose=env.gym.get_viewer_camera_transform(env.viewer,env.envs[0]);fwd=pose.r.rotate(gymapi.Vec3(0,0,1))
   origin=env.gym.get_env_origin(env.envs[0])
   camera=[pose.p.x+origin.x,pose.p.y+origin.y,pose.p.z+origin.z];target=[camera[0]+fwd.x,camera[1]+fwd.y,camera[2]+fwd.z]
   speeds=env.task_forward_velocity().cpu().tolist()
   rows=['4500 HW RESPONSE TEST',f'V CMD {float(command[0]):.2f} M S   YAW {float(command[1]):.2f}',f'HEIGHT {float(command[2])*100:.1f} CM',*[f'{name}  V {v:.2f}' for name,v in zip(names,speeds)],'W S SPEED   A D TURN','U J HEIGHT  SPACE STOP','R RESET  V CAMERA  P PAUSE',f'SIM {simtime:.1f} S   RATE {simtime/max(now-start,.01):.2f} X']
   if pause:rows.append('PAUSED')
   draw_time_hud(env.gym,env.viewer,camera,target,rows)
   env.gym.draw_viewer(env.viewer,env.sim,True);render_deadline=now+1/20
   if not snapshot and now-start>5:
    env.gym.write_viewer_image_to_file(env.viewer,str(root/'preview_hardware.png'));snapshot=True
  if pause:
   time.sleep(.01);continue
  height_timer+=env.dt
  while height_timer>=.01:
   height_timer-=.01;applied_height+=max(-1,min(1,height_target-applied_height))
  command[2]=applied_height/4096;env.commands[:,:3]=command
  action=policy(obs,hist);action=action[0] if isinstance(action,(tuple,list)) else action
  obs,_,_,done,_,hist=env.step(action);simtime+=env.dt
  if now>=status_deadline:
   status={'wall_time':time.strftime('%Y-%m-%d %H:%M:%S'),'checkpoint':checkpoint,'groups':['original','FPGA timing PID gain1','FPGA timing PID gain1.8'],'command':command.cpu().tolist(),'height_target_m':height_target/4096,'body_speeds':env.task_forward_velocity().cpu().tolist(),'tilts_deg':torch.rad2deg(torch.acos((-env.projected_gravity[:,2]).clamp(-1,1))).cpu().tolist(),'motor_faults':env.ever_fault.cpu().tolist(),'sim_time_s':simtime,'time_ratio':simtime/max(now-start,.01),'motor_assumption':'stall torque0.30Nm, no-load1.0m/s; not measured'}
   (root/'preview_state.json').write_text(json.dumps(status,indent=2));print(json.dumps(status),flush=True);status_deadline=now+1
  time.sleep(max(0.,start+simtime-time.monotonic()))
env.gym.destroy_viewer(env.viewer);env.gym.destroy_sim(env.sim)
