"""Local IsaacGym viewer for the exact saved car experiment config."""
import argparse
import json
from pathlib import Path
import sys
import time
from collections import deque
import numpy as np
for _name, _value in {'float': float, 'int': int, 'bool': bool}.items():
    if _name not in np.__dict__:setattr(np, _name, _value)
import isaacgym
sys.path.insert(0, str(Path(__file__).parent / 'deploy'))
sys.path.insert(0, '/home/cyd/fpga/rl_ws/_car_tools')
from parking_env import ParkingRobot
from isaacgym import gymapi
import torch
torch.set_num_threads(2)
from wheel_legged_gym.envs import task_registry
from wheel_legged_gym.utils import get_args
from wheel_legged_gym.utils.helpers import class_to_dict
from wheel_legged_gym.rsl_rl.runners.on_policy_runner import OnPolicyRunner
from viewer_time_hud import draw_time_hud
p=argparse.ArgumentParser()
p.add_argument('checkpoint')
p.add_argument('--config',required=True)
p.add_argument('--seconds',type=float,default=3600)
p.add_argument('--snapshot',help='Save one viewer image after five seconds for display verification')
p.add_argument('--device',choices=['cpu','cuda'],default='cpu',help='Physics and policy device for this one-robot preview')
p.add_argument('--render-fps',type=float,default=15.,help='Maximum render rate; physics continues between frames')
p.add_argument('--demo',action='store_true',help='Timed forward-jump demo for display verification')
a=p.parse_args()
saved=json.loads(Path(a.config).read_text())
sys.argv=['preview','--task=car_robot','--num_envs=1']
args=get_args()
if a.device=='cpu':
    args.sim_device='cpu';args.sim_device_type='cpu'
    args.rl_device='cpu';args.use_gpu=False;args.use_gpu_pipeline=False
cfg,train=task_registry.get_cfgs('car_robot')
task_registry.register('car_robot',ParkingRobot,cfg,train)
def apply(target, values):
    for key,value in values.items():
        if isinstance(value,dict) and hasattr(target,key) and not isinstance(getattr(target,key),dict):
            apply(getattr(target,key),value)
        else: setattr(target,key,value)
apply(cfg,saved['env']);apply(train,saved['train'])
# 老版本快照没有该字段，必须保持其 CAD 原点速度语义。
if 'forward_reference_x' not in saved['env']['asset']:
    cfg.asset.forward_reference_x=0.0
if 'min_reference_height' not in saved['env']['control']:
    cfg.control.min_reference_height=saved['env']['commands']['ranges']['height'][0]
cfg.env.num_envs=1
cfg.env.reset_velocity_scale=0.0
cfg.noise.add_noise=False
cfg.domain_rand.push_robots=False
cfg.terrain.curriculum=False
cfg.terrain.mesh_type='plane'
cfg.commands.curriculum=False
cfg.commands.heading_command=False
env,_=task_registry.make_env('car_robot',args=args,env_cfg=cfg)
feedback=cfg.control.motion_controller=='feedback'
if feedback:
    policy=lambda obs,hist: torch.zeros_like(env.actions)
else:
    runner=OnPolicyRunner(env,class_to_dict(train),None,device=args.rl_device)
    runner.load(a.checkpoint,load_optimizer=False)
    policy=runner.get_inference_policy(env.device)
command=torch.tensor([0.,0.,.1305],device=env.device)
def commands(ids): env.commands[ids,:3]=command
env._resample_commands=commands
env.reset();obs,hist=env.get_observations()
keymap={gymapi.KEY_W:'forward',gymapi.KEY_S:'backward',gymapi.KEY_A:'left',gymapi.KEY_D:'right',
        gymapi.KEY_U:'raise',gymapi.KEY_J:'lower',gymapi.KEY_SPACE:'stop',gymapi.KEY_R:'reset',
        gymapi.KEY_1:'low',gymapi.KEY_2:'high',gymapi.KEY_B:'jump'}
for key,action in keymap.items(): env.gym.subscribe_viewer_keyboard_event(env.viewer,key,action)
minimum_height=cfg.control.min_reference_height
maximum_speed=max(abs(v) for v in cfg.commands.ranges.lin_vel_x)
jump_names=['READY','CROUCH','PUSH','AIR','LAND']
start=time.monotonic();lastprint=0;steps=0;sim_time=0.
sim_start=env.gym.get_sim_time(env.sim)
timings=deque([(start,0.)]);rate=0.;snapshot_saved=False
next_render=0.
render_time=0.
costs=deque(maxlen=125)
jump_peak=0.

def render(sync_frame_time=True):
    global next_render,render_time,jump_peak
    now=time.monotonic()
    if now<next_render:
        return
    if env.gym.query_viewer_has_closed(env.viewer): raise SystemExit
    for event in env.gym.query_viewer_action_events(env.viewer):
        if event.value<=0: continue
        action=event.action
        if action=='QUIT': raise SystemExit
        if action=='forward': command[0]=min(maximum_speed,float(command[0])+.1)
        if action=='backward': command[0]=max(-maximum_speed,float(command[0])-.1)
        if action=='left': command[1]=min(2.9,float(command[1])+.5)
        if action=='right': command[1]=max(-2.9,float(command[1])-.5)
        if action=='raise': command[2]=min(.19,float(command[2])+.01)
        if action=='lower': command[2]=max(minimum_height,float(command[2])-.01)
        if action=='low': command[2]=minimum_height
        if action=='high': command[2]=.19
        if action=='stop': command[:2]=0
        if action=='reset':
            env.reset();jump_peak=0.
        if action=='jump':
            accepted=env.request_jump()
            if accepted: jump_peak=0.
            print('JUMP_REQUEST',accepted,flush=True)
    env.commands[:,:3]=command
    env.gym.fetch_results(env.sim,True)
    pos=env.root_states[0,:3].cpu().tolist()
    camera=[pos[0]+.4,pos[1]-.6,pos[2]+.30]
    target=[pos[0]-.08,pos[1],pos[2]+.02]
    env.gym.viewer_camera_look_at(env.viewer,None,gymapi.Vec3(*camera),gymapi.Vec3(*target))
    env.gym.step_graphics(env.sim)
    draw_time_hud(env.gym,env.viewer,camera,target,[
        'CLOCK '+time.strftime('%H:%M:%S'),
        'SIM   %8.2f S'%sim_time,
        'WALL  %8.2f S'%(time.monotonic()-start),
        'RATE  %8.2f X'%rate,
        'V %.2f CMD %.2f M S'%(float(env.task_forward_velocity()[0]),float(command[0])),
        'HEIGHT %.1f CM'%(float(env.base_height[0])*100),
        'MODEL 4500 - PARKING',
        'SPACE STOP | R RESET',
    ])
    env.gym.draw_viewer(env.viewer,env.sim,True)
    render_time=time.monotonic()-now
    next_render=time.monotonic()+1./max(a.render_fps,1.)
    global snapshot_saved
    snapshot_ready=sim_time>3.72 if a.demo else time.monotonic()-start>5
    if a.snapshot and not snapshot_saved and snapshot_ready:
        env.gym.write_viewer_image_to_file(env.viewer,a.snapshot)
        snapshot_saved=True
env.render=render
print('PREVIEW_READY', 'feedback motion+jump' if feedback else a.checkpoint,flush=True)
print('W/S velocity, A/D yaw, U/J height, 1=%.1fcm 2=19cm, Space=stop, R=reset'%(minimum_height*100),flush=True)
if cfg.control.jump_enabled: print('B=jump; movement commands remain active during jump',flush=True)
with torch.inference_mode():
    while time.monotonic()-start<a.seconds:
        stepstart=time.monotonic()
        if a.demo:
            command[0]=.5 if sim_time<8 else 0.
            if 3.<sim_time<3.+env.dt:
                accepted=env.request_jump()
                if accepted: jump_peak=0.
        out=policy(obs,hist);action=out[0] if isinstance(out,(tuple,list)) else out
        policy_time=time.monotonic()-stepstart
        render_time=0.
        physics_start=time.monotonic()
        obs,_,_,done,_,hist=env.step(action)
        if int(env.jump_phase[0])>0:
            jump_peak=max(jump_peak,float(env.wheel_clearances()[0].min()))
        costs.append((policy_time,render_time,time.monotonic()-physics_start-render_time))
        steps+=1
        sim_time=env.gym.get_sim_time(env.sim)-sim_start
        now=time.monotonic()
        timings.append((now,sim_time))
        while len(timings)>2 and timings[1][0]<now-3:
            timings.popleft()
        rate=(sim_time-timings[0][1])/max(now-timings[0][0],1e-6)
        elapsed=time.monotonic()-start
        if elapsed-lastprint>1:
            tilt=float(torch.rad2deg(torch.acos(torch.clamp(-env.projected_gravity[0,2],-1,1))))
            status=dict(checkpoint=None if feedback else a.checkpoint,controller=cfg.control.motion_controller,
                        jump_phase=jump_names[int(env.jump_phase[0])],device=a.device,elapsed=elapsed,sim_time_s=sim_time,
                        jump_peak_cm=jump_peak*100,
                        time_ratio=rate,command=command.cpu().tolist(),
                        mean_step_cost_ms={key:sum(c[i] for c in costs)/len(costs)*1000
                                           for i,key in enumerate(('policy','render','physics_state'))},
                        forward_mps=float(env.task_forward_velocity()[0]),yaw_rps=float(env.base_ang_vel[0,2]),
                        height_m=float(env.base_height[0]),tilt_deg=tilt,reset=bool(done[0]))
            Path(__file__).with_name('preview_state.json').write_text(json.dumps(status,indent=2))
            print(json.dumps(status),flush=True);lastprint=elapsed
        # Pace against cumulative time: fast physics steps can recover time
        # spent rendering without altering the physics/control timestep.
        time.sleep(max(0.,start+sim_time-time.monotonic()))
