"""Local IsaacGym viewer for the exact saved car experiment config."""
import argparse
import json
from pathlib import Path
import sys
import time
from collections import deque
import isaacgym
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
a=p.parse_args()
saved=json.loads(Path(a.config).read_text())
sys.argv=['preview','--task=car_robot','--num_envs=1']
args=get_args()
if a.device=='cpu':
    args.sim_device='cpu';args.sim_device_type='cpu'
    args.rl_device='cpu';args.use_gpu=False;args.use_gpu_pipeline=False
cfg,train=task_registry.get_cfgs('car_robot')
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
cfg.commands.curriculum=False
cfg.commands.heading_command=False
env,_=task_registry.make_env('car_robot',args=args,env_cfg=cfg)
runner=OnPolicyRunner(env,class_to_dict(train),None,device=args.rl_device)
runner.load(a.checkpoint,load_optimizer=False)
policy=runner.get_inference_policy(env.device)
command=torch.tensor([0.,0.,.1305],device=env.device)
def commands(ids): env.commands[ids,:3]=command
env._resample_commands=commands
env.reset();obs,hist=env.get_observations()
keymap={gymapi.KEY_W:'forward',gymapi.KEY_S:'backward',gymapi.KEY_A:'left',gymapi.KEY_D:'right',
        gymapi.KEY_U:'raise',gymapi.KEY_J:'lower',gymapi.KEY_SPACE:'stop',gymapi.KEY_R:'reset',
        gymapi.KEY_1:'low',gymapi.KEY_2:'high'}
for key,action in keymap.items(): env.gym.subscribe_viewer_keyboard_event(env.viewer,key,action)
minimum_height=cfg.control.min_reference_height
start=time.monotonic();lastprint=0;steps=0;sim_time=0.
sim_start=env.gym.get_sim_time(env.sim)
timings=deque([(start,0.)]);rate=0.;snapshot_saved=False
next_render=0.
render_time=0.
costs=deque(maxlen=125)

def render(sync_frame_time=True):
    global next_render,render_time
    now=time.monotonic()
    if now<next_render:
        return
    if env.gym.query_viewer_has_closed(env.viewer): raise SystemExit
    for event in env.gym.query_viewer_action_events(env.viewer):
        if event.value<=0: continue
        action=event.action
        if action=='QUIT': raise SystemExit
        if action=='forward': command[0]=min(.6,float(command[0])+.1)
        if action=='backward': command[0]=max(-.6,float(command[0])-.1)
        if action=='left': command[1]=min(2.9,float(command[1])+.5)
        if action=='right': command[1]=max(-2.9,float(command[1])-.5)
        if action=='raise': command[2]=min(.19,float(command[2])+.01)
        if action=='lower': command[2]=max(minimum_height,float(command[2])-.01)
        if action=='low': command[2]=minimum_height
        if action=='high': command[2]=.19
        if action=='stop': command[:2]=0
        if action=='reset': env.reset()
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
    ])
    env.gym.draw_viewer(env.viewer,env.sim,True)
    render_time=time.monotonic()-now
    next_render=time.monotonic()+1./max(a.render_fps,1.)
    global snapshot_saved
    if a.snapshot and not snapshot_saved and time.monotonic()-start>5:
        env.gym.write_viewer_image_to_file(env.viewer,a.snapshot)
        snapshot_saved=True
env.render=render
print('PREVIEW_READY',a.checkpoint,flush=True)
print('W/S velocity, A/D yaw, U/J height, 1=%.1fcm 2=19cm, Space=stop, R=reset'%(minimum_height*100),flush=True)
with torch.inference_mode():
    while time.monotonic()-start<a.seconds:
        stepstart=time.monotonic()
        out=policy(obs,hist);action=out[0] if isinstance(out,(tuple,list)) else out
        policy_time=time.monotonic()-stepstart
        render_time=0.
        physics_start=time.monotonic()
        obs,_,_,done,_,hist=env.step(action)
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
            status=dict(checkpoint=a.checkpoint,device=a.device,elapsed=elapsed,sim_time_s=sim_time,
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
