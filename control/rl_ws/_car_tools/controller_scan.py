"""Controlled wheel-feedback baseline: can the model track commands at all?
48 deterministic gain combinations run together. No original source is changed.
"""
import sys,json,itertools
import isaacgym
import torch
from wheel_legged_gym.envs import task_registry
from wheel_legged_gym.utils import get_args
sys.argv=['scan','--task=car_robot','--headless','--num_envs=48']
args=get_args();cfg,_=task_registry.get_cfgs('car_robot')
cfg.noise.add_noise=False;cfg.commands.curriculum=False;cfg.commands.heading_command=False
cfg.domain_rand.push_robots=False;cfg.env.reset_velocity_scale=.1
cfg.env.dof_vel_use_pos_diff=False;cfg.control.height_reference=True
cfg.control.pos_action_scale=.03;cfg.control.leg_integral_gain=0.
cfg.commands.ranges.height=[.1305,.1305]
env,_=task_registry.make_env('car_robot',args=args,env_cfg=cfg)
gains=list(itertools.product([.3,.6,1.,1.5],[.02,.05,.1],[.03,.08],[.003,.01]))
g=torch.tensor(gains,device=env.device)
cmd=torch.tensor([0.,0.,.1305],device=env.device)
env._resample_commands=lambda ids: env.commands.__setitem__((ids,slice(0,3)),cmd)
env.reset()
failed=torch.zeros(48,dtype=torch.bool,device=env.device)
poserr=torch.zeros(48,device=env.device)
errors=torch.zeros(48,4,device=env.device)
count=0
phases=[(0,[0,0,.1305]),(4,[.6,0,.1305]),(8,[-.6,0,.1305]),(12,[0,2.9,.1305]),(16,[0,-2.9,.1305]),(20,[0,0,.09]),(24,[0,0,.19])]
phase=0
for step in range(int(30/env.dt)):
    t=step*env.dt
    if phase+1<len(phases) and t>=phases[phase+1][0]:phase+=1
    cmd[:]=torch.tensor(phases[phase][1],device=env.device)
    env.commands[:,:3]=cmd
    r,l=env.right_indices[2],env.left_indices[2]
    velocity=.0325*.5*(env.dof_vel[:,r]-env.dof_vel[:,l])
    verror=velocity-cmd[0]
    poserr=torch.clamp(poserr+verror*env.dt,-1,1)
    lean=env.projected_gravity[:,1]
    leanrate=-env.base_ang_vel[:,0]
    common=g[:,0]*lean+g[:,1]*leanrate+g[:,2]*verror+g[:,3]*poserr
    turn=.01*(cmd[1]-env.base_ang_vel[:,2])
    action=torch.zeros(48,6,device=env.device)
    action[:,r]=torch.clamp(common+turn,-.1,.1)/.5
    action[:,l]=torch.clamp(-common+turn,-.1,.1)/.5
    env.step(action)
    failed.logical_or_(env.reset_buf & ~env.time_out_buf)
    tilt=torch.rad2deg(torch.acos(torch.clamp(-env.projected_gravity[:,2],-1,1)))
    errors+=torch.stack([torch.abs(env.base_lin_vel[:,1]-cmd[0]),
                         torch.abs(env.base_ang_vel[:,2]-cmd[1]),torch.abs(env.base_height-cmd[2]),tilt],dim=1)
    count+=1
score=errors[:,0]+.2*errors[:,1]+10*errors[:,2]+.01*errors[:,3]+failed.float()*count*10
indices=torch.argsort(score)[:10].cpu().tolist()
results=[dict(gains=gains[i],ever_failed=bool(failed[i]),mae_forward=float(errors[i,0]/count),
              mae_yaw=float(errors[i,1]/count),mae_height=float(errors[i,2]/count),mean_tilt=float(errors[i,3]/count)) for i in indices]
open('logs/car_balance/controller_scan.json','w').write(json.dumps(results,indent=2))
print('CONTROLLER_SCAN',json.dumps(results),flush=True)
