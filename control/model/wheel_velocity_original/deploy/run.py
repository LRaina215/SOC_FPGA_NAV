"""Original model_3000 training configuration, with wheel velocity actions."""
import argparse
import json
import sys
from pathlib import Path
import isaacgym
import torch
from wheel_legged_gym.envs import task_registry
from wheel_legged_gym.utils import get_args
from wheel_legged_gym.utils.helpers import class_to_dict, set_seed
from wheel_legged_gym.rsl_rl.runners.on_policy_runner import OnPolicyRunner

p=argparse.ArgumentParser()
p.add_argument('mode', choices=['smoke','train'])
p.add_argument('--iterations',type=int,default=3000)
p.add_argument('--output',default='logs/original_velocity_v1')
a=p.parse_args()
root=Path(__file__).resolve().parent.parent
saved=json.loads((root/'original_config.json').read_text())
modified=json.loads(json.dumps(saved))
modified['env']['control']['wheel_control']='velocity'
modified['env']['control']['vel_action_scale']=.6/.0325
def diff(x,y,path=''):
    found=[]
    for k in x:
        key=path+'.'+k if path else k
        if isinstance(x[k],dict):found+=diff(x[k],y[k],key)
        elif x[k]!=y[k]:found.append({'key':key,'old':x[k],'new':y[k]})
    return found
changes=diff(saved,modified)
assert {x['key'] for x in changes}=={'env.control.wheel_control','env.control.vel_action_scale'}
sys.argv=['original_velocity','--task=car_robot','--headless']
args=get_args()
cfg,train=task_registry.get_cfgs('car_robot')
def apply(obj,values):
    for k,v in values.items():
        if isinstance(v,dict) and hasattr(obj,k) and not isinstance(getattr(obj,k),dict):apply(getattr(obj,k),v)
        else:setattr(obj,k,v)
apply(cfg,modified['env']);apply(train,modified['train'])
set_seed(train.seed)
# Smoke uses fewer parallel robots only; formal training retains all 4096.
if a.mode=='smoke':cfg.env.num_envs=64
args.num_envs=cfg.env.num_envs
env,_=task_registry.make_env('car_robot',args=args,env_cfg=cfg)
out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
(out/'config.json').write_text(json.dumps({'env':class_to_dict(cfg),'train':class_to_dict(train)},indent=2))
(out/'config_changes.json').write_text(json.dumps(changes,indent=2))
print('CONFIG_CHANGES',json.dumps(changes),flush=True)
print('WHEEL_OUTPUT physical left=-0.6*a[LeftWheel], right=+0.6*a[RightWheel] m/s',flush=True)
if a.mode=='smoke':
    env.reset()
    action=torch.zeros_like(env.actions)
    left,right=env.left_indices[2],env.right_indices[2]
    action[:,left]=.2;action[:,right]=-.7
    velocity_before=env.dof_vel.clone()
    actual=env._compute_torques(action)
    expected=(env.d_gains*(action*cfg.control.vel_action_scale-velocity_before)*env.torques_scale)
    expected=expected.clamp(-env.torque_limits,env.torque_limits)
    assert torch.allclose(actual[:,[left,right]],expected[:,[left,right]],atol=1e-6)
    assert abs(float(action[0,left]) + float(action[0,right]))>.1
    for _ in range(100):
        obs,priv,reward,done,infos,hist=env.step(torch.zeros_like(action))
        assert obs.shape==(64,28) and priv.shape==(64,144) and hist.shape==(64,140)
        assert all(torch.isfinite(x).all() for x in (obs,priv,reward,env.torques))
    print('SMOKE mapping, independent wheels, finite physics, observation shapes PASS',flush=True)
runner=OnPolicyRunner(env,class_to_dict(train),str(out),device=args.rl_device)
runner.learn(2 if a.mode=='smoke' else a.iterations,init_at_random_ep_len=False)
print('ORIGINAL_VELOCITY_DONE',a.mode,flush=True)
