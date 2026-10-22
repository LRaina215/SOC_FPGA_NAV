"""Evaluate and fine-tune model_3000 without changing its actuator/network."""
import argparse,json,sys
from pathlib import Path
import isaacgym
import torch
from parking_env import ParkingRobot
from wheel_legged_gym.envs import task_registry
from wheel_legged_gym.utils import get_args
from wheel_legged_gym.utils.helpers import class_to_dict,set_seed
from wheel_legged_gym.rsl_rl.runners.on_policy_runner import OnPolicyRunner

p=argparse.ArgumentParser()
p.add_argument('mode',choices=['eval','train','smoke'])
p.add_argument('--checkpoint',required=True)
p.add_argument('--iterations',type=int,default=1500)
p.add_argument('--output',required=True)
p.add_argument('--all-commands',action='store_true')
a=p.parse_args()
root=Path(__file__).resolve().parent.parent
saved=json.loads((root/'logs/original_velocity_v1/config.json').read_text())
sys.argv=['parking','--task=parking','--headless']
args=get_args();cfg,train=task_registry.get_cfgs('car_robot')
def apply(obj,values):
    for k,v in values.items():
        if isinstance(v,dict) and hasattr(obj,k) and not isinstance(getattr(obj,k),dict):apply(getattr(obj,k),v)
        else:setattr(obj,k,v)
apply(cfg,saved['env']);apply(train,saved['train'])
cfg.commands.parking_extra_fraction=.45 if a.mode in ('train','smoke') else 0.
for k,v in {'parking_velocity':-60.,'parking_yaw':-2.,'parking_position':-12.,'parking_heading':-1.5}.items():setattr(cfg.rewards.scales,k,v)
if a.mode!='train':
    cfg.env.num_envs=64
    cfg.terrain.mesh_type='plane';cfg.terrain.curriculum=False
    cfg.env.spawn_radius=0.;cfg.env.reset_velocity_scale=0.
args.num_envs=cfg.env.num_envs
set_seed(train.seed)
task_registry.register('parking',ParkingRobot,cfg,train)
env,_=task_registry.make_env('parking',args=args,env_cfg=cfg)
out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
(out/'config.json').write_text(json.dumps({'env':class_to_dict(cfg),'train':class_to_dict(train)},indent=2))
runner=OnPolicyRunner(env,class_to_dict(train),str(out) if a.mode!='eval' else None,device=args.rl_device)
runner.load(a.checkpoint,load_optimizer=True)
if a.mode in ('train','smoke'):
    if a.mode=='smoke':
        env.reset();ids=torch.arange(env.num_envs,device=env.device)
        env.commands[:,:2]=0.;env.parking_was_stop[:]=True
        env.parking_anchor[:]=env.parking_center()-.1
        anchor=env.parking_anchor.clone()
        env.cfg.commands.parking_extra_fraction=1.
        env._resample_commands(ids)
        assert torch.equal(anchor,env.parking_anchor),'Repeated stop must preserve anchor'
        assert torch.all(env._reward_parking_position()>0)
        env.commands[:,0]=.2
        assert torch.all(env._reward_parking_position()==0),'Movement must not get parking penalty'
        env.reset_idx(ids)
        assert torch.allclose(env.parking_anchor,env.parking_center())
        env.cfg.commands.parking_extra_fraction=.45
        print('PARKING_SMOKE anchor persistence, movement gating, reset PASS',flush=True)
    start=runner.current_learning_iteration
    count=2 if a.mode=='smoke' else a.iterations
    runner.learn(count,init_at_random_ep_len=False)
    runner.current_learning_iteration=start+count
    runner.save(str(out/('model_%d.pt'%(start+count))))
    print('PARKING_TRAIN_DONE',start+count,flush=True)
else:
    actor=runner.get_inference_policy(env.device)
    cases=[('stop',0.,0.,.1305)]
    if a.all_commands:cases+=[('forward',.2,0.,.1305),('reverse',-.2,0.,.1305),('left',0.,1.,.1305),('right',0.,-1.,.1305),('low',0.,0.,.10),('high',0.,0.,.18)]
    results=[]
    for name,v,y,h in cases:
        command=torch.tensor([v,y,h,0.],device=env.device)
        def fixed(ids):env.commands[ids]=command
        env._resample_commands=fixed
        torch.manual_seed(1);env.reset();obs,hist=env.get_observations()
        origin=env.parking_center().clone();heading=env.parking_heading().clone()
        alive=torch.ones(env.num_envs,dtype=torch.bool,device=env.device)
        first=torch.full((env.num_envs,),20.,device=env.device)
        displacement=torch.zeros(env.num_envs,device=env.device)
        yaw_displacement=torch.zeros_like(displacement)
        sum_speed=sum_forward_error=sum_yaw_error=0.;samples=0
        for step in range(round(20/env.dt)):
            with torch.no_grad():actions=actor(obs,hist)
            actions=actions[0] if isinstance(actions,(tuple,list)) else actions
            obs,_,_,done,_,hist=env.step(actions)
            failed=done.bool()&~env.time_out_buf.bool()
            first=torch.where(alive&failed,torch.full_like(first,(step+1)*env.dt),first)
            valid=alive&~done.bool()
            if valid.any():
                d=torch.norm(env.parking_center()-origin,dim=1)
                displacement=torch.where(valid,d,displacement)
                e=env.parking_heading()-heading
                yaw_displacement=torch.where(valid,torch.atan2(torch.sin(e),torch.cos(e)).abs(),yaw_displacement)
                if step*env.dt>=1.:
                    sum_speed+=float(torch.norm(env.parking_velocity[valid],dim=1).sum())
                    sum_forward_error+=float((env.task_forward_velocity()[valid]-v).abs().sum())
                    sum_yaw_error+=float((env.base_ang_vel[valid,2]-y).abs().sum());samples+=int(valid.sum())
            alive&=~failed
        results.append(dict(case=name,command=[v,y,h],survived_20s_fraction=float(alive.float().mean()),mean_first_failure_seconds=float(first.mean()),mean_final_displacement_m=float(displacement.mean()),mean_final_heading_error_rad=float(yaw_displacement.mean()),mean_center_speed_m_s=sum_speed/max(samples,1),mean_forward_error_m_s=sum_forward_error/max(samples,1),mean_yaw_error_rad_s=sum_yaw_error/max(samples,1),samples_before_first_failure=samples))
        print('PARKING_EVAL',json.dumps(results[-1]),flush=True)
    (out/'evaluation.json').write_text(json.dumps({'checkpoint':a.checkpoint,'device':str(env.device),'environments':env.num_envs,'seconds':20,'exclude_after_first_failure':True,'results':results},indent=2))
