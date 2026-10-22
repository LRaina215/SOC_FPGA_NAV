"""Collect actual float-policy simulation observations for quantization QA."""
import argparse,json,sys
from pathlib import Path
import isaacgym
import numpy as np
import torch
sys.path.insert(0,'/home/cyd/fpga/model/wheel_velocity_original/deploy')
from parking_env import ParkingRobot
from wheel_legged_gym.envs import task_registry
from wheel_legged_gym.utils import get_args
from wheel_legged_gym.utils.helpers import class_to_dict
from wheel_legged_gym.rsl_rl.runners.on_policy_runner import OnPolicyRunner
torch.set_num_threads(2)
p=argparse.ArgumentParser();p.add_argument('--device',choices=['cpu','cuda'],default='cuda');opt=p.parse_args()
root=Path(__file__).resolve().parent
saved=json.loads((root/'config.json').read_text())
sys.argv=['collect','--task=parking_export','--headless','--num_envs=4']
args=get_args();args.sim_device=opt.device if opt.device=='cpu' else 'cuda:0';args.sim_device_type=opt.device;args.rl_device=opt.device;args.use_gpu=opt.device=='cuda';args.use_gpu_pipeline=opt.device=='cuda'
cfg,train=task_registry.get_cfgs('car_robot')
def apply(obj,values):
    for k,v in values.items():
        if isinstance(v,dict) and hasattr(obj,k) and not isinstance(getattr(obj,k),dict):apply(getattr(obj,k),v)
        else:setattr(obj,k,v)
apply(cfg,saved['env']);apply(train,saved['train'])
cfg.env.num_envs=4;cfg.env.spawn_radius=0.;cfg.env.reset_velocity_scale=0.
cfg.terrain.mesh_type='plane';cfg.terrain.curriculum=False
cfg.noise.add_noise=False;cfg.domain_rand.push_robots=False
task_registry.register('parking_export',ParkingRobot,cfg,train)
env,_=task_registry.make_env('parking_export',args=args,env_cfg=cfg)
runner=OnPolicyRunner(env,class_to_dict(train),None,device=opt.device)
runner.load(str(root/'model_4500.pt'),load_optimizer=False)
actor=runner.get_inference_policy(opt.device)
results=[]
for name,v,y,h in [('stop',0.,0.,.1305),('forward',.4,0.,.1305),('reverse',-.4,0.,.1305),('left',0.,2.,.1305),('right',0.,-2.,.1305),('low',0.,0.,.085),('high',0.,0.,.19)]:
    cmd=torch.tensor([v,y,h,0.],device=env.device)
    def fixed(ids):env.commands[ids]=cmd
    env._resample_commands=fixed
    torch.manual_seed(1);env.reset();obs,hist=env.get_observations()
    collected_obs=[];collected_history=[];actions=[]
    first_failure=torch.full((4,),20.,device=env.device)
    alive=torch.ones(4,dtype=torch.bool,device=env.device)
    with torch.no_grad():
        for step in range(round(20/env.dt)):
            out=actor(obs,hist);act=out[0] if isinstance(out,(tuple,list)) else out
            if step%25==0:
                collected_obs.append(obs.detach().cpu().clone().numpy());collected_history.append(hist.detach().cpu().clone().numpy());actions.append(act.clamp(-1,1).detach().cpu().clone().numpy())
            obs,_,_,done,_,hist=env.step(act)
            bad=done.bool()&~env.time_out_buf.bool()
            first_failure=torch.where(alive&bad,torch.full_like(first_failure,(step+1)*env.dt),first_failure)
            alive&=~bad
    path=root/'vectors'/('rollout_'+name+'.npz')
    np.savez_compressed(path,obs=np.array(collected_obs),history=np.array(collected_history),actions=np.array(actions))
    results.append(dict(case=name,command=[v,y,h],frames=len(collected_obs),environments=4,samples=len(collected_obs)*4,survived_20s=int(alive.sum()),first_failure_seconds=first_failure.tolist()))
    print('COLLECTED',json.dumps(results[-1]),flush=True)
(root/'reports/trajectory_collection.json').write_text(json.dumps(results,indent=2))
