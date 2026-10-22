"""Motor-driven motion/jump evaluation, with per-environment first-failure records.
No root state, external impulse, mass, motor limit or gravity is altered.
"""
import argparse
import itertools
import json
import sys
from pathlib import Path
import isaacgym
import torch
from isaacgym import gymtorch
from wheel_legged_gym.envs import task_registry
from wheel_legged_gym.utils import get_args
from wheel_legged_gym.utils.helpers import class_to_dict

p=argparse.ArgumentParser()
p.add_argument('--mode',choices=['motion','jump','jump-scan','motion-scan'],default='motion')
p.add_argument('--output',default='logs/car_motion_v6')
p.add_argument('--num-envs',type=int,default=64)
p.add_argument('--device',choices=['cpu','cuda'],default='cuda')
p.add_argument('--velocity',type=float,default=0.)
p.add_argument('--yaw',type=float,default=0.)
p.add_argument('--ride-height',type=float,default=.1305)
p.add_argument('--config')
p.add_argument('--seconds',type=float,default=16.)
p.add_argument('--jump-count',type=int,default=1)
a=p.parse_args()
sys.argv=['motion','--task=car_robot','--headless','--num_envs='+str(a.num_envs)]
args=get_args()
if a.device=='cpu':
    args.sim_device='cpu';args.sim_device_type='cpu';args.rl_device='cpu'
    args.use_gpu=False;args.use_gpu_pipeline=False
    torch.set_num_threads(2)
cfg,train=task_registry.get_cfgs('car_robot')
cfg.control.motion_controller='feedback'
cfg.control.height_reference_rate=.125
cfg.control.jump_enabled=True
cfg.control.pos_action_scale=0.
cfg.env.episode_length_s=max(a.seconds+2,20.)
cfg.env.reset_velocity_scale=.1
cfg.commands.boundary_sampling=False
cfg.commands.heading_command=False
cfg.commands.ranges.height=[.1305,.1305]
cfg.commands.ranges.lin_vel_x=[0.,0.]
cfg.commands.ranges.ang_vel_yaw=[0.,0.]
if a.config:
    values=json.loads(Path(a.config).read_text())['env']['control']
    for key,value in values.items(): setattr(cfg.control,key,value)
motion_keys=['balance_kp','balance_kd','speed_kp','speed_ki']
jump_keys=['jump_push_s','jump_push_height','jump_air_height','jump_leg_y','jump_push_hold_s']
if a.mode=='motion-scan':
    candidates=list(itertools.product([.3,.6,1.],[.03,.05,.08],[.08,.15,.25],[.01,.03,.06]))
    cfg.env.num_envs=len(candidates)
elif a.mode=='jump-scan':
    candidates=list(itertools.product([.08,.10,.12],[.19],[.085,.10],[-.03,-.027,-.024],[0.,.02,.04,.06]))
    cfg.env.num_envs=len(candidates)
else:
    candidates=None;cfg.env.num_envs=a.num_envs
Path(a.output).mkdir(parents=True,exist_ok=True)
args.num_envs=cfg.env.num_envs
env,_=task_registry.make_env('car_robot',args=args,env_cfg=cfg)
if candidates:
    params=torch.tensor(candidates,device=env.device)
    for i,key in enumerate(motion_keys if a.mode=='motion-scan' else jump_keys):
        env.jump_parameters[key]=params[:,i]
cmd=torch.tensor([a.velocity,a.yaw,a.ride_height],device=env.device)
env._resample_commands=lambda ids: env.commands.__setitem__((ids,slice(0,3)),cmd)
env.reset()
n=env.num_envs;device=env.device
bodies=gymtorch.wrap_tensor(env.gym.acquire_rigid_body_state_tensor(env.sim)).view(n,-1,13)
wheel_ids=[env.gym.find_actor_rigid_body_handle(env.envs[0],env.actor_handles[0],side+'Wheel_link')
           for side in ['Right','Left']]
failed=torch.zeros(n,dtype=torch.bool,device=device)
first_fail=torch.full((n,),a.seconds,device=device)
peak_clearance=torch.zeros(n,device=device)
flight_time=torch.zeros(n,device=device)
peak_base_rise=torch.zeros(n,device=device)
takeoff_base=torch.zeros(n,device=device)
ever_air=torch.zeros(n,dtype=torch.bool,device=device)
air_run=torch.zeros(n,device=device)
landing_time=torch.full((n,),-1.,device=device)
takeoff_time=torch.full((n,),-1.,device=device)
peak_vz=torch.zeros(n,device=device)
motion_error=torch.zeros(n,device=device)
yaw_error=torch.zeros(n,device=device)
tilt_sum=torch.zeros(n,device=device)
height_low=torch.zeros(n,device=device);height_high=torch.zeros_like(height_low)
records=[];last_values=None;settle_times={};stage=0
jump_runs=[];jump_started=False
def save_jump_run():
    jump_runs.append({key:value.cpu().tolist() for key,value in {
        'clearance_m':peak_clearance,'base_rise_after_takeoff_m':peak_base_rise,
        'airborne_s':flight_time,'takeoff_s':takeoff_time,'landing_s':landing_time}.items()})
original=env.check_termination
t=0.
def check():
    original()
    new=(~failed)&env.reset_buf&(~env.time_out_buf)
    first_fail[new]=t;failed.logical_or_(new)
    active=~failed
    contact=env.contact_forces[:,env.feet_indices,2].sum(1)>1.
    clearance=env.wheel_clearances().min(1).values
    jumping=(env.jump_phase>=2)&(env.jump_phase<=3)
    airborne=(~contact)&(clearance>.005)&active&jumping
    air_run[:]=torch.where(airborne,air_run+env.dt,torch.zeros_like(air_run))
    started=airborne&(~ever_air)
    takeoff_base[started]=env.root_states[started,2]
    takeoff_time[started]=t
    ever_air.logical_or_(airborne)
    relevant=airborne&ever_air
    peak_clearance[:]=torch.maximum(peak_clearance,torch.where(relevant,clearance,torch.zeros_like(clearance)))
    peak_base_rise[:]=torch.maximum(peak_base_rise,torch.where(relevant,env.root_states[:,2]-takeoff_base,torch.zeros_like(clearance)))
    flight_time.add_(relevant.float()*env.dt)
    peak_vz[:]=torch.maximum(peak_vz,torch.where(active,env.root_states[:,9],torch.zeros_like(clearance)))
    landed=ever_air&contact&(landing_time<0)&active&(flight_time>.08)
    landing_time[landed]=t
env.check_termination=check
zero=torch.zeros(n,6,device=device)
with torch.inference_mode():
    for step in range(round(a.seconds/env.dt)):
        t=(step+1)*env.dt
        if a.mode in ['motion','motion-scan']:
            if t<2: command=[0.,0.,.1305];label='settle'
            elif t<5: command=[.5,0.,.1305];label='forward'
            elif t<8: command=[-.5,0.,.1305];label='backward'
            elif t<10: command=[0.,2.9,.1305];label='left'
            elif t<12: command=[0.,-2.9,.1305];label='right'
            elif t<14: command=[0.,0.,.085];label='low'
            elif t<16: command=[0.,0.,.19];label='high'
            elif t<18: command=[0.,0.,.085];label='low_full'
            else: command=[0.,0.,.19];label='high_full'
            cmd[:]=torch.tensor(command,device=device)
        else:
            cmd[:]=torch.tensor([a.velocity,a.yaw,a.ride_height],device=device);label='jump'
            if any(step==round((3.+3*j)/env.dt) for j in range(a.jump_count)):
                if jump_started: save_jump_run()
                peak_clearance.zero_();peak_base_rise.zero_();flight_time.zero_()
                ever_air.zero_();air_run.zero_();takeoff_time.fill_(-1.);landing_time.fill_(-1.)
                jump_started=True
                print('JUMP_REQUESTED',env.request_jump(),flush=True)
        env.commands[:,:3]=cmd
        env.step(zero)
        v=env.task_forward_velocity()
        tilt=torch.rad2deg(torch.acos(torch.clamp(-env.projected_gravity[:,2],-1.,1.)))
        motion_error+=torch.abs(v-cmd[0]);yaw_error+=torch.abs(env.base_ang_vel[:,2]-cmd[1]);tilt_sum+=tilt
        if (a.mode in ['motion','motion-scan'] and t>2) or a.mode in ['jump','jump-scan']:
            if step%10==0:
                records.append({'t':t,'stage':label,'v':v.cpu().tolist(),
                                'yaw':env.base_ang_vel[:,2].cpu().tolist(),
                                'height':env.base_height.cpu().tolist(),
                                'tilt':tilt.cpu().tolist(),
                                'phase':env.jump_phase.cpu().tolist()})
total=round(a.seconds/env.dt)
if jump_started: save_jump_run()
results=[]
for i in range(n):
    r=dict(env=i,ever_failed=bool(failed[i]),first_failure_s=float(first_fail[i]),
           clearance_m=float(peak_clearance[i]),base_rise_after_takeoff_m=float(peak_base_rise[i]),
           airborne_s=float(flight_time[i]),takeoff_s=float(takeoff_time[i]),landing_s=float(landing_time[i]),
           peak_vz_mps=float(peak_vz[i]),mean_tilt_deg=float(tilt_sum[i]/total),
           speed_mae_all=float(motion_error[i]/total),yaw_mae_all=float(yaw_error[i]/total))
    if candidates:r['parameters']=dict(zip(motion_keys if a.mode=='motion-scan' else jump_keys,candidates[i]))
    if a.mode in ['motion','motion-scan']:
        stages={}
        for name,end in [('forward',5),('backward',8),('left',10),('right',12),('low',14),('high',16),('low_full',18),('high_full',20)]:
            selected=[x for x in records if x['stage']==name and x['t']>end-.7]
            if selected:
                stages[name]={k:sum(x[k][i] for x in selected)/len(selected) for k in ['v','yaw','height','tilt']}
        r['stages']=stages
        for name,start,target in [('low',12,.085),('high',14,.19),('low_full',16,.085),('high_full',18,.19)]:
            # First time within 2 mm, with all remaining samples in the
            # stage staying within 2 mm: conservative settling-time estimate.
            selected=[x for x in records if x['stage']==name]
            good=[abs(x['height'][i]-target)<=.002 for x in selected]
            reached=next((j for j in range(len(good)) if all(good[j:])),None)
            r[name+'_settle_s']=selected[reached]['t']-start if reached is not None else None
        r['score']=float(failed[i])*1000+abs(stages['forward']['v']-.5)+abs(stages['backward']['v']+.5)+.2*(abs(stages['left']['yaw']-2.9)+abs(stages['right']['yaw']+2.9))
    else:
        r['jumps']=[{key:values[i] for key,values in run.items()} for run in jump_runs]
        r['clearance_m']=min((run['clearance_m'][i] for run in jump_runs),default=0.)
        r['airborne_s']=min((run['airborne_s'][i] for run in jump_runs),default=0.)
        r['takeoff_s']=min((run['takeoff_s'][i] for run in jump_runs),default=-1.)
        r['landing_s']=min((run['landing_s'][i] for run in jump_runs),default=-1.)
        r['score']=float(failed[i])*1000-max(0,float(peak_clearance[i]))+max(0,.1-float(peak_base_rise[i]))
    results.append(r)
results.sort(key=lambda x:x['score'])
report=dict(options=vars(a),env=class_to_dict(cfg),train=class_to_dict(train),
            results=results,passed_jump_count=sum(not x['ever_failed'] and x['clearance_m']>=.1 and x['airborne_s']>=.08 and x['landing_s']>0 for x in results))
Path(a.output,'summary.json').write_text(json.dumps(report,indent=2))
Path(a.output,'trace.json').write_text(json.dumps(records))
Path(a.output,'config.json').write_text(json.dumps({'env':class_to_dict(cfg),'train':class_to_dict(train)},indent=2))
print('MOTION_JUMP_RESULT',json.dumps({'passed_jump_count':report['passed_jump_count'],'best':results[:8]}),flush=True)
