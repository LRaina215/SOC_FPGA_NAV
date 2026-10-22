"""Flat-ground payload sensitivity test; never edits the original robot.

Payload is a rigid 10x6x4 cm box merged into base mass, COM and inertia using
the parallel-axis theorem. Offsets are relative to the unloaded base COM;
local -Y is rear, +Z is up. Collision geometry is unchanged (no box contacts).
First failures are permanent in the report, even if the framework resets.
"""
import argparse
import json
import sys
from pathlib import Path
import isaacgym
from isaacgym import gymapi
import numpy as np
import torch
from wheel_legged_gym.envs import task_registry
from wheel_legged_gym.utils import get_args
from wheel_legged_gym.utils.helpers import class_to_dict
from wheel_legged_gym.rsl_rl.runners.on_policy_runner import OnPolicyRunner

p = argparse.ArgumentParser()
p.add_argument('--controller', choices=['feedback', 'policy'], default='feedback')
p.add_argument('--mode', choices=['stand', 'motion', 'jump'], default='stand')
p.add_argument('--samples', type=int, default=8)
p.add_argument('--seconds', type=float, default=20.)
p.add_argument('--seed', type=int, default=1)
p.add_argument('--output', required=True)
a = p.parse_args()
root = Path(__file__).resolve().parent.parent
cases = [('unloaded', 0., 0., 0.), ('1kg_center', 1., 0., 0.),
         ('1kg_rear3cm', 1., .03, 0.), ('1kg_rear6cm', 1., .06, 0.),
         ('1kg_rear10cm', 1., .10, 0.), ('1kg_rear6cm_up5cm', 1., .06, .05)]
n = len(cases)*a.samples
sys.argv = ['payload', '--task=car_robot', '--headless', '--num_envs='+str(n)]
args = get_args()
args.sim_device = args.sim_device_type = args.rl_device = 'cpu'
args.use_gpu = args.use_gpu_pipeline = False
torch.set_num_threads(2)
cfg, train = task_registry.get_cfgs('car_robot')
directory = root/'_car_tools/results'
config_path = (directory/'motion_v6_config.json' if a.controller == 'feedback'
               else directory/'logs/car_balance/full_center_v4/config.json')
saved = json.loads(config_path.read_text())
def apply(target, values):
    for key, value in values.items():
        if isinstance(value, dict) and hasattr(target, key) and not isinstance(getattr(target, key), dict):
            apply(getattr(target, key), value)
        else:
            setattr(target, key, value)
apply(cfg, saved['env']); apply(train, saved['train'])
cfg.env.num_envs = n
cfg.seed = train.seed = a.seed
cfg.env.episode_length_s = a.seconds+2
cfg.env.reset_velocity_scale = .1
cfg.noise.add_noise = False
cfg.domain_rand.push_robots = False
cfg.commands.curriculum = cfg.commands.heading_command = cfg.commands.boundary_sampling = False
cfg.terrain.curriculum = False
env, _ = task_registry.make_env('car_robot', args=args, env_cfg=cfg)

def inertia_array(body):
    return np.array([[getattr(getattr(body.inertia, row), col) for col in 'xyz'] for row in 'xyz'])
def parallel_axis(mass, delta):
    return mass*(np.dot(delta, delta)*np.eye(3)-np.outer(delta, delta))
body_reports = []
for i in range(n):
    name, m, rear, up = cases[i//a.samples]
    props = env.gym.get_actor_rigid_body_properties(env.envs[i], env.actor_handles[i])
    base = props[0]
    old_m = float(base.mass)
    old_com = np.array([base.com.x, base.com.y, base.com.z])
    payload_com = old_com+np.array([0., -rear, up])
    new_com = (old_m*old_com+m*payload_com)/(old_m+m)
    # x dimension is sideways, y dimension is length, z dimension is height.
    sx, sy, sz = .06, .10, .04
    box_inertia = m/12*np.diag([sy*sy+sz*sz, sx*sx+sz*sz, sx*sx+sy*sy])
    inertia = (inertia_array(base)+parallel_axis(old_m, old_com-new_com)
               +box_inertia+parallel_axis(m, payload_com-new_com))
    base.mass = old_m+m
    base.com = gymapi.Vec3(*new_com.tolist())
    for j, row in enumerate('xyz'):
        for k, col in enumerate('xyz'):
            setattr(getattr(base.inertia, row), col, float(inertia[j, k]))
    env.gym.set_actor_rigid_body_properties(env.envs[i], env.actor_handles[i], props, False)
    actual = env.gym.get_actor_rigid_body_properties(env.envs[i], env.actor_handles[i])
    assert abs(actual[0].mass-(old_m+m)) < 1e-6
    assert np.allclose(inertia_array(actual[0]), inertia, atol=1e-7)
    assert np.allclose([actual[0].com.x, actual[0].com.y, actual[0].com.z], new_com, atol=1e-7)
    body_reports.append(dict(case=name, total_mass_kg=sum(b.mass for b in actual),
                             payload_mass_kg=m, payload_com_xyz=payload_com.tolist(),
                             base_com_xyz=new_com.tolist(), base_inertia=inertia.tolist()))
command = torch.tensor([0., 0., .1305], device=env.device)
env._resample_commands = lambda ids: env.commands.__setitem__((ids, slice(0, 3)), command)
env.reset()
obs, hist = env.get_observations()
if a.controller == 'policy':
    checkpoint = directory/'logs/car_balance/full_center_v4/model_1000.pt'
    runner = OnPolicyRunner(env, class_to_dict(train), None, device='cpu')
    runner.load(str(checkpoint), load_optimizer=False)
    policy = runner.get_inference_policy('cpu')
else:
    checkpoint = None
    policy = lambda obs, hist: torch.zeros_like(env.actions)
failed = torch.zeros(n, dtype=torch.bool)
first_failure = torch.full((n,), a.seconds)
peak_tilt = torch.zeros(n)
peak_clearance = torch.zeros(n)
air_time = torch.zeros(n)
landed = torch.zeros(n, dtype=torch.bool)
original = env.check_termination
t = 0.
def check():
    original()
    newly = (~failed)&env.reset_buf&(~env.time_out_buf)
    first_failure[newly] = t
    failed.logical_or_(newly)
    tilt = torch.rad2deg(torch.acos(torch.clamp(-env.projected_gravity[:, 2], -1., 1.)))
    active = ~failed
    peak_tilt[:] = torch.maximum(peak_tilt, torch.where(active, tilt, torch.zeros_like(tilt)))
    if a.mode == 'jump':
        contact = env.contact_forces[:, env.feet_indices, 2].sum(1)>1.
        clearance = env.wheel_clearances().min(1).values
        airborne = active&(~contact)&(clearance>.005)&(env.jump_phase>=2)&(env.jump_phase<=3)
        peak_clearance[:] = torch.maximum(peak_clearance, torch.where(airborne, clearance, torch.zeros_like(clearance)))
        air_time.add_(airborne.float()*env.dt)
        landed.logical_or_(active&contact&(air_time>.08))
env.check_termination = check
records = []
with torch.inference_mode():
    for step in range(round(a.seconds/env.dt)):
        t = (step+1)*env.dt
        if a.mode == 'motion':
            if t<2: command[:]=torch.tensor([0.,0.,.1305]); stage='settle'
            elif t<6: command[:]=torch.tensor([.5,0.,.1305]); stage='forward'
            elif t<10: command[:]=torch.tensor([-.5,0.,.1305]); stage='backward'
            elif t<13: command[:]=torch.tensor([0.,1.,.1305]); stage='left'
            elif t<16: command[:]=torch.tensor([0.,-1.,.1305]); stage='right'
            elif t<18: command[:]=torch.tensor([0.,0.,.085]); stage='low'
            else: command[:]=torch.tensor([0.,0.,.19]); stage='high'
        else:
            stage=a.mode
        if a.mode == 'jump' and step == round(3./env.dt):
            print('JUMP_REQUESTED', env.request_jump(), flush=True)
        env.commands[:,:3]=command
        actions=policy(obs, hist)
        if isinstance(actions, (list, tuple)): actions=actions[0]
        obs, _, _, _, _, hist = env.step(actions)
        if step%10 == 0:
            tilt=torch.rad2deg(torch.acos(torch.clamp(-env.projected_gravity[:,2],-1.,1.)))
            records.append(dict(t=t,stage=stage,active=(~failed).tolist(),
                                v=env.task_forward_velocity().tolist(),yaw=env.base_ang_vel[:,2].tolist(),
                                height=env.base_height.tolist(),tilt=tilt.tolist(),
                                base_contact_n=torch.norm(env.contact_forces[:, 0, :], dim=1).tolist(),
                                wheel_contact_n=env.contact_forces[:, env.feet_indices, 2].sum(1).tolist()))
results=[]
for j, (name,m,rear,up) in enumerate(cases):
    ids=list(range(j*a.samples,(j+1)*a.samples))
    alive=[i for i in ids if not failed[i]]
    selected=[r for r in records if r['t']>a.seconds-2]
    means={key:(sum(r[key][i] for r in selected for i in alive)/(len(selected)*len(alive))
                if alive and selected else None) for key in ['v','yaw','height','tilt','base_contact_n','wheel_contact_n']}
    results.append(dict(case=name,payload_kg=m,rear_offset_m=rear,up_offset_m=up,
                        survivors=len(alive),samples=a.samples,final_2s=means,
                        first_failure_s=first_failure[ids].tolist(),peak_tilt_deg=peak_tilt[ids].tolist(),
                        jump_clearance_m=peak_clearance[ids].tolist(),
                        passed_jump_count=sum(bool((~failed[i])&landed[i]&(peak_clearance[i]>=.1)) for i in ids)))
report=dict(options=vars(a),checkpoint=str(checkpoint) if checkpoint else None,
            config=str(config_path),payload_geometry_m=[.06,.10,.04],
            offset_reference='unloaded base COM; -Y rear, +Z up',
            rigid_bodies=body_reports,results=results)
out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
(out/'summary.json').write_text(json.dumps(report,indent=2))
(out/'trace.json').write_text(json.dumps(records))
print('PAYLOAD_RESULT',json.dumps(results),flush=True)
