"""Reproducible car balance experiments; uses only car task overrides.
Run with IsaacGym/python and framework root in PYTHONPATH.
"""
import argparse
import json
import os
import sys
import time

import isaacgym
import torch
from wheel_legged_gym.envs import task_registry
from wheel_legged_gym.utils import get_args
from wheel_legged_gym.utils.helpers import class_to_dict
from wheel_legged_gym.rsl_rl.runners.on_policy_runner import OnPolicyRunner

parser = argparse.ArgumentParser()
parser.add_argument('mode', choices=['diagnose', 'train', 'eval'])
parser.add_argument('--profile', choices=['standing', 'motion', 'full'], default='standing')
parser.add_argument('--num-envs', type=int, default=4096)
parser.add_argument('--iterations', type=int, default=1000)
parser.add_argument('--checkpoint')
parser.add_argument('--output', default='logs/car_balance')
parser.add_argument('--seed', type=int, default=1)
parser.add_argument('--seconds', type=float, default=20)
parser.add_argument('--case', default='0,0,0.1305')
parser.add_argument('--reset-velocity-scale', type=float, default=0.1,
                    help='Eval: random initial six-axis velocities in +/-0.5*scale')
parser.add_argument('--height-sweep', action='store_true',
                    help='Eval: command 0.09m then 0.19m halfway through the episode')
parser.add_argument('--wheel-torque', type=float, default=0.5,
                    help='Full-scale open-loop wheel torque approximation, Nm')
parser.add_argument('--noise-std', type=float, default=None,
                    help='Train: reset action exploration std after checkpoint loading')
parser.add_argument('--boundary-sampling', action='store_true')
parser.add_argument('--min-height', type=float, default=.09)
parser.add_argument('--velocity-origin', choices=['cad', 'center'], default='cad')
parser.add_argument('--fixed-learning-rate', type=float, default=None)
parser.add_argument('--reset-critic', action='store_true')
opt = parser.parse_args()
sys.argv = ['balance', '--task=car_robot', '--headless', '--num_envs='+str(opt.num_envs)]
args = get_args()
cfg, train = task_registry.get_cfgs('car_robot')
cfg.seed = train.seed = opt.seed
cfg.commands.curriculum = False
cfg.commands.heading_command = False
cfg.commands.boundary_sampling = opt.boundary_sampling
cfg.asset.forward_reference_x = -0.08075 if opt.velocity_origin == 'center' else 0.0
cfg.domain_rand.push_robots = False
cfg.noise.add_noise = False
cfg.env.reset_velocity_scale = opt.reset_velocity_scale if opt.mode == 'eval' else 0.0
cfg.env.dof_vel_use_pos_diff = False
cfg.control.leg_integral_gain = 0.0
cfg.control.vel_action_scale = opt.wheel_torque / cfg.control.damping['Wheel']
cfg.control.min_reference_height = opt.min_height
# Fixed legs isolate the wheel balancing task first.
cfg.control.pos_action_scale = 0.0 if opt.profile == 'standing' else 0.03
cfg.control.height_reference = opt.profile != 'standing'
cfg.commands.ranges.lin_vel_x = [0, 0] if opt.profile == 'standing' else [-0.6, 0.6]
cfg.commands.ranges.ang_vel_yaw = [0, 0] if opt.profile == 'standing' else [-2.9, 2.9]
cfg.commands.ranges.height = [0.1305, 0.1305] if opt.profile != 'full' else [opt.min_height, 0.19]
for key in class_to_dict(cfg.rewards.scales):
    setattr(cfg.rewards.scales, key, 0.0)
for key, value in dict(upright=3., orientation=-3., base_height=1.,
                       tracking_lin_vel=2., tracking_ang_vel=1.5,
                       yaw_error=-0.3, forward_error=-2.,
                       action_rate=-0.005, termination=-5.).items():
    setattr(cfg.rewards.scales, key, value)
cfg.rewards.clip_single_reward = 100
train.algorithm.learning_rate = 0.0003
if opt.fixed_learning_rate is not None:
    train.algorithm.learning_rate = opt.fixed_learning_rate
    train.algorithm.schedule = 'fixed'
train.algorithm.entropy_coef = 0.001
train.policy.init_noise_std = 0.3
train.runner.save_interval = 200
os.makedirs(opt.output, exist_ok=True)
with open(os.path.join(opt.output, 'config.json'), 'w') as f:
    json.dump({'options': vars(opt), 'env': class_to_dict(cfg), 'train': class_to_dict(train)}, f, indent=2)
env, _ = task_registry.make_env('car_robot', args=args, env_cfg=cfg)
print('DOF_NAMES', env.dof_names, flush=True)
print('PHYSICS', env.dt, env.torque_limits.tolist(), flush=True)

if opt.mode == 'diagnose':
    env.reset()
    env.projected_gravity[:] = torch.tensor([0., 0., -1.], device=env.device)
    env.base_lin_vel[:] = 0.
    env.base_ang_vel[:] = 0.
    env.commands[:, 0] = 0.3
    env.base_lin_vel[:, 1] = 0.3
    assert torch.allclose(env._reward_tracking_lin_vel(), torch.ones(env.num_envs, device=env.device))
    print('FORWARD_Y_REWARD_OK', flush=True)
    env.base_ang_vel[:, 2] = 2.9
    env.base_lin_vel[:, 1] = .3 - 2.9*cfg.asset.forward_reference_x
    assert torch.allclose(env.task_forward_velocity(), torch.full((env.num_envs,), .3, device=env.device), atol=1e-6)
    print('ROTATING_REFERENCE_VELOCITY_OK', flush=True)
    env.base_ang_vel[:, 2] = 0
    env.base_lin_vel[:, 1] = .3
    results = []
    for deg in [0, 10, 20, 30, 55]:
        import math
        a = math.radians(deg)
        env.projected_gravity[:] = torch.tensor([math.sin(a), 0., -math.cos(a)], device=env.device)
        results.append({'tilt_deg': deg, 'rewards': {
            name: {'raw': float(fun()[0]), 'weighted': float(fun()[0]*env.reward_scales[name]),
                   'clipped': float(torch.clamp(fun()[0]*env.reward_scales[name], -100*env.dt, 100*env.dt))}
            for name, fun in zip(env.reward_names, env.reward_functions)}})
    with open(os.path.join(opt.output, 'reward_diagnostic.json'), 'w') as f:
        json.dump(results, f, indent=2)
    print('REWARD_DIAGNOSTIC', json.dumps(results), flush=True)
    # The exact URDF FK should reproduce both wheel centers.
    from isaacgym import gymtorch
    bodies = gymtorch.wrap_tensor(env.gym.acquire_rigid_body_state_tensor(env.sim)).view(env.num_envs, -1, 13)
    for s, side in enumerate(['Right', 'Left']):
        i = env.gym.find_actor_rigid_body_handle(env.envs[0], env.actor_handles[0], side+'Wheel_link')
        relative = bodies[:, i, :3] - env.root_states[:, :3]
        idx = env.right_indices if s == 0 else env.left_indices
        expected = env._leg_end_y(*idx[:2], 1 if s == 0 else -1)+env.hip_y_offset
        print('FK_SIDE', s, 'expected', float(expected[0]), 'body_y', float(relative[0, 1]), flush=True)
    env.reset()
    for _ in range(100):
        env.step(torch.zeros(env.num_envs, 6, device=env.device))
    assert torch.isfinite(env.obs_buf).all()
    print('SMOKE_OK', flush=True)
    sys.exit(0)

runner = OnPolicyRunner(env, class_to_dict(train), opt.output if opt.mode == 'train' else None, device=args.rl_device)
if opt.checkpoint:
    runner.load(opt.checkpoint, load_optimizer=opt.mode == 'train' and not opt.reset_critic)
if opt.mode == 'train' and opt.reset_critic:
    for layer in runner.alg.actor_critic.critic:
        if isinstance(layer, torch.nn.Linear):
            layer.reset_parameters()
    torch.nn.init.zeros_(runner.alg.actor_critic.critic[-1].weight)
    torch.nn.init.zeros_(runner.alg.actor_critic.critic[-1].bias)
if opt.mode == 'train' and opt.fixed_learning_rate is not None:
    runner.alg.learning_rate = opt.fixed_learning_rate
    for group in runner.alg.optimizer.param_groups:
        group['lr'] = opt.fixed_learning_rate
if opt.mode == 'train':
    # 文件编号只表示本轮更新次数；起点检查点另存在 config.json。
    runner.current_learning_iteration = 0
if opt.mode == 'train' and opt.noise_std is not None:
    with torch.no_grad():
        runner.alg.actor_critic.std.fill_(opt.noise_std)
if opt.mode == 'train':
    runner.learn(opt.iterations, init_at_random_ep_len=False)
    print('TRAIN_DONE', opt.output, flush=True)
    sys.exit(0)

case = [float(x) for x in opt.case.split(',')]
def fixed_commands(ids):
    env.commands[ids, :3] = torch.tensor(case, device=env.device)
env._resample_commands = fixed_commands
env.reset()
obs, hist = env.get_observations()
failed = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
first_failure = torch.full((env.num_envs,), opt.seconds, device=env.device)
records = []
height_records = []
original_check = env.check_termination
current_t = 0.
def record_check():
    original_check()
    tilt = torch.rad2deg(torch.acos(torch.clamp(-env.projected_gravity[:, 2], -1, 1)))
    newly_failed = (~failed) & env.reset_buf & (~env.time_out_buf)
    first_failure[newly_failed] = current_t
    failed.logical_or_(newly_failed)
    # First trajectory only: reset episodes never count as surviving.
    active = ~failed
    if active.any():
        height_records.append((current_t, float(env.base_height[active].mean())))
        forward = env.task_forward_velocity()
        records.append(torch.stack((tilt[active].mean(), forward[active].mean(),
                                   env.base_ang_vel[active, 2].mean(), env.base_height[active].mean(),
                                   torch.abs(forward[active]-case[0]).mean(),
                                   torch.abs(env.base_ang_vel[active, 2]-case[1]).mean(),
                                   torch.abs(env.base_height[active]-case[2]).mean())).cpu().tolist())
env.check_termination = record_check
policy = runner.get_inference_policy(device=env.device)
with torch.inference_mode():
    for step in range(int(opt.seconds/env.dt)):
        current_t = (step+1)*env.dt
        if opt.height_sweep:
            case[2] = opt.min_height if current_t < opt.seconds/2 else 0.19
            env.commands[:, 2] = case[2]
        act = policy(obs, hist)
        if isinstance(act, (tuple, list)):
            act = act[0]
        obs, _, _, _, _, hist = env.step(act)
mean = torch.tensor(records[len(records)//4:]).mean(0).tolist() if records else [None]*7
report = dict(checkpoint=opt.checkpoint, profile=opt.profile, case=case, seed=opt.seed,
              velocity_origin=opt.velocity_origin,
              num_envs=env.num_envs, seconds=opt.seconds, survivors=int((~failed).sum()),
              first_failure_s_mean=float(first_failure.mean()),
              mean_tilt_deg=mean[0], mean_forward_mps=mean[1], mean_yaw_rps=mean[2], mean_height_m=mean[3],
              forward_mae=mean[4], yaw_mae=mean[5], height_mae=mean[6])
if opt.height_sweep:
    low = [h for t, h in height_records if opt.seconds/2-2 < t < opt.seconds/2]
    high = [h for t, h in height_records if t > opt.seconds-2]
    report['height_sweep'] = {'low_mean_m': sum(low)/len(low) if low else None,
                            'high_mean_m': sum(high)/len(high) if high else None}
    if low and high:
        report['height_sweep']['actual_travel_m'] = sum(high)/len(high)-sum(low)/len(low)
print('EVAL_RESULT', json.dumps(report), flush=True)
with open(os.path.join(opt.output, 'eval_'+opt.case.replace(',', '_')+'_'+str(opt.seed)+'.json'), 'w') as f:
    json.dump(report, f, indent=2)
