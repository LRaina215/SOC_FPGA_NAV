"""跟踪训练过程中机器人状态：倾角/高度/动作/力矩，找"为什么学不会"。"""
import isaacgym
from wheel_legged_gym.envs import task_registry
from wheel_legged_gym.utils import get_args
import torch
import sys

sys.argv = ["t", "--task=car_robot", "--headless", "--num_envs=32"]
args = get_args()
cfg, _ = task_registry.get_cfgs(name="car_robot")
cfg.env.num_envs = 32
cfg.noise.add_noise = False
cfg.domain_rand.push_robots = False
cfg.env.episode_length_s = 100     # 别超时重置

env, _ = task_registry.make_env(name="car_robot", args=args, env_cfg=cfg)

print(f"{'t':>6s}{'tilt':>8s}{'z':>8s}{'cmd_h':>8s}{'base_h':>8s}"
      f"{'act_hip':>9s}{'act_knee':>10s}{'act_wheel':>10s}"
      f"{'tau_hip':>9s}{'tau_wheel':>10s}", flush=True)

for i in range(1500):
    env.step(torch.zeros(32, 6, device=env.device))
    if i % 150 == 0:
        pg = env.projected_gravity
        tilt = torch.rad2deg(torch.arccos(torch.clip(-pg[:, 2], -1, 1))).mean().item()
        a = env.actions
        t = env.torques
        print(f"{i*env.dt:6.2f}{tilt:8.2f}"
              f"{env.root_states[:,2].mean().item():8.4f}"
              f"{env.commands[:,2].mean().item():8.4f}"
              f"{env.base_height.mean().item():8.4f}"
              f"{a[:,0].mean().item():9.4f}{a[:,1].mean().item():10.4f}"
              f"{a[:,2].mean().item():10.4f}"
              f"{t[:,0].mean().item():9.4f}{t[:,2].mean().item():10.4f}", flush=True)

print(f"\n=== 终止统计 ===")
print(f"  reset_buf 数量 = {int(env.reset_buf.sum().item())}/{env.num_envs}")
print(f"  fail_buf 均值   = {env.fail_buf.float().mean().item():.1f}")
print(f"  倾角分布: <10={int((tilt<10).sum()) if tilt.numel()>1 else '?'}")
pg = env.projected_gravity
t = torch.rad2deg(torch.arccos(torch.clip(-pg[:, 2], -1, 1)))
print(f"  final tilt: mean={t.mean().item():.2f} min={t.min().item():.2f} max={t.max().item():.2f}")
