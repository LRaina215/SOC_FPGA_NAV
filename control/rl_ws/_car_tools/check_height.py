"""检查 base_height 奖励是否正常工作。

怀疑点：cfg.terrain.measure_heights=True 但 mesh_type="plane"，
measured_heights 可能无效，导致 base_height 是垃圾值，
进而 base_height 奖励无意义。
"""
import isaacgym
from wheel_legged_gym.envs import task_registry
from wheel_legged_gym.utils import get_args
import torch
import numpy as np
import sys

sys.argv = ["c", "--task=car_robot", "--headless", "--num_envs=4"]
args = get_args()
cfg, _ = task_registry.get_cfgs(name="car_robot")
cfg.env.num_envs = 4
cfg.noise.add_noise = False
cfg.domain_rand.push_robots = False

print(f"terrain.mesh_type        = {cfg.terrain.mesh_type}")
print(f"terrain.measure_heights  = {cfg.terrain.measure_heights}")

env, _ = task_registry.make_env(name="car_robot", args=args, env_cfg=cfg)

print(f"\n=== 初始状态 ===")
print(f"  root_states[:,2]  = {env.root_states[:2,2].cpu().numpy()}")
print(f"  measured_heights  = {env.measured_heights}")
mh = env.measured_heights
if torch.is_tensor(mh):
    print(f"    shape={mh.shape}  min={mh.min().item():.5f} max={mh.max().item():.5f}")
print(f"  base_height       = {env.base_height[:2].cpu().numpy()}")
print(f"  commands[:,2]     = {env.commands[:2,2].cpu().numpy()}")
print(f"  高度误差          = {(env.base_height-env.commands[:,2])[:2].cpu().numpy()}")

print(f"\n=== 步进 100 步后 ===")
for i in range(100):
    env.step(torch.zeros(4, 6, device=env.device))
print(f"  root_states[:,2]  = {env.root_states[:2,2].cpu().numpy()}")
print(f"  measured_heights  = {env.measured_heights if not torch.is_tensor(env.measured_heights) else env.measured_heights[:2].cpu().numpy()}")
print(f"  base_height       = {env.base_height[:2].cpu().numpy()}")
print(f"  commands[:,2]     = {env.commands[:2,2].cpu().numpy()}")

# 直接算奖励
r = env._reward_base_height()
print(f"\n  _reward_base_height() = {r[:2].cpu().numpy()}  (范围 0~1，越大越好)")
print(f"  权重 = {env.reward_scales['base_height']}")
print(f"  每步实际贡献 = {(r*env.reward_scales['base_height'])[:2].cpu().numpy()}")

print(f"\n=== 其他奖励项的当前值 ===")
for name in ["orientation", "leg_together", "nominal_state", "lin_vel_z",
             "tracking_lin_vel", "tracking_ang_vel", "dof_acc", "torques"]:
    fn = getattr(env, "_reward_" + name, None)
    if fn:
        try:
            v = fn()
            s = env.reward_scales.get(name, 0)
            print(f"  {name:20s} raw={v[:2].cpu().numpy()}  scale={s}  贡献={(v*s)[:2].cpu().numpy()}")
        except Exception as e:
            print(f"  {name:20s} ERROR {e}")
