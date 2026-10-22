"""诊断：回合到底被什么终止？统计各终止原因。"""
import isaacgym
from wheel_legged_gym.envs import task_registry
from wheel_legged_gym.utils import get_args
import torch
import math
import sys

sys.argv = ["d", "--task=car_robot", "--headless", "--num_envs=32"]
args = get_args()
cfg, _ = task_registry.get_cfgs(name="car_robot")
cfg.env.num_envs = 32
cfg.noise.add_noise = False
cfg.domain_rand.push_robots = False

env, _ = task_registry.make_env(name="car_robot", args=args, env_cfg=cfg)
print(f"max_episode_length = {env.max_episode_length}")
print(f"fail 阈值步数 = {cfg.env.fail_to_terminal_time_s / env.dt:.0f}")
print(f"tilt_fail_cos = {env.tilt_fail_cos:.4f} (倾角阈值 {cfg.env.tilt_fail_deg}deg)")
print()

# 统计
from collections import Counter
reasons = Counter()
ep_lens = []
cur_len = torch.zeros(env.num_envs, device=env.device)

tilt_hist = []
for i in range(3000):
    env.step(torch.zeros(env.num_envs, 6, device=env.device))
    cur_len += 1

    # 记录倾角分布
    if i % 500 == 0:
        pg = env.projected_gravity
        tilt = torch.rad2deg(torch.arccos(torch.clip(-pg[:, 2], -1, 1)))
        print(f"  step {i:5d}: 倾角 mean={tilt.mean().item():6.2f} "
              f"max={tilt.max().item():6.2f}  min={tilt.min().item():6.2f}  "
              f"fail_buf均值={env.fail_buf.float().mean().item():7.1f}", flush=True)

    dones = env.reset_buf.nonzero(as_tuple=False).flatten()
    if len(dones) > 0:
        for d in dones.tolist():
            ep_lens.append(cur_len[d].item())
            # 判断原因
            if env.time_out_buf[d]:
                reasons["timeout"] += 1
            elif env.fail_buf[d] > 0:
                reasons["fail_consecutive"] += 1
            elif env.edge_reset_buf[d]:
                reasons["edge"] += 1
            else:
                reasons["unknown"] += 1
        cur_len[dones] = 0

print()
print("=== 终止原因统计 ===")
for k, v in reasons.most_common():
    print(f"  {k:20s} {v}")
print()
if ep_lens:
    import statistics
    print(f"=== 回合长度 ===")
    print(f"  样本数 {len(ep_lens)}")
    print(f"  mean {statistics.mean(ep_lens):.1f}")
    print(f"  median {statistics.median(ep_lens):.1f}")
    print(f"  min {min(ep_lens)}  max {max(ep_lens)}")
