"""通用评估：对任意 task + checkpoint 测倾角/存活。

用法: python eval_generic.py <task> <load_run> <checkpoint>
"""
import isaacgym
from wheel_legged_gym.envs import task_registry
from wheel_legged_gym.utils import get_args
import torch, sys

TASK = sys.argv[1]
LOAD_RUN = sys.argv[2]
CKPT = int(sys.argv[3])

sys.argv = ["e", f"--task={TASK}", "--headless", "--num_envs=64"]
args = get_args()
cfg, _ = task_registry.get_cfgs(name=TASK)
cfg.env.num_envs = 64
cfg.noise.add_noise = False
cfg.domain_rand.push_robots = False

args.resume = True
args.load_run = LOAD_RUN
args.checkpoint = CKPT

env, _ = task_registry.make_env(name=TASK, args=args, env_cfg=cfg)
_, tcfg = task_registry.get_cfgs(name=TASK)
tcfg.runner.resume = True
tcfg.runner.load_run = LOAD_RUN
tcfg.runner.checkpoint = CKPT
runner, _ = task_registry.make_alg_runner(env=env, name=TASK, args=args, train_cfg=tcfg)
pol = runner.get_inference_policy(device=env.device)

obs, hist = env.get_observations()
print(f">>> {TASK} 策略已加载", flush=True)
print(f"{'t(s)':>7s}{'tilt_mean':>11s}{'tilt_max':>10s}{'z':>9s}", flush=True)

tilt = torch.zeros(env.num_envs)
with torch.inference_mode():
    for i in range(2500):
        out = pol(obs, hist)
        act = out[0] if isinstance(out, (tuple, list)) else out
        obs, _, rew, done, info, hist = env.step(act)
        if i % 250 == 0:
            pg = env.projected_gravity
            tilt = torch.rad2deg(torch.arccos(torch.clip(-pg[:, 2], -1, 1)))
            print(f"{i*env.dt:7.2f}{tilt.mean().item():11.2f}{tilt.max().item():10.2f}"
                  f"{env.root_states[:,2].mean().item():9.4f}", flush=True)

pg = env.projected_gravity
tilt = torch.rad2deg(torch.arccos(torch.clip(-pg[:, 2], -1, 1)))
print(f"\n>>> {TASK}: 倾角 mean={tilt.mean().item():.2f}  "
      f"<10deg={int((tilt<10).sum())}/{env.num_envs}  "
      f">45deg={int((tilt>45).sum())}/{env.num_envs}")
