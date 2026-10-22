#!/usr/bin/env python3
"""IsaacGym 兼容性快速测试 —— 拿到新服务器后第一件事就运行它。

用法（服务器上）:
    python test_compat.py /path/to/IsaacGym_Preview_4_Package/isaacgym/python

通过 = 可以继续配置环境
失败 = 换显卡（5090 不兼容时退 4090）
"""
import sys
import os

if len(sys.argv) < 2:
    print("用法: python test_compat.py <isaacgym/python 目录>")
    sys.exit(2)

sys.path.insert(0, sys.argv[1])

print("=" * 60)
print("IsaacGym 兼容性测试")
print("=" * 60)

# 0. 先把 isaacgym 导进来（必须在 torch 之前，否则 .so 加载会冲突）
try:
    import isaacgym
    print(f"[0] isaacgym   : {isaacgym.__file__}")
except Exception as e:
    print(f"[0] ❌ isaacgym 导入失败: {e}")
    sys.exit(1)

# 1. GPU 信息
try:
    import torch
    print(f"[1] torch      : {torch.__version__}")
    print(f"    CUDA 可用  : {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        p = torch.cuda.get_device_properties(0)
        print(f"    GPU        : {p.name}")
        print(f"    计算能力   : sm_{p.major}{p.minor}")
        print(f"    显存       : {p.total_memory/1024**3:.1f} GB")
        CAP = (p.major, p.minor)
    else:
        print("    ❌ CUDA 不可用，无法继续")
        sys.exit(1)
except Exception as e:
    print(f"[1] ❌ torch 导入失败: {e}")
    sys.exit(1)

# 2. IsaacGym 导入
print()
try:
    import isaacgym
    print(f"[2] isaacgym   : {isaacgym.__file__}")
except Exception as e:
    print(f"[2] ❌ isaacgym 导入失败: {e}")
    sys.exit(1)

# 3. gymtorch（需要编译，这一步最容易暴露架构问题）
print()
print("[3] 加载 gymtorch（首次会编译，约 1-2 分钟）...")
try:
    from isaacgym import gymtorch
    print("    ✅ gymtorch OK")
except Exception as e:
    print(f"    ❌ gymtorch 失败: {e}")
    print("    → 检查 ninja / gcc 是否安装")
    sys.exit(1)

# 4. 创建 sim（这一步会加载 libPhysXGpu_64.so，5090 兼容性的关键）
print()
print("[4] 创建 PhysX GPU 仿真（关键步骤）...")
from isaacgym import gymapi
try:
    gym = gymapi.acquire_gym()
    sp = gymapi.SimParams()
    sp.dt = 1 / 500
    sp.up_axis = gymapi.UP_AXIS_Z
    sp.physx.solver_type = 1
    # graphics_device_id 必须是 -1：服务器是无头的（无 DISPLAY / 无 X socket），
    # 传 0 会去建图形上下文 -> 段错误 SIGSEGV (exit 139)。
    # 训练脚本带 --headless 时 gymutil 本来就把这个值设成 -1，所以训练没事。
    sim = gym.create_sim(0, -1, gymapi.SIM_PHYSX, sp)
    if sim is None:
        print("    ❌ create_sim 返回 None")
        sys.exit(1)
    print("    ✅ PhysX GPU 仿真创建成功")
except Exception as e:
    print(f"    ❌ 创建失败: {e}")
    print("    → 可能是显卡架构不被 libPhysXGpu_64.so 支持")
    sys.exit(1)

# 5. 实际加载一个 URDF 并步进（端到端验证）
print()
print("[5] 加载球体模型并步进（端到端）...")
import glob
guess = [
    os.path.join(os.path.dirname(sys.argv[1]), "assets"),   # isaacgym/assets
    os.path.join(sys.argv[1], "..", "..", "assets"),
]
assets = next((g for g in guess if os.path.exists(os.path.join(g, "urdf", "ball.urdf"))), None)

if assets is None:
    print("    ⚠️ 找不到 assets/urdf/ball.urdf，跳过端到端测试")
    print("       但前 4 步已通过 —— PhysX GPU 可用")
    print()
    print("=" * 60)
    print("结论: ✅ 通过（未做端到端验证）")
    print("=" * 60)
    sys.exit(0)

try:
    pp = gymapi.PlaneParams()
    pp.normal = gymapi.Vec3(0, 0, 1)
    gym.add_ground(sim, pp)
    ao = gymapi.AssetOptions()
    asset = gym.load_asset(sim, assets, "urdf/ball.urdf", ao)
    env = gym.create_env(sim, gymapi.Vec3(-1, -1, 0), gymapi.Vec3(1, 1, 2), 1)
    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(0, 0, 1.0)
    gym.create_actor(env, asset, pose, "ball", 0, 1)
    gym.prepare_sim(sim)
    for _ in range(200):
        gym.simulate(sim)
        gym.fetch_results(sim, True)
    print("    ✅ 200 步仿真完成，无崩溃")
except Exception as e:
    print(f"    ❌ 仿真失败: {e}")
    sys.exit(1)

print()
print("=" * 60)
print("结论: ✅ 全部通过 —— 这台机器可以跑 IsaacGym")
print(f"      GPU: {p.name} (sm_{p.major}{p.minor})")
print("=" * 60)
