# 原 model_3000 配置：只改轮子输出为线速度

用户在2026-10-15撤回了新奖励、动作范围和网络的重新设计，要求沿用原来输出轮扭矩的版本，只改变轮子输出语义。

基线为 `/home/cyd/fpga/model/reference/training_config.json`，原检查点为 `/home/cyd/fpga/model/model_3000.pt`。新训练不加载旧扭矩权重，从头学习速度动作。

正式训练配置仅修改：

| 配置项 | 原值 | 新值 |
|---|---|---|
| control.wheel_control | torque | velocity |
| control.vel_action_scale | 20 | 0.6 / 0.0325 ≈ 18.461538 rad/s |

原框架已有速度反馈模式：轮子力矩 = 0.025 ×（目标关节角速度 − 实际关节角速度），随后受URDF effort限幅。这是P轮速闭环；本次未另加PI，也未改控制器增益。

左右轮动作独立，不投影为相等或反向。保留原URDF关节轴约定：物理左轮目标线速度为 −0.6×左轮动作，物理右轮为 +0.6×右轮动作，两者单位均为m/s，身体前进为正。真机部署时需按该符号换算。

舵机仍沿用原版：高度IK基础姿态 + action×0.03rad（约±1.72°）修正，不再使用1500+1000×action的全范围脉宽映射。升降基础姿态随原高度指令范围0.085–0.19m变化。

网络仍为ActorCriticSequence：28维观测、140维历史、3维潜变量、原encoder/actor/critic尺寸。奖励、PPO、探索参数和地形配置均沿用原配置。权威配置中的地形为heightfield及原地形课程，尽管原部署包README称作平地版；这里以实际保存配置为准。

远端目录 `/root/autodl-tmp/car_original_velocity`；正式训练4096环境、3000轮，输出 `logs/original_velocity_v1`。预检使用64环境和2轮PPO，仅用于验证轮速控制公式、左右轮独立性、有限物理状态与观测/网络维数，不作为已稳定跟踪或已收敛的证明。正式输出的config_changes.json列出仅两项配置差异。

旧7000轮速度版及旧model_3000部署文件均保留，不烧录FPGA。
