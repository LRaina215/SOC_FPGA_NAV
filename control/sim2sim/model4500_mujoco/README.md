# model_4500 Isaac Gym → MuJoCo

原4500权重已在MuJoCo 3.14.0运行，未重新训练或修改权重。先完成单车迁移验证，不使用Isaac Gym运行时。

## 打开窗口

```bash
bash /home/cyd/fpga/sim2sim/model4500_mujoco/open_mujoco.sh
```

默认打开一个MuJoCo机器人窗口及独立实时曲线窗口。相机可用鼠标自由操作，不自动跟随。

- W/S：车身目标速度加减0.1m/s，范围±0.6m/s。
- A/D：目标偏航角速度加减0.5rad/s，范围±2.9rad/s。
- U/J：目标高度加减1cm，范围8.5～19cm；1/2选择最低/最高。
- 空格：前进、偏航指令归零，继续执行平衡策略。
- R：重置；倾倒超过30度或机身接地持续超过50ms自动重置，并打印EPISODE_FAILED。
- 操作机器人时先点击MuJoCo窗口；曲线显示最近20秒仿真时间，CSV在telemetry/。
- 关闭曲线不影响仿真；关闭仿真后曲线保留最后的数据。`--no-plot`可禁用曲线。

## 保持的模型接口

- 相同训练URDF及STL：`rl_ws/Wheel-Legged-Gym-SF_small/resources/robots/chy/urdf/chy.urdf`。
- 总质量2.171117111228kg，7个刚体、6个驱动关节；保留CAD质量、惯量、关节位置与轴向。
- 使用原训练等效开链机构，没有另外改成真实五连杆闭链。
- 2ms物理步长，4倍降采样，8ms/125Hz策略周期。
- 28维观测，5帧历史（140维，旧→新且包含当前帧），ELU编码器与actor；动作限幅±1。
- 动作顺序：左髋、左膝、左轮、右髋、右膝、右轮。MuJoCo关节排列不同，按名称映射。
- 观测使用CAD机身轴的角速度和投影重力，不使用惯量主轴坐标。腿位置输入为上次动作×0.03，腿速度输入为0，轮子位置输入为0。
- 原高度IK及0.05m/s参考高度变化率；腿位置残差±0.03rad。
- 腿PD=60/0.06，最大1.96Nm；轮速度P=0.025，最大0.5Nm；轮动作最大18.461538rad/s，对应0.6m/s。
- 用MuJoCo原生affine执行器表达相同饱和PD公式，让implicitfast处理速度导数。没有改为关节位置直接赋值或根速度注入。
- 原训练设置replace_cylinder_with_capsule=true，因此碰撞圆柱同样转换成胶囊；外观仍是CAD网格。

## 接口检查

```bash
/home/cyd/miniforge3/envs/mujoco/bin/python /home/cyd/fpga/sim2sim/model4500_mujoco/verify_contract.py
```

结果见contract_verification.json：256组随机输入与原导出的TorchScript参考网络一致（动作和latent最大误差均0）；原始可执行观测函数误差小于1e-6；200次IK误差小于1e-10；32组PD饱和力矩误差小于1e-12；CAD角速度坐标和重力方向、历史末帧均已检查。

## MuJoCo固定指令评估

```bash
bash /home/cyd/fpga/sim2sim/model4500_mujoco/open_mujoco.sh --eval
```

每项20秒、单次确定性初始化，误差统计跳过初始1秒；不等同于大样本鲁棒性测试。所有七项均未倾倒。

| 工况 | 20秒存活 | 平均前进m/s | 平均偏航rad/s | 倾角RMS度 |
|---|---|---:|---:|---:|
| stop | 通过 | 0.0064 | 0.0173 | 3.74 |
| forward | 通过 | 0.1635 | -0.0041 | 3.57 |
| reverse | 通过 | -0.1174 | 0.0127 | 3.84 |
| left | 通过 | -0.0436 | 1.3090 | 3.62 |
| right | 通过 | 0.0699 | -1.1915 | 3.87 |
| low | 通过 | -0.0170 | 0.0553 | 5.00 |
| high | 通过 | 0.0091 | -0.0168 | 2.84 |

前进0.2m/s时实际约0.1635m/s，后退-0.2m/s时实际约-0.1174m/s，与Gym中后退偏弱的现象一致。停车轮子路程约0.211m，最终位置偏移约0.117m；位置偏移仅为辅助指标，路程与位移分开记录。

完整数值：results/evaluation.json；采样曲线：results/*.csv。

## 适用范围

这是同一策略在另一物理引擎中的基本运行验证，未进行实车测试。MuJoCo接触求解、静/动态摩擦混合、恢复系数与PhysX不同；地面滑动摩擦取0.6，恢复系数0.15没有直接等价映射。不可将本次结果称为两引擎动力学完全相同，也未覆盖外推扰动、岩石地形或真车内八轮胎。

模型生成与差异记录见build_model.py、model_manifest.json。重建：`python build_model.py`（使用mujoco环境）。

参考：[MuJoCo执行器及积分器](https://mujoco.readthedocs.io/en/stable/computation/)、[XML接口](https://mujoco.readthedocs.io/en/latest/XMLreference.html)。

## 固定后退指令偏置

`open_mujoco_bias.sh` 启动时增加固定 −0.01 m/s 车身速度指令偏置。输入零指令时送入模型 −0.01 m/s；+0.2送入+0.19，-0.2送入-0.21，转向和重置后也保留偏置，最后限幅±0.6。偏置不累计，权重和FPGA代码不变。曲线Command显示偏置后的实际模型指令；viewer_status.json另存requested_command。

```bash
bash /home/cyd/fpga/sim2sim/model4500_mujoco/open_mujoco_bias.sh
```

原`open_mujoco.sh`默认偏置仍为0，可用于无偏置基准测试。偏置是模型输入指令，不保证实际车速恰好-0.01m/s。
