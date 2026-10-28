# 云拓 · FPGA 自主导航与轮腿运动控制

目标板：小梅哥 ACX750（高云 GW5AT-LV138PG484AC1/I0；导航历史工程记录版本B，控制工程记录C，统一构建前核对实物）。架构依据《嵌赛-FPGA-选题3.pdf》第 13–14 页：
感知、定位、规划、避障与轮腿控制都在这一块 FPGA 上完成（RTL + 软核），RK3588 只提供视觉目标与任务，**导航不依赖 RK3588**。

```text
XT-M60 --UDP--> 帧接收 -> 深度转点云 -> 坐标变换 -> 滤波/栅格化 -> 滚动高程图 -> 可通行性分析 v3(代价图)
                                              \-> 竖直结构点 -> SLF 匹配定位(结构图/打分) --修正量--> 位姿
BMI088 --I2C--> 既有控制采集/融合 --G3导航姿态/运动反馈                                  |
RK3588 / PC --板载CH导航串口；P11-40独立控制遥测--> 邮箱 -> 软核固件：定点导航(A*，记忆栅格) /
                                                    目标跟踪 / 探索 -> 局部规划 --(候选)--> 轨迹评估加速器
                                                                                  |
                              安全限幅/斜坡/看门狗/急停 -> vx vy wz -> 轮腿 RL 运动控制（同一 FPGA）-> odo -> 位姿
```

工程采用 RTL 与 PicoRV32 软核协同实现导航，在同一片 FPGA 中集成原 4500 轮腿运动控制。导航源码位于 `rtl/` 与 `fw/`，控制源码位于 `control/`，联合构建与板级适配位于 `integration/`。系统分工见[总体设计](docs/system_architecture.md)。

开发进度见[整车总计划](docs/1_plan/system_integration_plan_20261021.md)，构建和实测结果见[PNR63 报告](docs/4_design/g2_PNR63_review_20261028.md)。

两种模式（选题 3 方案），局部高程图 / 可通行性分析 / 局部规划共用：

| 模式 | 定位 | 全局规划 | 探索（Search） |
|---|---|---|---|
| 无图（上电默认） | 里程计 + IMU，SLF 匹配自己的局部结构图（激光里程计，抑制漂移） | 记忆栅格（在线累积） | 记忆栅格上的边界探索 |
| 有图（下发先验地图 + SET_POSE） | **只定位不建图**：SLF 匹配先验定位图；初始对准、质量判定、丢失自动重定位 | 先验代价图 + 观测到的动态障碍 | 找相机还没看过的区域 |

先验地图通过场地几何 JSON 生成点云，再使用 SLF128 配套工具生成 NVMP v3 地图包。Web 提供地图上传、下发进度与预览入口；制作和导入步骤见[落地调试与建图指南](docs/3_guide/ground_debug_and_mapping_20261027.md)。

## 文档导航

整车总体设计见[云拓系统架构](docs/system_architecture.md)，覆盖RK3588边缘模型、FPGA导航与原4500控制、Web和微信小程序，以及任务、测量和视频的数据流。

文档按用途分在 `docs/` 的五个子目录里，完整目录见 [docs/README.md](docs/README.md)。

| 目录 | 给谁 | 文档 |
|---|---|---|
| `docs/1_plan/` 进度与计划 | 接手先读新总计划与核对报告 | [当前总计划](docs/1_plan/system_integration_plan_20261021.md)、[roadmap](docs/1_plan/roadmap.md)（历史记录）、[integration_plan](docs/1_plan/integration_plan.md)（整车联调）、[mapping_plan](docs/1_plan/mapping_plan.md)（扫图） |
| `docs/2_interface/` 接口（唯一真值） | 接口实现者 | [icd](docs/2_interface/icd.md)（接口定义）、[regmap](docs/2_interface/regmap.md)（寄存器表，自动生成）、[pinout](docs/2_interface/pinout.md)（引脚与接线） |
| `docs/3_guide/` 操作手册 | 上板、改代码的人 | [hardware_guide](docs/3_guide/hardware_guide.md)（实机搭建与测试）、[board_bringup](docs/3_guide/board_bringup.md)（上板要点）、[dev_guide](docs/3_guide/dev_guide.md)（开发规范） |
| `docs/4_design/` 设计与参数 | 算法开发者 | [algorithms](docs/4_design/algorithms.md)（仿真与算法记录）、[platform](docs/4_design/platform.md)（机器人 / 传感器 / 场地参数） |
| `docs/5_teams/` 与其他组对接 | 底盘 / RL 组、RK3588 组 | [chassis_integration](docs/5_teams/chassis_integration.md)、[chassis_review](docs/5_teams/chassis_review.md)、[rk3588_design](docs/5_teams/rk3588_design.md)、`reference/`（其他组原始资料） |

## 目录

| 目录 | 内容 |
|---|---|
| `config/` | **唯一参数来源**：`nav_params.yaml`（结构参数）、`regmap.yaml`（寄存器表）、`robot_calib.json`（整车 URDF 标定结果，俯仰为旧 CAD 值）。改完运行 `python tools/gen_config.py` |
| `control/` | 原4500完整控制副本，来源与维护约定见[control/FORK.md](control/FORK.md)；后续控制侧修改在本仓库提交 |
| `rtl/common` | FIFO、异步 FIFO、RAM、跨时钟域、时基 |
| `rtl/lidar` | `xt_frame_rx` 帧接收（厂家 SDK 格式：逐分包校验、跳过 IMU 消息、锁存帧尾 FrameInfo）、`pc_project` 深度（径向）转点云、`pc_transform` 坐标变换、`pc_filter` 栅格化 |
| `rtl/map` | `elev_map` 滚动高程图、`trav_v3` 可通行性分析（`trav_analysis` 为旧版 v1，保留作对照）、`map_subsys` 双缓冲代价图 |
| `rtl/match` | SLF 匹配定位：`slf_extract` 结构点提取、`slf_lf` 似然场建图、`slf_match` 打分/并图/控制（有图模式：先验槽字写口、固定窗口、不并图、不门控对准） |
| `rtl/plan` | `traj_eval` 候选轨迹评估、`safety_guard` 限幅/斜坡/看门狗/急停 |
| `rtl/net` | `rgmii_mac100` 板上 RJ45 的百兆 RGMII MAC（含 MDIO 初始化）、`udp_rx`/`udp_tx`/`net_port` UDP+ARP 协议栈、`raw_cap` 雷达口原始帧接收、`host_mbx` 主机邮箱（含串口注入与发送回读） |
| `rtl/io` | `imu_uart`、`uart_bus_bridge` 调试串口桥、`nav_bus_arb` |
| `rtl/regs` | 生成的寄存器文件与常量 |
| `rtl/top` | `nav_subsystem`：交付给整车顶层的唯一模块 |
| `rtl/soc` | 软核 SoC：`nav_soc`（PicoRV32 + 片上 RAM + 导航总线主口 + 控制台 + 调试口），`third_party/picorv32.v`（开源，打了寄存器堆补丁） |
| `gowin/` | 板级顶层 `nav_board_top.v`、约束 `nav_board.cst`、综合脚本 `syn/syn.py` |
| `fw/` | 软核固件：`bsp/` 启动与交叉编译、`nav/` 算法（记忆栅格 + A*、局部规划、探索、目标跟踪、位姿、SLF 求解、有图定位管理、地图包接收、协议、雷达启动 `xt_link`、方向表 `lidar_cal`）、`drv/` 上板 HAL、`app/` 主循环、`test/` 主机仿真测试 |
| `model/` | Python：XT-M60 协议与参考收包模型、虚拟场景与雷达、定点逐位参考模型（RTL 标准答案）、方向表、先验地图包 |
| `sim/` | xsim 回归（`run.py`、`tests.py` 文件清单、激励生成 `gen/`）；`closed_loop/` 闭环仿真（场景 + 虚拟雷达 + FPGA 感知模型 + 固件动态库 + 机器人误差模型） |
| `tools/` | `nav_host.py` 主机工具、`map_tool.py` 点云 → 地图包、`nav_uart.py` 串口寄存器调试、`board_init.py` 上电初始化、`robot_calib.py` 整车标定、`fake_lidar.py` 伪雷达、`xt_config.py` 雷达 TCP 配置、`xt_reboot_test.py` 经板子远程重启雷达做冷启动回归 |

## 常用命令（Python 用 `D:\Anaconda\python.exe`）

```bash
python tools/gen_config.py          # 由 yaml 生成 .vh/.h/.py/.md、寄存器文件、正弦表
python sim/run.py                   # 全部 RTL 回归（Vivado 2026.1 xsim）；python sim/run.py -l 列出
python fw/test/build.py             # 固件主机测试（gcc），含与定点模型的交叉验证与闭环仿真
python gowin/syn/syn.py             # 综合 nav_subsystem，资源与 Fmax
python gowin/syn/syn.py --board --app=nav --ram-kb=128   # 板级：固件 + 综合 + 布局布线 + 比特流
python sim/closed_loop/batch.py --tag x -j 10   # 批量闭环评测（5 m 场地：无图 5 + 有图 5 用例 × 5 种子，约 30 分钟）
python sim/closed_loop/run_cl.py --scene field5 --start 0.0,-2.0,0 --goal 4.0,2.1 --gif --out sim/build/cl/demo          # 闭环仿真（动画）
python sim/closed_loop/run_cl.py --scene field5 --start 0.0,-2.0,0 --goal 4.0,2.1 --prior auto --init-err rand --gif --out sim/build/cl/pdemo  # 有图
python tools/fake_lidar.py 192.168.0.100 --fps 10     # PC 冒充雷达（SDK 格式）
python tools/map_tool.py build map.pcd -o field.nvmp --step 20 --slope 12   # 地图点云 -> 先验地图包（附检查图）
python tools/nav_host.py --link uart:COM5 map field.nvmp    # PC 下发地图；pose / goal / wp / mode / listen 见 --help
```

## 验证与报告

| 层级 | 内容 | 入口 |
|---|---|---|
| RTL 与定点模型 | 雷达、点云、地图、匹配、轨迹评估、网络和软核 | `sim/run.py`、`model/` |
| 固件与导航闭环 | 协议、位姿、路径、探索、目标跟踪与虚拟场景 | `fw/test/build.py`、`sim/closed_loop/` |
| 地形动力学 | 原 4500 在坡道、台阶和沟槽中的运动、接触与地图判断 | [地形可行性分析](docs/4_design/local_map_terrain_feasibility_report_20261021.md) |
| 联合实现与实板 | 片上资源、时序、通信、传感器与轮腿执行 | [联合资源记录](docs/4_design/g2_joint_resources_20261022.md)、[PNR63 报告](docs/4_design/g2_PNR63_review_20261028.md) |
