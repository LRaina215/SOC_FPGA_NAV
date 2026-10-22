# FPGA 导航子系统（嵌赛 2026 FPGA 选题 3）

目标板：小梅哥 ACX750（高云 GW5AT-LV138PG484AC1/I0；导航历史工程记录版本B，控制工程记录C，统一构建前核对实物）。架构依据《嵌赛-FPGA-选题3.pdf》第 13–14 页：
感知、定位、规划、避障与轮腿控制都在这一块 FPGA 上完成（RTL + 软核），RK3588 只提供视觉目标与任务，**导航不依赖 RK3588**。

```text
XT-M60 --UDP--> 帧接收 -> 深度转点云 -> 坐标变换 -> 滤波/栅格化 -> 滚动高程图 -> 可通行性分析 v3(代价图)
                                              \-> 竖直结构点 -> SLF 匹配定位(结构图/打分) --修正量--> 位姿
BMI088 --I2C--> 既有控制采集/融合 --导航适配（待实现）                                      |
RK3588 / PC --USB 串口（现在）/ P11 串口（计划）/ SFP（可选）--> 邮箱 -> 软核固件：定点导航(A*，记忆栅格) /
                                                    目标跟踪 / 探索 -> 局部规划 --(候选)--> 轨迹评估加速器
                                                                                  |
                              安全限幅/斜坡/看门狗/急停 -> vx vy wz -> 轮腿 RL 运动控制（同一 FPGA）-> odo -> 位姿
```

上图是整车目标架构。共同工程已开始连接导航命令与原4500真实控制，完整姿态/时间适配和整车验收仍未完成；当前状态以[整车总计划](docs/1_plan/system_integration_plan_20261021.md)为准，历史边界见[控制仓库审阅](docs/5_teams/control_repo_review_20261021.md)。

当前任务顺序和状态只维护在[整车总计划](docs/1_plan/system_integration_plan_20261021.md)。实验记录与历史板级成绩分别保留，不能代替整车验收。

两种模式（选题 3 方案），局部高程图 / 可通行性分析 / 局部规划共用：

| 模式 | 定位 | 全局规划 | 探索（Search） |
|---|---|---|---|
| 无图（上电默认） | 里程计 + IMU，SLF 匹配自己的局部结构图（激光里程计，抑制漂移） | 记忆栅格（在线累积） | 记忆栅格上的边界探索 |
| 有图（下发先验地图 + SET_POSE） | **只定位不建图**：SLF 匹配先验定位图；初始对准、质量判定、丢失自动重定位 | 先验代价图 + 观测到的动态障碍 | 找相机还没看过的区域 |

先验地图：卷尺量测或赛前扫图得到地图点云，`tools/map_tool.py` 转成 `.nvmp`；PC 或 RK3588 用 `tools/nav_host.py` 下发
（PC 经板载 USB 串口即可下发地图、初始位姿、目标点、路径点并读状态）。

## 文档导航

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

## 验证现状

- RTL 回归 15 项全部通过，全部与 Python 定点模型逐位比较：公共库、XT-M60 帧接收（含真雷达抓包）、点云链路 + 地图、SLF 匹配（在线 + 有图）、
  轨迹评估、安全限幅、网口协议栈、百兆 RGMII MAC（上升沿 / 下降沿两种接收采样）、串口桥、软核 SoC、`nav_subsystem` 端到端。
- 固件主机测试：定点数学、协议、位姿、A*、C 版轨迹评估与模型逐位一致、闭环（绕障、巡航、探索找目标并跟踪、急停）；
  有图模式（`test_prior.py`）、方向表生成（`test_lidar_cal.py`，与模型差 ≤ 1 LSB）、雷达启动 TCP 客户端（`test_xt_link.py`，对假雷达 20 个场景）。
- 闭环仿真（5 m 场地、两轮自平衡、整车标定的车体、雷达正装下俯 10°，2026-10-01，[docs/4_design/algorithms.md](docs/4_design/algorithms.md) 第 10 节）：
  10 用例 × 5 种子 50/50 到达、0 碰撞，无图定位 xy RMS 均值 2.7 cm、有图 2.2 cm，地形误报均值 ≤ 1.9%。
- 板级（ACX750，50 MHz）：软核 + 导航固件 + 完整点云链路 + 百兆 MAC + 雷达口原始帧通道上板运行；真雷达直连 10 帧/秒、坏帧 0，
  软核自动启动雷达并在掉电重启后恢复。2026-10-01 用 10° 比特流补测：雷达冷启动复测 4 次、断流重启、PC 经 USB 串口下发报文 / 读状态（含地图包与 TARGET）、
  速度安全层台架测试都已通过；10° 安装的标定要等雷达装车后做（roadmap 第 8 节 N0）。

## 实物到货后

按 [docs/3_guide/hardware_guide.md](docs/3_guide/hardware_guide.md) 逐步搭建与测试（环境、烧录、上电初始化 `tools/board_init.py`、IMU / 雷达 / 底盘接入、实地测试、故障排查）。

## 未完成 / 需实机确认

以 [当前总计划](docs/1_plan/system_integration_plan_20261021.md) 为准：现有控制与导航的接口/时间/停止适配、同片资源和统一顶层、N2g新增退化修复，以及装车参数和真实响应验收。BMI088 I2C、原4500模型及转向接口已有实现，不再从零开发。RK3588任务与可选显示按资源和整车进度安排。
