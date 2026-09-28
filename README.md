# FPGA 导航子系统（嵌赛 2026 FPGA 选题 3）

目标板：小梅哥 ACX750（高云 GW5AT-LV138PG484AC1/I0，器件版本 B）。架构依据《嵌赛-FPGA-选题3.pdf》第 13–14 页：

```text
XT-M60 --UDP--> 帧接收 -> 深度转点云 -> 坐标变换 -> 滤波/栅格化 -> 滚动高程图 -> 可通行性分析(代价图)
                                                                                  |
RK3588 --UDP--> 邮箱 -> 软核固件：定点导航(A*) / 目标跟踪 / 自主探索 -> 局部规划 --(候选)--> 轨迹评估加速器
                                                                                  |
                                                        安全限幅/看门狗 -> vx vy wz -> 轮腿 RL 运动控制
```

## 目录

| 目录 | 内容 |
|---|---|
| `config/` | **唯一参数来源**：`nav_params.yaml`（结构参数）、`regmap.yaml`（寄存器表）。改完运行 `python tools/gen_config.py` |
| `rtl/common` | FIFO、异步 FIFO、RAM、跨时钟域、时基 |
| `rtl/lidar` | `xt_frame_rx` 帧接收、`pc_project` 深度转点云、`pc_transform` 坐标变换、`pc_filter` 栅格化 |
| `rtl/map` | `elev_map` 滚动高程图、`trav_analysis` 可通行性分析、`map_subsys` 双缓冲代价图 |
| `rtl/plan` | `traj_eval` 候选轨迹评估、`safety_guard` 限幅/看门狗/急停 |
| `rtl/net` | `udp_rx`/`udp_tx`/`net_port` UDP+ARP 协议栈、`host_mbx` 主机邮箱 |
| `rtl/io` | `imu_uart`、`uart_bus_bridge` 调试串口桥、`nav_bus_arb` |
| `rtl/regs` | 生成的寄存器文件与常量 |
| `rtl/top` | `nav_subsystem`：交付给整车顶层的唯一模块 |
| `gowin/` | 板级顶层 `nav_board_top.v`、约束 `nav_board.cst`、综合脚本 `syn/syn.py` |
| `fw/` | 软核固件：`nav/` 算法（A*、局部规划、探索、目标跟踪、位姿、协议）、`drv/hal_fpga.c` 上板 HAL、`app/` 主循环、`test/` 主机仿真测试 |
| `model/` | Python：XT 协议、虚拟场景与雷达、定点逐位参考模型（RTL 标准答案）、方向查找表生成 |
| `sim/` | xsim 回归（`run.py`、`tests.py`、激励生成 `gen/`） |
| `tools/` | `nav_uart.py` 串口调试、`fake_lidar.py` 伪雷达、`rk_host.py` RK3588 协议参考 |
| `sim/closed_loop/` | 闭环仿真：场景 + 虚拟雷达 + FPGA 感知模型 + 固件动态库 + 机器人误差模型，出指标与快照图 |
| `docs/` | `icd.md` 接口定义、`regmap.md` 寄存器表、`board_bringup.md` 上板步骤、`algorithms.md` 闭环仿真与算法改进记录 |

## 常用命令（Python 用 `D:\Anaconda\python.exe`）

```bash
python tools/gen_config.py          # 由 yaml 生成 .vh/.h/.py/.md、寄存器文件、正弦表、默认查找表
python sim/run.py                   # 全部 RTL 回归（Vivado 2026.1 xsim）；python sim/run.py -l 列出
python fw/test/build.py             # 固件主机测试（gcc），含与定点模型的交叉验证与闭环仿真
python gowin/syn/syn.py             # 综合 nav_subsystem，资源与 Fmax
python gowin/syn/syn.py --board     # 板级顶层：综合 + 布局布线 + 比特流
python sim/closed_loop/run_cl.py --scene terrain --goal 9.3,2.8 --trav v2 --calib 4   # 闭环仿真
```

## 验证现状

- RTL 回归 10 项全部通过：公共库、XT 帧接收（3 种像素格式 + 丢包/坏帧/灰度帧）、点云链路 + 地图（5 帧、窗口跳变，代价图逐格与模型一致）、轨迹评估（逐字一致）、安全限幅、网口协议栈（ARP/UDP/各类错误帧/发送）、串口桥、`nav_subsystem` 端到端（真实以太网帧 → 经总线读出代价图、轨迹评估、邮箱收发、限幅与看门狗、IMU）。
- 固件主机测试：定点数学、协议、位姿、A*、C 版轨迹评估与模型逐位一致、闭环（绕障定点导航、路径点巡航、探索找目标并跟踪接近、急停）。
- 综合（nav_subsystem，GW5AT-138B）：逻辑 13.0K/138K（10%），寄存器 11.6K（9%），BSRAM 157/340（46%），DSP 25，核心时钟估计 Fmax 100.3 MHz（目标 100 MHz）。板级顶层 50 MHz 布局布线 0 违例。

## 未完成 / 需实机确认

见 `docs/board_bringup.md`。主要是：三个 IP（PLL、TSE_MAC、1G Serial Ethernet）的生成与端口核对；XT-M60 帧格式细节与内参；软核选型与总线桥；底盘能力参数；IMU088 协议；先验地图定位（`pose_correct` 已预留）。
