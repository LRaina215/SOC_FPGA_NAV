# FPGA 导航子系统（嵌赛 2026 FPGA 选题 3）

目标板：小梅哥 ACX750（高云 GW5AT-LV138PG484AC1/I0，器件版本 B）。架构依据《嵌赛-FPGA-选题3.pdf》第 13–14 页：

```text
XT-M60 --UDP--> 帧接收 -> 深度转点云 -> 坐标变换 -> 滤波/栅格化 -> 滚动高程图 -> 可通行性分析 v3(代价图)
                                              \-> 竖直结构点 -> SLF 匹配定位(结构图/打分) --修正量--> 位姿
                                                                                  |
RK3588 --UDP(SFP)--> 邮箱 -> 软核固件：定点导航(A*，记忆栅格) / 目标跟踪 / 探索 -> 局部规划 --(候选)--> 轨迹评估加速器
PC --UDP 或 USB 串口注入--^                                                        |
                                                        安全限幅/看门狗 -> vx vy wz -> 轮腿 RL 运动控制
```

两种模式（选题 3 方案），局部高程图 / 可通行性分析 / 局部规划共用：

| 模式 | 定位 | 全局规划 | 探索（Search） |
|---|---|---|---|
| 无图（上电默认） | 里程计 + IMU，SLF 匹配自己的局部结构图（激光里程计，抑制漂移） | 记忆栅格（在线累积） | 记忆栅格上的边界探索 |
| 有图（下发先验地图 + SET_POSE） | **只定位不建图**：SLF 匹配先验定位图；初始对准、质量判定、丢失自动重定位 | 先验代价图 + 观测到的动态障碍 | 找相机还没看过的区域 |

先验地图：赛前扫图，Linux SLAM 出点云，`tools/map_tool.py` 转成 `.nvmp`；PC 或 RK3588 用 `tools/nav_host.py` 下发，
**导航不依赖 RK3588**（PC 经板载 USB 串口即可下发地图、初始位姿、目标点、路径点并读状态）。

## 目录

| 目录 | 内容 |
|---|---|
| `config/` | **唯一参数来源**：`nav_params.yaml`（结构参数）、`regmap.yaml`（寄存器表）。改完运行 `python tools/gen_config.py` |
| `rtl/common` | FIFO、异步 FIFO、RAM、跨时钟域、时基 |
| `rtl/lidar` | `xt_frame_rx` 帧接收、`pc_project` 深度转点云、`pc_transform` 坐标变换、`pc_filter` 栅格化 |
| `rtl/map` | `elev_map` 滚动高程图、`trav_v3` 可通行性分析（`trav_analysis` 为旧版 v1，保留作对照）、`map_subsys` 双缓冲代价图 |
| `rtl/match` | SLF 匹配定位：`slf_extract` 结构点提取、`slf_lf` 似然场建图、`slf_match` 打分/并图/控制（有图模式：先验槽字写口、固定窗口、不并图、不门控对准） |
| `rtl/plan` | `traj_eval` 候选轨迹评估、`safety_guard` 限幅/看门狗/急停 |
| `rtl/net` | `udp_rx`/`udp_tx`/`net_port` UDP+ARP 协议栈、`host_mbx` 主机邮箱（含串口注入与发送回读，PC 不经网口也能收发报文） |
| `rtl/io` | `imu_uart`、`uart_bus_bridge` 调试串口桥、`nav_bus_arb` |
| `rtl/regs` | 生成的寄存器文件与常量 |
| `rtl/top` | `nav_subsystem`：交付给整车顶层的唯一模块 |
| `gowin/` | 板级顶层 `nav_board_top.v`、约束 `nav_board.cst`、综合脚本 `syn/syn.py` |
| `fw/` | 软核固件：`nav/` 算法（记忆栅格 + A*（含先验层）、局部规划、边界探索 / 有图搜索、目标跟踪、位姿、SLF 匹配求解、`loc` 有图定位管理（对准 / 质量 / 重定位）、`prior` 地图包接收、协议）、`drv/hal_fpga.c` 上板 HAL、`app/` 主循环、`test/` 主机仿真测试 |
| `model/` | Python：XT 协议、虚拟场景与雷达、定点逐位参考模型（RTL 标准答案）、方向查找表生成、`prior_map.py` 先验地图包（点云 / 虚拟扫图 -> 定位图 + 代价图） |
| `sim/` | xsim 回归（`run.py`、`tests.py`、激励生成 `gen/`） |
| `tools/` | `nav_host.py` 主机工具（PC / RK3588 通用：地图、初始位姿、目标点、路径点、模式、状态；UDP 或串口）、`map_tool.py` 点云 -> 先验地图包、`nav_uart.py` 串口寄存器调试、`fake_lidar.py` 伪雷达 |
| `sim/closed_loop/` | 闭环仿真：场景 + 虚拟雷达 + FPGA 感知模型 + 固件动态库 + 机器人误差模型，出指标与快照图；无图 / 有图两套批量用例，`test_prior.py` 有图固件单元测试 |
| `docs/` | `icd.md` 接口定义（含主机报文与地图包）、`regmap.md` 寄存器表、`board_bringup.md` 上板步骤（含扫图与 PC 单独操作）、`algorithms.md` 闭环仿真与算法改进记录、`roadmap.md` 进度与后续计划 |

## 常用命令（Python 用 `D:\Anaconda\python.exe`）

```bash
python tools/gen_config.py          # 由 yaml 生成 .vh/.h/.py/.md、寄存器文件、正弦表、默认查找表
python sim/run.py                   # 全部 RTL 回归（Vivado 2026.1 xsim）；python sim/run.py -l 列出
python fw/test/build.py             # 固件主机测试（gcc），含与定点模型的交叉验证与闭环仿真
python gowin/syn/syn.py             # 综合 nav_subsystem，资源与 Fmax
python gowin/syn/syn.py --board     # 板级顶层：综合 + 布局布线 + 比特流
python sim/closed_loop/run_cl.py --scene terrain --goal 9.3,2.8 --seed 2 --gif --out sim/build/cl/demo   # 闭环仿真（动画）
python sim/closed_loop/batch.py --tag x   # 批量闭环评测（无图 4 用例 × 5 种子）
python sim/closed_loop/batch.py --tag y --cases p_terrain_a p_terrain_b p_search p_loop p_perturb p_kidnap   # 有图 6 用例
python sim/closed_loop/run_cl.py --scene terrain --goal 9.3,2.8 --prior auto --init-err 0.3,-0.2,8 --gif --out sim/build/cl/pdemo  # 有图
python tools/map_tool.py build map.pcd -o field.nvmp        # 地图点云 -> 先验地图包（附检查图）
python tools/nav_host.py --link uart:COM5 map field.nvmp    # PC 下发地图；pose / goal / wp / mode / listen 见 --help
```

## 验证现状

- RTL 回归 11 项全部通过，全部与 Python 定点模型逐位比较：公共库、XT 帧接收（3 种像素格式 + 丢包/坏帧/灰度帧）、
  点云链路 + 地图（terrain 场景 8 帧，覆盖基线坡度/遮挡落差/空洞填补/近区保持）、SLF 匹配（5 帧在线：结构点数、245 候选打分、
  整张结构图；有图 2 帧：先验槽字写入、不门控 / 门控打分、UPDATE 不改先验图）、轨迹评估、安全限幅、网口协议栈、串口桥、
  `nav_subsystem` 端到端（以太网帧 → 代价图、高程读口、SLF、轨迹评估、邮箱、串口注入与发送回读、限幅、IMU）。
- 固件主机测试：定点数学、协议、位姿、A*、C 版轨迹评估与模型逐位一致、闭环（绕障、巡航、探索找目标并跟踪、急停）；
  有图模式（`sim/closed_loop/test_prior.py`）：地图包分块下发、乱序重发、CRC 错、SET_POSE。
- 闭环仿真（`docs/algorithms.md` 第 2、8 节，车体按实车 180 × 213 mm）：
  - 无图 4 用例（定点 ×2、搜索、回环两圈）× 5 种子 20/20 到达，定位 xy RMS 0.20–0.34 m，搜索平均 99.8 s。
  - 有图 6 用例（定点 ×2、搜索、回环、扫图后新增障碍、行驶中被撞）× 5 种子 30/30 到达，xy RMS 1.3–4.9 cm，
    初始位姿误差 ±0.4 m / ±8° 下对准 1.6 s，被撞 5/5 检出并在 1.6 s 内重定位，搜索平均 78 s。
  - 全部 0 碰撞，地形误报 ≤ 0.3%。
- 综合（nav_subsystem）：逻辑 15%，BSRAM 82%（估计），核心时钟 Fmax 102.5 MHz；板级顶层 50 MHz 布局布线 0 违例（布线后 71 MHz），BSRAM 68%，已出比特流。

## 未完成 / 需实机确认

见 `docs/board_bringup.md`。主要是：三个 IP（PLL、TSE_MAC、1G Serial Ethernet）的生成与端口核对；XT-M60 帧格式细节与内参；软核（GoRV32 / PicoRV32）总线桥；底盘能力参数（轮径、可跨台阶、坡度）；IMU088 协议；实地扫图与 SLAM 点云导出。进度与计划见 `docs/roadmap.md`。
