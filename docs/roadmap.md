# 导航模块进度、交接要点与后续计划（接续工作请先读本文件）

详细实验记录与数据见 [algorithms.md](algorithms.md)，实物参数（机器人 / 传感器 / 场地，已知与假设）见 [platform.md](platform.md)，
上板步骤见 [board_bringup.md](board_bringup.md)，接口见 [icd.md](icd.md)。

## 1. 当前状态（2026-09-29）

- **架构**（选题 3 核心文件第 13–14 页，只把架构边界当硬约束，具体算法按闭环仿真效果选）：
  - 无图模式（上电默认）：局部高程图 / 可通行性 + 记忆栅格 A* + 边界探索；SLF 匹配自建局部结构图作激光里程计（增强项）。
  - 有图模式：外部电脑建图（Linux SLAM 点云 -> `tools/map_tool.py` -> `.nvmp`），PC 或 RK3588 下发；板上只定位不建图
    （SET_POSE 后 ±0.5 m / ±10° 对准、跟踪质量判定、丢失自动重定位），先验代价图 + 动态障碍层规划，有图搜索。
  - 主机工具 `tools/nav_host.py`（PC / RK3588 通用，UDP 或 USB 串口注入），导航不依赖 RK3588。
- **RTL**：全链路与 Python 定点模型逐位一致，xsim 回归 12/12（含 tb_soc）；nav_subsystem 综合约 102 MHz。
- **软核已上板**：PicoRV32 SoC（`rtl/soc`）+ 导航固件在 ACX750 上运行（主循环约 5.7 万次/秒，无输入时单次最长 0.17 ms），
  PC 经 USB 串口装载程序、看控制台。
- **仿真**（按实车：两轮自平衡、轮径 65 mm、机身 180 × 213 mm、5 m 场地 field5）：10 个用例 × 5 种子 49/50 到达，
  定位 xy RMS 1.0–4.2 cm（algorithms.md 第 9 节）。旧的 12 m 大场地用例（terrain / loop）保留作回归。

## 2. 环境与工具（本机）

| 项目 | 位置 / 说明 |
|---|---|
| Python | `D:/Anaconda/python.exe`（PATH 里的 msys python 没有 numpy）；pyserial 3.5 已装进 Anaconda |
| RISC-V GCC | `D:\tools\xpack-riscv-none-elf-gcc-15.2.0-1`（xPack 15.2.0-1，校验过 sha256）；环境变量 `RISCV_GCC` 可改 |
| xsim | Vivado 2026.1，`E:\2026.1\Vivado\bin`（`sim/run.py` 里的 `VIVADO_BIN`） |
| 高云 | `D:\Gowin2\Gowin_V1.9.12.04_x64`：`IDE\bin\gw_sh.exe`（综合 / 布局布线）、`Programmer\bin\programmer_cli.exe`（下载） |
| 板卡连接 | JTAG = FT232H（"Gowin USB Cable(FT2CH)"，同时出现 COM16，不用）；调试串口 = CH9102 **COM11**（921600） |
| 器件 | `programmer_cli --scan` 识别 ID 0x0011081B，下载时 `--device GW5AT-138B` |
| 写权限 | 本会话里 E: 盘根目录不可写，新装的东西放 D:\tools；**不要装到 C 盘**（用户要求） |
| PDF | 内置阅读器误报 `嵌赛-FPGA-选题3.pdf` 加密，用 PyMuPDF 可读（`PYTHONPATH=D:/pytools`） |
| GitHub | https://github.com/LRaina215/SOC_FPGA_NAV.git，推 main；网络时好时坏，失败就间隔 20–30 s 重试 |

## 3. 上板流程（已验证）

```bash
PY=/d/Anaconda/python.exe
$PY fw/bsp/build_rv.py                                   # 导航固件 -> fw/build/rv/nav/（约 35 KB 代码 + 106 KB 数据）
$PY gowin/syn/syn.py --board --app=nav --ram-kb=160      # 比特流（RAM 初始内容 = 固件），约 20 分钟
#   自检小镜像：--app=selftest --ram-kb=32（约 15 分钟）
/d/Gowin2/Gowin_V1.9.12.04_x64/Programmer/bin/programmer_cli.exe --device GW5AT-138B --run 2 \
    --fsFile "D:\Desktop\SOC FPGA\nav\gowin\syn\build_board\impl\pnr\nav_board_top.fs"   # 下载到 SRAM（掉电即失）
$PY tools/nav_uart.py COM11 soc                          # 软核信息：RAM、状态字、控制台字节数
$PY tools/nav_uart.py COM11 con                          # 软核控制台
$PY tools/nav_uart.py COM11 load fw/build/rv/nav/nav_fw.bin   # 换固件（不重新综合）
```

备份比特流：`D:\tools\nav_fw_160k.fs`（导航固件，验证无误）、`D:\tools\nav_selftest_32k.fs`（自检）。

## 4. 踩过的坑（改动前先看）

1. **PicoRV32 寄存器堆**：高云综合把异步读的 `cpuregs` 推断成同步读 BSRAM，读写同址取到旧值，上板寄存器值错乱（自检打印乱码）。
   已在 `third_party/picorv32.v` 第 203 行加 `syn_ramstyle = "distributed_ram"`（`PICORV32_COMMIT` 里记了补丁）。
2. **读写同址的 RAM**：写成简单双口且读写地址相同时会被推成"先读后写"单口 RAM，布局布线报 PA2122。软核 RAM 用 `nav_ram_sp`（普通写模式）。
3. **板级布线**：BSRAM 90% 时缺省选项在布线阶段一小时以上不收敛；`-route_option 2`（快速布线）能布完但运行中控制台随机位错；
   现在用 `-place_option 1`（可布性优先）+ `-route_option 0`，约 20 分钟、运行无错（`gowin/syn/syn.py`）。时序余量很小（50.03 MHz / 50 MHz）。
4. **板级 BSRAM 数字会"变少"**：雷达 MAC IP 没接时，点云链路（方向表、帧缓冲、pc_* 模块）被综合掉，实际接上后会多约 67 块（见第 6 节）。
5. **自平衡车**：机身俯仰是平衡姿态不是坡度：不能按 IMU 俯仰分解速度算高度（`pose.decomp_pitch = false`）；姿态要用角速度辅助，
   否则地形误报 14%；高度锚定的采样点必须落在轮子着地处。
6. **Python heredoc 写文件**：`"\n"`、Windows 路径里的反斜杠会被 Python 当转义，改代码优先用编辑工具。
7. **仿真动态库**：批量评测运行中不要重新编译 `nav_fw.dll`（文件被占用会编译失败）。

## 5. 关键算法参数（当前缺省）

| 项目 | 值 | 位置 |
|---|---|---|
| 地形阈值 | 台阶 30 mm、坡度 16°（20 cm 基线净升高 57 mm）、跳变 25 mm、粗糙度 50 mm | `model/trav_v3.py DEFAULT_V3`，`MAP.THR0/THR1` 复位值 |
| 障碍膨胀 | 圆形 3 格（15 cm），RTL 支持半径 <= 4 | `MAP.INFL = 0x13` |
| 记忆栅格 / A* | 10 cm × 64 格（6.4 m）；离致命 1 格罚 150，2–3 格按距离罚；同处卡两次放 30 s 虚拟障碍 | `gmap.h`、`nav_task_init` |
| 姿态 | 有角速度：预测 + 1/16 校正；没有：α-β（1/4, 1/32）；帧时刻按姿态历史插值 | `pose.c` |
| 高度锚定 | 轮子着地处 ±60 × ±85 mm，增益 64/256 | `nav_app_init` |
| 结构点高度带 | 无图 +60..1500 mm、有图 −300..1500 mm（相对足底） | `APP_ZLO_*` |
| 有图匹配 | 增益 208/256、退化门限 200/千点；对准内点率 >= 40%、第二峰代价差 >= 0.3 | `nav_app_init`、`loc.c` |
| 雷达安装（仿真） | 离地 200 mm、下俯 10°、前伸 80 mm；建议实车下俯 10–12° | `run_cl.py mount_pose` |

## 6. BSRAM 预算（340 块 × 18 Kbit ≈ 765 KB；DSP 另算，只用了约 16%）

板级构建（软核 RAM 160 KB，雷达 MAC 未接）的综合报告：

| 模块 | 块数 | 说明 |
|---|---|---|
| 软核 RAM 160 KB | 96 | 4 个字节通道各 24 块（40K 深度不是 2 的幂，按 8K×2 拼出来浪费约 16 块） |
| 软核控制台 | 3 | |
| SLF 结构图 | 92 | 256 × 256 × 23 位（有图模式存先验定位图；无图模式存自建局部结构图） |
| SLF 点缓冲等 | 23 | 点缓冲 9、似然场 4、写标记 4、结构点提取 3 等 |
| 高程图 | 52 | 128 × 128 × 52 位（局部地形，两种模式都要） |
| 代价图双缓冲 + 地形 | 21 | |
| 主机邮箱 | 4 | |
| **小计** | **291**（布线后 306） | |
| 接上雷达后恢复 | +67 | 方向表 48（9600 像素 × 48 位）、帧缓冲 19 |
| **合计** | **约 358 > 340** | |

压缩候选（都不改变算法效果，按收益排序）：
1. 软核 RAM 160 -> 128 KB（2 的幂深度，64 块，-32）：固件数据 106 -> 约 85 KB（代价图副本改为直接读窗口 -16 KB、A* 与 BFS 共用缓冲）。
2. 结构图槽字 23 -> 18 位（标签 4->2、质心 4->3，64 块，-28）。
3. 方向表只存两个分量（-16）：需要雷达到货后确认深度是"沿光轴 Z"还是"径向距离"。
4. 若确定比赛只用有图模式：结构图 256 -> 128（6.4 m 够 5 m 场地，-69）；但无图模式的激光里程计会变差（仿真 xy 误差 2 cm -> 15–30 cm）。
三项（1–3）合计约 -76 块 -> 约 282 块（83%）。

## 7. 已知问题

1. 被撞检测偶尔漏检（pf5_kidnap 1/5）；高速贴近箱角偶有擦碰（pf5_loop 1 个种子）。
2. 有图对准在"视野里只有平行结构"时要原地转 1–3 次（每次约 3.4 s），摆放时车头尽量对着有转角的地方。
3. 无图模式探索新区域时的打滑漂移（大场地 0.2–0.5 m / 10 m，5 m 场地 1–3 cm）。
4. 板级时序余量很小（布线后 50.03 MHz）；BSRAM 压缩后应能改善。
5. PC 经 USB 串口注入报文、读回状态只在 RTL 仿真（tb_nav_top）中验证，未上板实测。
6. 雷达、IMU088、底盘均未接：方向表内参、IMU 协议与轴向、底盘对地速度接口都待实物。

## 8. 下一阶段任务（2026-09-29 与用户确认的方案，按顺序做）

用户决定：**只做不影响算法效果的 BSRAM 压缩**（不缩小结构图尺寸），并把 **XT-M60 雷达数据帧解析做到"接上就能用"**。
雷达资料在 `D:\Desktop\SOC FPGA\XT-M60技术资料`：`XT-M60产品手册-202510.pdf`（23 页，比现有实现依据的手册 V2.0 新）、
`上位机操作使用说明-V1.4.pdf`（13 页）、`芯探雷达-常见问题列表.pdf`、`不同FOV覆盖区域计算-芯探科技.xlsx`、厂家上位机
（Qt 程序 `XT-Toffuture-V2.11.6`，只有可执行文件，**没有 SDK 源码**，协议只能以手册为准，实物到后再抓包核对）。

背景（给用户解释过）：无图模式里有两张"局部图"——高程图（避障、地形判断，赛题要求）和结构图（记住最近看到的墙和箱体边，
每帧拿来匹配、抑制里程计漂移，即"激光里程计"，不对外输出地图、没有回环，不算建图）。结构图从 256 缩到 128 格（12.8 m -> 6.4 m）
会让远处的墙掉出窗口，仿真中无图定位误差从约 2 cm 变成 15–30 cm（2 次运行的结果），所以不缩。

### B1. 读雷达手册，列出协议事实（先做，A3 依赖它）
- UDP 包结构：包头字段、帧号 / 包序号、每包像素数、分辨率（160 × 60？）、深度 / 幅值排布、字节序、无效值编码、深度单位；
- 深度定义（沿光轴 Z 还是径向距离）、内参与畸变参数及读取命令、帧率与时间戳、网络缺省配置（IP / 端口）、TCP 控制命令；
- 对照 `rtl/lidar/xt_frame_rx.v`、`model/xt_proto.py`、`docs/icd.md` 第 2 节的"待确认项"表，逐项标出一致 / 不一致 / 手册未写。

### A. BSRAM 无损压缩（目标：接上雷达后总量约 358 -> 约 280 块）
- **A1 软核 RAM 160 -> 128 KB（-32 块）**：固件数据 106 -> 约 85 KB。
  - A* 的 g / 父方向 / 距离 / 堆 / 关闭表（约 29 KB）与探索 BFS 的距离 / 队列（16 KB）从不同时使用，合成一块共用缓冲；
  - 局部代价图副本（16 KB）：评估改为直接按窗口读 FPGA（`gmap_update` 每帧扫一遍约 1.6 万字节），或压到只保留需要的部分；
  - 128 KB 是 2 的幂深度，拼块不浪费（现在 160 KB 按 8K×2 拼成 96 块）。改完跑固件测试、xsim tb_soc、5 m 场地批量评测确认结果不变。
- **A2 结构图槽字 23 -> 18 位（-28 块）**：标签 4 -> 2 位（5 m 场地不会绕回）、命中数 3 -> 2 位（HIT_MIN 2、冻结 3）、质心 4 -> 3 位
  （1/8 格 = 6.25 mm）；法向 5 位、occ / has / gate 各 1 位不变。改 `model/slf_match.py`、`rtl/match/*.v`、`model/prior_map.py`、
  固件 `prior.c` 的槽字格式；先在仿真里比较 5 m 场地批量评测（无图 + 有图）确认定位精度不降，再改 RTL、重生成 tb_slf 向量。
- **A3 方向表 3 -> 2 个分量（约 -16 块）**：若手册确认深度是沿光轴 Z，方向向量第三分量恒为 1，只存 x/z、y/z；若是径向距离则保留。
- **A4** 全部回归（xsim、固件测试、批量评测）后重新板级构建，检查时序余量（现在 50.03 MHz / 50 MHz）。

### B2–B6. 雷达接入做到"接上就能用"
- **B2** 按 B1 的结论修改 `model/xt_proto.py` 与 `rtl/lidar/xt_frame_rx.v`（逐位一致），寄存器缺省值改为手册值，
  `model/lidar_lut.py` 用手册标称内参生成缺省方向表。
- **B3** 测试向量：按手册格式生成多包帧（含丢包、乱序、重复、坏帧、不同模式），扩充 tb_xt_rx_* 与 tb_nav_top。
- **B4** PC 工具：`tools/xt_config.py` 按手册的 TCP 命令配置雷达（目标 IP / 端口、输出模式、读内参）——FPGA 不实现 TCP；
  `tools/fake_lidar.py` 支持按手册格式发包和回放抓包文件。
- **B5** 板级：生成 TSE MAC IP（RGMII、百兆；IP 生成器是图形界面，先试命令行，不行请用户在 IDE 里点一次），在 `nav_board_top`
  打开 `NAV_USE_TSE_MAC`；雷达没到之前，用 PC 网口接板子 RJ45、`fake_lidar.py` 发包，经串口读帧统计与代价图验证整条链路。
- **B6** 雷达到货：上位机配置 -> Wireshark 抓包核对 -> 读内参写方向表 -> 标定安装角 -> 静止 / 行驶数据回放到仿真。

### C. 之后（算法）
按离障距离限速、内点率突变辅助被撞检测、多帧对准、有图模式动态障碍过期、终点朝向对准；IMU088（需角速度输出 >= 100 Hz）；
实地扫图（XT-M60 + IMU088 + 轮速 -> Linux SLAM -> `map_tool.py`）；底盘接口联调。

## 9. 常用命令

```bash
PY=/d/Anaconda/python.exe
$PY sim/run.py                                   # RTL 回归（12 项，约 7 分钟）
$PY fw/test/build.py                             # 固件主机测试
$PY sim/closed_loop/test_prior.py                # 有图固件单元测试
$PY sim/closed_loop/batch.py --tag x -j 5        # 5 m 场地 10 用例 × 5 种子（缺省用例集，约 40 分钟）
$PY sim/closed_loop/batch.py --tag y --cases terrain_a terrain_b search loop       # 旧大场地回归
$PY sim/closed_loop/run_cl.py --scene field5 --start 0.0,-2.0,0 --goal 4.0,2.1 --gif --out sim/build/cl/demo
$PY sim/closed_loop/run_cl.py --scene field5 --start 0.0,-2.0,0 --goal 4.0,2.1 --prior auto --init-err rand --gif --out sim/build/cl/pdemo
#   选项：--bot car（旧四点着地模型）、--osc 幅度,频率,冲击、--imu-hz、--imu-gyro 0、--tcfg infl_disk=0,fuse_r=2500、--pmcfg
$PY tools/map_tool.py build map.pcd -o field.nvmp            # 地图点云 -> 先验地图包 + 检查图
$PY tools/nav_host.py --link uart:COM11 map field.nvmp       # PC 下发；pose / goal / wp / mode / listen
$PY gowin/syn/syn.py                                         # 只综合 nav_subsystem（资源、Fmax）
```
