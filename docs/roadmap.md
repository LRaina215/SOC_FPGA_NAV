# 导航模块进度、交接要点与后续计划（接续工作请先读本文件）

详细实验记录与数据见 [algorithms.md](algorithms.md)，实物参数（机器人 / 传感器 / 场地，已知与假设）见 [platform.md](platform.md)，
上板步骤见 [board_bringup.md](board_bringup.md) 与 [hardware_guide.md](hardware_guide.md)（给硬件同学的逐步手册），接口见 [icd.md](icd.md)。

## 1. 当前状态（2026-09-30）

- **架构**（选题 3 核心文件第 13–14 页，只把架构边界当硬约束，具体算法按闭环仿真效果选）：
  - 无图模式（上电默认）：局部高程图 / 可通行性 + 记忆栅格 A* + 边界探索；SLF 匹配自建局部结构图作激光里程计（增强项）。
  - 有图模式：外部电脑建图（Linux SLAM 点云 -> `tools/map_tool.py` -> `.nvmp`），PC 或 RK3588 下发；板上只定位不建图
    （SET_POSE 后 ±0.5 m / ±10° 对准、跟踪质量判定、丢失自动重定位），先验代价图 + 动态障碍层规划，有图搜索。
  - 主机工具 `tools/nav_host.py`（PC / RK3588 通用，UDP 或 USB 串口注入），导航不依赖 RK3588。
- **雷达接入**（roadmap 第 8 节 B 部分，已完成到"接上就能用"，等实物确认）：帧格式、FrameInfo、TCP 命令都按厂家 SDK 源码实现
  （[icd.md](icd.md) 第 2 节）；厂家上位机能正确显示 `tools/fake_lidar.py` 发的包；深度按径向距离，方向表由软核按帧尾内参在板上生成；
  板上 RJ45 用自写的百兆 RGMII MAC（不需要 IP）。
- **RTL**：全链路与 Python 定点模型逐位一致，xsim 回归 15 项（含真雷达抓包 `tb_xt_rx_capture`）。
- **雷达启动（D1，2026-09-30，真雷达上板通过）**：板上软核自己当 TCP 客户端启动 XT-M60（`fw/nav/xt_link.c` + 雷达口原始帧通道），
  不再需要电脑。雷达直连板子：上电约 3.3 s 打印 `lidar start ok (1)`，随后 10 帧/秒、坏帧 0、网口错误 0，帧尾内参锁存后按本机内参
  重算方向表；把 `NETL_UDP_PORT` 临时改掉模拟断流，约 2 s 后自动重新启动、恢复 10 帧/秒。上板发现并修了三个问题（第 4 节第 18、20、21 条）。
- **板级**：软核（PicoRV32，RAM 128 KB）+ 导航固件 + 完整点云链路 + 百兆 MAC，BSRAM 布线后 324 / 340，50 MHz 0 违例；
  已上板：方向表 1.6 s 生成，主循环约 4.3 万次/秒。
- **网口上板实测**（2026-09-30，电脑网线直连板子 RJ45，`fake_lidar.py` 发 50 帧 + IMU 消息）：
  - 电脑 -> 板子**通过**：链路 100 Mbps（`NETL_PHY_STAT` = 0x01870007），UDP 750 个 / 错误 0，雷达帧 50 / 坏帧 0，IMU 分包 50 个全部跳过，
    帧尾内参读到（`LENS0` = 78.21），每帧有效点 8819（= 9600 − 四角切除 781），固件处理 50 帧；代价图里两个箱子和台阶边沿
    的位置与虚拟场景一致，左右正确。
  - 板子 -> 电脑**不通**：板子应答了 ARP（`NETL_STAT_ARP` 增长），电脑一帧没收到，所以查不到板子的 MAC。上面的测试是在电脑上
    加静态 ARP 绕过的。改了 TXC 相位（第 4 节第 13 条）后已上板确认修好（见下一条，雷达收到了板子的 ARP 应答）。
- **真雷达上板**（2026-09-30，XT-M60 固件 V2.34.2）：细节见第 4 节第 14–16 条与 icd.md 第 2 节。
  - 雷达接电脑：`xt_config.py` 读到 SN / 固件 / 本机内参；真实报文经参考模型逐位解析 30 帧、坏帧 0（片段存为 `sim/data/xt_m60_capture.txt`，
    回归 `tb_xt_rx_capture` 通过）。雷达**上电不自动出流**、**UDP 只发给连它 TCP 的主机**（命令 19 不生效），TCP 断开后继续出流。
  - 雷达直连板子（板子雷达口临时顶替电脑网口的 IP / MAC，hardware_guide 6.4 节第四步）：UDP 139 个/秒全部收到（MAC 错误约 0.5%），
    但旧比特流只有 0.5 帧/秒通过、9.5 帧/秒判坏——帧内超时 3 ms 太短；好帧里的帧尾内参正确锁存（fx 79.94 等），固件处理正常。
    超时改为 200 ms（`xt_frame_rx.v`）后重新布线，接收错误反而变成 100%（第 4 节第 17 条）；再把 RGMII 接收改为下降沿采样（构建 6）：
    **10 帧/秒全部通过、坏帧 0、网口错误 0**（20 s 统计），每帧有效点 8819，帧尾内参每帧锁存，固件每帧处理（主循环最长约 38 ms）。
  - 发送方向确认：板子只保留雷达的目标 IP、换回自己的 MAC 后流先断，约 150 s 后雷达重新发 ARP，收到板子的应答后改发给板子、恢复 10 帧/秒。
  - 代价图目前是一整片不可通行：雷达放在桌上，没按车上的安装位姿（离地 200 mm、下俯 10°）摆放和标定，地面不在模型以为的位置，属预期。
    静止时位姿有 2–3 cm 的漂移（没有 IMU / 里程计，安装位姿也不对），标定后再看。
- **整车标定**（2026-09-30，`tools/robot_calib.py` 读机械组的 URDF / STL）：雷达相对机身下俯 20.3°、在轮轴前 123 mm、离地 = 腿长 L0 + 25 mm；
  整车宽 332 mm（腿在轮子外侧）、外接圆半径 209 mm、轮半径 33.65 mm、轮距 270 mm。已写进固件、寄存器缺省值与仿真（platform.md 第 1、2 节）。
- **仿真**（按实车：两轮自平衡、CAD 标定的外形与雷达安装、5 m 场地 field5，虚拟雷达按 SDK 内参）：10 个用例 × 5 种子 50/50 到达，
  定位 xy RMS 2.0–7.2 cm（algorithms.md 第 10 节；雷达下俯 20.3° 比 10° 差，无图均值 4.1 对 2.7 cm）。旧的 12 m 大场地用例保留作回归。

## 2. 环境与工具（本机）

| 项目 | 位置 / 说明 |
|---|---|
| Python | `D:/Anaconda/python.exe`（PATH 里的 msys python 没有 numpy）；pyserial 3.5 已装进 Anaconda |
| RISC-V GCC | `D:\tools\xpack-riscv-none-elf-gcc-15.2.0-1`（xPack 15.2.0-1，校验过 sha256）；环境变量 `RISCV_GCC` 可改 |
| xsim | Vivado 2026.1，`E:\2026.1\Vivado\bin`（`sim/run.py` 里的 `VIVADO_BIN`） |
| 高云 | `D:\Gowin2\Gowin_V1.9.12.04_x64`：`IDE\bin\gw_sh.exe`（综合 / 布局布线）、`Programmer\bin\programmer_cli.exe`（下载） |
| 板卡连接 | JTAG = FT232H（"Gowin USB Cable(FT2CH)"，同时出现 COM16，不用）；调试串口 = CH9102 **COM11**（921600） |
| 电脑有线网卡 | 自带 Killer E2600（"以太网"，ifIndex 会变，现 28）192.168.0.1/24；USB 网卡 ASIX AX88179（"以太网 6"，ifIndex 14，MAC 9C-69-D3-4B-D9-FB）192.168.0.2/24，接雷达用。改 IP 要管理员权限 |
| 器件 | `programmer_cli --scan` 识别 ID 0x0011081B，下载时 `--device GW5AT-138B` |
| 写权限 | 本会话里 E: 盘根目录不可写，新装的东西放 D:\tools；**不要装到 C 盘**（用户要求） |
| PDF | 内置阅读器误报 `嵌赛-FPGA-选题3.pdf` 加密，用 PyMuPDF 可读（`PYTHONPATH=D:/pytools`） |
| 雷达厂家 SDK | 源码在 GitHub `XT-Toffuture/xtsdk_cpp`，本机参考副本 `D:\tools\xtsdk_ref`（含手册文字版）；上位机目录 `XT-M60技术资料\…\XT-Toffuture` 里只有可执行文件和调试符号。**没有许可证，只作参考，不要把代码拷进仓库** |
| 整车模型 | 机械组 URDF / STL：`D:\Desktop\SOC FPGA\Structure\car\car`（SolidWorks 导出；每侧腿的五连杆闭环 URDF 表达不了） |
| 雷达 | XT-M60，192.168.0.101，SN XTM60B20250324000234，固件 XTFW-RT-V2.34.2；真实抓包 `D:\tools\xt_capture\m60_dist_10fps.txt`（3 s，30 帧，每行一个 UDP 负载） |
| GitHub | https://github.com/LRaina215/SOC_FPGA_NAV.git，推 main；网络时好时坏，失败就间隔 20–30 s 重试 |

## 3. 上板流程（已验证）

```bash
PY=/d/Anaconda/python.exe
$PY fw/bsp/build_rv.py                                   # 导航固件 -> fw/build/rv/nav/（-Os，约 32 KB 代码 + 83 KB 数据）
$PY gowin/syn/syn.py --board --app=nav --ram-kb=128      # 比特流（RAM 初始内容 = 固件），约 25–35 分钟
#   自检小镜像：--app=selftest --ram-kb=32（约 15 分钟）
/d/Gowin2/Gowin_V1.9.12.04_x64/Programmer/bin/programmer_cli.exe --device GW5AT-138B --run 2 \
    --fsFile "D:\Desktop\SOC FPGA\nav\gowin\syn\build_board\impl\pnr\nav_board_top.fs"   # 下载到 SRAM（掉电即失，不动 Flash）
$PY tools/nav_uart.py COM11 soc                          # 软核信息：RAM、状态字、控制台字节数
$PY tools/nav_uart.py COM11 con                          # 软核控制台
$PY tools/nav_uart.py COM11 load fw/build/rv/nav/nav_fw.bin   # 换固件（不重新综合）
```

备份比特流：`D:\tools\nav_fw_128k_mac.fs`（当前版本：128 KB + 雷达链路 + 百兆 MAC）、`D:\tools\nav_fw_160k.fs`（旧版，无雷达 MAC）、
`D:\tools\nav_selftest_32k.fs`（自检）。

## 4. 踩过的坑（改动前先看）

1. **PicoRV32 寄存器堆**：高云综合把异步读的 `cpuregs` 推断成同步读 BSRAM，读写同址取到旧值，上板寄存器值错乱（自检打印乱码）。
   已在 `third_party/picorv32.v` 第 203 行加 `syn_ramstyle = "distributed_ram"`（`PICORV32_COMMIT` 里记了补丁）。
2. **读写同址的 RAM**：写成简单双口且读写地址相同时会被推成"先读后写"单口 RAM，布局布线报 PA2122。软核 RAM 用 `nav_ram_sp`（普通写模式）。
3. **板级布线**：BSRAM 90% 以上时缺省选项在布线阶段一小时以上不收敛；`-route_option 2`（快速布线）能布完但运行中控制台随机位错；
   现在用 `-place_option 1`（可布性优先）+ `-route_option 0`（`gowin/syn/syn.py`）。时序余量很小（最近几次 50.02–50.63 MHz / 50 MHz，当前 50.48）。
4. **板级 BSRAM 以布线后为准**：布局布线报告比综合报告多约 23 块（317 -> 340、297 -> 320），预算时按布线后的数字留余量。
5. **板级关键路径**：两次都卡在"寄存器 -> 乘法 / 比较 -> 计数或 RAM 片选"的长组合路径上：`LIDAR_CTRL.HDR_LE` -> 帧长乘法 ->
   `stat_bad`（已把长度检查挪到偏移 16、先寄存像素字节数），`frm_iok` -> ROI 选择 -> 方向表地址 -> 分块 RAM 片选（已把 `pp_*` 打一拍）。
   改 RTL 后先看 `impl/pnr/*_tr_content.html` 的最差路径。
6. **Verilog 越界位选**：`vv[AW-1:0]`（`vv` 只有 10 位、AW = 14）在 xsim 里得 X，点全部无效；零扩展要显式写 `{{(AW-10){1'b0}}, vv}`。
7. **量化改动要看方向性偏差，不只看 RMS**：结构图质心 1/8 格时按 `{c, 1}`（1/16 格）取代表点，相对"向下取整到 1/16 格"的点整体偏
   +1/32 格；累计平均"加 n/2 再向下取整"每逢平局再往正方向偏。无图模式里结构图随机器人前进不断新建，偏差累积成往 +x/+y 的漂移
   （无图 xy RMS 大 1 cm，逐帧误差的均值两个轴都是正的）。改为 1/32 格算代价（代表点 4c+1）+ 正负对称取整后消失（algorithms.md 第 6 节）。
8. **厂家 SDK 的几何约定**：深度是径向距离（z = d / √(1 + x′² + y′²)）；方向表的列号要左右翻转（数据第 q 列对应标定第 159 − q 列）；
   FrameInfo 在帧尾最后 206 字节，偏移要从帧尾倒着数。雷达到货后第一件事是核对左右（hardware_guide 6.5 节）。
9. **自平衡车**：机身俯仰是平衡姿态不是坡度：不能按 IMU 俯仰分解速度算高度（`pose.decomp_pitch = false`）；姿态要用角速度辅助，
   否则地形误报 14%；高度锚定的采样点必须落在轮子着地处。
10. **Python heredoc 写文件**：`"\n"`、Windows 路径里的反斜杠会被 Python 当转义，改代码优先用编辑工具。
11. **仿真动态库与模型文件**：批量评测运行中不要重新编译 `nav_fw.dll`（文件被占用会编译失败），也不要改 `model/*.py`
    （之后启动的用例会用到改了一半的模型）；xsim 回归运行中不要改 RTL。
12. **有图单元测试的场景**：`test_prior.py` 用 field5（terrain 场景的代价图 128×88 超过固件记忆栅格 64×64）。
13. **RGMII 发送时钟相位**：原来 TXC 和 TXD / TX_CTL 在同一个 clk 沿翻转（数据在 TXC 下降沿变）。RGMII 的 PHY 在 TXC 下降沿还要采
    TX_CTL（= TX_EN ^ TX_ER），正好采在跳变上；上板表现为板子发的 ARP 应答电脑一帧都收不到（收方向正常）。
    现在 TXC 由 clk 下降沿寄存器产生，比数据晚 1/4 周期，两个沿都落在数据中间；tb_rgmii_mac 加了"下降沿与上升沿一致"的检查。
    已上板确认：修好前雷达 / 电脑收不到板子的 ARP 应答，修好后雷达收到应答并改发给板子。
14. **帧内超时**：`xt_frame_rx.v` 原来 3 ms 内没有新字节就放弃当前帧。`fake_lidar.py` 一帧的分包一口气发完，测不出来；
    真雷达分包之间常有几毫秒间隔，95% 的帧被判坏（坏帧 9.5 帧/秒、跳过的零散分包约为坏帧的 5 倍、`NETL_STAT_RX_UDP` 正常）。
    已改 200 ms。仿真激励不带时间间隔，这类问题只能靠实物或带时间戳的抓包回放发现。
15. **雷达的 TCP 行为**：同一时间只理一个 TCP 连接（厂家上位机开着时 `xt_config.py` 收不到应答，要先关掉上位机）；UDP 发给最近一次
    连它 TCP 的主机，命令 19 返回 OK 但不生效；上电不自动出流。所以 FPGA 必须自己连 TCP 发开始命令（第 8 节 D1，已做）。
16. **真雷达 UDP 源端口是 49154**（不是 7687）：按源端口过滤的抓包 / 回放一帧都拿不到，统一按目的端口 7687 过滤（`fake_lidar.py` 已改）。
17. **RGMII 接收采样沿**：原来在 RXC 上升沿采 RXD / RX_CTL。百兆时 PHY 每周期只换一次数据、换在上升沿附近（RTL8211F 内部延时 2 ns），
    建立时间只有约 2 ns，板级 I/O 又没有时序约束，结果随布线变：构建 4 错误帧约 0.5%，只改了 `xt_frame_rx` 超时的构建 5 变成 100%
    （`NETL_STAT_RX_ERR` = `NETL_STAT_RX_FRAMES`）。改为下降沿采样（数据窗口中间；下降沿的 RX_CTL = DV ^ ER，按 DV 用，
    RX_ER 让帧提前结束、FCS 错而丢弃），`tb_rgmii_mac`（下降沿）/ `tb_rgmii_mac_pos`（上升沿）两种都测。
    教训：I/O 没有时序约束时，"上次布线能用"不代表下次还能用，接口采样点要放在数据窗口中间。
18. **`NETL.LIDAR_IP` 复位值写错过**：第一版写成 0xC0A80165（= 192.168.1.101），上板控制台打印 `tcp -> 192.168.1.101` 才发现，
    改为 0xC0A80065。现在 `test_xt_link.py` 的固件侧直接用寄存器复位值、假雷达用 `xt_proto.DEFAULT_LIDAR_IP`，`tb_nav_top` 也读回核对。
19. **原始帧发送缓冲只有一份**：`hal_lraw_send` 先等 `RAW_STAT.TX_BUSY` 清零（最多 200 us）再写，否则连发两帧（确认 + FIN）时后一帧会
    覆盖正在发的前一帧。主机测试的 HAL 是队列，测不出这一点。
20. **32 位微秒时间的有符号比较**：`xt_link` 用 `(int32_t)(now - t_frame) > 2 s` 判断断流，一直没帧时 t_frame 不更新，差值约 36 分钟后
    超出 int32 变成负数，就再也不启动（板子开了 50 分钟才接雷达时实测如此）。改为超时后把 t_frame 钳在"刚超时"的位置；
    `test_xt_link.py` 加了 40 / 75 分钟后才接雷达的场景（75 分钟跨过 `TIME_US` 约 71.6 分钟的回绕）。
21. **串口桥连发写丢命令**：`uart_bus_bridge` 在写应答 'K'（10 位）发完之前不收新字节，而 PC 连发时下一条命令的首字节 10 位后就到，
    两者几乎同时，约 0.7% 的写丢失（`nav_uart.py load` 报"连续写应答缺失"，丢了之后后面的字全部错位）。改为写命令发出 'K' 后立即
    回去收（下一条总线访问等 'K' 发完）；`tb_uart_bridge` 加了无间隔连发 3 条写的检查（旧 RTL 过不了）。

## 5. 关键算法参数（当前缺省）

| 项目 | 值 | 位置 |
|---|---|---|
| 地形阈值 | 台阶 30 mm、坡度 16°（20 cm 基线净升高 57 mm）、跳变 25 mm、粗糙度 50 mm | `model/trav_v3.py DEFAULT_V3`，`MAP.THR0/THR1` 复位值 |
| 障碍膨胀 | 圆形 4 格（20 cm，按外接圆 209 mm；RTL 上限 4 格） | `MAP.INFL = 0x14` |
| 记忆栅格 / A* | 10 cm × 64 格（6.4 m）；离致命 1–2 格罚 150（车半宽 166 mm），4 格内按距离罚；同处卡两次放 30 s 虚拟障碍；A* 堆 1536 项（实测最大 469） | `gmap.h`、`astar.h`、`nav_task_init` |
| 姿态 | 有角速度：预测 + 1/16 校正；没有：α-β（1/4, 1/32）；帧时刻按姿态历史插值 | `pose.c` |
| 高度锚定 | 轮子着地处 ±60 × ±85 mm，增益 64/256 | `nav_app_init` |
| 结构点高度带 | 无图 +60..1500 mm、有图 −300..1500 mm（相对足底） | `APP_ZLO_*` |
| 结构图槽字 | 18 位 {标签 每轴 1 位, 命中 2 位, 质心 3+3 位（1/8 格）, occ, 法向 5 位, has, gate}；前 3 次命中按格号对称取整平均后冻结 | `model/slf_match.py`、`rtl/match/` |
| 有图匹配 | 增益 208/256、退化门限 200/千点；对准内点率 >= 40%、第二峰代价差 >= 0.3 | `nav_app_init`、`loc.c` |
| 雷达 | SDK 缺省内参（fx 78.2、fy 79.1、cx 80.7、cy 26.8、k1 −0.30），有效视场约 111° × 43°（光轴上 19°、下 23°）；方向表 12 位 Q0.11 × 3，四角切除半径 30 像素 | `model/lidar_lut.py`、`fw/nav/lidar_cal.c` |
| 雷达安装 | CAD：下俯 20.3°、轮轴前 123 mm、离地 L0 + 25 = 155 mm（L0 暂按 130） | `pose_init`、`MATCH.MOUNT_Z`、`board_init.py`、`sim_world.MOUNT_*` |
| 车体 | 宽 332 mm、外接圆 209 mm、轮距 270 mm、轮半径 33.65 mm | `world.Robot`、`lp_cfg.caps.radius_mm`、`anchor_hw` |

## 6. BSRAM 预算（340 块 × 18 Kbit ≈ 765 KB；DSP 另算，只用了约 16%）

板级构建（软核 RAM 128 KB + 雷达链路 + 百兆 MAC + 雷达口原始帧通道，18 位槽字）的综合报告，布线后 324 / 340（96%）：

| 模块 | 块数 | 说明 |
|---|---|---|
| 软核 RAM 128 KB | 64 | 4 个字节通道各 16 块（原 160 KB 时 96 块） |
| 软核控制台 | 4 | |
| SLF 结构图 | 72 | 256 × 256 × 18 位（原 23 位 92 块）；综合按 16K×1 拼，改成 1K×18 分块可到 64 |
| SLF 点缓冲等 | 26 | 点缓冲 9、似然场 4、写标记 4、结构点提取 3、候选 / 结果 / 余弦表 5、槽列表 1 |
| 高程图 | 52 | 128 × 128 × 52 位（局部地形，两种模式都要） |
| 代价图双缓冲 + 地形 | 23 | |
| 雷达方向表 | 20 | 9600 × 36 位，10 块 1K × 36（原 48 位 48 块） |
| 雷达帧缓冲 | 19 | 双缓冲 |
| 轨迹评估 | 4 | |
| 网口 / 邮箱 / IMU | 15 | 含雷达口原始帧收发（D1，2 块） |
| **合计** | **299**（布线后 324） | 压缩前（160 KB、23 位、48 位方向表）约 358，超出 340 |

还能再压的（目前不需要）：结构图按 1K×18 分块（−8）、高程图按 1K×18 拼（−4）。结构图 256 -> 128 会让无图定位变差（仿真 xy 2 cm -> 15–30 cm），不做。

## 7. 已知问题

1. 被撞检测偶尔漏检；高速贴近箱角偶有擦碰（需要按离障距离限速）。
2. 有图对准在"视野里只有平行结构"时要原地转 1–3 次（每次约 3.4 s），摆放时车头尽量对着有转角的地方。
3. 无图模式探索新区域时的打滑漂移（大场地 0.2–0.5 m / 10 m，5 m 场地 1–3 cm）。
4. 板级时序余量很小（D1 构建布线后 50.04 MHz / 50 MHz），每次改 RTL 都可能变成负 slack。
5. PC 经 USB 串口注入报文、读回状态只在 RTL 仿真（tb_nav_top）中验证，未上板实测。
6. 待实物确认：真雷达的左右方向与深度定义、安装位姿；IMU088 协议与轴向；底盘对地速度接口（整车顶层还没有，`ch_*` / `odo_*` 没接引脚）。
7. 雷达流进板子后固件主循环单次最长约 38 ms（没有雷达时 0.2 ms），速度指令看门狗是 200 ms，目前够用；上车后若控制周期要求更高，
   要把每帧处理拆成几段。
8. RGMII 接收采样沿（第 4 节第 17 条）：已改为 RXC 下降沿（`NETL.CTRL.RX_NEG` 缺省 1，可串口切回上升沿对比），构建 6 上板 0 错误。
   I/O 时序仍没有约束，改 RTL 重新布线后要再看一次 `NETL_STAT_RX_ERR`。
9. 雷达启动 TCP 客户端（第 8 节 D1）已对真雷达上板通过（上电启动、断流后重启）。还没试过的：雷达掉电重启（约 6 s 才应答 ARP，
   按设计会退避后重试）、板子与雷达之间隔一个交换机。不成功时看控制台 `lidar start #n` / `ok`，`NETL_CTRL` bit 6 写 0 可关掉它，
   回到电脑手工启动（hardware_guide 6.4 节第四步旧办法）。
10. 整车标定（CAD）的遗留：雷达支架下俯 20.3°，比建议的 10–12° 大，定位变差（仿真无图 xy RMS 均值 4.1 cm，10° 时 2.7 cm），
    请机械组评估能否改支架；站立腿长 L0 待底盘给出（现按 130 mm，离地 155 mm）；雷达正装 / 倒装模型里判断不了，实物核对。
    整车宽 332 mm：场地通道要 ≥ 55 cm（platform.md 第 3 节）。

## 8. 下一阶段任务

2026-09-29 与用户确认的方案：**只做不影响算法效果的 BSRAM 压缩**（不缩小结构图尺寸），并把 **XT-M60 雷达接入做到"接上就能用"**。
2026-09-30 完成情况：

| 项 | 内容 | 状态 |
|---|---|---|
| B1 | 协议事实：以厂家 SDK 源码（GitHub，第 2 节）为准写进 icd.md 第 2 节 | 完成 |
| A1 | 软核 RAM 160 -> 128 KB：A* 与探索 BFS 共用缓冲（`fw/nav/scratch.c`）、A* 堆按实测、`-Os`（−32 块） | 完成 |
| A2 | 结构图槽字 23 -> 18 位（−20 块）；第一版有量化偏差（第 4 节第 7 条），修正后批量评测不变差 | 完成 |
| A3′ | 深度是径向距离，原 A3（只存两个分量）不成立；改为 12 位 × 3 分量 + 分块 RAM（48 -> 20 块），软核按帧尾内参生成（与模型差 ≤ 1 LSB） | 完成 |
| A4 | 全部回归（xsim 13 项、固件测试、有图单元测试、方向表测试、5 m 场地批量）+ 板级构建 | 完成 |
| B2 | `model/xt_proto.py`（含与 RTL 同检查点的参考收包模型）、`rtl/lidar/xt_frame_rx.v`（逐分包偏移校验、跳过 IMU / 非图像分包、FrameInfo 锁存）、寄存器缺省值、FPGA 地址 192.168.0.100 + 组播 | 完成 |
| B3 | tb_xt_rx 三个场景（SDK 深度 / 深度 + 幅值 / 大端旧格式，含丢包、乱序、重复、截断、坏帧）、tb_nav_top 用新格式 | 完成 |
| B4 | `tools/fake_lidar.py`（SDK 格式、pcap / pcapng / xbin 回放）、`tools/xt_config.py`（TCP 命令）；厂家上位机验证过格式 | 完成 |
| B5 | 板上 RJ45：自写 `rgmii_mac100.v`（TSE IP 生成器只有图形界面，而且 BSRAM 不够），tb_rgmii_mac / tb_rgmii_mac_pos；修了发送时钟相位与接收采样沿，收发都已上板确认 | 完成 |
| B6 | 雷达到货：`xt_config.py` 读配置与内参、自动出流与 UDP 目标核对（不自动、只发 TCP 主机）、真实抓包进回归、直连板子（修了帧内超时） | 完成；安装位姿见 E1 |
| E1 | 整车标定：`tools/robot_calib.py` 由机械组 URDF / STL 算雷达安装位姿、车体尺寸、轮距、轮半径，写进固件 / 寄存器 / 仿真，批量评测 50/50 | 完成；L0、正倒装、左右 / 深度待实车核对 |
| D1 | **FPGA 最小 TCP 客户端**：雷达口加原始帧收发通道（`udp_tx` 原始帧模式、`rtl/net/raw_cap.v` 4 格 × 128 字节接收，`NETL.RAW_*` 寄存器与 `LRAW_TX/RX` 窗口，+2 块 BSRAM），软核 `fw/nav/xt_link.c`：帧 2 s 不涨就 ARP → SYN → 命令 1 → 等应答 → FIN，帧再断就重来（icd.md 第 2 节"雷达启动"）。测试：`tb_net`、`tb_nav_top`（经总线收发）、`test_xt_link.py`（假雷达 19 个场景）、`fake_lidar.py --tcp-server`（PC 冒充雷达 TCP 端） | 完成；真雷达上板通过（上电 3.3 s 出流、断流 2 s 后自动重启） |

### C. 之后（算法）
按离障距离限速、内点率突变辅助被撞检测、多帧对准、有图模式动态障碍过期、终点朝向对准；IMU088（需角速度输出 >= 100 Hz）；
实地扫图（XT-M60 + IMU088 + 轮速 -> Linux SLAM -> `map_tool.py`）；底盘接口联调（整车顶层）。

## 9. 常用命令

```bash
PY=/d/Anaconda/python.exe
$PY sim/run.py                                   # RTL 回归（15 项，约 10 分钟）
$PY fw/test/build.py                             # 固件主机测试（会重编 nav_fw.dll）
$PY sim/closed_loop/test_prior.py                # 有图固件单元测试
$PY sim/closed_loop/test_lidar_cal.py            # 板上方向表生成 vs 模型
$PY sim/closed_loop/test_xt_link.py              # 雷达启动 TCP 客户端 vs 假雷达（19 个场景）
$PY sim/closed_loop/batch.py --tag x -j 10       # 5 m 场地 10 用例 × 5 种子（缺省用例集，约 30 分钟）
$PY sim/closed_loop/batch.py --tag y --cases terrain_a terrain_b search loop       # 旧大场地回归
$PY sim/closed_loop/run_cl.py --scene field5 --start 0.0,-2.0,0 --goal 4.0,2.1 --gif --out sim/build/cl/demo
$PY sim/closed_loop/run_cl.py --scene field5 --start 0.0,-2.0,0 --goal 4.0,2.1 --prior auto --init-err rand --gif --out sim/build/cl/pdemo
#   选项：--bot car（旧四点着地模型）、--osc 幅度,频率,冲击、--imu-hz、--imu-gyro 0、--tcfg infl_disk=0,fuse_r=2500、--pmcfg
$PY tools/map_tool.py build map.pcd -o field.nvmp            # 地图点云 -> 先验地图包 + 检查图
$PY tools/nav_host.py --link uart:COM11 map field.nvmp       # PC 下发；pose / goal / wp / mode / listen
$PY tools/fake_lidar.py 192.168.0.100 --fps 10 --imu         # PC 冒充雷达（SDK 格式）；--pcap / --xbin 回放
$PY tools/fake_lidar.py --tcp-server --stream-for 20         # 连 TCP 启动也冒充（板子 NETL_LIDAR_IP 写电脑 IP）
$PY tools/xt_config.py info | lens | udp-dest 192.168.0.100 7687 | start dist   # 雷达 TCP 配置
$PY gowin/syn/syn.py                                         # 只综合 nav_subsystem（资源、Fmax）
```
