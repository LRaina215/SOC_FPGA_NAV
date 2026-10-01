# 导航子系统接口定义（ICD）

本文件定义导航子系统与外部（雷达、RK3588、底盘 RL 控制、IMU、软核）的全部接口。寄存器级细节见 [regmap.md](regmap.md)（由 `config/regmap.yaml` 生成）。
引脚与电平见 [pinout.md](pinout.md)。其他文档只能引用本文件，不能另立定义。

| 版本 | 日期 | 内容 |
|---|---|---|
| v0.9（当前） | 2026-10-01 | 雷达接口已按实物冻结；主机报文已定义（串口注入未上板）；底盘端口已定义但未接实物；IMU 按 WIT 0x55 帧。D2.1 阶段与底盘组冻结为 v1.0（拟增加的字段见 [chassis_review.md](chassis_review.md) 第 4 节） |

版本规则：主版本号变 = 语义不兼容（坐标、单位、位宽、周期、急停语义）；次版本号变 = 只增加字段或报文。

## 1. 全局约定

| 项目 | 约定 |
|---|---|
| 坐标系 | 无图模式：里程计系，上电位置为原点。有图模式：地图系（先验地图的坐标系，SET_POSE 之后生效）。X 前 Y 左 Z 上；机身系同向，原点 = 两轮轴线中点在地面的投影（雷达光心在其前 123 mm、上 L0 + 25 mm）；雷达本体系同向（标定坐标系到本体系的换算已在方向查找表中完成） |
| 长度 / 速度 | mm（int32）/ mm·s⁻¹（int16）；角速度 mrad·s⁻¹（int16） |
| 角度 | 报文中为 0.01°（int32，cdeg）；寄存器与固件内部为 uint16，65536 = 一周 |
| 时间 | FPGA 全局微秒计数（`SYS.TIME_US`，32 位回绕，按模比较）；所有到达时间戳都由 FPGA 打，不依赖对端时钟 |
| 栅格 | 分辨率默认 50 mm（`MAP.INV_RES = 65536/50 = 1311`）；世界栅格 = floor(mm × INV_RES / 65536)；局部窗口 128×128（6.4 m） |
| 代价 | 0–200 可通行（越大越难走），253 膨胀区，254 不可通行，255 未知 |

## 2. 雷达（XT-M60）

格式依据厂家公开的 SDK 源码 [XT-Toffuture/xtsdk_cpp](https://github.com/XT-Toffuture/xtsdk_cpp)（`communicationNet.cpp`、
`xtdaemon.cpp`、`frame.cpp`、`cartesianTransform.cpp`、`cmdstructs.h`），并用厂家上位机 V2.11.6 的调试符号核对过结构体
大小与字段偏移；手册 202510 版 7.4 节本身也写"详细见 SDK cmdstructs.h"。参考副本（不含二进制）放在本机 `D:\tools\xtsdk_ref`，
仓库里不拷厂家代码（无许可证）。我们的实现：模型 `model/xt_proto.py`，RTL `rtl/lidar/xt_frame_rx.v`，发包 `tools/fake_lidar.py`。
`fake_lidar.py` 发到本机时厂家上位机能正常解析（其日志出现 `reportImage: imageversion=3`）。

- 链路：雷达网口（百兆）→ 板载 RJ45（RTL8211F，RGMII）→ `rtl/net/rgmii_mac100.v`（自写百兆 MAC；上电复位 PHY 并经 MDIO
  把自协商限制到 100BASE-TX，所以接雷达或接电脑千兆网卡都工作在百兆）→ `nav_subsystem.l_*`。
- 网络缺省：雷达 192.168.0.101/24，TCP 控制端口 7787，UDP 数据发往 7687；FPGA 雷达口 `NETL.IP` = 192.168.0.100（必须与雷达
  同网段），`NETL.UDP_PORT` = 7687，另收组播 239.255.255.76（`NETL.MCAST_IP`，新版 SDK 加入的组）。FPGA 应答 ARP；雷达的 IP
  写在 `NETL.LIDAR_IP`（缺省 192.168.0.101），软核用它当 TCP 客户端启动雷达（见下面"雷达启动"）。
- 实物确认（2026-09-30，SN XTM60B20250324000234，固件 XTFW-RT-V2.34.2，`tools/xt_config.py`）：
  - 上电自动运行 = 0：上电后不出流，要连 TCP 发命令 1（开始测量）；**TCP 断开后继续出流**。
  - UDP 发给最近一次连 TCP 的主机 IP、端口 7687；**命令 19（设 UDP 目标）返回 OK 但不生效**（单播、组播都试过，重启后也不变）。
    所以 FPGA 要收到流，必须由 FPGA 自己连 TCP 发开始命令（下面"雷达启动"）。
  - 雷达同一时间只理一个 TCP 连接（厂家上位机连着时，别的连接收不到应答）。
  - 报文源端口 49154；一帧 14 个分包（13 × 1400 + 1226 字节），分包之间常有几毫秒间隔（FPGA 帧内超时改为 200 ms，原 3 ms 让约 95% 的帧作废）。
  - 本机内参 fx 79.94、fy 82.73、cx 79.88、cy 26.92、k1 −0.302、k2 0.0958、k3 −0.0142、p1 0.00035、p2 −0.00056（每帧帧尾也带，
    FPGA 锁存到 `LIDAR.LENS0..8`，固件据此重算方向表）；帧尾温度约 46–62 °C。
  - 真实报文片段（5 帧）存在 `sim/data/xt_m60_capture.txt`，回归测试 `tb_xt_rx_capture` 用它逐位核对 RTL 与模型。

雷达启动（`fw/nav/xt_link.c`，软核上的最小 TCP 客户端；`NETL.CTRL.LIDAR_TCP` = 1 时工作，缺省 1）：
- 流程：上电等 2 s；`LIDAR.STAT_FRAMES` 2 s 不涨就 ARP 找 `NETL.LIDAR_IP` → SYN 到 7787（MSS 1460）→ 与 ACK 合发命令 1 `{1 深度, 1 连续}`
  （`7E FF AA 55 05 00 00 00 01 01 01 00 01 FF 7E 55 AA`）→ 等应答（跳过其他命令号的上报帧）→ FIN 关闭 → 给雷达 5 s 出流（实测冷启动应答 OK 后约 3 s 才出第一帧）；
  之后帧再中断 2 s（雷达掉电重启等）就整套重来。已完成一次真实雷达掉电重启自动恢复验证；当前 5 s 暖机保护版本已构建并下载，3.5 s 延迟场景已由闭环仿真验证，但该版本对应的真实冷启动尚未再次强制复测。已经有别人把雷达启动了（帧在涨）就什么都不发。
- 超时与重试：ARP 0.5 s × 10 次（雷达重启后约 6 s 才应答）、SYN 1 s × 3 次、命令数据段 0.6 s 未确认重发、应答等 1.5 s
  （超时发 RST 后交给出流监视判断）、被 RST 或失败后退避 2 s。不做重排和窗口：乱序段只回重复确认，丢包靠超时重来。
- 收发走雷达口的原始帧通道：发送 `NETL.RAW_TX`（先把整帧写进 `LRAW_TX` 窗口，≤ 128 字节，MAC 补齐到 60 字节并加 FCS）；
  接收 `NETL.CTRL.RAW_EN` = 1 时把给本机的 ARP 应答和发给本机 IP 的 TCP 段（FCS 正确、目的 MAC 是本机或广播）存进 4 格 × 前
  128 字节（`LRAW_RX` 窗口，`NETL.RAW_STAT` 看格数与长度，`NETL.RAW_POP` 弹出；满了整帧丢、`RAW_STAT.RX_DROP` 计数）。
  UDP 数据流不经过这里。
- 串口日志：`lidar start #n (tcp -> 192.168.0.101:7787)`、`lidar start ok (n)`。
- 测试：`sim/closed_loop/test_xt_link.py`（假雷达按实测行为建模：一次一个连接、断开后继续出流；20 个场景：丢 SYN、校验和错、
  丢数据段、RST、应答前有上报、应答拆段、应答码失败、雷达重启、雷达不在、开机 40 / 75 分钟后才接雷达等）；`tb_net`、`tb_nav_top`
  覆盖 RTL 原始帧通道。排查用：`tools/fake_lidar.py --tcp-server` 让电脑冒充雷达的 TCP 端（hardware_guide 6.4 节第一步）。
- 实物（2026-09-30，固件 V2.34.2）：雷达直连板子，上电约 3.3 s 启动成功、10 帧/秒；断流 2 s 后自动重新启动。

UDP 分包（一个 UDP 报文可连放多个分包，全部小端）：

| 偏移 | 字段 | 说明 |
|---|---|---|
| 0 | u16 帧序号 | 同一帧的分包相同 |
| 2 | u32 整帧字节数 | START 到 END 的总长；< 1000 的是 IMU 等非图像消息（FPGA 跳过，`LIDAR.STAT_SKIP`） |
| 6 | u16 本包数据字节数 | ≤ 1400 |
| 8 | u32 本包在帧内的偏移 | FPGA 要求连续：丢包 / 乱序 / 重复整帧丢弃（`LIDAR.STAT_BAD`）；偏移 0 = 新帧 |
| 12 | 8 字节 | 未用 |
| 20 | 帧数据 | |

帧：`7E FF AA 55` | u32 len | 负载（len 字节）| `FF 7E 55 AA`（首尾标记按字节顺序发送）。负载：

| 偏移 | 内容 |
|---|---|
| 0 | cmdid = 251（252 = IMU 消息） |
| 1 | 类型：0 深度+幅值（逐像素交织、深度在前），1 深度，3 灰度（FPGA 忽略） |
| 2 / 4 / 6 | u16 帧号 / 宽 / 高（M60 160 × 60；竖直合并时 160 × 30） |
| 8 | 像素：u16 深度（mm × unit_div，沿视线的**径向距离**）[+ u16 幅值]；**≥ 64000 为无效码**（幅值过低、饱和、过曝、坏点、被滤） |
| … | 可选附加平面（灰度、HDR 档位图）、IMU 块（按 FrameInfo 标志；FPGA 按长度跳过） |
| 末 206 字节 | FrameInfo_t（V3）：偏移 53 起 9 个 float 内参 fx fy cx cy k1 k2 k3 p1 p2；93 imageflags；113/117 ROI 起点；124 合并（bit1 水平 bit0 竖直）；136 unit_div；141 温度；202 u16 结构体大小（= 206）；204 设备状态；205 协议版本 3 |

FPGA 对帧尾的处理：内参、深度单位、ROI、合并方式锁存到 `LIDAR.LENS0..8` / `INFO_*`；`LIDAR.CTRL.AUTO_INFO=1`（缺省）时深度单位、
ROI、合并直接用于本帧的点云换算（否则用 `DEPTH_CFG` / `ROI_OFS`）。固件（`fw/nav/lidar_cal.c`）按收到的内参在板上生成方向查找表
（上电先用 SDK 的 M60 缺省内参，内参变化才重算，约 1–2 s，其间的帧丢弃），不再需要 PC 生成、烧写方向表。

方向与坐标（`model/lidar_lut.py`，与 SDK `pcltransCamparm` 一致）：数据第 q 列对应标定坐标第 159−q 列；去畸变为 Brown-Conrady
（固件与模型都用"4 次不动点迭代 + 3 步牛顿"，四角也收敛）；点 = 距离 × 单位向量（本体系 前 = z、左 = −x、上 = −y，标定坐标为
OpenCV 约定）；厂家对 M60 默认切掉四角约 8% 像素（半径 30 像素的倒角），表中这些像素为零向量 = 无效。按缺省内参，视场约
111° × 43°、光轴上方约 19° / 下方约 23°（手册标称 120° × 45°，以每帧自带的内参为准）。**左右正负、上下方向需实物核对一次**
（在雷达左侧放一个箱子，看点云 Y 是否为正）。

TCP 命令（`tools/xt_config.py`）：请求 `7E FF AA 55 | len | cmd | 数据 | 00 01 | FF 7E 55 AA`（len = 1 + 数据 + 2），应答
`… | cmd | 数据 | 返回码 | 设备状态 | 03 | …`。常用：0 测试，1 开始（{1 深度 / 2 深度+幅值 / 3 灰度, 1 连续 / 0 单次}），2 停止，
3 设 IP（IP、掩码、网关各 4 字节），4 设备信息（DevInfo_t，含当前 UDP 目标），5 配置（DevCfg_t，含 bautorun），9 最小幅值，
10 HDR，13 重启（数据 "XINTAN"），18 内参（36 字节 float），19 UDP 目标（IP 4 字节 + u16 端口），56 最大帧率。

## 3. 主机报文（RK3588 或 PC）

导航不依赖 RK3588：下面的报文由 RK3588 或 PC 发出都一样，PC 可以脱离 RK3588 完成下发地图、初始位姿、目标点、路径点、模式切换和状态查看。
参考实现 `tools/nav_host.py`（PC / RK3588 通用命令行，`tools/rk_host.py` 为兼容入口），C 侧 `fw/nav/proto.c`。

| 链路 | 状态 | 用法 |
|---|---|---|
| 板载 USB 串口（uart_bus_bridge，921600） | 可用（2026-10-01 上板通过：状态、目标点、模式、地图包、TARGET） | PC 把报文逐字写 `MBX.INJ_DATA`，写 `MBX.INJ_LEN` 后 `MBX.INJ_GO`，报文进入与网口相同的接收队列（源 IP 记 0）；FPGA 发出的报文（状态、地图应答）留在发送缓冲，经 `MBX.TX_SEQ` / `MBX.TXB_ADDR` / `MBX.TXB_Q` 读回（只保留最新一条）。`nav_host.py --link uart:COM5`；RK3588 经 USB 也可这样用 |
| 专用主机串口（P11 第 3 / 4 针） | 计划（[rk3588_design.md](rk3588_design.md) 方案 R1） | 链路层（帧定界 + CRC-16）实现时补进本节；报文内容不变 |
| 网口 UDP（SFP 口，1000BASE-X） | RTL 已有，**IP 未生成，目前不可用** | FPGA 地址 `NETH.IP`（默认 192.168.2.100），端口 `NETH.UDP_PORT`（9000）；FPGA 回发到最近一次收到合法报文的源地址/端口（`NETH.CTRL.LEARN_PEER`），也可写 `NETH.PEER_*` 固定。`nav_host.py --link udp:192.168.2.100` |

- 负载小端。公共头 4 字节：`'N' 'V' 版本=1 类型`。

| 类型 | 方向 | 内容（头之后） |
|---|---|---|
| 0x10 GOAL | 主机→FPGA | int32 x_mm, int32 y_mm, int32 yaw_cdeg（0x7FFFFFFF = 不关心） |
| 0x11 WAYPOINTS | 主机→FPGA | uint8 n(≤32), uint8 loop, uint16 保留, n × {int32 x, int32 y} |
| 0x20 TARGET | 主机→FPGA | uint8 found, uint8 cls, uint16 conf(‰), int32 x, y, z（机身系 mm）, uint16 latency_ms, uint16 track_id |
| 0x30 MODE | 主机→FPGA | uint8 mode（0 空闲 1 定点 2 跟踪 3 探索 4 急停）, 3 字节保留 |
| 0x12 SET_POSE | 主机→FPGA | int32 x, y, yaw_cdeg（地图系）, uint16 flags（bit0 = 在先验图上对准）, uint16 range_mm, uint16 range_cdeg（对准搜索半宽，0 = 缺省 ±500 mm / ±10°）, uint16 保留。无图模式下即重置里程计原点 |
| 0x40 MAP_BEGIN | 主机→FPGA | uint32 total_len（先验地图包 .nvmp 的字节数） |
| 0x41 MAP_DATA | 主机→FPGA | uint32 offset, 数据（≤ 1024 字节）。只接受 offset = 下一个期望偏移的块 |
| 0x42 MAP_END | 主机→FPGA | —（校验 CRC；通过即进入有图模式，等待 SET_POSE） |
| 0x43 MAP_CLEAR | 主机→FPGA | —（回到无图模式） |
| 0x80 STATUS | FPGA→主机 | uint8 mode, uint8 state, uint16 map_seq, int32 x, y, yaw_cdeg, goal_x, goal_y, int16 vx, wz, uint16 flags, uint16 保留, uint32 t_us, uint8 loc_state（0 无图 1 对准中 2 正常 3 退化 4 丢失）, uint8 loc_quality（%）, uint16 保留（5 Hz） |
| 0x81 PATH | FPGA→主机 | uint8 n, 3 字节保留, n × {int32 x, int32 y}（预留） |
| 0x82 MAP_ACK | FPGA→主机 | uint32 next_offset, uint8 map_state（0 无 1 接收中 2 已加载 3 校验错 4 格式错）, 3 字节保留。每个 MAP_* 报文回一次，主机按 next_offset 重发 |

- TARGET 的 `latency_ms` = RK3588 采图到发出报文的时间。FPGA 用"到达时刻 − latency"查历史位姿，把目标换算到里程计系，所以两边时钟不需要同步。目标 1.5 s 未更新判为丢失（跟踪 → 探索）。
- STATUS.state：0 空闲 1 行进 2 到达 3 受阻 4 跟踪 5 探索 6 急停；flags：bit0 目标有效，bit2 地图正常，bit3 里程计正常。
- 有图模式流程：MAP_BEGIN → MAP_DATA… → MAP_END（map_state = 2）→ SET_POSE（大致位置与朝向，误差 0.5 m / 10° 以内）→ 板上对准（约 1.6 s；视野里只有平行结构时原地转 35° 再试）→ loc_state = 2 后才执行 GOAL / WAYPOINTS / 探索（坐标均为地图系）。定位丢失时原地保持、自动重定位。
- 先验地图包 `.nvmp`：由 `tools/map_tool.py build <地图点云>` 生成（Linux SLAM 导出的点云，地图系，单位 m），格式见 `model/prior_map.py` 文件头：40 字节头（版本 2 = 18 位槽字，固件只收版本 2）+ 结构图条目 {u16 槽地址, u24 槽字} + 20 cm 代价图。场地不超过 12.8 m × 12.8 m。

## 4. 底盘（RL 运动控制）接口 —— FPGA 内部信号

`nav_subsystem` 输出（与核心时钟同域；每 `SAFE.TICK_US` 一次，`ch_valid` 单拍，缺省 1000 us）：

| 信号 | 含义 |
|---|---|
| `ch_vx, ch_vy`（int16 mm/s） | 机身系期望速度；`SAFE.CTRL.HOLONOMIC=0` 时 vy 恒为 0 |
| `ch_wz`（int16 mrad/s） | 期望角速度，正值 = 从上往下看逆时针（左转） |
| `ch_mode`（8 位） | 固件当前只写两个值：0 = 正常；4 = 主机下发了 `mode estop`（此时速度指令为 0）。其他值预留（如姿态请求），D2.1 定义 |
| `ch_stop` | 急停（`SAFE.CTRL.ESTOP` 或使能的 `ext_stop`）：**下一个输出节拍起速度直接为 0，不经斜坡**。含义为"速度归零、底盘保持平衡"，不是断电 |
| `ch_wdog` | 超过 `SAFE.WD_US`（缺省 200 ms）未收到新指令：目标改为 0，**按加速度限制减到 0**；复位后、第一次指令前也为 1 |
| `ch_valid` / `ch_seq` | 输出节拍单拍 / 16 位节拍序号（回绕） |

- 已限幅（`SAFE.LIM_V/LIM_W`）和限加速度（`SAFE.LIM_A`：每节拍最大变化量 DV mm/s、DW mrad/s；1 ms 节拍时 DV = 1 即 1 m/s²）。
- 固件每 100 ms（规划 10 Hz）提交一次指令（先写 `SAFE.CMD_V`，写 `SAFE.CMD_W` 即提交）；节拍之间输出保持，底盘按 `ch_valid` 取最新值。
- 导航实际会发出的范围：前进 ≤ `lp_cfg.v_max`（500 mm/s，再受 `SAFE.LIM_V` 限制，板上初始化缺省 300）、倒车 150 mm/s、
  角速度 ≤ 1200 mrad/s，包括纯原地转向（400–600 mrad/s）。底盘必须在这个范围内都能跟踪，否则把两处上限改小。

底盘回传：`odo_valid` 单拍 + `odo_vx/odo_vy/odo_wz`（同单位，同一时钟域），FPGA 在 `odo_valid` 时打时间戳（`SAFE.ODO_TS`）并计数（`SAFE.ODO_CNT`）。固件的用法与由此产生的要求：

| 项目 | 要求 | 原因（`fw/nav/pose.c`） |
|---|---|---|
| `odo_vx` | 机身前向**对地**速度：轮速已扣掉机身俯仰角速度（自平衡车轮子相对车身来回转） | 直接积分成位置 |
| `odo_wz` | 机身 z 轴角速度，用 IMU 陀螺（不用轮速差；零偏可以不扣） | 航向由它积分（IMU 航向角不用） |
| `odo_vy` | 两轮车恒为 0 | |
| 频率 | 固定 50–100 Hz；不要 1 kHz | 位姿历史 64 项、每个样本一项：100 Hz 时覆盖 0.64 s，雷达帧和 RK3588 目标都靠它插值 |
| 间隔 | 两个样本间隔 > 200 ms 的那一段不积分 | 回传中断时宁可不动 |
| 静止 | 导航指令为 0 且车不动时，`odo_vx = odo_vy = 0`（精确 0），`odo_wz` 绝对值 < 50 mrad/s | 持续 0.3 s 后把 `odo_wz` 当陀螺零偏估计并扣除 |

固件每轮只读最新一组样本（不排队）。底盘调好后需要提供：能否横移、速度 / 角速度 / 加速度上限、外接圆半径（填入 `lp_cfg.caps` 与 `SAFE.LIM_*`），
以及站立腿长 L0（雷达离地 = L0 + 25 mm）。拟在 v1.0 增加的底盘状态字、腿长回传和时间基准输出见 [chassis_review.md](chassis_review.md) 第 4 节。

底盘调试、RL 代码分层、外部控制器桥接、故障降级和架空 / 落地联调流程见 [chassis_integration.md](chassis_integration.md)。本文件与 `config/regmap.yaml` 是信号、位宽和寄存器语义的唯一真值；说明文档不得替代本文件。

## 5. IMU

- 引脚：IMU TX → P11 第 1 针（FPGA C18，`imu_rxd`）；IMU RX ← P11 第 2 针（C19，`imu_txd`）；3.3 V TTL（[pinout.md](pinout.md) 第 5 节）。
  一颗 IMU 由导航与底盘共用：整车顶层把 `imu_rxd` 同时接给两边的接收器，`imu_txd` 只由导航驱动。
- 串口 8N1，波特率 = 核心时钟 / `IMU.BAUD_DIV`（复位值 868 按 100 MHz / 115200；板上 50 MHz，由 `board_init.py --imu-baud` 设置）。
  硬件只收原始字节，并给每个"突发"（空闲 > `IMU.IDLE_US`，缺省 500 us，后的首字节）打到达时间戳；协议解析在 `fw/nav/imu_parse.c`。
- IMU088 协议文档未拿到，当前按常见 0x55 帧（WIT 系列）实现，拿到手册后只改该文件：
  - 用到的帧：0x52 角速度（按 ±2000°/s 量程换算，只用 x、y 轴辅助俯仰 / 横滚滤波）、0x53 角度（横滚、俯仰；航向角不用）；其他帧跳过；
  - 俯仰约定：pitch > 0 为低头（绕机身 Y 轴正转）；IMU 实际安装方向与机身系（X 前 Y 左 Z 上）不同时，在解析处换轴；
  - 频率：至少 50 Hz（姿态滤波按 50 Hz 设计），建议 100 Hz 以上；
  - 容量限制：接收 FIFO 256 字节，固件每轮主循环最多取 64 字节，雷达流下主循环最长约 38 ms，所以 IMU 字节率的硬上限约 6.7 KB/s（256 B / 38 ms），
    留余量按 5 KB/s 以内配置（100 Hz × 3 帧 × 11 字节 = 3.3 KB/s 可以；200 Hz × 3 帧 = 6.6 KB/s 会偶发溢出，`IMU.STAT_OVF` 增长）。FIFO 扩容或改为硬件解析之前，IMU 按 100 Hz 配置。

## 6. 软核总线

- 32 位数据、16 位字节地址、按字访问；`bus_req` 保持到 `bus_ack`（单拍）。多数寄存器 2–3 拍完成，代价图窗口每字约 10 拍，邮箱读每字约 6 拍。
- 主控两个：软核 PicoRV32（`rtl/soc/nav_soc.v`，m0）与调试串口桥（m1），经 `nav_bus_arb` 仲裁后接 `nav_subsystem`。
- `nav_subsystem` 对地址空间内未定义的地址也应答（读回 0），所以导航之外的模块（底盘块 CHAS 等）要在整车顶层按地址译码分出去；
  预留 `0xD800–0xDFFF` 给底盘（[dev_guide.md](dev_guide.md) 第 4 节）。
- 调试：板级顶层把板载 USB 串口接到 `uart_bus_bridge`（协议：写 `'W' A1 A0 D3..D0` → `'K'`，读 `'R' A1 A0` → `'D' D3..D0`，大端），
  PC 用 `tools/nav_uart.py` 按寄存器名读写；`0xE000–0xFFFF` 转到软核调试口（保持复位、装载程序、读控制台）。
