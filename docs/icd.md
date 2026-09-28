# 导航子系统接口定义（ICD）

本文件定义导航子系统与外部（雷达、RK3588、底盘 RL 控制、IMU、软核）的全部接口。寄存器级细节见 [regmap.md](regmap.md)（由 `config/regmap.yaml` 生成）。

## 1. 全局约定

| 项目 | 约定 |
|---|---|
| 坐标系 | 里程计系：上电位置为原点，X 前 Y 左 Z 上；机身系同向；雷达本体系同向（相机模式的 Z前X左Y上 已在方向查找表中换算） |
| 长度 / 速度 | mm（int32）/ mm·s⁻¹（int16）；角速度 mrad·s⁻¹（int16） |
| 角度 | 报文中为 0.01°（int32，cdeg）；寄存器与固件内部为 uint16，65536 = 一周 |
| 时间 | FPGA 全局微秒计数（`SYS.TIME_US`，32 位回绕，按模比较）；所有到达时间戳都由 FPGA 打，不依赖对端时钟 |
| 栅格 | 分辨率默认 50 mm（`MAP.INV_RES = 65536/50 = 1311`）；世界栅格 = floor(mm × INV_RES / 65536)；局部窗口 128×128（6.4 m） |
| 代价 | 0–200 可通行（越大越难走），253 膨胀区，254 不可通行，255 未知 |

## 2. 雷达（XT-M60 Pro）

- 链路：百兆以太网 → 板载 RJ45（RTL8211F，RGMII）→ TSE_MAC → `nav_subsystem.l_*`。
- 雷达需预先配置（厂家上位机或 TCP 命令）：UDP 目标 IP = `NETL.IP`（默认 192.168.1.100），目标端口 = `NETL.UDP_PORT`（默认 7687），连续输出、仅深度（命令 1：type=1, continuous=2）。
- FPGA 应答 ARP；不实现 TCP（控制命令由 PC 或以后的软核协议栈发送）。
- 帧格式按手册 V2.0 第 7 章实现，下列未明确项做成寄存器，接实机后抓包确认：

| 待确认项 | 寄存器 | 默认 |
|---|---|---|
| 分包头长度 | `LIDAR.UDP_SKIP` | 20 |
| 头部字段字节序 | `LIDAR.CTRL.HDR_LE` | 0（大端，手册"网络序"） |
| 像素字节序 | `LIDAR.CTRL.PIX_LE` | 0 |
| 深度+幅值排布 | `LIDAR.CTRL.PIX_LAYOUT` / `AMP_FIRST` | 逐像素交织、深度在前 |
| size 字段与数据长度差 | `LIDAR.DEPTH_CFG.SIZE_ADJ` | 0 |
| 深度单位 | `LIDAR.DEPTH_CFG.DEPTH_MUL`（Q8.8，mm/LSB） | 256（1 mm） |
| 无效深度编码 | `LIDAR.VALID_CFG.RAW_INV`、`AMP_MIN` | 0 视为无效 |
| 深度定义（Z / 径向） | 方向查找表生成参数 `--depth-def` | z |
| 内参 fx fy cx cy k1 k2 k3 p1 p2 | 方向查找表（`model/lidar_lut.py`，经 `LIDAR.LUT_*` 写入） | 理想针孔 120°×45° |

## 3. RK3588 主机报文（UDP，SFP 口）

- FPGA 地址 `NETH.IP`（默认 192.168.2.100），端口 `NETH.UDP_PORT`（9000）；FPGA 回发到最近一次收到合法报文的源地址/端口（`NETH.CTRL.LEARN_PEER`），也可写 `NETH.PEER_*` 固定。
- 负载小端。公共头 4 字节：`'N' 'V' 版本=1 类型`。

| 类型 | 方向 | 内容（头之后） |
|---|---|---|
| 0x10 GOAL | RK→FPGA | int32 x_mm, int32 y_mm, int32 yaw_cdeg（0x7FFFFFFF = 不关心） |
| 0x11 WAYPOINTS | RK→FPGA | uint8 n(≤32), uint8 loop, uint16 保留, n × {int32 x, int32 y} |
| 0x20 TARGET | RK→FPGA | uint8 found, uint8 cls, uint16 conf(‰), int32 x, y, z（机身系 mm）, uint16 latency_ms, uint16 track_id |
| 0x30 MODE | RK→FPGA | uint8 mode（0 空闲 1 定点 2 跟踪 3 探索 4 急停）, 3 字节保留 |
| 0x80 STATUS | FPGA→RK | uint8 mode, uint8 state, uint16 map_seq, int32 x, y, yaw_cdeg, goal_x, goal_y, int16 vx, wz, uint16 flags, uint16 保留, uint32 t_us（5 Hz） |
| 0x81 PATH | FPGA→RK | uint8 n, 3 字节保留, n × {int32 x, int32 y}（预留） |

- TARGET 的 `latency_ms` = RK3588 采图到发出报文的时间。FPGA 用"到达时刻 − latency"查历史位姿，把目标换算到里程计系，所以两边时钟不需要同步。目标 1.5 s 未更新判为丢失（跟踪 → 探索）。
- STATUS.state：0 空闲 1 行进 2 到达 3 受阻 4 跟踪 5 探索 6 急停；flags：bit0 目标有效，bit2 地图正常，bit3 里程计正常。
- 参考实现：`tools/rk_host.py`（Python），`fw/nav/proto.c`（C）。

## 4. 底盘（RL 运动控制）接口 —— FPGA 内部信号

`nav_subsystem` 输出（每 `SAFE.TICK_US` 一次，`ch_valid` 单拍）：

| 信号 | 含义 |
|---|---|
| `ch_vx, ch_vy`（int16 mm/s） | 机身系期望速度；`SAFE.CTRL.HOLONOMIC=0` 时 vy 恒为 0 |
| `ch_wz`（int16 mrad/s） | 期望角速度 |
| `ch_mode`（8 位） | 运动模式（预留：平地 / 过坡 / 越障，由固件填写） |
| `ch_stop` | 急停（寄存器或外部急停源）。含义为"速度指令归零、底盘保持平衡"，不是断电 |
| `ch_wdog` | 超过 `SAFE.WD_US` 未收到新指令，输出已按加速度限制减到 0 |

已限幅（`SAFE.LIM_V/LIM_W`）和限加速度（`SAFE.LIM_A`）。底盘回传：`odo_valid` 单拍 + `odo_vx/odo_vy/odo_wz`（同单位），导航用它积分位姿。底盘调好后需要提供：能否横移、速度/角速度上限、外接圆半径（填入 `lp_cfg.caps`）。

## 5. IMU

- 串口 8N1，波特率 = 核心时钟 / `IMU.BAUD_DIV`。硬件只收原始字节，并给每个"突发"（空闲 > `IMU.IDLE_US` 后的首字节）打到达时间戳；协议解析在 `fw/nav/imu_parse.c`。
- IMU088 协议文档未拿到，当前按常见 0x55 帧（WIT 系列）实现，拿到手册后只改该文件。

## 6. 软核总线

- 32 位数据、16 位字节地址、按字访问；`bus_req` 保持到 `bus_ack`（单拍）。多数寄存器 2–3 拍完成，代价图窗口每字约 10 拍，邮箱读每字约 6 拍。
- 软核确定后只需写一个总线桥（AHB/APB/自定义 → 本协议），固件改 `NAV_BASE`。
- 调试：板级顶层把板载 USB 串口接到 `uart_bus_bridge`，PC 可用 `tools/nav_uart.py` 读写全部寄存器。
