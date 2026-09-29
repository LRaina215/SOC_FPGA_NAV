# 上板与实机接入步骤

## 0. 当前可直接上板的内容

`python gowin/syn/syn.py --board` 已完成综合、布局布线并生成比特流：
`gowin/syn/build_board/impl/pnr/nav_board_top.fs`（50 MHz、不含 MAC IP，满足时序，0 违例）。
下载后：LED0 心跳；PC 经板载 USB 串口（921600）运行 `python tools/nav_uart.py COMx id`，应读到 `ID=0x4E415630`。

## 1. 生成 IP（Gowin IDE → IP Core Generator），名字要和顶层一致

| IP | 模块名 | 关键选项 | 顶层宏 |
|---|---|---|---|
| PLL | `Gowin_PLL_Nav` | 输入 50 MHz，clkout0 = 100 MHz，带 lock | `NAV_USE_PLL` |
| Triple Speed Ethernet MAC | `Gowin_TSE_MAC_Lidar` | RGMII 接口；关闭 FCS 转发（MAC 剥离/添加 FCS） | `NAV_USE_TSE_MAC` |
| SerDes → 1G Serial Ethernet | `Gowin_1GSERETH_Host` | Quad0 通道 0（SFP1），参考时钟 MGT_CLK1（125 MHz），1000BASE-X，勾选 MAC 功能 | `NAV_USE_SFP` |

生成后对照 IP 的实际端口核对 `gowin/nav_board_top.v` 中三个 `ifdef` 块，特别是 1G Serial Ethernet：IP 生成的顶层会同时包含 SerDes 与 PCS/MAC，端口名以生成结果为准。

注意：
- 底板 SFP1 接收发器通道 0，SFP2 接通道 1，PCIe 接通道 2（以管脚信息表为准）。
- 电口 SFP 模块选"1000BASE-X 主机侧、仅千兆"的型号。
- `ge_pcs.v` 为加密源码，能否综合取决于授权文件是否覆盖该 IP。

## 2. 雷达到货前：PC 冒充雷达（板级半实物）

1. PC 网卡设为 192.168.1.10，网线接板载 RJ45。
2. `python tools/fake_lidar.py 192.168.1.100 --fps 10` 按真实帧格式发送深度帧。
3. `python tools/nav_uart.py COMx stat`：`NETL_STAT_ARP`、`LIDAR_STAT_FRAMES` 应增长，`LIDAR_STAT_BAD=0`。
4. 固件未上板前可用串口手动放行：`LIDAR.CTRL` 不开 WAIT_POSE（位 3 = 0），帧会按寄存器中的位姿直接进地图。
5. `python tools/nav_uart.py COMx map out.png` 读出代价图检查。

## 3. 雷达到货后

1. 厂家上位机：设置雷达 IP、UDP 目标地址为 192.168.1.100:7687，连续输出、仅深度。
2. Wireshark 抓一帧，逐项核对 [icd.md](icd.md) 第 2 节的待确认项；把抓包导出后用 `fake_lidar.py --replay` 回放，确认 `LIDAR_STAT_BAD=0`。
3. TCP 命令 18 读内参，`python model/lidar_lut.py --out lut.hex --fx .. --fy .. --cx .. --cy .. --k1 ..`，再用 `nav_uart.py COMx lut lut.hex` 写入（或放进 `rtl/lidar/lut_default.hex` 重新综合）。
4. 用厂家 SDK 对同一帧输出点云，与 `model/nav_fixed.py:pc_project` 的结果比较，确定 `--depth-def`（z 或 radial）和深度单位。

## 4. 软核接入

1. 在 IP 生成器中生成软核（GowinGoRV32 / GowinPicoRV32 等），写一个总线桥接到 `nav_bus_arb` 的 m0 口。
2. 固件：`fw/app/main.c` + `fw/app/nav_app.c` + `fw/nav/*.c` + `fw/drv/hal_fpga.c`，定义 `NAV_BASE`；启动代码、链接脚本放 `fw/bsp/`。
3. 固件内存需求：代价图副本 16 KB + A* 约 28 KB + 记忆栅格约 37 KB（含先验层 9 KB）+ 探索约 50 KB（BFS 37 KB、相机已看过 9 KB）+ 其余约 10 KB。

## 5. 底盘接入

`ch_*` 接 RL 运动控制模块，`odo_*` 接底盘速度回传。按实测填写 `lp_cfg.caps`（能否横移、速度上限、半径），以及 `SAFE.CTRL.HOLONOMIC`、`SAFE.LIM_*`。

## 6. 有图模式（先验地图）与 PC 单独操作

不接 RK3588 也能完成全部操作：PC 经板载 USB 串口（或网线接 SFP 口）运行 `tools/nav_host.py`。

1. 扫图：推着车在场地里走一圈（经过所有要去的地方，转角处多停留），录 XT-M60 + IMU088（+ 轮速）。
   Linux 上用激光惯性 SLAM（FAST-LIO 等）建图，导出地图点云（地图系，单位 m，z 轴向上）。这个地图系就是之后目标点的坐标系。
2. 转包：`python tools/map_tool.py build map.pcd -o field.nvmp`，打开同名 .png 检查：左图应能看到墙、箱体、路沿的轮廓，
   右图黑色为不可通行。场地（含墙）须 ≤ 12.8 m × 12.8 m。
3. 下发：`python tools/nav_host.py --link uart:COM5 map field.nvmp`，应答"已加载"。
4. 初始位姿：把车放到大致已知的位置（误差 0.5 m / 10° 以内），`nav_host.py --link uart:COM5 pose <x_mm> <y_mm> <yaw_度>`。
   板上对准约 1.6 s；`listen` 里 loc 变为"正常"后才会执行任务。视野里只有平行墙面时车会原地转 35° 再试。
5. 任务：`goal x y`、`wp x1,y1 x2,y2 --loop`、`mode search`（有图搜索），坐标都是地图系 mm。
6. `nav_host.py ... unmap` 回到无图模式。
