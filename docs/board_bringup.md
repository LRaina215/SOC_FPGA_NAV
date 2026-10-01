# 上板与实机接入步骤

实物到货后的完整操作步骤（含通过标准与故障排查）见 [hardware_guide.md](hardware_guide.md)；本文件是开发侧的要点记录。
IMU 串口引脚已从 C20/D20（拨码开关 SW7/SW6）改到扩展口 P11 的 C18/C19；雷达口 PHY 管理口 MDC R14、MDIO U21。全部引脚与电平见 [pinout.md](pinout.md)。

## 0. 当前可直接上板的内容

`python gowin/syn/syn.py --board --app=nav --ram-kb=128` 综合、布局布线并生成比特流
`gowin/syn/build_board/impl/pnr/nav_board_top.fs`（50 MHz，0 违例）：软核 + 导航固件 + 雷达口百兆 MAC（自写，不需要 IP）+
完整点云链路 + 雷达口原始帧通道（软核 TCP 客户端启动雷达）。下载后：LED0 心跳；PC 经板载 USB 串口（921600）运行 `python tools/nav_uart.py COMx id`，应读到 `ID=0x4E415630`。
最近一次构建：2026-10-01（固件含 10° 安装参数），布线 BSRAM 324 / 340、逻辑 18%、DSP 67 / 298，Fmax 50.085 MHz；已另存为 `D:\tools\nav_fw_128k_d1_10deg_20261001.fs`，尚未上板。

## 1. 生成 IP（Gowin IDE → IP Core Generator），名字要和顶层一致

雷达口不需要 IP：`rtl/net/rgmii_mac100.v`（RGMII 百兆，MDIO 把 PHY 限制在 100BASE-TX，状态在 `NETL.PHY_STAT`）。
原来计划的 TSE MAC 已不用。下面两个 IP 都是可选的：

| IP | 模块名 | 关键选项 | 顶层宏 |
|---|---|---|---|
| PLL | `Gowin_PLL_Nav` | 输入 50 MHz，clkout0 = 100 MHz，带 lock | `NAV_USE_PLL` |
| SerDes → 1G Serial Ethernet | `Gowin_1GSERETH_Host` | Quad0 通道 0（SFP1），参考时钟 MGT_CLK1（125 MHz），1000BASE-X，勾选 MAC 功能 | `NAV_USE_SFP` |

生成后对照 IP 的实际端口核对 `gowin/nav_board_top.v` 中对应的 `ifdef` 块，特别是 1G Serial Ethernet：IP 生成的顶层会同时包含 SerDes 与 PCS/MAC，端口名以生成结果为准。

注意：
- 底板 SFP1 接收发器通道 0，SFP2 接通道 1，PCIe 接通道 2（以管脚信息表为准）。
- 电口 SFP 模块（1000BASE-T）的主机侧多数是带自协商的 SGMII，少数是 1000BASE-X：选型时确认，IP 里选对应模式。RK3588 自己不能插 SFP，用网线接这个电口模块（[rk3588_design.md](rk3588_design.md) 第 3.4 节）。
- `ge_pcs.v` 为加密源码，能否综合取决于授权文件是否覆盖该 IP。

## 2. 雷达到货前：PC 冒充雷达（板级半实物）

1. PC 网卡设为 192.168.0.10，网线接板载 RJ45；`nav_uart.py COMx rd NETL_PHY_STAT` 最低位为 1（链路建立）。
2. `python tools/fake_lidar.py 192.168.0.100 --fps 10 --imu` 按厂家 SDK 的格式发送深度帧（帧尾带 SDK 缺省内参，每帧夹一条 IMU 消息）。
3. `python tools/nav_uart.py COMx stat`：`NETL_STAT_ARP`、`LIDAR_STAT_FRAMES` 应增长，`LIDAR_STAT_BAD=0`，`LIDAR_STAT_SKIP` = 帧数。
4. 导航固件会自己打开 WAIT_POSE；只想看帧进地图、不跑固件时，暂停软核后写 `LIDAR.CTRL` 位 3 = 0，帧按寄存器中的位姿直接进地图。
5. `python tools/nav_uart.py COMx map out.png` 读出代价图检查。
6. 格式自检（不需要板子）：`fake_lidar.py 127.0.0.1` 发给本机，厂家上位机能显示深度图（已验证：上位机日志 `imageversion=3`）。

## 3. 雷达到货后

1. 雷达接电脑（192.168.0.10），厂家上位机能出图即正常；`python tools/xt_config.py info / config / lens` 记下固件版本、配置、内参。
2. 已在 2026-09-30 用实物确认（[icd.md](icd.md) 第 2 节）：雷达上电不自动出流、UDP 只发给最近一次连它 TCP 的主机、命令 19（`udp-dest`）返回 OK 但不生效。
   因此板上软核自己当 TCP 客户端启动雷达（`fw/nav/xt_link.c`，D1，已上板通过），雷达直连板子即可，不需要电脑。
3. Wireshark 抓包或上位机录 `.xbin`，用 `fake_lidar.py --pcap` / `--xbin` 回放给板子，确认 `LIDAR_STAT_BAD=0`、`LIDAR_LENS0..8` 等于内参、
   控制台出现 `lidar LUT ready (device lens)`。
4. 左右方向：左前方放箱子，代价图里应在左侧（反了改 `fw/nav/lidar_cal.c` 的 `flip_u` 与 `model/lidar_lut.py` 的列翻转）；
   深度定义（径向）按 hardware_guide 6.5 节用平墙核对。

## 4. 软核（已上板）

软核用开源 PicoRV32（RV32IMC，`rtl/soc/third_party/picorv32.v`，ISC 许可）。高云 IP 库里的 GoRV32 / PicoRV32 是加密网表，
配置只能走图形界面，所以没有用。SoC 在 `rtl/soc/nav_soc.v`：片上 RAM（代码 + 数据）、导航总线主口（接 `nav_bus_arb` m0）、
控制台环形缓冲、调试口（串口桥地址 0xE000–0xFFFF：保持复位、装载程序、读控制台）。

1. 工具链：xPack riscv-none-elf-gcc 15.2，装在 `D:\tools\xpack-riscv-none-elf-gcc-15.2.0-1`（环境变量 `RISCV_GCC` 可改路径）。
2. 编译固件：`python fw/bsp/build_rv.py`（导航固件，-Os，2026-10-01 为 36012 字节代码 + 85324 字节 `.bss`，另留 8 KB 栈，RAM 余量约 1.5 KB）；`--app selftest` 为自检程序（1 KB）。
3. 比特流：`python gowin/syn/syn.py --board --app=nav --ram-kb=128`（RAM 初始内容 = 编译好的固件）；自检小镜像 `--app=selftest --ram-kb=32`。
4. 下载到 SRAM（掉电即失，不动 Flash）：
   `programmer_cli.exe --device GW5AT-138B --run 2 --fsFile gowin\syn\build_board\impl\pnr\nav_board_top.fs`
5. 串口（板载 CH9102，本机 COM11）：`python tools/nav_uart.py COM11 soc`（软核信息）、`con`（控制台）、
   `load fw/build/rv/nav/nav_fw.bin`（不重新综合，直接换固件）、`reset`。
6. 实测（2026-09-29）：自检程序通过（导航寄存器读写、乘除法、计时）；导航固件主循环约 5.7 万次/秒、单次最长 0.17 ms（没有雷达输入时）。
   2026-09-30（RAM 128 KB、-Os、雷达链路 + 百兆 MAC）：上电约 1.6 s 生成方向表（控制台 `lidar LUT ready (M60 default lens), 1606 ms`），
   主循环约 4.3 万次/秒、单次最长约 0.2 ms；`NETL_PHY_STAT` = 0x00270006（MDIO 找到 PHY、地址 0、初始化完成；没接网线所以无链路）。
   同日真 XT-M60 直连雷达口完成板级验证：软核自动发 TCP 启动命令，10 帧/秒、坏帧 0、`NETL_STAT_RX_ERR` 0；已完成一次真实掉电重启后自动重新启动并恢复出流。当前固件将启动后的暖机保护设为 5 s，覆盖冷启动应答 OK 后约 3 s 才出第一帧；该版本已构建并下载，5 s 版本的真实冷启动尚未再次强制复测。
7. 注意：
   - PicoRV32 的寄存器堆已强制为分布式 RAM：高云综合会把它推断成同步读 BSRAM，读写同址时取到旧值，上板后寄存器值错乱。
   - BSRAM 占用高时布线要用 `-place_option 1`（可布性优先），缺省选项在布线阶段长时间不收敛。
   - 布局布线后的 BSRAM 块数比综合报告多约 23 块（综合 297 -> 布线后 320 / 340），预算按布线后的数字算。

## 5. 底盘接入

`ch_*` 接 RL 运动控制模块，`odo_*` 接底盘速度回传。按实测填写 `lp_cfg.caps`（能否横移、速度上限、半径），以及 `SAFE.CTRL.HOLONOMIC`、`SAFE.LIM_*`。
整车顶层（导航 + 底盘同一比特流）的做法、接口扩展与联调顺序见 [chassis_integration.md](chassis_integration.md)、[chassis_review.md](chassis_review.md)、[integration_plan.md](integration_plan.md)。

## 6. 有图模式（先验地图）与 PC 单独操作

不接 RK3588 也能完成全部操作：PC 经板载 USB 串口（或网线接 SFP 口）运行 `tools/nav_host.py`。

1. 扫图：推着车在场地里走一圈（经过所有要去的地方，转角处多停留），录 XT-M60 + IMU088（+ 轮速）。
   Linux 上用激光惯性 SLAM（FAST-LIO 等）建图，导出地图点云（地图系，单位 m，z 轴向上）。这个地图系就是之后目标点的坐标系。
2. 转包：`python tools/map_tool.py build map.pcd -o field.nvmp --step 20 --slope 12`（按当前车体能力；缺省值是旧大车的），打开同名 .png 检查：
   左图应能看到墙、箱体、路沿的轮廓，右图黑色为不可通行。场地（含墙）须 ≤ 12.8 m × 12.8 m。5 m 场地也可以不扫图、用卷尺量测建图（[mapping_plan.md](mapping_plan.md)）。
3. 下发：`python tools/nav_host.py --link uart:COM5 map field.nvmp`，应答"已加载"。
4. 初始位姿：把车放到大致已知的位置（误差 0.5 m / 10° 以内），`nav_host.py --link uart:COM5 pose <x_mm> <y_mm> <yaw_度>`。
   板上对准约 1.6 s；`listen` 里 loc 变为"正常"后才会执行任务。视野里只有平行墙面时车会原地转 35° 再试。
5. 任务：`goal x y`、`wp x1,y1 x2,y2 --loop`、`mode search`（有图搜索），坐标都是地图系 mm。
6. `nav_host.py ... unmap` 回到无图模式。
