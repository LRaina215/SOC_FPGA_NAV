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

## 4. 软核（已上板）

软核用开源 PicoRV32（RV32IMC，`rtl/soc/third_party/picorv32.v`，ISC 许可）。高云 IP 库里的 GoRV32 / PicoRV32 是加密网表，
配置只能走图形界面，所以没有用。SoC 在 `rtl/soc/nav_soc.v`：片上 RAM（代码 + 数据）、导航总线主口（接 `nav_bus_arb` m0）、
控制台环形缓冲、调试口（串口桥地址 0xE000–0xFFFF：保持复位、装载程序、读控制台）。

1. 工具链：xPack riscv-none-elf-gcc 15.2，装在 `D:\tools\xpack-riscv-none-elf-gcc-15.2.0-1`（环境变量 `RISCV_GCC` 可改路径）。
2. 编译固件：`python fw/bsp/build_rv.py`（导航固件，约 35 KB 代码 + 106 KB 数据）；`--app selftest` 为自检程序（1 KB）。
3. 比特流：`python gowin/syn/syn.py --board --app=nav --ram-kb=160`（RAM 初始内容 = 编译好的固件）；自检小镜像 `--app=selftest --ram-kb=32`。
4. 下载到 SRAM（掉电即失，不动 Flash）：
   `programmer_cli.exe --device GW5AT-138B --run 2 --fsFile gowin\syn\build_board\impl\pnr\nav_board_top.fs`
5. 串口（板载 CH9102，本机 COM11）：`python tools/nav_uart.py COM11 soc`（软核信息）、`con`（控制台）、
   `load fw/build/rv/nav/nav_fw.bin`（不重新综合，直接换固件）、`reset`。
6. 实测（2026-09-29）：自检程序通过（导航寄存器读写、乘除法、计时）；导航固件主循环约 5.7 万次/秒、单次最长 0.17 ms（没有雷达输入时）。
7. 注意：
   - PicoRV32 的寄存器堆已强制为分布式 RAM：高云综合会把它推断成同步读 BSRAM，读写同址时取到旧值，上板后寄存器值错乱。
   - RAM 160 KB 时 BSRAM 占用 90%，布线要用 `-place_option 1`（可布性优先），缺省选项在布线阶段长时间不收敛。
   - 接上雷达 MAC 后，点云链路（方向表 48 块、帧缓冲 19 块）会恢复，BSRAM 总量超出 340 块，需要压缩（见 roadmap）。

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
