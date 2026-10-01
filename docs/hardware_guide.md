# 实机搭建与测试手册

本手册给实物到货后的阶段用：从一台新电脑装环境开始，到烧录、逐个接入外设、实地导航测试。按章节顺序做，
每一步都写了"通过标准"；不通过时先查第 9 节，不要跳到下一步。算法与接口细节分别见 [icd.md](icd.md)、[regmap.md](regmap.md)、
[platform.md](platform.md)，进度见 [roadmap.md](roadmap.md)。

约定：
- 命令在 `nav/` 目录下用 Git Bash 执行。下面两个变量每次打开终端先设一次（路径按本机安装位置改）：
  ```bash
  cd "/d/Desktop/SOC FPGA/nav"
  PY=/d/Anaconda/python.exe
  PRG=/d/Gowin2/Gowin_V1.9.12.04_x64/Programmer/bin/programmer_cli.exe
  ```
- `COM11` 是本机调试串口号，换电脑后以设备管理器里的为准（第 3 节）。
- 一个串口同一时间只能被一个程序打开：`nav_uart.py con`、`nav_host.py listen` 要先 Ctrl-C 退出，才能运行下一条命令。

## 0. 现状：哪些已经实测，哪些还没有

| 部分 | 状态（2026-09-30） | 影响 |
|---|---|---|
| 开发板 + 软核 + 导航固件 | **已上板实测**：寄存器读写、装载固件、控制台正常；软核 RAM 128 KB（原 160 KB） | 第 3–5 节可以直接做 |
| PC 经 USB 串口下发报文 / 读状态（`nav_host.py --link uart`） | 只在 RTL 仿真中验证过 | 第 6.1 节是第一次实测 |
| 雷达网口（RJ45） | **已上板实测收发**：自写百兆 MAC（`rtl/net/rgmii_mac100.v`），不需要生成 IP；接真雷达 10 帧/秒、坏帧 0 | 第 6.3 节只做检查 |
| 雷达启动（软核 TCP 客户端） | **真雷达上板通过**：雷达直连板子，上电约 3.3 s 自动出流；冷启动应答 OK 后约 3 s 才出第一帧，客户端已留 5 s 保护；断流 2 s 后自动重新启动 | 第 6.4 节第四步只需插线 |
| 雷达方向表 | 固件上电后按内参自动生成（实测约 1.6 s，控制台打印 `lidar LUT ready`） | 不需要 PC 生成、烧写 |
| XT-M60 帧格式 | 已按厂家 SDK 源码实现并测试（[icd.md](icd.md) 第 2 节）；厂家上位机能解析 `fake_lidar.py` 发的包 | 第 6.4 节接真雷达确认 |
| 片上存储（BSRAM） | 雷达链路、百兆 MAC、雷达口原始帧通道都接上后布线后 324 / 340（96%），逐模块见 [roadmap.md](roadmap.md) 第 6 节 | 再加大模块要先压缩 |
| IMU088 | 协议手册未到；固件按 WIT 0x55 帧解析 | 协议不同时要改 `fw/nav/imu_parse.c` |
| 底盘 | 板级顶层里速度输出 `ch_*` / 里程计输入 `odo_*` **没有接到任何引脚** | 第 6.6 节需要先完成整车顶层 |
| SFP 口 / RK3588 | IP 未生成 | 不影响基础导航，PC 用 USB 串口代替 |
| 时钟 | 板级 50 MHz（未用 PLL），布线后时序余量很小 | 改 RTL 后注意看时序报告 |

另外，**固件目前不会自己设置下面几个寄存器**，每次上电后要用 `tools/board_init.py` 写一次（第 5 节）：
IMU 波特率分频（复位值按 100 MHz 算，板上是 50 MHz）、雷达安装高度、速度限幅与速度输出使能（`SAFE_CTRL` 复位为关闭，输出恒为 0）。
雷达帧位姿握手（`LIDAR_CTRL.WAIT_POSE`）导航固件上电会自己打开，`board_init.py` 也会再写一次。

## 1. 物料与工具清单

| 类别 | 物品 | 备注 |
|---|---|---|
| 主板 | 小梅哥 ACX750（GW5AT-LV138）+ 原配电源 | |
| 线缆 | USB 线 2 根 | 一根接 JTAG 下载口（FT232H），一根接 USB 串口（CH9102），以底板丝印为准 |
| 雷达 | XT-M60 + M12 防水线 + 12–25 V / ≥ 3 A 直流电源 | 雷达单独供电，不从开发板取电 |
| 网络 | 网线 2 根、电脑有线网口（或 USB 网卡） | 可选：一台傻瓜交换机，方便同时接电脑抓包 |
| IMU | IMU088、杜邦线、USB-TTL 模块（3.3 V） | USB-TTL 用于先在电脑上确认 IMU 协议 |
| 底盘 | 轮腿底盘、架空支架（轮子离地） | 首次通电动作必须架空 |
| 安全 | 电机电源的物理急停开关 / 断电开关 | 实车测试时必须在手边 |
| 测量 | 万用表、卷尺、手机水平仪 App | 测雷达安装高度与俯仰角 |
| 场地 | 纸板 / 木板围墙、快递箱、可调坡道与台阶 | 搭法见 [platform.md](platform.md) 第 3 节 |
| 可选 | 电口 SFP 模块（1000BASE-X 主机侧、仅千兆）、RK3588 | 只在第 6.9 节用 |

## 2. 电脑环境（换电脑时从这里开始）

| 软件 | 获取 | 用途 |
|---|---|---|
| 代码 | `git clone https://github.com/LRaina215/SOC_FPGA_NAV.git`（即本仓库 `nav/`） | |
| 高云 Gowin V1.9.12.04 | 工作区里的 `Gowin_V1.9.12.04_x64_win.zip` | 综合、布局布线（`gw_sh.exe`）与下载（`programmer_cli.exe`） |
| 高云授权 | 工作区里的 `gowin_E_BC6EE2274D68.lic` | 授权与电脑网卡绑定，换电脑要重新向高云申请 |
| Python | Anaconda；`pip install pyserial numpy scipy matplotlib pyyaml pillow` | 所有工具脚本 |
| RISC-V GCC | xPack `riscv-none-elf-gcc` 15.2.0-1（GitHub：xpack-dev-tools/riscv-none-elf-gcc-xpack），解压到 `D:\tools\` | 编译软核固件 |
| 串口驱动 | `小梅哥…资料/驱动和常用软件/驱动/CH343SER.ZIP`（CH9102 用） | 调试串口 |
| JTAG 驱动 | 装 Gowin 时一起装（FTDI） | 下载 |
| Wireshark | 官网新版（资料里的 1.12 版太旧） | 雷达抓包 |
| 雷达上位机 | `XT-M60技术资料/XT-S240-M60-M120系列上位机V2.11.6+` | 配置雷达、读内参 |

安装位置和脚本里的缺省路径不一样时，用环境变量指定，不用改代码：

```bash
export GW_SH="D:/Gowin/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe"          # gowin/syn/syn.py 用
export RISCV_GCC="D:/tools/xpack-riscv-none-elf-gcc-15.2.0-1/bin"        # fw/bsp/build_rv.py 用
```

检查环境：

```bash
$PY -c "import serial, numpy, scipy, matplotlib, yaml; print('python ok')"
$PY fw/bsp/build_rv.py             # 应打印"镜像 约 33000 字节，.bss 约 85000 字节，RAM 128 KB"
$PRG --help | head -3              # 应打印 Programmer 版本号
```

## 3. 第一次接板（只有开发板）

1. 断电状态下接好电源、两根 USB 线。拨码开关保持出厂位置（本工程不用）。
2. 上电。打开设备管理器 → 端口（COM 和 LPT）：
   - `USB-Enhanced-SERIAL CH9102 (COMx)`：这是调试串口，**记下 COMx**，后面所有命令里的 `COM11` 换成它；
   - Gowin 下载器（FT232H）也会多出一个 COM 口，不用管。
3. 识别芯片：
   ```bash
   $PRG --scan-cables          # 应列出 "Gowin USB Cable(FT2CH)"
   $PRG --scan                 # 应识别到 GW5AT-138B（ID 0x0011081B）
   ```

通过标准：两条命令都能看到下载器和芯片。看不到就换 USB 口 / 线，重装驱动（第 9 节）。

## 4. 生成比特流与烧录

### 4.1 三种来源，按需要选

| 来源 | 什么时候用 | 做法 |
|---|---|---|
| A. 已验证的备份 | 只想先让板子跑起来 | `D:\tools\nav_fw_128k_mac.fs`（导航固件，RAM 128 KB，带雷达网口 MAC 与新帧解析）、`D:\tools\nav_selftest_32k.fs`（自检）；旧的 `nav_fw_160k.fs` 没有雷达网口。**建议拷到 U 盘 / 网盘**，换电脑时直接用 |
| B. 从源码构建 | 改了 RTL、约束、IP，或第一次在新电脑上 | 第 4.2 节，约 20 分钟 |
| C. 只换固件 | 只改了 `fw/` 下的 C 代码 | 第 4.5 节，不用重新综合，约 10 秒 |

注意：`nav_fw_128k_mac.fs`（2026-09-30）已经是新的 IMU 引脚 C18/C19（第 6.2 节），可以直接接 IMU。
`nav_fw_160k.fs` 和 `nav_selftest_32k.fs` 是 2026-09-29 的旧版：IMU 串口在拨码开关的引脚上、没有雷达网口、方向表格式也是旧的，
只能用来做第 4.6 节的基本检查；需要自检版时按 4.2 重新生成。

### 4.2 从源码构建

```bash
$PY fw/bsp/build_rv.py                                   # 编译导航固件 -> fw/build/rv/nav/
$PY gowin/syn/syn.py --board --app=nav --ram-kb=128      # 综合 + 布局布线 + 比特流，约 20 分钟
# 自检小镜像（约 15 分钟）：$PY gowin/syn/syn.py --board --app=selftest --ram-kb=32
```

- 结果：`gowin/syn/build_board/impl/pnr/nav_board_top.fs`。脚本最后会打印资源占用和 `pnr:` 文件列表。
- 通过标准：没有打印 `ERROR`；打开 `gowin/syn/build_board/impl/pnr/` 下的时序报告（`*.tr.html`），没有负的 slack（setup / hold）。
- 失败时看 `gowin/syn/build_board/gw_sh.log`。
- 每次成功构建并上板验证过的 `.fs`，按日期改名另存一份（例如 `nav_20261015_imu.fs`），出问题时能退回。

### 4.3 下载到 SRAM（调试时用，掉电即失，不动 Flash）

```bash
$PRG --device GW5AT-138B --run 2 --fsFile "D:\Desktop\SOC FPGA\nav\gowin\syn\build_board\impl\pnr\nav_board_top.fs"
```

图形界面做法：打开 Gowin Programmer → Scan Device → 双击器件行 → Access Mode 选 `SRAM Mode`，Operation 选 `SRAM Program`
→ 选 `.fs` 文件 → Save → Program/Configure。

### 4.4 固化到 Flash（上电自动运行，整车测试和比赛用）

调试阶段不要固化，改一次就要重烧一次。确认比特流稳定后：

1. 第一次请用图形界面：Access Mode 选 `External Flash Mode`，Operation 选 `exFlash Erase,Program,Verify`（GW5AT 属于 Arora V，
   列表里有带 "Arora V" 的项就选它），选 `.fs` → Program/Configure。完成后断电再上电，确认现象与第 4.6 节一致。
2. 图形界面成功后，可以改用命令行（操作号 54 = exFlash Erase,Program,Verify Arora V）：
   `$PRG --device GW5AT-138B --run 54 --fsFile "…\nav_board_top.fs"`
3. 固化后仍然可以用第 4.3 节临时下载 SRAM 调试，断电后回到 Flash 里的版本。
4. Flash 启动后，第 5 节的初始化**仍然要做**（寄存器每次上电都回到复位值）。

### 4.5 只换固件（不重新综合）

```bash
$PY fw/bsp/build_rv.py                                        # 编译
$PY tools/nav_uart.py COM11 load fw/build/rv/nav/nav_fw.bin   # 保持软核复位 -> 写 RAM -> 抽查回读 -> 放开运行
```

- 通过标准：打印"抽查回读错误 0"。
- 条件：当前比特流是导航版（RAM 128 KB），且 `build_rv.py` 打印的镜像 + .bss 不超过 RAM（链接脚本会检查，超了直接报错）。
- 报"连续写应答缺失"：旧比特流的串口桥连发写会丢命令（roadmap 第 4 节第 21 条），2026-09-30 之后的比特流已修；旧比特流上重试几次，
  或先按 4.3 节下载新比特流。
- 这样装的固件断电就没了，恢复为比特流里的固件。要长期保留，按 4.2 重新生成比特流。

### 4.6 下载后检查（每次下载后都做）

| 检查 | 命令 / 现象 | 通过标准 |
|---|---|---|
| 心跳 | 板上 LED0 | 亮、灭各约 0.67 s 交替（50 MHz 时钟在跑） |
| 软核 | LED4–LED7 / LED8 | LED4–7 每秒变化一次（固件主循环在跑）；LED8 灭（亮 = 软核异常停机） |
| 寄存器通路 | `$PY tools/nav_uart.py COM11 id` | `ID=0x4E415630`，TIME_US 每次读都在增加 |
| 软核信息 | `$PY tools/nav_uart.py COM11 soc` | `RAM 128 KB`（自检版为 32 KB），保持复位 0 |
| 控制台 | `$PY tools/nav_uart.py COM11 con`（Ctrl-C 退出） | 上电后约 2 s 内出现 `lidar LUT ready (M60 default lens), 约 1600 ms`；之后每秒一行 `t=… loop 约 32000–35000/s max 约 200 us frames 0 msgs 0 … lut 1`；没接雷达时约每 7 s 一行 `lidar start #n (tcp -> 192.168.0.101:7787)`（雷达启动客户端在重试，正常） |
| 雷达网口 PHY | `$PY tools/nav_uart.py COM11 rd NETL_PHY_STAT` | 低 3 位 `110`（找到 PHY、初始化完成；接上网线后最低位变 1 = 链路建立）；高 16 位约 390（百兆）或约 39（没接网线时 PHY 输出 2.5 MHz） |

自检版比特流的控制台应出现 `SELFTEST PASSED`，之后每秒一行 `alive`。LED1–LED3 分别是中断、急停、速度指令超时，
不作为这一步的判据。

## 5. 每次上电后：运行期初始化（必须做）

```bash
$PY tools/board_init.py COM11 --no-safe --mount-z 155
```

| 参数 | 含义 | 何时改 |
|---|---|---|
| `--mount-z` | 雷达光心离地高度 mm | 缺省 155（CAD，腿长 130 mm）；第 6.5 节量过后填实测值 |
| `--imu-baud` | IMU 波特率（缺省 115200） | 按 IMU 实际设置 |
| `--clk-mhz` | 核心时钟（缺省 50） | 以后打开 PLL（100 MHz）时改 |
| `--v-max` / `--w-max` | 速度 mm/s / 角速度 mrad/s 上限（缺省 300 / 800） | 首次上车用 150 / 500 |
| `--no-safe` | 速度输出保持 0 | 底盘接入并架空测试（第 6.6 节）之前一直加上 |

通过标准：每一行都显示 `OK`，最后打印"初始化完成"。先用 `--dry-run` 可以只看要写哪些值、不连板子。
按了复位键 S0 或者重新下载 / 断电后，都要重新运行一次。

## 6. 分阶段接入与测试

顺序：6.1 串口主机链路 → 6.2 IMU → 6.3 雷达网口检查 → 6.4 雷达 → 6.5 雷达标定 → 6.6 底盘 → 6.7 无图导航 → 6.8 有图导航 →
6.9 RK3588。每一阶段只新接一样东西，出问题时范围最小。每一阶段的结果填进附录 B 的记录表。

### 6.1 PC 经 USB 串口下发任务、读状态（不接任何外设）

```bash
$PY tools/nav_host.py --link uart:COM11 listen          # 看状态，约 5 次/秒，Ctrl-C 退出
$PY tools/nav_host.py --link uart:COM11 goal 1000 0     # 下发目标点（x=1 m, y=0）
$PY tools/nav_host.py --link uart:COM11 listen
$PY tools/nav_host.py --link uart:COM11 mode estop      # 急停
$PY tools/nav_host.py --link uart:COM11 listen
$PY tools/nav_host.py --link uart:COM11 mode idle
```

通过标准：
- 第一次 `listen` 连续打印 `{'type': 'status', 'mode': 0, 'state': '空闲', … 'loc': '无图' …}`，`t_us` 递增；
- 下发目标后 `goal` 字段变成 `(1000, 0)`，state 变为 `行进` 或 `受阻`（没有雷达时地图全是未知，`受阻` 是正常的）；
- 急停后 state 为 `急停`；`con` 控制台里 `msgs` 计数随每次下发增加。

这是串口注入第一次上板。`listen` 什么都不打印时：先用 `nav_uart.py COM11 rd MBX_TX_SEQ` 看发送计数是否在涨
（在涨 = 固件在发状态、问题在读回；不涨 = 固件没发），把结果记下来再排查。

### 6.2 IMU

**第一步：先在电脑上确认 IMU 协议（不接板子）。** IMU 接 USB-TTL 模块，用串口助手（资料里的"串口调试助手"）以 IMU
的波特率打开，十六进制显示：
- 看到反复出现 `55 51 … / 55 52 … / 55 53 …`（每帧 11 字节）= WIT 协议，固件可以直接用；
- 否则记下帧格式和厂家手册，需要改 `fw/nav/imu_parse.c`（只改这一个文件），改完按 4.5 节装固件。
- 同时确认：输出频率（建议 ≥ 100 Hz、带角速度）、电平（3.3 V TTL / 5 V TTL / RS232）。

**第二步：接线**（扩展口 P11，40 针；针号按管脚信息表的顺序推断，以底板丝印和原理图为准）：

| IMU | 开发板 | FPGA 引脚 |
|---|---|---|
| TX | P11 第 1 针 GPIO0_0 | C18（`imu_rxd`） |
| RX | P11 第 2 针 GPIO0_1 | C19（`imu_txd`） |
| GND | P11 第 12 针 GND | — |
| VCC | 按 IMU 要求：P11 第 29 针 3.3 V 或第 11 针 5 V，或单独供电 | — |

- 只有 **3.3 V TTL** 可以直接接。5 V TTL 要加电平转换，RS232 要加 MAX3232 类转换板，否则会损坏 FPGA 引脚。
- 这个引脚分配是 2026-09-30 改的（原来的 C20/D20 是拨码开关 SW7/SW6）。备份 `nav_fw_128k_mac.fs` 和之后从源码生成的比特流都已包含；
  **不要用 `nav_fw_160k.fs` / `nav_selftest_32k.fs` 接 IMU**（第 4.1 节）。

**第三步：上板验证**
```bash
$PY tools/board_init.py COM11 --no-safe --imu-baud 115200     # 换成 IMU 的实际波特率
$PY tools/nav_uart.py COM11 wr 0xF000 1                      # 暂停软核（让字节留在缓冲里）
$PY tools/nav_uart.py COM11 rd IMU_RX_COUNT                  # 隔 1 秒读两次
$PY tools/nav_uart.py COM11 rd IMU_RX_DATA                   # 多读几次，每次弹出 1 字节
$PY tools/nav_uart.py COM11 reset                            # 恢复软核运行
$PY tools/nav_uart.py COM11 rd IMU_STAT_OVF                  # 隔 1 秒读两次
```
通过标准：暂停软核时 `IMU_RX_COUNT` 在增长（涨到 256 停住是正常的，缓冲满了）；`IMU_RX_DATA` 的低 8 位
应和第一步在电脑上看到的字节一致（如 `0x…55`）；恢复软核后 `IMU_STAT_OVF` 不再增长（暂停期间涨的不算）。
全是乱码或不增长 → 波特率 / 接线 / 电平不对（第 9 节）。

### 6.3 雷达网口检查（RJ45）

板上 RJ45 用自写的百兆 MAC（`rtl/net/rgmii_mac100.v`），已经在导航比特流里，**不需要生成 IP**。它上电时复位 PHY，
经 MDIO 把 PHY 的自协商限制到 100BASE-TX（接雷达或接电脑千兆网卡都工作在百兆），之后每 200 ms 读一次链路状态。

```bash
$PY tools/nav_uart.py COM11 rd NETL_PHY_STAT      # 先不接网线读一次，再接上网线（另一端接电脑或雷达）读一次
```

| 位 | 含义 | 通过标准 |
|---|---|---|
| bit 2 / bit 1 | 找到 PHY / MDIO 初始化完成 | 都为 1（上板已实测，PHY 地址 = bit 7:3 = 0） |
| bit 0 | 链路 | 接上网线、对端上电后变 1，RJ45 的连接灯亮 |
| bit 31:16 | RX 时钟测频（值 × 64 = kHz） | 链路建立后约 390（25 MHz = 百兆）；约 39 = 十兆或没接网线 |

链路建立不了：换网线；电脑侧在网卡属性里把"速度和双工"设为自动或 100 Mbps 全双工；记下 `NETL_PHY_STAT` 的值。

### 6.4 雷达数据接入

格式已按厂家 SDK 源码实现（[icd.md](icd.md) 第 2 节），寄存器缺省值就是雷达的格式，正常情况下不需要改任何格式寄存器。

**第一步：PC 冒充雷达**（验证 FPGA 网口链路，不需要雷达）
1. 电脑有线网卡设静态 IP `192.168.0.10`、掩码 `255.255.255.0`；网线直连板上 RJ45；Windows 防火墙允许 Python 通过（或临时关闭）。
2. 发帧并看统计（导航固件；`fake_lidar.py` 按雷达的格式发深度帧，帧尾带 SDK 的 M60 缺省内参）：
   ```bash
   $PY tools/fake_lidar.py 192.168.0.100 --fps 10 --frames 100 --imu
   $PY tools/nav_uart.py COM11 stat
   $PY tools/nav_uart.py COM11 map fake.png
   arp -a | findstr 192.168.0.100
   ```
通过标准：`NETL_STAT_ARP` > 0（`arp -a` 里 192.168.0.100 对应 `02-4e-56-00-00-01`）；`NETL_STAT_RX_UDP`、`LIDAR_STAT_FRAMES`
随发送增长；`LIDAR_STAT_BAD` = 0；`LIDAR_STAT_SKIP` 与帧数相同（`--imu` 每帧夹一条 IMU 消息，应被跳过）；`LIDAR_LAST_WH` =
`0x003C00A0`（160 × 60）；`LIDAR_INFO_SEQ` 随帧增长、`LIDAR_LENS0` = `0x429C6CD0`（fx = 78.21）；`fake.png` 里能看到虚拟场景的障碍。
FPGA 不回 ping，ping 不通是正常的。

3. 雷达启动（TCP 客户端）也冒充：电脑只在收到板子发来的"开始测量"后才出流，并且发给连它 TCP 的那个 IP（与实物一致）：
   ```bash
   $PY tools/nav_uart.py COM11 wr NETL_LIDAR_IP 0xC0A8000A      # "雷达 IP"改成电脑网卡的 IP（这里 192.168.0.10；掉电恢复 192.168.0.101）
   $PY tools/fake_lidar.py --tcp-server --fps 10 --stream-for 20   # 每次启动后出流 20 s 就停，看板子会不会重新启动它
   ```
   通过标准：`fake_lidar.py` 打印 `TCP 连接 #1 来自 192.168.0.100`、`开始测量（数据 0101）`、`TCP 断开`，随后开始出流；板子串口
   控制台（`$PY tools/nav_uart.py COM11 con`）打印 `lidar start #1 (tcp -> 192.168.0.10:7787)`、`lidar start ok (1)`；
   每次停流约 2 s 后出现新的一次连接与启动。`nav_uart.py COM11 stat` 里 `NETL_RAW_STAT` 的 bit 31:24（接收格满丢帧）应为 0。
   Windows 防火墙第一次会弹窗，允许 Python 在"专用网络"上通信。

**第二步：雷达接电脑，确认雷达正常、读配置**（先不接开发板）
1. 雷达接 12–25 V（≥ 3 A）电源，网线接电脑（USB 网卡也可以）。厂家说明：先插电源、再插网线。电脑网卡设 `192.168.0.x`
   （不能用 100、101；2026-09-30 实测用的是 192.168.0.2）。
2. 打开厂家上位机，连接后能看到深度图即雷达正常。
3. **关掉上位机**（雷达同一时间只理一个 TCP 连接，上位机连着时 `xt_config.py` 收不到应答），再读设备信息、配置、内参：
   ```bash
   $PY tools/xt_config.py info                     # SN、固件、IP、MAC、当前 UDP 目标
   $PY tools/xt_config.py config                   # 输出类型、HDR、帧率、上电自动运行
   $PY tools/xt_config.py lens                     # 内参（每帧帧尾也带，固件会自己用；这里留底）
   $PY tools/xt_config.py start dist               # 开始连续输出、只出深度
   ```

2026-09-30 用实物（SN XTM60B20250324000234，固件 XTFW-RT-V2.34.2）确认的行为，下面的步骤都以此为准：

| 行为 | 结果 | 影响 |
|---|---|---|
| 上电自动运行（`config` 里的"上电自动运行"） | 0：上电后不出流，要有人发"开始测量" | 板上固件自己当 TCP 客户端发开始命令（`fw/nav/xt_link.c`，第四步） |
| UDP 发往哪里 | 发给最近一次连 TCP 的主机 IP，端口 7687；**命令 19（`udp-dest`）返回 OK 但不生效**（重启后也不变） | 不能把目标改成 FPGA；FPGA 当 TCP 客户端后自然就是目标 |
| TCP 断开后 | 继续出流 | 电脑发完"开始测量"就可以断开 |
| 报文 | 源端口 49154（不是 7687）；一帧 14 个分包（13 × 1400 + 1226 字节），分包之间常有几毫秒间隔 | Wireshark 过滤用 `udp.dstport == 7687` |

**第三步：抓包回放**（雷达接电脑，PC 冒充雷达回放真实数据）
1. 电脑上抓几秒原始负载（关掉上位机、`xt_config.py start dist` 之后）：Wireshark 过滤 `udp.dstport == 7687`、保存为
   `xt.pcapng`；或在厂家上位机里点 Record 录几十帧（`.xbin`，默认在 `C:\xt-pcds`）。
2. 网线换到开发板、电脑用另一个网口接板子时，回放给 FPGA：
   ```bash
   $PY tools/fake_lidar.py 192.168.0.100 --pcap xt.pcapng           # 按原时间间隔
   $PY tools/fake_lidar.py 192.168.0.100 --xbin C:/xt-pcds/某次录制 --fps 10
   $PY tools/nav_uart.py COM11 stat
   ```
3. 通过标准同第一步；另外 `LIDAR_LENS0..8` 应等于第二步读到的内参，控制台出现 `lidar LUT ready (device lens)`。

**第四步：雷达直连开发板**
雷达网线直接插板子 RJ45，雷达 IP 保持缺省 192.168.0.101（改过就写 `NETL_LIDAR_IP`）。板上固件发现没有帧就自己连雷达 TCP 发
"开始测量"（icd.md 第 2 节"雷达启动"）：板子先上电时约 2 s 后开始尝试，雷达后上电时等它启动完（约 6 s）自动连上；雷达中途掉电重启，
帧中断 2 s 后会重新启动它。收到 `lidar start ok` 后客户端保留 5 s 暖机保护，覆盖冷启动约 3 s 才出第一帧的情况，因此不会在暖机期间重复发送启动命令。没接雷达时控制台约每 7 s 打印一次 `lidar start #n`（ARP 没人应答，退避后重试），属正常。
2026-09-30 实测（雷达直连板子、下载比特流后）：约 2.5 s 打印 `lidar start #1`、3.3 s 打印 `lidar start ok (1)`，冷启动时应答 OK 后约 3 s 才有第一帧；新版本已把保护窗口调整为 5 s。之后 10 帧/秒、坏帧 0、
`NETL_STAT_RX_ERR` 0，约 2.6 s 后 `lidar LUT ready (device lens)`；已完成一次真实雷达掉电重启自动恢复验证。当前 5 s 保护版本已构建并下载，3.5 s 暖机场景已在仿真中通过，但该版本的真实冷启动尚未再次强制复测；把 `NETL_UDP_PORT` 临时改成 7688 模拟断流，约 2 s 后出现
`lidar start #2` / `ok (2)`，改回 7687 后恢复 10 帧/秒。电脑冒充雷达（第一步第 3 条）只在排查时用，不是必做项。

旧的临时办法（固件没有 TCP 客户端时用，2026-09-30 实测可用；`NETL_CTRL` 的 bit 6 `LIDAR_TCP` 写 0 可关掉板子的启动客户端）：
雷达只发给最后连它 TCP 的主机，所以让板子的雷达口**临时用电脑那个网口的 IP**（MAC 可选）：
1. 雷达接电脑，`xt_config.py start dist`（第二步），确认电脑能收到数据。记下电脑这个网口的 IP 和 MAC
   （`ipconfig /all`，例如 `192.168.0.2`、`9C-69-D3-4B-D9-FB`）。
2. 串口写板子雷达口（掉电恢复缺省）：
   ```bash
   $PY tools/nav_uart.py COM11 wr NETL_IP 0xC0A80002         # 192.168.0.2
   # 可选：再写 MAC，换线后立刻出流；不写则要等雷达重新发 ARP（实测约 2–3 分钟），板子应答后自动改发给板子
   $PY tools/nav_uart.py COM11 wr NETL_MAC_HI 0x00009C69     # MAC 前 2 字节
   $PY tools/nav_uart.py COM11 wr NETL_MAC_LO 0xD34BD9FB     # MAC 后 4 字节
   ```
3. **雷达不断电**，网线从电脑拔下插到板子 RJ45；`NETL_PHY_STAT` 最低位为 1。
4. `$PY tools/nav_uart.py COM11 stat` 隔几秒读两次，`$PY tools/nav_uart.py COM11 map lidar.png`。

通过标准：`LIDAR_STAT_FRAMES` 约 10 帧/秒增长，`LIDAR_STAT_BAD` 与 `LIDAR_STAT_DROP` 基本不涨，`LIDAR_STAT_POINTS` 明显大于 0
（上限 9600，四角切除 781 个像素），`LIDAR_LENS0..8` 等于 `xt_config.py lens` 的值；`lidar.png` 中雷达前方的地面为深色（低代价），
墙 / 箱子为亮色（254 不可通行；没看到的区域 255 也是亮色，靠形状区分）。2026-09-30 实测：10 帧/秒、坏帧 0、`NETL_STAT_RX_ERR` 0。
雷达还没按车上的位姿安装时（例如放在桌上），代价图会是一整片不可通行，这是正常的，先做 6.5 节的安装与标定。

### 6.5 雷达标定（方向、安装位姿）

**方向表**：固件按雷达每帧帧尾自带的内参自动生成（深度按径向距离，与厂家 SDK 一致），不需要手工操作。
`$PY model/lidar_lut.py --out lut.hex --fx … …` + `nav_uart.py COM11 lut lut.hex` 只在排查时用（手工写入的表会在内参变化时被固件覆盖）。

**核对左右 / 上下**（只需做一次）：车静止，在雷达**左前方** 1 m 放一个箱子，`nav_uart.py COM11 map` 看箱子是否在地图的左侧
（机器人前方为 +X、左为 +Y）。反了就把 `fw/nav/lidar_cal.c` 里的 `flip_u` 改成 false（左右）后按 4.5 节装固件，并告诉开发
（`model/lidar_lut.py` 要同步改）。上下反了（地面出现在上方）同样反馈。

**核对深度定义**：雷达正对一面平墙、距离约 1 m 且垂直。上位机里点墙中间和墙两端的像素看"距离"：两端比中间大（视场边缘约
大 1.8 倍）= 径向距离，与 SDK 和我们的实现一致；两端和中间基本一样 = 沿光轴，与实现不符，记下来反馈给开发。

**安装位姿**：已由整车 URDF / STL 标定（`$PY tools/robot_calib.py`，结果 `config/robot_calib.json`，[platform.md](platform.md)
第 1、2 节），固件与仿真的缺省值就是标定值：

| 量 | 当前标定值 | 写在哪里 | 实车要确认的 |
|---|---|---|---|
| 俯仰 | **正常正装、相对机身下俯 10°** | `fw/nav/nav_task.c` `pose_init` 第 1 个参数 `ANGLE_FROM_CDEG(1000)` | 已按实车支架修改；车体水平时仍可用手机水平仪复核约 10°；如支架再次调整，需同步固件和仿真 |
| 前伸 x | 镜头在轮轴前 123 mm | `pose_init` 第 2 个参数 | — |
| 高度 z | 腿长 L0 + 25 mm（L0 = 130 时 155 mm） | `pose_init` 第 3 个参数、`board_init.py --mount-z`、`MATCH.MOUNT_Z` 复位值 | **底盘站立时的 L0**：卷尺量镜头中心到地面，与 L0 + 25 差 1 cm 以上就改这三处 |

腿长或支架改了：`$PY tools/robot_calib.py --leg <L0>` 打印要改的数，改完按 4.5 节装固件，`--mount-z` 与第 3 个参数必须相同。
机械图纸改了：机械组重新导出 URDF 后再跑一次这个脚本。

**验证**（导航固件、第 5 节初始化后，车静止放在场地里）：`$PY tools/nav_uart.py COM11 map calib.png`

| 摆放 | 应该看到 |
|---|---|
| 前方 0.5–3 m 空地 | 大片低代价，没有散落的不可通行点（有 = 俯仰角或高度不准） |
| 15 cm 以上的箱子 | 不可通行，位置和实际距离一致（卷尺量，误差 < 5 cm） |
| 1 cm 地垫边沿 | 可通行 |
| 5 cm 木条 | 不可通行 |

远处地面整片变成障碍 → 俯仰角偏了，改 1–2° 再试；箱子距离整体偏近 / 偏远 → 深度单位或内参不对。

### 6.6 底盘接入

**架构和接口说明**：底盘 RL 代码分层、`ch_*` / `odo_*` / `ext_stop` 语义、外部控制器桥接及故障降级见 [chassis_integration.md](chassis_integration.md)。本节负责现场接线和测试步骤，接口真值以 [icd.md](icd.md) 第 4 节为准。

**先要完成的开发工作**（本手册之外）：现在的 `gowin/nav_board_top.v` 里速度输出没有接出去、里程计输入接的是 0。
需要一个整车顶层把 `nav_subsystem` 和运动控制连起来：
- 运动控制（RL）也在这块 FPGA 里：`ch_vx / ch_wz / ch_valid / ch_stop` 直接接它的指令输入，它估计的**对地**速度接 `odo_valid / odo_vx / odo_wz`；
- 运动控制在别的控制器上：要加一个串口 / CAN 桥（仓库里还没有）。
- 接口约定见 [icd.md](icd.md) 第 4 节：速度 mm/s、角速度 mrad/s；`ch_stop` 的含义是"速度归零、保持平衡"，不是断电。
- 同时把一个物理急停按钮接到 `ext_stop`，并用 `SAFE_EXT_MASK` 打开。

**架空测试**（轮子离地，电机电源急停开关在手边）：

1. 上电后先 `board_init.py … --no-safe`，确认轮子不动。
2. 暂停软核（否则固件会不停写速度指令，覆盖手动指令），打开速度输出，手动发指令：
   ```bash
   $PY tools/nav_uart.py COM11 wr 0xF000 1
   $PY tools/board_init.py COM11 --v-max 150 --w-max 500 --mount-z 155     # 这次不加 --no-safe
   $PY -c "
   import sys, time; sys.path[:0] = ['tools', 'model']
   import nav_uart, nav_regs_map as m; u = nav_uart.NavUart('COM11'); R = m.REGS
   for i in range(30):                      # 3 秒，前进 100 mm/s；改第二个值测转向（wz，左转为正）
       u.wr(R['SAFE_CMD_V'], 100); u.wr(R['SAFE_CMD_W'], 0); time.sleep(0.1)
       print(hex(u.rd(R['SAFE_OUT_V'])), hex(u.rd(R['SAFE_ODO_V'])))
   "
   ```
   负数按 16 位补码写：-100 写 `0xFF9C`，例如 `u.wr(R['SAFE_CMD_W'], 0xFE0C)` 是右转 500 mrad/s。
3. 检查：

| 项目 | 通过标准 |
|---|---|
| 方向 | vx > 0 车轮向前转；wz > 0 车头向左（逆时针，从上往下看） |
| 限幅 | 指令 300 mm/s 时 `SAFE_OUT_V` 不超过 150 |
| 里程计 | `SAFE_ODO_V` 符号与指令一致，稳定后与指令相差 < 10 % |
| 看门狗 | 脚本结束后约 0.2 s 内输出按加速度限制降到 0，LED3 亮 |
| 软件急停 | `wr SAFE_CTRL 5`（ENABLE + ESTOP）后下一个安全输出节拍开始归零，LED2 亮；`wr SAFE_CTRL 1` 恢复 |
| 物理急停 | 按下后输出为 0 |

4. 恢复软核：`$PY tools/nav_uart.py COM11 reset`。
5. 落地测里程计（导航固件运行，`--no-safe`，用手推车）：推直线 2 m，`nav_host.py … listen` 里 x 应为 2000 ± 60 mm；
   原地转一圈，yaw 回到起始值 ± 5°。
6. 把实测能力填进 `fw/nav/local_planner.c` 开头的 `caps`（`vx_max`、`wz_max`、`radius_mm`；两轮车 `holonomic = false`）和
   `v_max / w_max`，以及 [platform.md](platform.md)，按 4.5 节装固件。

### 6.7 实地导航：无图模式（上电缺省）

准备：场地按 [platform.md](platform.md) 第 3 节搭好；车放在起点，车头方向就是 +x，左边是 +y，起点为原点（上电或
`nav_host.py … pose 0 0 0` 时重置）。每次测试前：第 5 节初始化（不加 `--no-safe`，首次 `--v-max 150 --w-max 500`）。
一人看车、手放在急停上，一人操作电脑。

由近到远做，前一项通过再做下一项：

| 编号 | 操作 | 通过标准 |
|---|---|---|
| N1 | 空地 `goal 1000 0` | 到达（state `到达`），卷尺量终点误差 < 10 cm |
| N2 | 空地 `goal 2000 1000` | 到达，路线平滑，无原地打转 |
| N3 | 起点和目标之间放一个箱子 `goal 3000 0` | 绕开箱子到达，无碰撞 |
| N4 | 路径点 `wp 2000,0 2000,1500 0,1500 0,0 --loop` | 连续跑 3 圈，回到起点误差 < 20 cm |
| N5 | 可跨台阶（10–15 mm）、8° 坡 | 通过 |
| N6 | 不可跨台阶（≥ 50 mm）、20° 陡坡 | 判为不可通行，绕行或停下（state `受阻`） |
| N7 | `mode search` | 探索全场，不撞墙 |
| N8 | 行驶中 `mode estop` | 立即停车；`mode idle` 后可重新下任务 |

```bash
$PY tools/nav_host.py --link uart:COM11 goal 1000 0
$PY tools/nav_host.py --link uart:COM11 listen | tee logs/N1_$(date +%H%M%S).txt    # 先 mkdir logs
```
N1–N3 先用 150 mm/s 跑通，再逐步把 `--v-max` 提到 300、500。每次失败保存 `listen` 日志、`map` 截图和控制台输出（第 8 节）。

### 6.8 实地导航：有图模式（先验地图）

流程与命令见 [board_bringup.md](board_bringup.md) 第 6 节，这里补充实机要点：
1. 地图来源二选一：
   - 扫图 + SLAM：需要把雷达和 IMU 数据录到电脑再转给 Linux SLAM（FAST-LIO 等），**录包与格式转换工具仓库里还没有**，要另外准备；
   - 5 m 场地更简单的办法：用卷尺量出墙和箱子的位置，写成 `.xyz` 点云（每行 `x y z`，单位 m；墙面和箱子侧面每 2 cm 一个点、
     高度 0–0.5 m，地面每 5 cm 一个点、z = 0），再 `map_tool.py build`。需要一个小脚本生成点云。
2. `$PY tools/map_tool.py build field.xyz -o field.nvmp`，打开 `field.png` 检查墙和箱子轮廓。
   地图包现在是版本 2（18 位槽字），2026-09-30 之前生成的 `.nvmp` 会被固件拒收（状态显示"格式错"），用同一命令重新生成即可。
3. 下发与定位：
   ```bash
   $PY tools/nav_host.py --link uart:COM11 map field.nvmp          # 应答"已加载"
   $PY tools/nav_host.py --link uart:COM11 pose 500 500 0          # 车实际位置（地图系 mm、度），误差 0.5 m / 10° 内
   $PY tools/nav_host.py --link uart:COM11 listen                  # loc 变为"正常"后再下任务
   ```
4. 测试项：P1 定点（终点误差 < 5 cm）；P2 路径点循环；P3 `mode search`；P4 扫图后新增一个箱子（应绕开）；
   P5 行驶中把车抬起挪 0.5 m（应检测到丢失、原地停下、自动重定位）。
5. `unmap` 回到无图模式。

### 6.9 RK3588（可选，不影响基础导航）

需要先生成 1G Serial Ethernet IP（[board_bringup.md](board_bringup.md) 第 1 节表格第 3 行），把文件加进 `gowin/ip/ip_files.txt`，
构建时加 `--define=NAV_USE_SFP`；SFP1 插电口 SFP 模块，网线接 RK3588。RK3588 网口设 `192.168.2.10`，在 RK3588 上：
`python3 tools/nav_host.py --link udp:192.168.2.100 listen`。通过标准：`nav_uart.py stat` 里 `NETH_STAT_RX_UDP` 增长，RK3588 上能收到状态。

## 7. 改了什么之后要做什么

| 改动 | 必须做 | 上板 |
|---|---|---|
| `fw/` 下的 C 代码 | `$PY fw/test/build.py`（主机测试）→ `$PY fw/bsp/build_rv.py` | 临时：`nav_uart.py load`；长期：4.2 重新构建 |
| `config/nav_params.yaml`、`config/regmap.yaml` | `$PY tools/gen_config.py` → `$PY sim/run.py`（RTL 回归，需要 Vivado）→ `$PY fw/test/build.py` | 4.2 重新构建 |
| `rtl/` 下的 Verilog | `$PY sim/run.py` → 4.2 | 重新构建、下载，看时序 |
| `gowin/nav_board.cst`（引脚） | 核对管脚信息表 → 4.2 | 重新构建、下载 |
| IP（`gowin/ip/`，目前只有 SFP 口需要） | 更新 `ip_files.txt`，核对端口 → 4.2（带 `--define`） | 重新构建、下载 |
| 雷达格式寄存器（缺省即 SDK 格式，一般不用改） | 临时：`nav_uart.py wr` / `board_init.py --lidar-ctrl`；长期：改 `regmap.yaml` 复位值 | 同上一行 |
| 方向查找表 | 不用管：固件按雷达帧尾的内参自动生成（6.5 节）；排查时 `nav_uart.py lut` 临时写入 | — |
| 雷达安装位姿 / 腿长 | `tools/robot_calib.py --leg <L0>` 算出新值，改 `fw/nav/nav_task.c` 的 `pose_init`，`board_init.py --mount-z` 同步 | 4.5 |

没有 Vivado 的电脑不能跑 `sim/run.py`；这时只改 `fw/` 和运行期寄存器，RTL 与 yaml 改动留到有仿真环境时做。

## 8. 出问题时收集什么

记录下来再改，不要边猜边改。把下面这些放进一个以日期命名的文件夹：
1. 比特流版本（文件名 / 构建日期）、固件版本（控制台第一行 `nav fw <日期>`）、`board_init.py` 的输出；
2. `$PY tools/nav_uart.py COM11 stat` 的输出（出问题时读一次，隔 5 秒再读一次）；
3. 控制台 30 秒：`$PY tools/nav_uart.py COM11 con > con.txt`；
4. `$PY tools/nav_uart.py COM11 map fail.png`；
5. `nav_host.py … listen` 的日志、现场照片或视频、当时的操作步骤。

## 9. 故障排查

| 现象 | 可能原因 | 处理 |
|---|---|---|
| `--scan` 找不到下载器 | JTAG 驱动、USB 线只能充电、接错 USB 口 | 换线换口；重装 Gowin Programmer 驱动；Gowin 图形界面的 Programmer 能否识别 |
| 找到下载器但识别不到芯片 | 板子没上电、器件型号选错 | 查电源灯；`--device GW5AT-138B` |
| 下载成功但 LED0 不闪 | 下载了别的工程的 `.fs`；时钟没起来 | 确认文件路径与时间；重下备份比特流 |
| LED8 亮（软核异常） | RAM 里的固件坏了 / 版本不对 | `nav_uart.py load` 重装固件，或重下比特流 |
| `nav_uart.py … id` 报"无应答" | 串口号错、串口被其他程序占用、比特流不对 | 设备管理器看 COM 号；关掉 `con` / `listen` / 串口助手；先过 4.6 节 |
| 读到的 ID 不是 `0x4E415630` | 连到了别的串口（如下载器附带的 COM） | 换成 CH9102 那个 COM |
| 控制台 `loop` 很小（< 1 万/s）或 `max` 很大（> 50 ms） | 固件卡在某处轮询 | 保存控制台，按第 8 节收集后分析 |
| `nav_host.py listen` 无输出 | 串口注入 / 回读未上板验证过（第 0 节） | 按 6.1 节最后一段先判断是发送还是回读的问题 |
| `NETL_STAT_ARP` = 0 | 链路没建立；网线、网卡 IP 网段不对；用了旧比特流 | 6.3 节看 `NETL_PHY_STAT`；电脑 IP 必须是 192.168.0.x（不能是 100、101） |
| `NETL_STAT_ARP` 在涨，但电脑 `arp -a` 里没有 192.168.0.100 | 板子发送方向不通（2026-09-30 遇到过，见 roadmap 第 4 节第 13 条） | 确认用的是修复后的比特流；临时绕过：管理员 PowerShell `New-NetNeighbor -InterfaceIndex <网卡编号> -IPAddress 192.168.0.100 -LinkLayerAddress 02-4E-56-00-00-01 -PolicyStore ActiveStore` |
| ARP 有，`NETL_STAT_RX_UDP` 不涨 | 目的端口不是 7687、电脑防火墙拦了发送 | 查雷达 / `fake_lidar` 的目的端口；防火墙放行 |
| `NETL_STAT_RX_ERR` 增长（甚至等于 `NETL_STAT_RX_FRAMES`） | RGMII 接收采样时序（随布线变，见 roadmap 第 4 节第 17 条）；线缆 | 确认 `NETL_CTRL` 位 4（RX_NEG，下降沿采样）为 1；换网线；仍有错误时 `wr NETL_CTRL 0x0D` 切到上升沿对比，把两种的错误率反馈给开发 |
| `LIDAR_STAT_BAD` 增长（`NETL_STAT_RX_UDP` 正常、`LIDAR_STAT_SKIP` 约为坏帧的 4–5 倍） | 旧比特流的帧内超时只有 3 ms，真雷达分包间隔更长（2026-09-30 实测 95% 的帧被判坏） | 用 2026-09-30 之后的比特流（超时 200 ms） |
| `LIDAR_STAT_BAD` 增长（其他情况） | 帧格式和 SDK 不一致（雷达固件版本不同） | 6.4 节第三步，抓包反馈 |
| `LIDAR_STAT_FRAMES` 涨，`LIDAR_STAT_POINTS` 一直是 0 | 方向表没生成（控制台没有 `lidar LUT ready`）；深度全是无效码 | 看控制台；上位机看深度图是否正常 |
| `LIDAR_STAT_DROP` 增长 | 地图处理跟不上帧率 | 雷达最大帧率降到 10 |
| `LIDAR_STAT_FRAMES` 涨，但控制台 `frames` 一直是 0 | `LIDAR_CTRL` 的 WAIT_POSE（位 3）被手动写掉了（`wr LIDAR_CTRL` 或 `--lidar-ctrl` 的值里没有 8）；或者跑的是自检版 | `rd LIDAR_CTRL` 应为 `0x1F`；不带 `--lidar-ctrl` 重新运行 `board_init.py` |
| 控制台 `frames` 不涨，`LIDAR_STAT_DROP` 在涨 | 固件没有放行帧（卡在上一帧的匹配里） | 按第 8 节收集控制台和 `stat` 反馈 |
| 地图里地面大片变成障碍 | 俯仰角 / 高度不准；IMU 轴向不对 | 6.5 节；车静止时对比 IMU 读数 |
| 障碍位置左右颠倒 | 方向表列方向 | 6.5 节（`fw/nav/lidar_cal.c` 的 `flip_u`） |
| IMU 无数据 / 乱码 | TX、RX 接反；波特率；电平；用了旧比特流 | 6.2 节；`board_init.py --imu-baud` |
| 车不动 | `--no-safe` 还在；急停；定位未就绪（有图模式 loc 不是"正常"） | `nav_uart.py … rd SAFE_STAT`；`listen` 看 state / loc |
| 车走一段就停，LED3 亮 | 固件没有及时发指令（看门狗） | 看控制台 `max` 是否超过 200 ms |
| 转向方向反了 | 底盘 wz 符号约定不同 | 在整车顶层对 `ch_wz` / `odo_wz` 取反，不要改导航固件 |
| 有图模式一直"对准中" | 初始位姿偏差太大、视野里只有平行墙 | 位姿误差控制在 0.5 m / 10° 内；车头对着有转角的地方 |
| 布线一小时以上不结束 | BSRAM 占用太高 | 保持 `syn.py` 里的 `-place_option 1`；做 BSRAM 压缩 |

## 10. 安全须知

- 底盘第一次通电、改过运动控制或速度方向相关代码之后，**先架空测试**（6.6 节）。
- 实车测试时，一人手放在电机电源急停上。软件急停（`mode estop`）要经过串口，不能代替物理急停：串口正被别的命令占用时发不出去。
- 首次上车用低速（150 mm/s）；每次只提高一档。
- 雷达的使用与供电要求按 `XT-M60产品手册` 执行，不用不符合要求的电源或破损线缆。
- 雷达、电机与开发板各自供电，只共地，不要把电源输出接在一起。
- 改接线前先断电。扩展口只能接 3.3 V 信号。

## 附录 A. 命令速查

```bash
# 构建与下载
$PY fw/bsp/build_rv.py                                         # 编译固件
$PY gowin/syn/syn.py --board --app=nav --ram-kb=128            # 比特流（导航版）
$PY gowin/syn/syn.py --board --app=selftest --ram-kb=32        # 比特流（自检版）
#   附加选项：--define=NAV_USE_SFP（SFP 口，需先生成 IP） --tcl-only（只生成脚本）；雷达口 MAC 已内置
$PRG --device GW5AT-138B --run 2  --fsFile <路径>.fs           # 下载到 SRAM
$PRG --device GW5AT-138B --run 54 --fsFile <路径>.fs           # 固化到外部 Flash（先用图形界面确认一次）
$PY tools/nav_uart.py COM11 load fw/build/rv/nav/nav_fw.bin    # 只换固件
# 上电初始化
$PY tools/board_init.py COM11 [--no-safe] [--mount-z 155] [--imu-baud 115200] [--lidar-ctrl 0x1F] [--v-max 300 --w-max 800]
# 调试
$PY tools/nav_uart.py COM11 id | soc | con | stat | reset
$PY tools/nav_uart.py COM11 rd <寄存器名>   /   wr <寄存器名> <值>        # 名字见 docs/regmap.md，写成 块_名，如 SAFE_CTRL
$PY tools/nav_uart.py COM11 map out.png                        # 代价图
$PY tools/nav_uart.py COM11 wr 0xF000 1                        # 暂停软核（reset 恢复）
# 任务
$PY tools/nav_host.py --link uart:COM11 goal <x_mm> <y_mm> | wp x,y x,y … [--loop] | mode idle|goto|search|estop | pose x y 度 | map f.nvmp | unmap | listen
# 雷达
$PY tools/fake_lidar.py 192.168.0.100 --fps 10 [--imu]         # PC 冒充雷达（SDK 格式）
$PY tools/fake_lidar.py 192.168.0.100 --pcap xt.pcapng         # 回放 Wireshark 抓包；--xbin <目录> 回放上位机录制
$PY tools/xt_config.py info | config | lens | udp-dest 192.168.0.100 7687 | start dist | stop   # 雷达 TCP 配置
$PY model/lidar_lut.py --out lut.hex [--fx … …]               # 内参 -> 方向表（只在排查时用，参数见 --help）
```

LED：LED0 心跳；LED1 中断；LED2 急停；LED3 速度指令超时；LED4–7 软核每秒计数；LED8 软核异常。

网络缺省：FPGA 雷达口 `192.168.0.100`（UDP 7687，MAC `02:4E:56:00:00:01`，另收组播 239.255.255.76）；FPGA 主机口（SFP）`192.168.2.100`（UDP 9000）；
雷达出厂 `192.168.0.101`（TCP 7787；UDP 发往连它 TCP 的主机的 7687 端口，源端口 49154）。

## 附录 B. 测试记录表（每次测试复制一份）

| 项目 | 内容 |
|---|---|
| 日期 / 测试人 | |
| 比特流文件 / 固件版本 | |
| `board_init.py` 参数 | |
| 场地布置（照片编号） | |
| 测试编号（如 6.4-4、N3） | |
| 操作 | |
| 结果（通过 / 不通过） | |
| 测量值（终点误差、用时、碰撞次数） | |
| 日志文件 | |
| 雷达（6.4 节第二步）：固件版本、不连 TCP 上电是否自动发 UDP / 发往哪个 IP、内参截图 | |
| 问题与下一步 | |
