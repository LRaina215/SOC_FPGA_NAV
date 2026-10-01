# 开发者指南（新成员、其他模块接入前必读）

本文写给要在本仓库里改代码或接入新模块的人（导航组新成员、底盘 / RL 组、视频组、RK3588 组）。
它不重复接口细节，只说明：每样东西的唯一真值在哪里、改了什么要跑什么、新模块怎样接进同一个工程、哪些约束不能碰。
环境安装见 [hardware_guide.md](hardware_guide.md) 第 2 节，本机工具路径见 [roadmap.md](roadmap.md) 第 2 节。

## 1. 先认清"唯一真值"

| 内容 | 唯一来源 | 由它生成 / 引用它的 |
|---|---|---|
| 寄存器地址、字段、复位值 | `config/regmap.yaml` | `tools/gen_config.py` 生成 `rtl/regs/nav_regs_map.vh`、`rtl/regs/nav_subsystem_rf.vh`、`rtl/regs/nav_regfile.v`、`fw/include/nav_regs_map.h`、`model/nav_regs_map.py`、`docs/regmap.md` |
| 结构参数（位宽、存储深度、时钟） | `config/nav_params.yaml` | 生成 `rtl/regs/nav_params.vh`、`fw/include/nav_params.h`、`model/nav_params.py`，以及正弦表 `rtl/plan/sin_lut.hex`、`fw/include/nav_sin_lut.h` |
| 外部接口语义（雷达、主机报文、底盘、IMU） | [icd.md](icd.md) + `config/regmap.yaml` | 其他文档只能引用，不能另立定义 |
| 实物参数（车体、传感器、安装、场地） | [platform.md](platform.md) | 固件 / 仿真里的对应常量（表的最后一列） |
| 引脚与电平 | `gowin/nav_board.cst` + [pinout.md](pinout.md) | 板级顶层 `gowin/nav_board_top.v` |
| RTL 文件清单 | `sim/tests.py`（`ALL_RTL`、`SOC`） | `sim/run.py`（xsim 回归）与 `gowin/syn/syn.py`（综合）共用 |
| 进度、已知问题、计划 | [roadmap.md](roadmap.md) | 其余规划文档（[integration_plan.md](integration_plan.md) 等）展开细节 |

生成文件（上表第三列）**不要手改**，改 yaml 后运行 `python tools/gen_config.py`。

## 2. 改了什么，必须跑什么

| 改动 | 必跑 | 说明 |
|---|---|---|
| `config/*.yaml` | `gen_config.py` → `sim/run.py` → `fw/test/build.py` | 生成文件一并提交 |
| `rtl/` | `sim/run.py`（15 项，约 10 分钟，需要 Vivado xsim）→ `gowin/syn/syn.py --board …` | 看布线后时序（当前余量只有约 0.1 MHz）与 BSRAM |
| `fw/` | `fw/test/build.py`（主机测试，会重编 `nav_fw.dll`）→ `fw/bsp/build_rv.py` | 涉及雷达启动 / 有图 / 方向表时再跑对应的 `sim/closed_loop/test_*.py` |
| `model/`、`sim/closed_loop/`、定位 / 规划参数 | `sim/closed_loop/batch.py --tag <名字> -j 10`（约 30 分钟） | 与上一次批量结果逐用例对比，结论记进 [algorithms.md](algorithms.md) |
| `gowin/*.cst`、板级顶层 | `syn.py --board` | 核对 `impl/pnr/nav_board_top.rpt.txt` 第 7 节（Pinout by Port Name）的管脚与 Bank |
| 文档 | 无构建 | 涉及接口 / 参数的，先改真值文件再改说明文档 |

并发约束（踩过的坑）：xsim 回归或 Gowin 构建运行中不要改 RTL；批量闭环运行中不要重编 `nav_fw.dll`、不要改 `model/*.py`。

## 3. 新 RTL 模块怎样接进来（底盘 / 视频 / 主机串口等）

整块 FPGA 只有**一个比特流**：导航、底盘 RL、视频必须在同一个 Gowin 工程里一起综合、布局布线、下载。
任何一组单独生成的比特流下载后都会把其他功能整个覆盖掉。所以新模块一律按下面的方式进本仓库：

1. **文件位置**：`rtl/<领域>/xxx.v`，测试平台放 `rtl/<领域>/tb/tb_xxx.sv`。建议的新领域目录：`rtl/chassis/`（底盘 / RL）、`rtl/video/`（摄像头、HDMI、叠加）、`rtl/host/`（RK3588 串口等）。
2. **编码约定**（与现有代码一致）：
   - Verilog-2001 可综合子集；同步、高有效复位 `rst`；时钟参数化 `parameter integer CLK_HZ`（板级 50 MHz，`NAV_USE_PLL` 时 100 MHz）。
   - 尽量与导航同一个 `clk` 域。确需别的时钟时，跨域只用 `rtl/common/nav_cdc.v`（电平 / 脉冲同步）与 `nav_async_fifo.v`，单拍脉冲不能直接跨域。
   - 不用锁存器；除 RAM 初值外不用 `initial`；RAM 用 `rtl/common/nav_ram_*.v`（高云综合对"读写同址"的推断有坑，见 roadmap 第 4 节第 1、2 条）。
   - 时间一律用导航的 32 位微秒计数 `time_us`（约 71.6 分钟回绕），差值用无符号模运算比较。
   - 厂家加密 IP（PLL、DDR3、1G Serial Ethernet 等）放 `gowin/ip/`，在 `gowin/ip/ip_files.txt` 登记，用顶层宏（`NAV_USE_*`）开关，没有 IP 时工程仍能构建。
3. **登记文件清单**：在 `sim/tests.py` 里加到对应列表（综合脚本直接用 `ALL_RTL`，不在清单里的文件不会被综合），并加一个 `TESTS["tb_xxx"]` 条目。
4. **逐位参考模型**：算法类模块（推理核、滤波、编码器测速）在 `model/` 写 Python 定点参考，`sim/gen/gen_xxx.py` 生成激励与期望值，testbench 逐位比较——这是本仓库所有 RTL 的验收方式。
5. **资源先报预算**：接入前在 [integration_plan.md](integration_plan.md) 第 4 节登记 BSRAM / DSP / 逻辑 / 引脚 / 时钟需求。当前板级 BSRAM 已用 324 / 340。
6. **跑 `python sim/run.py tb_xxx`**，再跑全量回归和 `syn.py --board`，确认导航原有 15 项回归不受影响、时序不出现负 slack。

## 4. 寄存器与地址空间

软核与调试串口桥共用一条 32 位数据、16 位字节地址的总线（[icd.md](icd.md) 第 6 节）。当前占用：

| 地址 | 内容 | 地址 | 内容 |
|---|---|---|---|
| `0x0000–0x0014` | SYS | `0x6000–0x601C` | IMU |
| `0x1000–0x104C` | NETL（雷达口） | `0x7000–0x7044` | MATCH |
| `0x1100–0x1138` | NETH（主机口） | `0x8000–0xBFFF` | COSTMAP 窗口 |
| `0x1200–0x1244` | MBX（主机邮箱） | `0xC000–0xCFFF` | MATCH_CAND / MATCH_RES 窗口 |
| `0x2000–0x20C0` | LIDAR | `0xD000–0xD17F` | LRAW_TX / LRAW_RX 窗口 |
| `0x3000–0x3054` | MAP | `0xD200–0xDFFF` | **空闲**（建议 `0xD800–0xDFFF` 留给底盘块 CHAS） |
| `0x4000–0x47FF` | TRAJ 及其窗口 | `0xE000–0xFFFF` | 软核调试口（只有串口桥能访问） |
| `0x5000–0x503C` | SAFE（底盘接口） | | |

- 导航内部的新寄存器优先放进已有块的空余偏移（例如底盘回传的扩展字段放 SAFE `0x5040` 之后）。
- `nav_subsystem` 对它自己范围内没定义的地址也会应答（读回 0），所以**导航之外的模块不能直接挂在同一条总线上**：
  要在整车顶层按地址把 `0xD800–0xDFFF` 分到底盘模块（仲裁器 `nav_bus_arb` 之后加一级地址译码），其余仍给 `nav_subsystem`。
- 新块同样写进 `config/regmap.yaml`，这样 `tools/nav_uart.py COM11 rd CHAS_xxx` 按名字就能读写，PC 调试不需要另写工具。

## 5. 固件约束（`fw/`）

- 算法只经 `fw/hal/hal.h` 访问硬件：上板实现 `fw/drv/hal_fpga.c`，主机测试实现 `fw/test/hal_sim.c`。新增硬件访问先加 HAL 函数，两边都实现。
- **RAM 很紧**：软核 RAM 128 KB，当前镜像 36012 字节 + `.bss` 85324 字节，链接脚本另外保留 8 KB 栈，剩余可用只有约 1.5 KB。
  新增数组前先看 `build_rv.py` 打印的数字（超限时链接直接报错）；要加大功能，先在 roadmap 第 6 节的 BSRAM 预算里找空间，或复用 `fw/nav/scratch.c` 的共用缓冲。
- 主循环不能阻塞：雷达 10 帧/秒时单次循环最长约 38 ms（速度看门狗 200 ms）。不要在固件里忙等外设；需要微秒级实时性的控制（底盘平衡、电机）一律放 RTL，不放软核。
- 32 位微秒时间比较一律 `(uint32_t)(now - t) > dt`，不要用有符号差（roadmap 第 4 节第 20 条）。
- 浮点：PicoRV32 没有浮点单元，固件全部定点（`fw/nav/fixmath.c`）。

## 6. 板级调试要点

- 下载只用 SRAM（`programmer_cli --run 2`），调试阶段不动 Flash；只换固件用 `nav_uart.py COM11 load`（约 10 秒）。
- 一个串口同时只能被一个程序打开：`nav_uart.py con`、`nav_host.py listen`、串口助手不要同时开。
- 暂停软核 `nav_uart.py COM11 wr 0xF000 1`、恢复 `reset`；暂停期间速度看门狗会让底盘输出减到 0。
- 每个上板验证过的比特流按日期另存到 `D:\tools\`（命名见 [hardware_guide.md](hardware_guide.md) 第 4.1 节）。

## 7. Git 与协作

- 仓库：`https://github.com/LRaina215/SOC_FPGA_NAV.git`，主分支 `main`。网络不稳时间隔 20–30 s 重试推送。
- 提交说明：第一行写结果（中文），下面列改动要点与验证结果（回归几项通过、板级 BSRAM / 时序、实测数据）。
- 不提交：构建输出（`fw/build/`、`gowin/syn/build*/`、`sim/build/`）、厂家 SDK 代码（没有许可证，参考副本放 `D:\tools\xtsdk_ref`）、授权文件。
- **接口变更规则**：改坐标、单位、位宽、周期、超时、急停语义之一，必须同时改 [icd.md](icd.md)、`config/regmap.yaml`、
  [chassis_integration.md](chassis_integration.md)（涉及底盘时）并重跑接口回归，提交说明里写明接口版本号（见 [integration_plan.md](integration_plan.md) 第 9 节）。
- Windows 上会看到 `CRLF will be replaced by LF` 警告，属换行格式提示，`.gitattributes` 已统一为 LF。

## 8. 遇到问题先查

- [roadmap.md](roadmap.md) 第 4 节（踩过的坑，21 条）与第 7 节（已知问题）；
- [hardware_guide.md](hardware_guide.md) 第 9 节（上板故障排查表）；
- [algorithms.md](algorithms.md)（每个算法参数为什么是现在的值）。
