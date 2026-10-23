# SFP1 → 电口模块 → RK3588：实施与交接记录

记录日期：2026-10-23。本记录只负责 SFP 专项；整车状态仍以 [总计划](../1_plan/system_integration_plan_20261021.md) 为准。

## 1. 给接手对话的结论

已把高云官方 1G Serial Ethernet 参考方案适配到本板，形成可重建的独立调试工程，并完成实际综合、布局布线、下载文件生成及两项专项仿真。核心代码在 [integration/sfp](../../integration/sfp/README.md)。

当前结论是 **独立工程实现与本地验证完成，硬件链路尚未验收**。用户提供的模块目前没有插入。本次没有下载 FPGA、写 Flash、修改 RK3588 配置或运行电机/舵机。不能写成“RK3588 已连通”或“整车 SFP 已合入”。

只新增 `integration/sfp/` 和本文；没有修改现行 G2 构建、控制算法、权重、原网络 RTL 或其他对话的在改文件。检查时仓库 HEAD 已由其他工作推进到 `08aca0a`，这些新增文件当前尚未提交。不要根据本报告覆盖或回退其他对话的修改，也不要沿用本报告替代最新的板上镜像记录。

本轮产物固定在 `F:/nav_sfp_20261023/`，可与另一个对话正在进行的布线工作并存。这里没有继续运行的 SFP 构建进程。

## 2. 找到的可复用资料

1. [板厂 ACG750-GW5AT138 自助服务帖](https://fpga.cn/forum.php?mod=viewthread&tid=29660)：确认器件和配套资料入口；帖子说明该板是硬件评估板，教学资料有限。本轮在论坛搜索/浏览中未找到直接匹配本板 SFP1 电口模块的现成工程，不能据此断言全站不存在。
2. [高云 1G Serial Ethernet 官方 IP 页面](https://www.gowinsemi.com/en/support/ip_detail/127/)和[官方参考设计 ZIP](https://www.gowinsemi.com/upload/database_doc/2505/document/69558f8490732.zip)：实际使用的 MAC/PCS/SerDes 来源。原例程为另一器件/封装、Q1 Lane3、100 MHz 参考时钟和内部环回，不能原样下载到本板。
3. [IPUG1021-1.4E 用户指南](https://www.gowinsemi.com/upload/database_doc/2506/document/68313482bc8e1.pdf)：协议选择、MAC 接口、配置/状态向量与时钟要求。实际生成器以本机 V1.9.12.04 输出为准。

本地官方源码解压位置：`D:/tools/sfp_gowin_ref_20261023/source/Gowin_1G_Serial_Ethernet_RefDesign`。只复用了源码/生成配置，没有运行下载包内附带的可执行程序。

板级连接依据为厂商图纸 `ACX750-GW5A138-CB-250928.pdf` 第 5 页、`ACX750-GW5A138-CORE-25092801.pdf` 第 3 页，位于 `D:/Desktop/SOC FPGA/小梅哥ACG750型国产高云GW5AT-138K FPGA开发板资料/硬件图纸/`；此前已渲染核对。图纸连接关系不等于实板电气测量结果。

## 3. 实际采用的配置

| 项目 | 本轮配置/依据 |
|---|---|
| 器件 | GW5AT-LV138PG484AC1/I0，工程 device_version=C，与现有控制工程目标一致 |
| 插槽 | SFP1；Q0 Lane0 |
| 高速 RX | B8/A8，P/N |
| 高速 TX | B4/A4，P/N |
| 参考时钟 | 板载 U13 125 MHz；MGT_CLK1 F10/E10 → Q0 REFCLK1 |
| PLL / 线速率 | QPLL1 / 1.25 Gbit/s，8b/10b，MAC 侧 1 Gbit/s |
| 环回 | OFF，外部链路候选 |
| MAC | 启用；RX FCS 不转发，TX FCS 由 MAC 添加；全双工，jumbo 关闭 |
| PHY MIIM | 关闭；不把该端口误当本板 SFP 管理 I²C |
| 初始协议 | 1000BASE-X、主机侧 AN 开启；可改为 BASE-X 强制或 SGMII 系统/MAC 侧 AN |
| 链路定时参数 | BASE-X=488、SGMII=49，单位 32.768 μs；不是简单抄原例程的 BASE-X=305 |
| 外部控制 | TX_DISABLE=R17，LOS=P15；默认不以 LOS 阻止建链，保留原始诊断值及启用选项 |
| 核心 / UART | W19 50 MHz；D21 低有效复位；CH9102 RX=L21、TX=M21，921600、8N1 |

高速引脚由 `GTR12_QUAD` 专用硬核及 Q0 配置占用，不作为普通 GPIO 在顶层重复定义。已确认实际 PNR 命令含 `-csr .../serdes.csr`。只编译包装层而漏 CSR 不构成有效 SFP 工程。

用户商品图确认的是 SFP-GE-T、3.3 V、1000 Mbps、RJ45、100 m 等商品参数；**没有确认具体制造商、上电主机协议、AN 或 LOS 的实现**。可切换模式用于匹配模块，不会改写模块自身。早期“千兆电口 SFP 就必然 BASE-X 即插即用”的表述不成立。

图纸中的 SFP MOD_DEF1/2 管理线没有直接布到 FPGA；本轮未添加假想 I²C 引脚，也未要求焊接。若模块依赖管理写入才能进入所需模式，应先取得实际型号资料，再决定后续处理。

## 4. 软件与 RTL 边界

### 独立诊断顶层

`sfp_bringup_top.v` 接 `sfp_host_mac.v` 和 `sfp_udp_echo.v`。上电与每次协议配置后保持 IP 复位约 1.31 ms；链路状态稳定 256 个核心周期后释放各域的同步复位。

链路完成条件沿用官方例程：`status[0] && status[1] && (!AN || status[2])`。状态和对端能力经过两级同步供诊断，读取后应在稳定链路下复读；多位快照不承诺事务原子性。链路丢失会清空网络状态及网络计数；`link_losses` 只在全局复位时清零，模式切换引起的掉链也计入。

十路执行器输出全部固定低：M1/M2 的 DIR1、DIR2、PWM 和 SERVO1～4。本轮不仅检查了 RTL，也在实际布局布线 `.vo` 中确认十路 OBUF 输入均为 GND。该镜像不实例化整车控制功能。

### 网络端点

复用 `rtl/net/net_port.v`、`udp_rx.v`、`udp_tx.v` 及已有 FIFO/RAM/CDC 模块。FPGA MAC=`02:4E:56:00:00:02`，IP=`192.168.2.100`，UDP 端口=`9000`。有 ARP 回复，无 DHCP、ICMP ping、TCP 或 NV 命令解析。

回显只接受 1～1472 字节负载；一次保留一个正在提交给发送器的负载缓冲。忙时新报文丢弃并计数，避免覆盖正在读出的 RAM。UDP 回复地址从有效入站报文学习，因此客户端应主动发包。TX 使用已有整帧缓存，避免 50 MHz 核心向 125 MHz MAC 直接逐拍供数造成欠载。

50 MHz 核心处理能力不等于持续满千兆吞吐。既有 UDP 接收器验证 IPv4 头和 MAC 错误标志，本专项没有新增 UDP 负载校验和验证；本次仿真也不声称覆盖厂商 MAC 的实际 CRC 算法。

### UART 寄存器

复用 `uart_bus_bridge`：读 `R + 16bit地址`，回复 `D + 32bit数据`；写 `W + 16bit地址 + 32bit数据`，回复 `K`，多字节大端。

| 地址 | 含义 |
|---|---|
| 0x0000 | 标识 0x53465031，即 SFP1；客户端写配置前必须匹配 |
| 0x0004 | 配置：bit0 SGMII、bit1 AN、bit2 使用 LOS；写入重新复位 IP |
| 0x0008 / 0x000C | IP 原始状态 11bit / 对端能力 16bit |
| 0x0010 | bit0 QPLL锁定、1 RX锁定、2 对齐、3 原始LOS、4 link_ready、5 IP复位 |
| 0x0014 | 链路下降次数 |
| 0x0020 / 24 / 28 / 2C | 接收帧 / UDP / 接收错误 / ARP请求计数 |
| 0x0030 / 34 / 38 | 发送帧 / 提交回显 / 回显丢弃计数 |
| 0x0040 / 44 | IPv4 / UDP端口 |

除 0x0004 外均只读。此映射属于独立诊断镜像，不应冒充导航子系统已有寄存器映射。

## 5. IP 重建的完整依赖

配置冻结在 `integration/sfp/config/`。当前 GUI 完整导出保留在 `F:/nav_sfp_20261023/gui138c/`，匹配本器件的受保护 MAC 和模板缓存于 `D:/tools/nav_sfp_ip_20261023/`。

`generate_ip.py` 使用同版本 `serdes_toml_to_csr_138k.exe` 生成 CSR，使用 `GowinModGen.exe` 生成 SerDes 包装，再复制匹配的 MAC 导出文件。包装层只增加三个已有状态线的诊断输出，没有改厂商 PMA/PCS 行为。厂商包装中高位 TX 总线未驱动警告来自 10bit 模式只连接低 10 位；已检查原始输出，未擅自修改厂商其他端口。

新机器如没有 MAC 缓存，需要在 V1.9.12.04 GUI 中先建立目标器件为 C/PG484 的项目，再加载配置或按第 3 节生成：Q0 Lane0、Q0 REFCLK1 125 MHz、QPLL1、1.25G、MAC=true、PHY MIIM=false、Loopback=OFF。保存完整生成结果后，将包含 MAC `.v` 和 `_tmp.v` 的目录传给 `--mac-export`。生成器版本升级后应重新导出配置，不能猜测 TOML 内部枚举。

旧参考 TOML 在新生成器中缺 `bias_tc` 等字段，本轮最初两次尝试因此或因旧时钟选择编码失败；这些失败目录保留用于溯源，**最终使用的是当前 GUI 完整导出配置**。首次独立综合还遇到 RAM 初始化循环超过默认 2000 次，已按现有项目做法加入 `-looplimit 100000`，没有缩减 RAM。

## 6. 本轮验证和证据

可直接读取仓库内的 [机器结果汇总](../../integration/sfp/evidence_20261023.json)。已直接比较当前 12 个 RTL 源文件与成功构建目录中的冻结副本，内容一致；没有使用哈希。

### 实际构建

成功目录：`F:/nav_sfp_20261023/build_v2/`，2026-10-23 13:40 完成。该目录含冻结源码、IP、CSR、run.tcl、build_result.json、gw_sh.log 和完整物理报告。

| 项目 | 实际结果 |
|---|---|
| 工具 | Gowin V1.9.12.04；目标 GW5AT-138C / PG484 |
| 流程 | 综合、布局布线、下载文件生成成功；进程返回 0 |
| 核心时钟 | 要求 50 MHz；报告 Fmax 107.447 MHz |
| RX MAC 时钟 | 要求 125 MHz；报告 Fmax 163.865 MHz |
| TX MAC 时钟 | 要求 125 MHz；报告 Fmax 170.981 MHz |
| Setup / Hold | 三个域报告负裕量端点均为 0 |
| 最小报告裕量 | setup 1.897 ns；hold 0.176 ns；recovery 2.996 ns；removal 0.363 ns；pulse width 1.127 ns |
| 物理资源 | Logic 3570、寄存器 3114、BSRAM 5、GTR12_QUAD 1（只启用 Lane0） |
| 输出核对 | 十路执行器 OBUF 接 GND；PNR 命令携带 CSR |

下载文件：`F:/nav_sfp_20261023/build_v2/impl/pnr/sfp_bringup.fs`。
证据摘要：`F:/nav_sfp_20261023/build_v2/inspection.json`。
原始时序：`.../impl/pnr/sfp_bringup_tr_content.html`。
资源与引脚：`.../impl/pnr/sfp_bringup.rpt.txt`。
物理网表：`.../impl/pnr/sfp_bringup.vo`。

**时序结论的边界：** SDC 对独立核心/RX/TX 时钟采用官方例程的异步域例外；网络使用异步 FIFO，状态使用同步器，配置切换带复位。上述通过属于已约束路径的 STA，未专项完成 Gray 指针最大物理时延/偏斜检查，也不等于厂商受保护 IP 的完整 CDC 签收。合入 G2 时不能把这些宽泛例外直接扩展到全工程并声称完整时序通过，应沿用 G2 的精确 CDC 审核规则。

### 专项仿真

1. `F:/nav_sfp_20261023/sim_echo_v1/result.json`：通过。50 MHz 核心、125 MHz RX/TX、有相位差和随机 TX 反压；检查 ARP 回复和五个实际回显包，负载长度 1、18、1472、255、21 B，逐字节比较目的地址/端口/负载与 IPv4 头校验和。另覆盖忙时丢弃、1473 B 超限、MAC 错误标志、截断、错误目标端口，以及随后恢复正常收发。
2. `F:/nav_sfp_20261023/sim_control_v1/result.json`：通过。通过真实 UART 字节收发访问新顶层，检查标识、默认 BASE-X/AN、切换 SGMII/AN、强制 BASE-X、重配复位、LOS/链路恢复、掉链计数与十路恒低。

第一项处于 MAC 字节接口边界；第二项使用行为 MAC 桩。两者都不模拟铜缆或模拟 SerDes，不证明实物模块兼容性。本轮没有使用 SHA 哈希校验，也没有扩大到整车回归。

## 7. 接手后的执行顺序

1. 先读取最新总计划和板上镜像记录，再安排此独立诊断镜像的 SRAM 使用。不要依据本记录恢复旧控制位流；当前硬件状态由另一专项持续更新。
2. 插入 SFP1 电口模块，用网线接 PC/RK3588 的实际千兆网卡，设置无冲突的 `192.168.2.10/24` 等同网段地址。RK3588 具体板型、Linux 网卡名尚未确认，不预填 `eth0`。
3. 用 `sfp_client.py status` 检查 SFP1 标识、QPLL/RX锁定、对齐、PCS/AN 状态。先默认 BASE-X/AN；若不通，结合模块资料和这些状态分别验证 BASE-X 强制、SGMII/AN。不能把“切了三次模式”当作模块已识别。
4. 对端用 `sfp_client.py echo` 主动发包，先少量小包，再做 1～1472 B、持续编号回显和拔插恢复；记录串口快照、对端千兆/全双工状态及抓包。该阶段不使用 ping 作为唯一成败依据。
5. 独立链路通过后，在届时冻结的共同工程中接入 `sfp_host_mac.v`。替换 `gowin/nav_board_top.v` 中旧 `Gowin_1GSERETH_Host` 占位实例的真实接口；复用 `nav_subsystem` 的 `h_*` 和现有主机 net_port，不把诊断回显端点重复实例化到正式系统。共同工程还须加入匹配 IP、CSR、引脚与时钟/复位约束，并把诊断状态按正式映射接入。
6. 基于同一冻结基线做实际联合 PNR 和资源差分。独立工程 5 块 BSRAM 不等于共同工程净增 5 块；正式网络、既有缓冲优化、物理拆分和调试逻辑都会影响增量。若同片已有其他 Q0 SerDes 功能，需统一该 Quad 的生成配置，避免重复实例化占用同一硬核。

第 5～6 步是待办，未在本轮执行。SFP 专项不改变 G2 的接受标准，也不证明控制/导航共同工程已经布通。

## 8. 可直接复用的命令

在仓库根目录，使用全新输出目录：

```powershell
python integration/sfp/build.py --out F:/nav_sfp_next --stage all
python integration/sfp/inspect_build.py F:/nav_sfp_next
python integration/sfp/test_echo.py --out F:/nav_sfp_echo_next
python integration/sfp/test_control.py --out F:/nav_sfp_control_next
```

下载独立镜像并接好模块后，在有对应硬件连接的机器执行：

```powershell
python integration/sfp/sfp_client.py status --port COM11
python integration/sfp/sfp_client.py status --port COM11 --mode basex-an
python integration/sfp/sfp_client.py echo --bind 192.168.2.10 --count 100 --size 64
```

`COM11` 是此前 CH9102 的枚举值，须以实际枚举为准；UART 子命令需要 pyserial，UDP 子命令只需 Python 标准库。`build.py` 默认上限 600 秒、180 秒无日志进展停止其自有进程，具体可用 `--max-seconds` 调整；不会停止其他会话的构建。
