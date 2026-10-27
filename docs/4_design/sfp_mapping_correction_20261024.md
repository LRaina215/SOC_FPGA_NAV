# SFP1 实物通道映射更正与后续测试

日期：2026-10-24。**重要更正：P6/U11 是用户所接的 SFP1，实际使用 Q0 Lane2；此前 Lane0 判断错误。**

**00:11 最终结果：正确 Lane2 镜像91FB已上板，RK eth2以1000Mb/s全双工建链；五种长度共100个UDP包全部原样返回。FPGA接收错误0、回显丢弃0、掉链0。** 当前仅为独立诊断短测通过，正式NV应用/完整G2仍按各自门禁验证。

## 原因和完整连接依据

此前核对了底板 SFP 页和核心板 SerDes 引脚，却漏掉底板连接器页的网络别名映射，把 SFP1 直接对应到了 MGT0。这是工程核对错误，不应归因于用户插错位置或模块不兼容。

完整连接链如下，已提取文字并渲染图纸核对：

| 连接点 | 实际信号 |
|---|---|
| 底板第 5 页 | P6/U11 为 SFP1；TX_DISABLE/RX_LOS 对应 SFP1 控制线 |
| 底板第 1 页 P4 | 15/17 脚 SFP1_RX_P/N 与 MGT_RX2_P/N 同网；16/18 脚 SFP1_TX_P/N 与 MGT_TX2_P/N 同网 |
| 核心板第 7 页 | P4 同位置接 MGT_RX2/TX2 |
| 核心板第 3 页 U1H | MGT2 对应 Q0 Lane2；RX P/N=B10/A10，TX P/N=B6/A6 |
| 参考时钟 | 仍为板载 125 MHz、MGT_CLK1、Q0 REFCLK1 F10/E10 |
| 其他通道 | Lane0/MGT0 接 PCIe；SFP2 接 MGT1/Q0 Lane1 |

图纸：`D:/Desktop/SOC FPGA/小梅哥ACG750型国产高云GW5AT-138K FPGA开发板资料/硬件图纸/ACX750-GW5A138-CB-250928.pdf`、同目录 `ACX750-GW5A138-CORE-25092801.pdf`。渲染核对图保存在 `F:/nav_sfp_20261023/mapping_audit/`。

## 已修正范围

- `integration/sfp/config/serdes_tmp.toml`：将已启用 Lane 的完整配置从 q0.ln0 移至 q0.ln2，关闭 ln0，并更新 RX Quad 时钟选择和 Lane 内引用；参考时钟、PLL、AFE、线速率不变。
- `serdes.mod` / `serdes.ipc`：选择 Q0 Lane2，并更新路由描述。
- `generate_ip.py`：检查仅 q0.ln2 启用，导出 Lane2 的 RX lock/align 诊断线。
- `sfp_bringup.sdc`：125 MHz MAC 时钟定位到 GTR 的 LANE2 RX/TX 引脚。
- `inspect_build.py`：新增实际时钟源必须为 Lane2 的检查，避免错误通道也仅凭时序通过而被接受。
- 本板仍使用 R17 TX_DISABLE、P15 RX_LOS 和原 UART/执行器引脚，逻辑功能及 UDP 端点不变。

受保护 MAC 来自同版本、同器件、同 PLL 的生成缓存，MAC 通过通用 SerDes 接口连接。物理 Lane 选择在 SerDes 包装与 CSR，本次已重新生成二者。

## 修正版独立构建

`F:/nav_sfp_20261023/build_lane2_v3/`，00:06 完成，UserCode=`91FB`。生成、综合、PNR 和实际报告检查通过，十路执行器 OBUF 均接 GND，PNR 命令包含新 CSR。

| 项目 | 实际结果 |
|---|---|
| 核心 / RX / TX 要求 | 50 / 125 / 125 MHz |
| 报告 Fmax | 111.964 / 171.965 / 175.461 MHz |
| 资源 | Logic 3568、寄存器 3114、BSRAM 5、GTR12_QUAD 1 |
| 最小报告裕量 | setup 2.185、hold 0.176、recovery 3.491、removal 0.326、pulse width 1.137 ns |
| 时序口径 | 已约束路径通过；独立工程整域异步例外的局限仍保留，不能代替联合 CDC/Gray 物理签收 |

00:07 已将修正版下载至SRAM，读回UserCode=91FB；Flash未写，十路执行器恒低。现场初始成功记录位于 `F:/nav_sfp_20261023/hardware_lane2_initial/session.json`。

## 实物建链与100包通信结果

RK远程主机为 `robomaster@100.113.173.115`（hky），通过用户本次提供的密码认证成功；密码未写入项目文件、远端配置或报告。其device-tree自报 `RK3588 OPi 5 Plus`。实际连接网卡是 **eth2 / r8168**，不是eth1。

修正Lane后，默认 **1000BASE-X、AN开启，配置值2** 即成功工作。初期Lane0上的三种模式失败不能用于判断该模块协议；本轮正确Lane上的默认配置已实证兼容用户这只模块。

| 端点/项目 | 最终实测 |
|---|---|
| FPGA状态 | status=23 (0x17)，partner=416 (0x01A0)，diagnostic=23 |
| FPGA链路 | QPLL锁定、RX锁定、对齐、PCS同步/链路、自协商完成均为1；LOS=0 |
| RK eth2 | carrier=1；1000Mb/s；Full；Auto-negotiation on；Link detected yes |
| ARP | 192.168.2.100 → 02:4e:56:00:00:02，邻居状态REACHABLE |
| 独立测试端点 | RK 192.168.2.10/24 → FPGA 192.168.2.100:9000 |

完整复测结果：

| UDP负载字节 | 发出/完整原样返回 | RTT最小～最大 (ms) |
|---|---|---|
| 1 | 20/20 | 0.132～0.727 |
| 18 | 20/20 | 0.134～0.372 |
| 64 | 20/20 | 0.151～0.457 |
| 256 | 20/20 | 0.274～0.585 |
| 1472 | 20/20 | 0.282～0.541 |

包内使用序号和确定性数据，客户端逐字节比较返回负载；每包等待回复再发送下一包，间隔10ms。这是低负载短测，未做持续满千兆吞吐、长时间耐久或拔插恢复验收。

首轮UDP因RK的旧 `Wired connection 1` 仍在等待DHCP，网络管理器重新激活连接并清掉手工临时IP，执行到途中遇到 `Network is unreachable`。当时已看到26个UDP进入FPGA并提交回显。失败原始记录与NetworkManager日志保留，未把中断归成FPGA丢包。

随后用 `nmcli connection add save no` 建立了 **不落盘** 的静态测试连接 `codex-sfp-test-20261024`，仅绑定eth2，设置 `192.168.2.10/24`、`ipv4.never-default=yes`、`connection.autoconnect=no`、`ipv6.method=disabled`。切换后100/100包全部通过。原Wired连接配置未改写，Wi-Fi/default route保留。

最终FPGA累计计数：RX UDP=126、echoes=126、RX errors=0、echo drops=0、link_losses=0；另有ARP请求2、TX frames=128。126包含首轮中断前的26和完整复测100，不能把累计值说成一次126包完整验收。

### 当前保留状态

- FPGA：SRAM为独立诊断91FB，SFP1/Lane2，BASE-X/AN，十路执行器恒低，Flash未写。
- RK：eth2为192.168.2.10/24，使用上述内存测试连接；Wi-Fi与Tailscale访问保留。该连接未持久化，重启或NetworkManager重启后应重新设置；没有替换正式启动配置。
- FPGA仍为诊断回显程序，不运行整车控制/NV任务。不要自动恢复旧有动力镜像。

### 证据与复测

仓库内入口：

- [完整100包结果与网络配置](evidence/sfp_20261023/rk_udp_lane2_pass.json)
- [最终FPGA寄存器和计数](evidence/sfp_20261023/fpga_lane2_after_udp.json)
- [修正后RK物理链路](evidence/sfp_20261023/rk_after_lane2.json)
- [修正后SRAM下载与初始状态](evidence/sfp_20261023/hardware_lane2_initial.json)
- [修正版独立物理时序/资源检查](evidence/sfp_20261023/inspection_lane2.json)
- [汇总与源码对应关系](../../integration/sfp/evidence_20261024.json)

RK临时连接失效时，可以在RK终端重建（若该内存连接仍存在，直接up即可）：

```bash
sudo nmcli connection add save no type ethernet ifname eth2 con-name codex-sfp-test-20261024 connection.autoconnect no ipv4.method manual ipv4.addresses 192.168.2.10/24 ipv4.never-default yes ipv6.method disabled
sudo nmcli connection up id codex-sfp-test-20261024 ifname eth2
```

把 `integration/sfp/sfp_client.py` 复制到RK后，运行：

```bash
python3 sfp_client.py echo --bind 192.168.2.10 --count 20 --size 64
```

PC读取板上状态使用本机带pyserial的解释器：

```powershell
& D:/Anaconda/python.exe integration/sfp/sfp_client.py status --port COM11 --image standalone
```

构建入口仍是 `python integration/sfp/build.py --out <全新目录> --stage all`；当前下载文件位于 `F:/nav_sfp_20261023/build_lane2_v3/impl/pnr/sfp_bringup.fs`。再次运行硬件读取器时使用新UserCode `--expect-usercode 91FB --read-only`。

## 旧证据和另一个对话的共同工程

旧 `build_v2` / UserCode 4785 使用 Lane0，其 RX 不锁、PCS 不通的结果**不能用于判定 SFP1 的模块、网线或协议兼容性**。旧记录保留用于追溯，不能当作正确 SFP1 工程的验收。

旧独立仿真通过只证明 MAC 字节接口以上的逻辑；旧 PNR 通过只证明当时网表已约束路径，不证明引脚映射正确。

另一个对话正在运行的G2冻结输入和进程未修改或停止。根目录 `integration/g2/build_g2.py` 的两条MAC时钟源已改成LANE2，与新生成配置一致；这只影响后续新构建，不改变任何已冻结候选。凡复用旧Lane0 IP的联合候选，下一轮需重新生成Lane2的CSR/包装，重新核对实际MAC时钟源、物理CDC节点及有效约束。不要覆盖旧产物或拿旧候选作SFP1硬件验收。联合资源/时序仍按G2自身门禁判断，本轮没有新增完整联合PNR。
