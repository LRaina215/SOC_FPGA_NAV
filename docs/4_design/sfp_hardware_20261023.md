# SFP1 实板简单测试记录

更新时间：2026-10-24 00:11 +08:00。首轮 2026-10-23 23:45，随后完成通道映射纠错和实物通信短测。

**最新结论：已发现并更正此前 SFP1 通道映射错误（应为 Q0 Lane2，旧配置为 Lane0/PCIe）。修正版91FB与RK3588已建立千兆全双工，100/100个UDP原样回显通过，错误/回显丢弃计数0。** 最新交接和复测方法见 [映射更正与成功实测](sfp_mapping_correction_20261024.md)。下文保留旧Lane0镜像的失败过程，其“不通”不能归因于SFP1模块/网线。

## 已执行

1. 枚举到 CH9102 `COM11`、FT232H `COM14`、Gowin JTAG location `579`。扫描 ID `0011081B`；该共享 ID 不能独立确认 B/C 版本。
2. 读取原 SRAM UserCode 为 `0x4B28`，与最后的恒零 B1 诊断记录相符；UserCode 不是完整镜像身份校验。
3. 重新核对已有独立镜像的实际物理时序、十路执行器 OBUF 接地、CSR 构建输入和下载文件；没有进行哈希校验或重跑联合构建。
4. 经 JTAG 操作 2，仅将 `F:/nav_sfp_20261023/build_v2/impl/pnr/sfp_bringup.fs` 下载至 SRAM。成功读回 UserCode `0x4785`。没有写 Flash、没有恢复有动力控制镜像。
5. 从 `COM11`、921600 baud 读取到 `SFP1` 标识 `0x53465031`。默认配置为 BASE-X/AN，连续取得三个状态快照。

## 实际读取结果

三个快照分别在 23:45:20、23:45:22、23:45:24 取得，结果一致：

| 项目 | 实际值 |
|---|---|
| config | 2：1000BASE-X，AN 开启，忽略 LOS 门控 |
| status / partner | 0 / 0 |
| diagnostic | 9，即 QPLL lock=1，RX lock=0，对齐=0，原始 LOS=1，link_ready=0，IP reset=0 |
| RX/UDP/ARP/TX/echo 计数 | 均为 0 |
| FPGA 固定端点 | 192.168.2.100:9000 |

确认的是 FPGA 参考时钟对应的 PLL 可以锁定、诊断程序运行和 UART 通信正常。**不能据此认定模块损坏、协议选错，或 RK3588 已连通。** 接收未锁定时，先核对实际插槽、模块输出和对端物理链路；仅切换 AN 不能替代这些检查。

## RK3588 访问与待补信息

从现有 Web 网关配置确认 RK 后端使用 `100.113.173.115`。尝试现有 SSH 配置 `robomaster@100.113.173.115`，服务器拒绝当前公钥/登录方式，返回 `Permission denied (publickey,password)`；未猜测其他账号或读取密码。尚未取得 RK 的接口名、carrier、速率、IP 和 UDP 运行结果。

已向用户请求：

- 当前插的是 SFP1 还是 SFP2（本次镜像仅启用 SFP1 / Q0 Lane0）。
- RK3588 正确的现成 SSH 访问方式，或由用户在 RK 终端执行一次检查。

**用户后续已确认模块在 SFP1，且平时 SSH 需要输入密码。** 复核表明 Tailscale 可达、TCP/22 建立、服务器正常提供 publickey/password 认证；本机提供的 id_rsa 公钥被拒绝，SSH agent 不可用。本次使用 BatchMode 未输入密码，因此认证失败，不能据此判断用户给错地址或账号。已请求此次测试所需的密码/现成免密方式，尚未尝试密码认证；没有保存或在文档中记录密码。

当前记录不包含完整链路故障归因，也不把诊断脚本的 `completed=true` 当作网络测试通过。

## 后续模式对比

确认 SFP1 后，23:51 再次只读取得三组相同状态。随后通过已核对 SFP1 标识的寄存器接口，分别设置一次 BASE-X 强制、SGMII/AN，再恢复默认 BASE-X/AN。每次带 IP 复位和等待，再读取三个快照；没有重新下载镜像。

| 模式 | config | status | diagnostic | 结果 |
|---|---|---|---|---|
| BASE-X 强制 | 0 | 0 | 9 | PLL锁定，RX未锁定，未对齐，LOS=1，未建链 |
| SGMII / AN | 3 | 0 | 9 | 相同，未取得有效对端能力 |
| 恢复 BASE-X / AN | 2 | 0 | 9 | 相同，当前保留此配置 |

机器摘要见 [模式对比结果](evidence/sfp_20261023/mode_comparison.json)。完整快照在 `F:/nav_sfp_20261023/hardware_mode_basex-forced/`、`hardware_mode_sgmii-an/`、`hardware_mode_basex-an/`。

该对比没有证明模块采用哪种协议；当前在接收锁定以前就未满足建链条件。下一步先从 RK 读取有线接口是否 UP、carrier、速率/双工与协商状态，结合模块与板端信号继续定位，不盲目扩大协议或布线试验。

## 23:45 期间板上镜像（已替换）

23:45 曾保留旧 Lane0 诊断镜像 UserCode=4785，替换了原B1的4B28。**00:07 已再替换为正确 Lane2 的91FB，十路执行器恒低，Flash未改。** 其他对话按最新镜像记录接手，不要自动恢复有动力控制。

本次没有改动或停止另一个对话的 G2 布线进程。独立 SFP1 镜像与其包含真实 SFP 的完整联合候选不是同一构建。

完整原始日志位于 `F:/nav_sfp_20261023/hardware_2347_initial/`：

- `session.json`：命令、读回、三个快照、未写 Flash/未发运动命令记录。
- `program_sram.log`：SRAM 下载工具原始结果。
- `usercode_before.log`、`usercode_after.log`：下载前后读回。

可重用入口为 `integration/sfp/hardware_probe.py`。再次读取可以使用 `--read-only --expect-usercode 4785`；该选项跳过下载，但仍核对当前 UserCode 和 SFP1 UART 标识。模式设置仅在标识匹配后进行，当前首轮没有改写默认协议配置。
