# SFP1 千兆网络独立调试工程

日期：2026-10-23。**交接先读 [SFP 实施与交接记录](../../docs/4_design/sfp_bringup_20261023.md)**。

面向 ACG750 / ACX750-GW5AT138、GW5AT-LV138PG484AC1/I0，使用板上 SFP1 插槽接用户已购的 SFP-GE-T 电口模块。FPGA 内部复用高云 1G Serial Ethernet MAC/PCS/SerDes，外部网线接 PC 或 RK3588。

已完成：IP 生成、独立工程综合/布局布线、有效约束下的时序检查、MAC 字节接口 ARP/UDP 回显仿真、UART 配置/复位仿真。**尚未下载本工程、尚未接模块实测。** 16:10已接入真实G2候选，联合SYN通过、PNR仍未通过，当前状态见总计划。独立工程跨域时序例外沿用官方参考设计，Gray 总线物理时延/偏斜尚未专项签收；联合工程采用逐路径审计，不照搬独立整域例外。

## 入口

| 文件 | 用途 |
|---|---|
| `sfp_bringup_top.v` | 独立顶层；50 MHz、921600 UART、配置寄存器、链路复位、十路执行器恒低 |
| `sfp_host_mac.v` | 高云 IP 的实际端口适配；BASE-X/SGMII、AN、LOS 可配置 |
| `sfp_udp_echo.v` | 复用现有 net_port，实现 1～1472 B UDP 回显和 ARP 回复 |
| `config/` | 当前版本 GUI 导出的 IPC、TOML、MOD 配置 |
| `generate_ip.py` | 重建 SerDes 包装与 CSR，复制同器件 MAC 导出文件 |
| `build.py` | 独立目录冻结源码并构建，不访问板子 |
| `inspect_build.py` | 检查实际时序报告、CSR 构建输入、十路输出接地及下载文件 |
| `sfp_client.py` | UART 状态/模式配置；PC/RK3588 上的 UDP 回显客户端 |
| `test_echo.py` / `test_control.py` | 两项专项仿真；均不模拟真实模拟收发器 |
| `evidence_20261023.json` | 本轮时序、资源、输出核对和两项仿真的机器结果汇总 |

## 重建

在仓库根目录执行；输出目录必须不存在。Python 3.11+，Gowin V1.9.12.04，仿真使用 Vivado XSim。

```powershell
python integration/sfp/build.py --out F:/nav_sfp_rebuild --stage all
python integration/sfp/inspect_build.py F:/nav_sfp_rebuild
python integration/sfp/test_echo.py --out F:/nav_sfp_echo_test
python integration/sfp/test_control.py --out F:/nav_sfp_control_test
```

MAC 受保护文件默认从 `D:/tools/nav_sfp_ip_20261023` 读取。`GowinModGen.exe` 本身只重建包装层，**不能凭该命令重建完整 MAC**。新机器需按交接文档在高云 GUI 生成匹配 MAC，或指定保留的导出目录：`--mac-export <目录>`；该目录包含 `giga_serial_ethernet.v` 和 `giga_serial_ethernet_tmp.v`。不要拿官方参考工程里其他封装/器件版本的网表代替。

本轮已验证下载文件：`F:/nav_sfp_20261023/build_v2/impl/pnr/sfp_bringup.fs`。

## 接线后使用

先按届时板上工作状态安排独立镜像的 SRAM 下载。该镜像整体替换当前 FPGA 功能，仅用于网络诊断；本次没有执行下载。

将电口模块插入 **SFP1**，网线连接 RK3588 或 PC 的千兆网口。对端选择实际有线接口，并设同网段静态地址，例如 `192.168.2.10/24`，避免地址冲突。FPGA 固定为 `192.168.2.100:9000`，MAC `02:4E:56:00:00:02`。当前实现支持 ARP 和 IPv4/UDP，不实现 ping。

下载本工程后，用 CH9102 串口读取状态；COM11 是本轮枚举值，插拔后应重新确认。FT232H 的串口不是这里的 UART。

```powershell
python integration/sfp/sfp_client.py status --port COM11
python integration/sfp/sfp_client.py status --port COM11 --mode basex-an
python integration/sfp/sfp_client.py echo --bind 192.168.2.10 --count 100 --size 64
```

UART 子命令需要 `pyserial`；UDP 子命令只用 Python 标准库。脚本默认只读识别`0x1300`的`SFP2`联合窗口或`0x0000`的`SFP1`独立窗口，只有签名匹配才允许写模式，并核对写后回读。也可用`--image joint`或`--image standalone`限定镜像，签名错误不写入。联合模式显示遥测计数，独立模式显示回显/网络计数；两者不混用。`echo`仅用于独立回显镜像，联合工程应使用NV协议。六项离线地址/签名/只读/写后回读合同通过，未访问板卡。

可用模式为 `basex-an`、`basex-forced`、`sgmii-an`；每次设置重置 IP 并等待 3 秒。默认忽略外部 LOS，必要时加 `--use-los`，其作用须与模块手册/实测对照。联合SGMII管理额外核对对端PHY链路位，不能仅以PCS同步宣称铜缆连接。

这些选项改变 **FPGA 侧协议**，不会向模块写管理寄存器。哪个模式匹配已购模块仍待实测；不能仅凭商品名称确定。

建议先低速单请求等待回包，再测 1、18、64、256、1472 字节，以及重连恢复。需要较长记录时，可运行 `--count 6000 --size 256 --interval 0.1`，保存输出并另存串口状态/抓包。该工程不承诺持续满千兆吞吐。
