# SFP1 千兆网络独立调试工程

本工程用于 ACX750-GW5AT138 的 SFP1 千兆网络调试，采用高云 1G Serial Ethernet MAC / PCS / SerDes，支持 ARP 与 UDP 回显，通过网线连接 PC 或 RK3588。

SFP1 位于 P6 / U11，连接 Q0 Lane2；Lane0 对应 PCIe。电口模块使用 SFP-GE-T，所测配置为 1000BASE-X 与自协商。独立顶层保持十路执行器输出为零，导航与原 4500 共同工程另由 `integration/g2/` 管理。

接线及实测记录见[通道映射与测试](../../docs/4_design/sfp_mapping_correction_20261024.md)。

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
| `hardware_probe.py` | 带当前 UserCode 和物理恒低检查的 SRAM 诊断下载/读回；不写 Flash |
| `test_echo.py` / `test_control.py` | 两项专项仿真；均不模拟真实模拟收发器 |
| `evidence_20261023.json` | 2026-10-23 时序、资源、输出与仿真记录 |

## 重建

在仓库根目录执行；输出目录必须不存在。Python 3.11+，Gowin V1.9.12.04，仿真使用 Vivado XSim。

```powershell
python integration/sfp/build.py --out F:/nav_sfp_rebuild --stage all
python integration/sfp/inspect_build.py F:/nav_sfp_rebuild
python integration/sfp/test_echo.py --out F:/nav_sfp_echo_test
python integration/sfp/test_control.py --out F:/nav_sfp_control_test
```

MAC 受保护文件默认从 `D:/tools/nav_sfp_ip_20261023` 读取。`GowinModGen.exe` 本身只重建包装层，**不能凭该命令重建完整 MAC**。新机器需按交接文档在高云 GUI 生成匹配 MAC，或指定保留的导出目录：`--mac-export <目录>`；该目录包含 `giga_serial_ethernet.v` 和 `giga_serial_ethernet_tmp.v`。不要拿官方参考工程里其他封装/器件版本的网表代替。

本轮已验证并实测的下载文件：`F:/nav_sfp_20261023/build_lane2_v3/impl/pnr/sfp_bringup.fs`，UserCode=91FB。旧build_v2是错误Lane0映射，保留追溯。

## 接线后使用

该镜像整体替换当前 FPGA 功能，仅用于网络诊断。本轮已完成SRAM下载并保留91FB恒低诊断；以后重新下载时按届时板上状态安排。

将电口模块插入 **SFP1**，网线连接 RK3588 或 PC 的千兆网口。对端选择实际有线接口，并设同网段静态地址，例如 `192.168.2.10/24`，避免地址冲突。FPGA 固定为 `192.168.2.100:9000`，MAC `02:4E:56:00:00:02`。当前实现支持 ARP 和 IPv4/UDP，不实现 ping。

下载本工程后，用 CH9102 串口读取状态；COM11 是本轮枚举值，插拔后应重新确认。FT232H 的串口不是这里的 UART。

```powershell
python integration/sfp/sfp_client.py status --port COM11
python integration/sfp/sfp_client.py status --port COM11 --mode basex-an
python integration/sfp/sfp_client.py echo --bind 192.168.2.10 --count 100 --size 64
```

UART 子命令需要 `pyserial`；UDP 子命令只用 Python 标准库。脚本默认只读识别`0x1300`的`SFP2`联合窗口或`0x0000`的`SFP1`独立窗口，只有签名匹配才允许写模式，并核对写后回读。也可用`--image joint`或`--image standalone`限定镜像，签名错误不写入。联合模式显示遥测计数，独立模式显示回显/网络计数；两者不混用。`echo`仅用于独立回显镜像，联合工程应使用NV协议。六项离线地址/签名/只读/写后回读合同通过，未访问板卡。

可用模式为 `basex-an`、`basex-forced`、`sgmii-an`；每次设置重置 IP 并等待 3 秒。默认忽略外部 LOS，必要时加 `--use-los`，其作用须与模块手册/实测对照。联合SGMII管理额外核对对端PHY链路位，不能仅以PCS同步宣称铜缆连接。

这些选项改变 **FPGA侧协议**，不会向模块写管理寄存器。当前这只实物已验证1000BASE-X/AN可工作，配置值2；这不代表所有同名商品都采用相同上电模式。

建议先低速单请求等待回包，再测 1、18、64、256、1472 字节，以及重连恢复。需要较长记录时，可运行 `--count 6000 --size 256 --interval 0.1`，保存输出并另存串口状态/抓包。该工程不承诺持续满千兆吞吐。
