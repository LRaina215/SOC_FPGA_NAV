# G2 双 UART 合同与 RK 联调步骤（2026-10-25）

本次审阅导航、控制与 RK 桥接代码，并于 19:29 经用户已保存的私有连接配置成功登录 RK，读取 USB/sysfs/服务状态/相关内核日志。定位后按明确授权实施了单设备 BRLTTY 排除与串口重绑；没有打开串口、发送任务、下载镜像或运行综合/仿真，亦未停止 ModemManager、Web 或相机。本次未改变无 SFP 冻结输入或启动重复构建。机器记录见 [dual_uart_contract_20261025.json](evidence/g2_20261023/dual_uart_contract_20261025.json)。

用户确认已有 P11-40 USB-TTL 接线，只把其 USB 端接到 RK，FPGA 侧保持不动。双串口方向可复用现有实现：CH9102 负责导航任务和低频状态，P11-40 负责完整控制 V2 遥测。当前新镜像与 RK 双口的实板接受仍未完成。

## 1. 已确认的接口

| 口 | 当前代码合同 | 边界 |
|---|---|---|
| CH9102；FPGA L21/M21；921600、8N1 | W/R 寄存器桥；MBX.INJ_* 注入 NV 任务；MBX.TX_SEQ/TX_LEN/TXB_* 回读应答 | 独占一个串口；不能同时开 nav_uart、nav_host 和 RK bridge |
| 已接 USB-TTL；P11-40/A16；230400、8N1 | 原完整 V2：126 字节/帧、100Hz、CRC16、序号、设备时间 | RK 只读；不改 FPGA 接线，不启用独立速度 RX 旁路 |

G2 冻结控制源为 `NAV_INPUT_ENABLE=1`、`UART_SPEED_ENABLE=0`，顶层 `UART_RXD=1`。速度、停止优先级继续由 NAV→SAFE→命令租约→控制处理。原独立速度 RX 具有合法包后保持最后目标的行为，其 10ms 超时只丢弃半包；不能把它当作已经实现的失联停车通道。

V2 原始字段包括六轴 BMI、融合角速度/重力、采用的前进/转向/高度命令、策略轮速、实测轮速、PWM、舵机脉宽、推理计数/周期、编码器增量、PID 参考/误差与控制序号。六轴、融合与执行器快照不是全部同一采样时刻；遥测序号不是运动命令 ACK，高度字段不是实测车高。

## 2. 导航 R0 的现状及可靠性

`tools/nav_host.py` 与 RK 的 `yuntuo/fpga_link/links.py` 已实现 GOAL、WAYPOINTS、MODE、SET_POSE、TARGET、MAP_BEGIN/DATA/END/CLEAR；固件发送 STATUS 与 MAP_ACK。PATH 只有组包函数，当前应用没有调用；实时地图上行仅有原始 MMIO 拉取入口，没有一致快照/分片上行的已完成合同。不能把“可构造字段”当成已经执行或上报的功能：GOAL 末端 yaw 尚未启用，TARGET 的 z/conf 不参与导航目标更新。

- W/R：写请求 `W+地址16+数据32`，回复 `K`；读请求 `R+地址16`，回复 `D+数据32`，地址/数据为大端。桥无 CRC/命令序号；`K` 只表示 MMIO 总线应答。
- NV v1：`NV+版本+类型` 和小端载荷，无通用命令序号、会话、执行 ACK 或 TTL。
- `MBX_TX_SEQ` 在 `TX_GO && !busy` 时递增，证明邮箱接受提交，不证明网络发完、RK 已收或任务执行。UART 通过前后读序号检查复制期间的已提交覆盖，只有最新一包，不是可靠消息队列。原 shadow 在新包写入时即更新，序号在 GO 才变；没有独立锁定快照的证明。
- 地图有 `MAP_ACK.next_offset/state`，先验包体在 MAP_END 做 CRC32。当前 `hal_host_send` 遇 busy 返回 false，应用的 ACK/STATUS 调用未重试；新报文也可覆盖旧应答。UART “reliable”标志不是新镜像的端到端无丢失证明。
- TARGET 约 1.5s 丢失后回 SEARCH，不是 ESTOP；GOAL 等自主任务也不能假定在 RK 断开后自动停止。RK bridge 的超时/停止状态处理不能替代 FPGA 内部的主机断连策略。

## 3. 移除 SFP 必须保留的结构

`host_control_tx` 是原 UART 字节的只读副本，使用独立两个 128 字节槽，把一帧 V2 加 NV 0x83 头形成 130 字节 SFP 包。它只读邮箱，邮箱优先，不写 `TXB_Q` 或 `TX_SEQ`。因此 100Hz tap 不会覆盖 UART NV 的应答 shadow。

可断开 NAV 侧 tap、移除无用 SFP 封装；原 P11-40 的 126 字节 V2 输出必须保留。仅把 tap 清零不会自动删除 MAC 或解决邮箱握手。

此前离线裁剪版把 `u_txs/TXB_Q` 固定为零。这在 SFP 为正式回传时是离线读口裁剪；现在 R0 以它为正式回传，必须恢复或提供等价、经验证的出口。无需为此恢复整套 SFP。

`HOST_EXTENSION_ENABLE=1` 时邮箱用 `h_core_rst`，不是普通 `rst`；无 SFP 版本必须将其接到仍有效的协议复位，不能因删除 SFP 的锁定/复位条件而永久复位。`mb_busy/GO` 还须有明确完成语义，不能等待不存在的 MAC 消费。保留 MBX 注入队列、完整回读内容、长度、序号和 CPU 接口，分别验证复位、连续提交和回读覆盖。

## 4. 带宽与已有实板证据

| 项目 | 已知结果或计算 | 不能推出 |
|---|---|---|
| CH9102 原始线路 | 921600/10=92160 字节/秒 | 当前应用吞吐就是 92KB/s |
| R0 回读 | 每 4 个载荷字节需 16 个线路字节（写地址+读数据），另有序号/长度轮询与 USB 往返 | 适合完整 V2 100Hz 或实时整图高频上传 |
| 历史 R0 | 2026-10-01 PC 实板：STATUS 约 5Hz，TARGET 10Hz；38964 字节先验包 1.7s 下发并显示已加载；文档记录 STATUS 25–50ms/包 | 本次新 G2、实际 RK 或断连停止已通过 |
| P11-40 V2 | 126×100=12600 字节/秒，按 8N1 为 126000bit/s，占 230400 的约 54.7% | RK 当前收到了完整新鲜数据、CRC/序号无损 |

历史来源为 `docs/5_teams/rk3588_design.md` §3.2 与 `hardware_guide.md`；不是本次重测。当前可采用 TARGET 10–15Hz、STATUS 5Hz，完整 V2 单走第二口。路径/地图大包应低优先，不能阻塞停止与状态。

## 5. RK 入口和稳定设备识别

本地 `D:/Desktop/SOC FPGA/QHXD_Web` 已有两个实现：

- `yuntuo/fpga_link/links.py::BoardUartLink`，接受 `uart:/dev/...@921600`，核对 NAV0 ID、流水写注入、TXB 回读；只允许 I/O 线程独占。
- `yuntuo/bridge/inputs.py::TelemetryReader`，接受 `serial:/dev/...@230400`，只读 V1/V2，CRC 检查并在设备异常后重连。`serial.read(512)` 会批量接收；接收批次时间不能冒充每个采样的设备时间。
- `scripts/udev/99-yuntuo.rules` 已约定 `/dev/yuntuo-fpga`（1a86:55d4）和 `/dev/yuntuo-ctl`（1a86:7523）。这是按 VID/PID 的规则，不是当前实物身份验证。若两个适配器同 VID/PID，必须用实际唯一序列号；无唯一序列号则用固定 USB 物理路径 `ID_PATH` 区分，不能依赖 ttyUSB0/ttyACM0 枚举次序。

既有 RK SSH 入口为 `robomaster@100.113.173.115`，本机 SSH 配置含该 Host。首个 BatchMode 请求因没有可用非交互认证被拒；用户随后授权持久复用连接，19:29 已从 `C:/Users/Li/.codex/private/rk3588_connection.md` 仅在内存使用认证信息成功登录。没有输出或提交口令。

**19:28–19:29 实际枚举与无串口原因：**

- 可见 `1a86:7523` CH340（Bus002 Device004，物理拓扑 `fc880000.usb/usb2/2-1/2-1.2`），没有 CH9102，也没有 ttyUSB/ttyACM/ttyCH 节点。CH9102 当前未枚举，不能判断是尚未接到 RK、线缆还是其它现场原因。
- CH340 的 USB `authorized=1`、configuration=1；接口 `2-1.2:1.0` 当前无 driver。`ch341` 模块已加载，故不是没安装驱动。
- kernel journal 记录 19:11:10 `ch341-uart converter now attached to ttyUSB0`，19:11:11 紧接 `interface 0 claimed by ch341 while 'brltty' sets config #1`，随后 ttyUSB0/converter 断开。dmesg 有相同顺序；两者墙钟转换不同，不能据其秒值推算独立两次事件。
- `/lib/udev/rules.d/85-brltty.rules:293` 把 `PRODUCT=1a86/7523/*` 作为 Baum 盲文设备匹配，设置 `BRLTTY_BRAILLE_DRIVER=bm` 并启动 BRLTTY；`udevadm` 的当前属性也有该值和 `SYSTEMD_WANTS=brltty-udev.service`。`brltty-udev.service` 实际 active/running，主 PID 322390；软件包为 `brltty 6.4-4ubuntu3`。
- ModemManager 和 Web bridge 虽在运行，此次解绑的日志证据直接指向 BRLTTY，不能把其它服务当根因而逐个停掉。
- 此 CH340 没有唯一 USB serial；`ID_SERIAL=1a86_USB_Serial` 是通用描述，不能当唯一身份。恢复绑定后应取得实际 tty 的 `ID_PATH/by-path` 并绑定原固定 USB 口；当前 `99-yuntuo.rules` 已有对应 `/dev/yuntuo-ctl` 规则，但没有 tty 就不会生成该链接。

**19:32 已授权的精确恢复：**备份实际厂商规则到 `/var/backups/yuntuo-ch340-brltty-20261025T193229/85-brltty.rules.vendor.before`，创建同名本地覆盖 `/etc/udev/rules.d/85-brltty.rules`，只在原 `PRODUCT=1a86/7523/*` 那一条加当前物理 DEVPATH 的排除条件。原厂 `/usr/lib/udev/rules.d/85-brltty.rules` 未改，所有其它规则保持原文。此前本地覆盖不存在，恢复时删除本次覆盖即可回到厂商版本。

```text
原：ENV{PRODUCT}=="1a86/7523/*", ENV{BRLTTY_BRAILLE_DRIVER}="bm", GOTO="brltty_usb_run"
新：ENV{PRODUCT}=="1a86/7523/*", DEVPATH!="/devices/platform/fc880000.usb/usb2/2-1/2-1.2", ENV{BRLTTY_BRAILLE_DRIVER}="bm", GOTO="brltty_usb_run"
```

随后只停止 `brltty-udev.service`、reload udev 规则，将准确接口 `2-1.2:1.0` 写入 `ch341/bind`，等待 udev 完成。停止 BRLTTY 耗尽其原 10s 退出时限，最终 `MainPID=0`、`ActiveState=failed`、`Result=timeout`；没有留下 BRLTTY 进程。没有卸包/卸载驱动/停止其它服务。恢复规则的 root 命令是 `rm -- /etc/udev/rules.d/85-brltty.rules` 后 `udevadm control --reload-rules`；这会撤回排除，原抢占可能再次发生，不能无目的地执行。备份目录另存 `85-brltty.rules.local.after` 和 `repair_record.json`。

**19:33 实际复核：**

| 身份 | 当前节点和稳定入口 | 实际属性 |
|---|---|---|
| 控制 CH340 | `/dev/yuntuo-ctl → ttyUSB0`；`/dev/serial/by-path/platform-fc880000.usb-usb-0:1.2:1.0-port0` | ch341；`ID_PATH=platform-fc880000.usb-usb-0:1.2:1.0`；无唯一 serial |
| 导航 CH9102 | `/dev/yuntuo-fpga → ttyACM0`；`/dev/serial/by-id/usb-1a86_USB_Single_Serial_5910012847-if00` | cdc_acm；VID/PID=1a86:55d4；serial=5910012847；USB 路径 2-1.4 |

CH9102 是在本轮期间新枚举的现场变化，不是这次 CH340 修复的效果。`fuser` 未报告两节点有占用者；ModemManager 和 Web bridge 的原 PID 仍 active。当前 RK `/home/robomaster/QHXD/.env` 实际仍为导航 `udp:127.0.0.1:9000`、`YUNTUO_FPGA_IS_MOCK=true`、遥测 `udp:127.0.0.1:9104`，本次未改。两个稳定入口已出现，只证明设备枚举/绑定，不证明 NAV0 或完整 V2 通信通过。当前路径例外与原固定 USB 拓扑绑定；本次没有重启/物理拔插验证，拓扑改变后应重新核对，不能假设任意 USB 口都自动排除 BRLTTY。

原始记录位于 `F:/nav_integration_g2_20261023/rk_dual_uart_readonly_inventory_20261025/` 的 `inventory1.json`、`ch340_diagnosis1.json`、`ch340_brltty_rule1.json`、`ch340_targeted_repair1.json`、`ch340_after_repair_verify1.json`。认证信息只在内存/SSH stdin 使用，没有输出、复制到报告或提交。

可复用的只读清单（本次未在 RK 成功执行）：

```sh
lsusb
ls -l /dev/serial/by-id /dev/serial/by-path
readlink -f /dev/yuntuo-fpga /dev/yuntuo-ctl
udevadm info --query=property --name=/dev/<实际枚举设备>
```

只保留 `ID_VENDOR_ID/ID_MODEL_ID/ID_SERIAL_SHORT/ID_PATH` 等识别字段；核对哪个 USB 接原 CH9102、哪个接已接 P11-40 的 TTL。读取这些清单不打开串口。改变 udev 或服务前先确认当前所有者，避免两个进程抢同一设备。

候选配置（不是本次已部署配置）：

```text
YUNTUO_FPGA_LINK=uart:/dev/yuntuo-fpga@921600
YUNTUO_CONTROL_TELEMETRY=serial:/dev/yuntuo-ctl@230400
YUNTUO_FPGA_IS_MOCK=false
```

不得直接运行批量安装脚本替代针对性配置，它会重启服务及改变其它栈。上面的候选双口配置尚未部署；19:33 实际重读仍为 mock UDP，不能把串口节点出现当成 Web 已切换到真 FPGA。

## 6. 过物理门槛后的双口测试顺序

以下为待执行步骤，不是通过记录。只在当前镜像布线/时序与功能门槛通过、现场设备/供电/支撑事实核对后执行；不写 Flash。原直接控制 RX 旁路保持不动。

1. **只读识别。** 记录镜像、两口真实身份及稳定符号链接；确认唯一进程持有每口，保留原 FPGA 接线。先关闭自主任务入口，建立零命令/停止状态，不发 GOAL、TARGET 或速度旁路。
2. **遥测先行。** 仅打开控制只读口，记录 V2 版本/长度/CRC/序号/设备时间与接收时间、帧率、缺帧/重复/设备重启；核对六轴、编码器、策略状态、停止/故障与零目标。没有新鲜遥测时界面必须标失效，不能沿用旧帧证明停稳。
3. **导航零命令。** 唯一 R0 所有者读 NAV0 ID，发送 MODE idle（必要时按原停止流程 ESTOP），观察新鲜 STATUS 模式/命令归零，并与控制 V2 的轮速/角速度持续确认。K、TX_SEQ 或 STATUS 指令归零均不能单独记为实车已停稳。
4. **断连优先。** 在零命令状态分别拔导航 USB、拔遥测 USB、结束 RK bridge，再重连。确认无意外使能、无旧任务重放、遥测过期明确显示、停止锁存不被自动清除。这只能证明零命令断连无意外运动，不能证明运行中失联停车。若要求失联停车，须先定义并实现 FPGA 主机会话/心跳到期行为，不能靠已断开的 RK 再发 idle；随后在支撑条件下低幅验证停机时限。
5. **任务与地图。** 保持停止，验证 MAP_BEGIN/DATA/END 的最终 offset/state 与 CRC 错误拒绝；模式/位姿等任务先观察接受后的真实状态。检查同时 STATUS 到来时 MAP_ACK 不会被误当成功。暂不把 PATH/实时地图当作已存在输出。
6. **最后运动。** 原 N2/N3 门槛满足后才在支撑下低幅短时执行 GOAL/受控 TARGET，验证 NAV 唯一所有权、停止优先级、反馈新鲜度、取消与失联实际反应；落地停止/地形验收另计。

本审阅完成了源合同和接续步骤整理，定位并可逆修复了 CH340 的 BRLTTY 抢占，实际双口枚举/稳定入口已复核。当前新镜像端到端通信、运行中主机断连停止仍未验证；未打开串口或发送任何运动字节。

## 21:10 实际基础只读采集

RK两个节点身份和空闲复核后，P11/CH340在20.04秒被动收到251904字节，解码1998帧完整V2，CRC0、缺序0；设备时间逐帧10ms递增。所有帧status10、sensor_error1、fusion_flags1、推理计数0，不能把新鲜传输当作新鲜BMI或控制就绪。板载CH9102仅发送`R 0000`只读SYS_ID三字节，0回复，版本未验证；未发任务、未写寄存器/切mock/重启服务或动作。两口结束后无占用。原始文件和限定见[真实基础采集](evidence/g2_20261023/rk_dual_uart_baseline_readonly_20261025.json)。

失联软件候选目前只归独立本地验证：提交前零需求门控、锁止后新鲜idle握手与主机断连清队列正在接完整冻结固件；首候选来自工作树，与SYN22冻结FW有差异，既有测试不能移作冻结联合接受。仍无USB迟到字节的会话标识证明、目标CPU完整停止上界或实车停稳接受。
