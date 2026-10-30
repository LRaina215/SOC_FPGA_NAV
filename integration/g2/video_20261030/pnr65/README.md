# PNR65：带摄像头/LCD 的联合镜像（时序收敛）

位流：`D:\tools\g2_video160_pnr65_20261031.fs`（sha 89146aca…，全部哈希见 `hashes.txt`）。构建目录 `F:/nav_integration_g2_20261023/K_pnr65_v160s3p2r0`。只写 SRAM，尚未上板。

## 内容

- 网表：Codex 的 `K_video160_mmio_predecode_syn3`（SYN50 的导航与控制 + 160×120 RGB565 双缓冲摄像头/LCD + 超声波/SG90 扫描 + PID 首样本修复 + 导航 MMIO 读地址预译码）。综合 0 条 EX3988（初值缺失）告警，BSRAM 307，PNR 拆宽后 318/340。
- 与 SYN50 的核对：`command_lease_guard.v`、`bmi088.v`、`oled_driver.v`、`mem_policy.v`、`imu_fusion.v`、`nav_command_units.v`、`bmi_att_uart.v` 逐字节相同；`control4500_core.v` 只多了视频/OLED/测距接线；顶层只新增视频、测距、扫描的端口和实例，视频复位由 `~rst && video_locked` 门控，不回灌到导航和控制。
- CPU RAM 初值为前移 1000 固件（`nav_fw.bin` sha cdebc9bd…，与 PNR63 相同）。

## 本次接手补的约束（`g2.sdc` 末尾）

PNR1（Codex，同一视频 RTL、place2）clk50 只有 47.3 MHz：170 个负裕量端点，其中 125 个是视频跨时钟路径（SDC 只声明了 `camera_pclk`、`lcd33`，没约束同步器首级），被当成同步路径分析；它们还挤占了真实关键路径的布线资源。

1. `audit_video_cdc.py` 在布线后的展平网表上，从每个触发器的 D/CE/复位端沿组合逻辑反查到发出寄存器，列出全部跨时钟对，每一对都必须归入一类已审结构，否则判失败。PNR65 网表：173 个跨域，全部归类。
   - 导航原有 26 个（原 SDC 已约束）；
   - 视频单比特同步器首级 7 个，逐个核对首级只驱动一个同时钟的第二级；
   - HUD 数据 120 位：clk50 侧先写好 `held`，再翻转 `req`；LCD 侧在 `req` 过两级同步后才采样 `page<=held`，数据至少稳定两个 LCD 周期；
   - 复位同步器的异步复位端 12 个，LCD 复位流水 8 个。
2. 对 7 个单比特首级和 120 位 HUD 数据各加 `set_false_path -hold` 加 `set_max_delay`，上限取源、目的时钟周期较小者（20 ns），与导航原有 26 组写法一致，没有使用 `set_clock_groups`，也没有放开 setup。
3. LCD 与相机像素时钟两组"异步置位、同步释放"复位同步器（D 恒为 1，下游只用第二级），只对这 4 个 CLEAR 脚免除 recovery/removal 检查（PNR64 在这两处报 −7.62 ns 的假违例），其余复位路径照常分析。

## 结果（place2/route0）

| 项目 | PNR63（板上） | PNR65 |
|---|---|---|
| clk50 | 50.075 MHz | 50.044 MHz |
| rgmii_rxc | 136.8 MHz | 132.5 MHz |
| camera_pclk / lcd33 | — | 97.9 / 65.7 MHz |
| setup / hold TNS（4 个时钟） | 0 | 0 |
| 最差 setup / hold | +0.030 / +0.162 ns | +0.017 / +0.160 ns |
| 最差 recovery / removal | +5.996 / +0.282 ns | +4.946 / +0.345 ns |
| 未布通 | 0 | 0 |
| 约束 Actived | 61 | 319（全部） |
| BSRAM / Logic / Register / CLS | 280 / 61054 / 32151 / 43136 | 318 / 63202 / 33433 / 44790 |

89 个 CST 引脚全部按约束落位，无重复。布线后的网表重新审计，生成的约束集与实际使用的完全一致。

## 还没验证

- 上板：摄像头 SCCB 初始化、完整帧采集、LCD 显示、超声波/SG90、HUD；以及导航/控制在新镜像上回归（BMI 启动、租约、遥控覆盖、遥测）。
- 外部 IO 时序（相机 DVP 输入、LCD 输出）没有板级延迟预算，SDC 里没有 input/output delay。
- 余量很小（setup +0.017 ns），之后任何 RTL 改动都可能需要重新收敛。
