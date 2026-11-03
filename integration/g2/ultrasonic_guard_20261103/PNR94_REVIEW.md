# PNR94独立审阅（2026-11-04）

结论：最新 `K_pnr94_ultrasonic_noresetp2r0` 确实完成布线，已约束路径的setup/hold均通过。此次只读审阅真实日志、TR、物理VO、冻结RTL和约束，没有重新仿真/综合/布线、计算哈希或操作车辆。

## 实际实现结果

- 厂商日志01:54:58完成输出，存在完整TR与位流，出现 `Placement and routing completed`，无PR0004/DesRoute失败。
- 四时钟与全部域内setup/hold端点违例、TNS均为0；全局setup/hold端点违例均为0。

| 时钟 | 目标MHz | 实际Fmax MHz | Setup违例 | Hold违例 |
| --- | ---: | ---: | ---: | ---: |
| clk50 | 50 | 50.446 | 0 | 0 |
| rgmii_rxc | 100 | 145.427 | 0 | 0 |
| camera_pclk | 50 | 102.676 | 0 | 0 |
| lcd9 | 8.967 | 62.931 | 0 | 0 |

- Setup/hold/recovery/removal/最小脉宽各实际列出5000项，最低裕量依次+0.177/+0.160/+5.040/+0.250/+2.123ns，列出项无负值。全局setup/hold结论另以STA汇总为依据，不用5000条冒充全部路径。
- 核心最差路径为NAV位姿提交到高程RAM地址；新保护模块已列出的最差setup/hold裕量为+2.916/+0.248ns。
- 实际Logic64616、FF33657、CLS44816/69120、BSRAM318/340、DSP178/298。对PNR85分别+517/+200/+25/0/0。
- PRIMARY8/8、LW8/8。TR最高已列拥塞R31C41为0.875；它是本次已布通设计的报告，不能反推旧失败网络身份。

## 输入、存储和复位

- PNR94实际VG逐字节等于完整SYN5产物；实际CST/SDC及规范路径后的实现命令均与PNR85保持。
- SYN5对SYN3的75个选定RTL仅 `ultrasonic_drive_guard.v` 改变；改动仅为数据通路去异步复位、增加上电初值。原0.3m/s、导航所有权、600/250mm阈值、固定1500us SG90和LED7连线保持。
- 全部318个物理BSRAM原语、几何参数及显式INIT与PNR85一致。所复制的发布FS逐字节等于本次实际PNR位流，无哈希校验。
- 保护实际185个FF全部为clk50_d；仅enabled/mode_key_held的2个FF接异步CLEAR，其余183个数据FF不接系统异步复位。物理INIT均明确：12个DFFSE为1，其余为0；enabled实际INIT0，默认关闭保持。其余非零初值对应speed/range_cap的1229。
- 普通运行与导航旁路的算术/选择表达式没有改变；**热复位的内部状态不再与旧版逐状态等价**。enabled复位关闭，停车积分在旁路时清除，但near_obstacle/range_cap保留并等待下一次有效测距更新；不能把“上电初值”写成“每次按复位键重新初始化所有数据”。这是此次无复位数据通路的实际语义，不另加故障/重武装功能。

## 两项需要保留的签核边界

1. 319条约束在TR中全部Actived：3基础钟、1生成钟、157条false-path、153条max-delay及5条报告命令。SDC与85相同，没有为本次改动增加例外。不过该SDC没有set_input_delay/set_output_delay，也没有以get_ports为起止点的max/min延迟预算，**外部器件到FPGA、FPGA到外部器件的板级IO窗口没有由本报告证明**。这是继承的缺口，不是此次新增违例。实际接口功能仍需结合原板级证据及新镜像实测，不能写成完整外部IO签核已通过。
2. PR1014是相机时钟存在普通路由段的警告，**不是布通标志**。资源表同时列camera为PRIMARY，二者应一起保留；本轮相机约束域STA通过，不能因此抹掉告警或替代真实图像验收。

## 对“已找到全部失败根因”的评价

同一组实现参数下，撤去新增保护的数据异步复位负载后由PNR88失败478转为PNR94布通，且318RAM和178DSP保持。这是强证据，支持以后将可先写后读的数据通路与必要复位控制位分开设计。

但两次综合后的局部逻辑、复制/打包和实际全局分配也改变，不能把一次成功推广成“所有历史失败都仅由这条网造成”或“与面积完全无关”。厂商TR本轮复位根高扇出报告为5258；源码/网表脚本的其他负载数必须注明计数口径，不能混用。也不应将结论扩大成“所有新增控制寄存器都不得复位”。

建议保留PNR94作为新候选，下一步转入SRAM功能验证：默认关闭及0键/LED、原0.3m/s遥控减速停车、导航旁路、复位后模式关闭。此审阅没有执行这些实机项目，亦未烧录。

实际来源：

- `F:/nav_integration_g2_20261023/K_pnr94_ultrasonic_noresetp2r0/impl/pnr/g2_top.tr`
- 同目录 `g2_top.rpt.txt`、`g2_top.vo`、`cmd.do`，构建根目录 `gw_sh.log`、`g2.sdc`、`g2.cst`
- `F:/nav_integration_g2_20261023/K_ultrasonic_noreset_syn5/ultrasonic_drive_guard.v`
- 发布文件 `D:/tools/g2_ultrasonic_noreset_pnr94_20261104.fs`
- 本次机器记录 `evidence/PNR94_independent_review.json`
