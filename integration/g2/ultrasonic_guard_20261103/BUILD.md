# 超声波联合实现（2026-11-03）

## 11-04阶段交付更新

PNR100已继承PNR94超声波RTL并固化Flash；CPU扩展136KiB、实际322BSRAM，四域STA通过。保护逻辑、0键/LED7、0.3m/s和固定SG90保持。当前应用选择与实物证据见[交接](../../../docs/3_guide/project_handover_20261104.md)，下文PNR94“当前SRAM”和未烧录描述按各自时点解释。

## 11-04 04:50 PNR94已下载SRAM

用户接通开发板主供电后，USB位置36重新扫描识别器件ID `0x0011081B`。执行官方Programmer的 `--device GW5AT-138C --run 2 --fsFile D:/tools/g2_ultrasonic_noreset_pnr94_20261104.fs --location 36`，9.62秒完成，退出0。之后独立Read Device Codes读回ID `0x0011081B`、UserCode `0x000026F0`、StatusCode `0x00026230`，UserCode与FS头一致。只写SRAM，Flash未改，没有新仿真/构建/哈希或运动命令。RK SSH持续超时，未改ACTIVE、未验证导航固件自动重载、传感器或实车保护；模式默认关闭，遥控0键切换、LED7表示开启。日志与机器证据见 `evidence/PNR94_SRAM_20261104/`，下文未烧录是当时状态。

## 11-04 01:53 PNR94 布通，时序通过（未烧录）

**当次排查依据。** 同族失败日志出现`DesRoute failed`，但该消息不提供失败网络身份，不能单凭它认定全局/专用走线阶段或排除面积、打包及局部拥塞影响。以下是指导复位减载的历史对照：

- PNR92 把行缓存改成块 RAM 以后，CLS 降到 64%，比 PNR85 还低，照样失败。
- 历史上 27 次 DesRoute 失败，没有一次布通。

当时关注控制核的异步复位根（LW）新增负载。下表保留当时脚本口径，不能与厂商STA扇出5258或后来直接引脚统计混用；该相关性不等于所有历史失败的唯一根因：

| 版本 | 复位网负载 | 结果 |
| --- | --- | --- |
| PNR66–85（只改导航侧） | 5232–5236 | 全部布通 |
| PNR86 | 5511 | 失败 |
| PNR88 | 5414 | 失败 |

保护模块的 183 个触发器全部接在这个异步复位上。

**改法。** `rtl/ultrasonic_drive_guard.v` 的数据通路寄存器去掉异步复位，改为上电初值。只有 `enabled`/`mode_key_held` 保留复位：复位后模式关闭，保护整体旁路。功能和阈值都不变。

**结果。**

- 构建：SYN5 `K_ultrasonic_noreset_syn5` → `K_pnr94_ultrasonic_noresetp2r0`，place2/route0，CST 和 SDC 与 PNR88 相同。
- 布线：0 条未布通，出现 PR1014（布通时都会有这条）。复位网负载降到 5239。
- 时序：clk50 50.446 MHz，rgmii 145.4 MHz，camera 102.7 MHz，lcd9 62.9 MHz。四个时钟的 setup 和 hold TNS 都是 0，recovery 和 removal 没有负 slack。
- 资源：Logic 64616，FF 33657，CLS 44816（65%），BSRAM 318，DSP 178。
- 内置固件：布线后网表的全部 BSRAM INIT 与 PNR85 一致，仍是 follow1+3°。
- 位流：`D:\tools\g2_ultrasonic_noreset_pnr94_20261104.fs`，md5 d8965d92…。按要求没有追加 RTL 仿真，也没有烧录。

**同一晚的其他尝试，都已排除：**

| 版本 | 改了什么 | 结果 |
| --- | --- | --- |
| PNR89 | PNR88 网表加 `CLOCK_LOC` 锁全局线 | 约束被静默忽略，581 条未布通 |
| PNR90 | place1 | 布通，但 clk50 只有 41.08 MHz，592 个 setup 端点违例，不可用 |
| PNR91 | place3 | 357 条未布通 |
| PNR92 | `slf_extract` 行缓存改成块 RAM（`STAGE2B_BLOCK=1`） | 579 条未布通，此改动不保留 |
| PNR93 | `-clock_route_order 1` | 与 PNR88 完全相同，478 条未布通 |

**后续适用范围：** 已有上电初值或先写后读保证的数据通路可避免新增高扇出异步复位负载；模式、有效性和协议控制状态仍按各自语义处理。不能推广成所有新增控制寄存器都禁止复位。

## 21:18 PNR88终局

PNR88于21:18:14自然退出1，cancelled=false、总时限0/静默时限0；厂商PR0004报告478条未布通，未生成TR或位流，setup/hold均未知。工具53508及监管53020已退出。相对PNR86的1043条减少565条，仍不满足布线门槛，不能推断时序改善。

实际Logic64445/FF33637/CLS45230/BSRAM318/DSP178；对已通过PNR85分别+346/+180/+439/0/0。全部318个BSRAM叶子原语及所有参数（含显式INIT）与85一致；183个保护FF实际仍为clk50_d，保护DSP0。PRIMARY8/8、LW7/8；原35族历史不能当作本轮分配，当前85对88实际分配另存机器证据：88把匹配器rb_ra[2:3]加入PRIMARY，LCD_BL由PRIMARY改LW，观测复位/遥测快照等原LW对象未再列出。全局分配变化不是478条失败网络身份，也不足以确立唯一根因。不原样重跑，不把资源减少当作布通证明。机器证据：`evidence/PNR88_final.json`，原现场完整保留，板上仍PNR85。

## 21:04 当前实际接续

最小版SYN3已完整结束，实际网表审阅通过；PNR88于21:04:23实际启动Gowin53508，父监管仍为53020，工具日志已进入placement。新增保护实际643 LUT、183 FF、0 DSP（原PNR86源保护1734 LUT、271 FF、2 DSP）；LED为组合连线。307源BSRAM全部参数及显式INIT保持，原617个SDC引脚引用全部解析，新增触发器均为clk50_d。318物理RAM仍待本轮报告实测。

PNR88总时限0、静默时限0，不追加测试，不改变板上PNR85。实际未布通数和setup/hold尚未出；不能把综合资源减少当作布通或时序结论。机器证据见 `evidence/minimal_SYN3_PNR88_start.json`。

**当前优先：用户要求删除新增防御性设计。** 旧compact综合Gowin12208已核对身份后撤下，只停止本会话这一旧SYN，没有中断PNR或其他会话。新最小版 `K_ultrasonic_minimal_syn3` 于20:59:16启动Gowin39688，监管53020；完成实际网表审阅后接唯一 `K_pnr88_ultrasonic_minimalp2r0`。实时job位于 `ultrasonic_minimal_joint_one_shot/job.json`。当前接口/行为以本目录README和冻结新源为准，下文旧PI超时/故障状态描述仅属于PNR86历史。

PNR86已20:30自然失败：1043未布通、318RAM、Logic65525/FF33798/CLS45493/DSP180，无TR。对PNR85分别+1426Logic/+341FF/+702CLS/+2DSP，RAM不变。新增保护SYN模块1734LUT/271FF/2DSP；全局分配同时变化，不把所有1043失败网络直接归因于该模块。旧失败现场完整保留，不原样重跑。

用户已要求立即开始联合综合与布线。本轮从实际PNR85的冻结源`K_terrain19_follow1`派生，**不是**把MuJoCo报告作为硬件时序通过结论。

- SYN：`F:/nav_integration_g2_20261023/K_ultrasonic_manual_hold_syn1`，20:06:12实际启动Gowin54796。
- PNR86：`F:/nav_integration_g2_20261023/K_pnr86_ultrasonicp2r0`。综合后审阅通过才启动，不重复综合。
- 实时状态：`F:/nav_integration_g2_20261023/ultrasonic_joint_one_shot/job.json`。PID和创建时间以此文件及系统现场为准。
- 原入场门槛≥2 GiB可用物理内存、≥4 GiB可用提交；综合40分钟和原运行内存保护；PNR总时限0、静默时限0。

实际接续：综合已完成，307源BSRAM全部几何/显式INIT不变、原617个SDC引脚引用全部解析、新保护/LED304个触发器均为clk50_d；PNR86于20:12:14启动Gowin44748，已开始布局。当前监管54472，身份和阶段以实时job及tool_runtime为准。首审阅器把层次SYN网表按平坦物理VO读取而拒绝，旧拒绝报告已保留；按实际层次展开重审通过后复用同一VG进入PNR，没有重复SYN或绕过检查。

当前RTL已接入停车态轮速保持：原100 Hz、整数cm/s粒度的编码器平均反馈来自`feedback_vx_mmps/feedback_valid`；比例0.5、积分0.5/s，积分和限制±10000，输出限制±205 Q12。共享36拍逐位除法完成精确取整，不加入宽组合除法器。新反馈超过50 ms未更新则撤销修正，关闭模式、遥控交权或原硬停时清状态；测距失效时不允许正向停车修正。该新增故障处理和逐拍RTL并未借用MuJoCo结果冒称已完成硬件功能签核。

正常遥控前进仍是原1229 Q12（0.3 m/s档），新环只在已锁定停车态工作；导航拥有控制权时全旁路。SG90始终固定朝前，600/250/350 mm阈值、LED7及原IR5停机优先级保持。没有改原4500、权重、CPU固件、地图或其他传感器功能。

`run_joint_build.py`冻结完整输入并依次执行SYN、实际网表审阅、PNR。网表审阅逐项比较原307源BSRAM全部参数及显式INIT，核对原SDC617个pin引用在新层次网表中可解析，并核新保护/LED触发器实际时钟。沿用宽拆分后318块只是本轮物理运行前的预期，必须以新报告验收。

PNR沿用原place2/route0、同一C版器件、原CST和完整SDC，包括50/100/50/8.967 MHz、已有I/O预算和例外、5000路径报告。未加新时序例外。实现完成后继续审实际资源、未布通、四域/全局setup和hold、复位及I/O，不因退出0或生成位流而下载。

本轮不运行额外RTL回归、不写Flash、不改变板上PNR85。
