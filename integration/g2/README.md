# G2共同资源工程

固定输入：控制`6443f56`的真实RTL/ROM，导航检查点`cd9b816`。不会烧录。实际构建读取当前仓库`control/fpga/src`，由`prepare_control.py`应用`control_nav_adapter.patch`生成导航入口/反馈适配；补丁不匹配就停止，不静默使用旧副本。`../control4500/original_src`与本目录`control4500_core.v`仅保留6443基线对照。控制副本的来源与提交方式见`control/FORK.md`。

```powershell
D:\Anaconda\python.exe -X utf8 integration/g2/build_g2.py --nav as --stage all --out F:/nav_integration_g2_20261022/example_as
D:\Anaconda\python.exe -X utf8 integration/g2/build_g2.py --nav coverage --packing directed --command-lease --stage all --out F:/nav_integration_g2_20261022/example_directed_coverage
D:\Anaconda\python.exe -X utf8 integration/g2/test_storage_equivalence.py --out F:/nav_integration_g2_20261022/example_equivalence
```

输出目录必须不存在。`--packing none|nav|control|both|ssram|hybrid|planes|directed`比较存储布局；`ssram`转两张完整代价图，`hybrid`还转完整帧缓冲/触碰位图。`planes`拆高程元数据；`directed`在导航/控制两侧显式区分大块存储和小分布式存储，避免自动提升小RAM消耗余量。旧失败报告使用开启宽RAM拆分的设置，已保留；不把相同布局在关闭拆分后的结果混写。`--stage syn|all`区分综合与完整构建。硬上限340块BSRAM、工作门槛336；只有综合数不构成通过。

必须显式记录`--wide-split 0|1`（现默认0）。安装工具的旧实际PNR命令开启了32/36位存储拆分，八次构建的额外需求与宽叶子数量逐一相等。`pnr_from_netlist.py`可以固定已有网表，仅改该开关配对；`audit_netlist_memories.py`给出原语、位宽和层次归因。关闭拆分可避免重复占用，不改变算法/数据，但仍需实际资源和时序通过。

宽叶还包括只读pROMX9等物理模式，不能仅按SDP名筛选；双流水实际为314＋18＝332（导航17/控制IK ROM1）。`audit_physical_memories.py`把后PNR的332个叶子与来源逐实例对齐，原314全在、18对拆分均匹配。`audit_pin_loads.py`只计实际网表直接非时钟输入引用，不代表未布通网络名单。

`pnr_from_netlist.py`也可读取SYN-only构建，从实际PRJ/有效配置获取频率；`--replicate-resources 0|1`显式覆盖工具高负载资源复制，否则保持来源设置。`--place-option`默认1（可布线优先），2为时序优先，均在预检回读。实际PNR二进制复制标志为`-replicate`。这是物理实现试验，不改RTL算法或权重，仍须重测实际占用/布线/时序。示例：`python integration/g2/pnr_from_netlist.py <SYN-build> --wide-split 1 --replicate-resources 1 --place-option 2 --out <new-build>`。

`--route-option 0|1|2`分别为默认、更深搜索、较快布线，预检回读。布线轮次必须显式记录预算，历史60/35分钟取消与延长后的自然失败分开保存。本次用户要求继续介入后的轮次采用`--place-option 1 --route-option 2 --max-runtime-minutes 120 --max-no-progress-minutes 0`，其余时钟/CDC/资源门槛不变。预算默认0表示不启用；对本任务新PNR必须显式指定。`tool_runtime.json`保存进程和预算，观察日志/中间产物变化，CPU忙或监管心跳不算进展；超限只终止本次启动的工具，保存取消说明并返回未通过。`test_bounded_tool.py`验证真实子进程生命周期，不跑FPGA或物理仿真。

`--capture-system-telemetry`可每30秒向`system_telemetry.jsonl`记录所属进程CPU/线程/工作集/私有内存/I/O及主机物理空闲/提交余量。依赖psutil，Windows提交值来自GlobalMemoryStatusEx；缺少观测记录错误，不伪造零值。系统观测文件明确排除在日志/产物进展集合之外，CPU、I/O或记录写入不会刷新静默预算；页错误累计也不能直接视为硬分页次数。当前一次延长诊断仍由总计划规定独立上限，不恢复定时监督。

`adopt_pnr_deadline.py <正在运行的冻结PNR目录> --deadline <带时区绝对时间>`仅用于用户明确延长现有一轮：校验实际工具PID/创建时间、包装器命令和构建目录，暂停原Python预算包装器，继续同一Gowin进程；新监管器按绝对截止收尾，保留旧预算快照。工具退出后恢复原包装器进行报告采集，再保存扩展后的实际运行记录，不重新启动厂商工具。只读系统观测继续、不计进展；历史2026-10-23T13:00:00+08:00延期只归旧轮次；各轮最新预算以总计划、对应deadline_extension.json及有效tool_runtime为准。静默预算取消时仍由绝对截止兜底，不能只改静态JSON来声称运行中的包装预算已改变。两种一次性子进程合同验证了自然结束/截止终止、PID保持和原包装器恢复，不代表已测试退出ChatGPT应用后任务存活。

`--diagnostic-only`显式标记受控实现对照。采集器及最终写出的`g2_gate.json`均强制不准入，即使报告的资源/时序满足条件也不会成为生产通过结果；缺报告同样拒绝。基础SDC对照直接取SYN来源的两条50/100MHz时钟，省略`--cdc-reference-audit`，不删除或修改旧候选目录。它仅用于判断CDC候选约束是否参与长时间搜索，后续生产仍需完整26路径/52约束核验，不作烧录来源。输出策略的合成指标夹具合同不冒充真实FPGA报告。

`--timing-driven 0|1`控制厂商PNR时序驱动优化，默认1。关闭时脚本要求`--diagnostic-only`，在创建输出目录前拒绝无诊断标志的请求；有效选项必须回读为0，实际`cmd.do`不再含`-timing`。它不更改SDC的50/100MHz时钟，也不等于满足时序，只用于隔离优化搜索的影响。预算与报告/物理CDC检查保持；关闭优化后完成的报告仍不得作为G2或部署准入。

`--command-lease`加入NAV唯一owner的命令年龄/有界减速/实测停稳监管原型；原4500继续维持平衡。入口RTL与24项CRC/坏帧/断线/切换等合同见`test_command_lease.py`，默认500ms停稳窗口在合同测试中缩放到10ms。外部旧UART协议没有序号/session，不能因此声称旧协议具备防重放。资源结论须归到带该标志的共同工程。

`--observation-pipeline`是隔离的轮速分子时序候选，生成器位于`control/tools/g2_observation_pipeline.py`。保持算法/数值及外部观测周期，仅写构建副本；原控制文件仍保留。`test_observation_equivalence.py`提供与原RTL四组符号、32帧逐周期比较，实际共同资源与时序另验。未指定标志时使用原观测RTL。

`--command-units-pipeline`（要求`--command-lease`）将NAV单位换算改为精确四级流水，正常接收多4拍/80ns，序号同步、停止/看门狗直接优先并清空在途命令。`test_nav_command_units.py`穷举两轴全部16位输入并验证连续输入、序号和复位/故障；不改变原4500算法、网络、权重或125Hz推理。

`--coverage-static-bounds`提前计算覆盖层每帧固定的窗口基点/边界，保留32位回绕算术、全部状态和外部周期。AS可指定相同标志用于同源码对照（覆盖层仍不实例化）。`test_coverage_collision.py --static-bounds`比较原/新模块所有外部信号与有效RAM端口每拍一致，沿用独立模型及碰撞X读值；`measure_coverage_timing.py`仅测模块综合。模块Fmax改善不等于共同工程或PNR通过，当前在跑的双流水配对没有此标志。

`--fusion-norm-pipeline`由`control/tools/g2_fusion_pipeline.py`生成BMI模长寄存边界调整版，滤波、标定参数及所有外部周期保持；`test_fusion_equivalence.py`对照完整原融合模块和24位极值。有符号平方的carry-save/显式扩展加法两版均触发综合器内部错误，记录保留；当前恢复原绝对值平方表达式，只调整寄存边界；`measure_fusion_timing.py`原/新模块综合均成功，联合结果另验。原控制源不覆盖。

`--gyro-sum-exact-widths`将三个原始16位陀螺仪绝对值之和精确限制到17位，显式零扩展回原32位消费者，保持live输入、参数比较、bias累计和周期。`test_fusion_equivalence.py --gyro-sum-exact-widths`增加786432组算术输入及组合版完整逐拍对照；`measure_fusion_timing.py --module fusion --gyro-sum-exact-widths`在模长候选上隔离此改动，双方有97位共同输入启动夹具，避免将未约束顶层输入当作已测路径。夹具不进产品；具体SYN/PNR是否包含此标志以冻结manifest为准，不能将旧版结果转归新候选。

`--pose-predecode`在原总线捕获拍寄存位姿提交译码，替换帧接收和覆盖层的两个消费者；点流延迟、帧归属不变。`test_pose_predecode.py`在实际NAV总线状态机内与未改寄存器文件脉冲逐拍比较。

`--action-map-exact-widths`保留原动作裁剪、四个私有增益、对称舍入和输出周期，只按可达范围缩短中间算术。生成器位于`control/tools/g2_action_map_widths.py`，原控制文件保留。`test_action_map_equivalence.py`覆盖每增益全部8193个裁剪值和完整模块逐拍输出；`measure_fusion_timing.py --module action-map`使用共同输入启动寄存器测量，寄存器仅属测试夹具，不进入产品。不能把未约束顶层输入时测出的反馈路径Fmax当作动作路径时序。

本工程统一使用器件C版、RTL核心50MHz，PNR SDC定义20ns核心与10ns接收时钟。截至directed_local，实际综合/全局提示均100MHz，旧`-frequency 50`设置未生效。脚本现改为文档支持的`-global_freq 50`，并在昂贵构建前用`saveto -all_options`回读频率和宽RAM选项，失配即停止；实际报告另读PRJ/cmd.do核对。所有PNR核心SDC约束仍为20ns，综合Fmax不能代替PNR。实板B/C对应、完整IO时序和BMI重力到导航姿态/时间适配尚待确认；旧导航IMU UART仍保留在A15，本工程不代表G3功能或实车通过。

存储优化保持原参数数值与推理调度，不量化、不跳MAC、不改训练。控制权重40个1K×16银行、ELU13bit精确常量尾、偏置26bit；导航仅改变大RAM物理分银行，容量、初始化、读前写、单拍读及保持语义不变。原始文件始终保留。

`--packed-weight9`可叠加含控制存储的布局，按符号/低字节、六位打包与精确例外把权重ROM重排为35块原生9/8位模式。它不删权重位或增加读拍；模块综合40→35已测，联合节省仍需测量。`test_storage_equivalence.py --nav-layout directed --packed-weight9 --control-only --out <new-dir>`只重测改变的控制及全部地址；`measure_weight_rom9.py --variant both|bank16|packed9 --out <new-dir>`测真实ROM模块；`test_weight_rom_netlist.py --build <module-build/packed9> --out <new-dir>`验证实际综合网表的厂商INIT/使能/延迟。预检配置保存后显式exit并限时60s，完整综合/PNR没有该限时。

`retry_frozen_synthesis.py <cancelled-SYN-build> --wait-pid <owned-PNR-wrapper-pid> --out <new-dir>`为单次串行重试：先复制冻结输入、重定位并回读选项，再等待已有构建退出。启动需物理空闲≥2GiB/可用提交余量≥4GiB；运行中连续提交余量<512MiB，或物理<256MiB且提交<1GiB才取消自己的子进程，SYN总预算40分钟。旧仅物理1GiB策略会过早中止，取消不等于OOM。`job.json`保存阶段、内存/耗时心跳；成功后保存SYN摘要和存储清单。它不读取之后的源码、不追加PNR、不编程，旧目录保留。

前一工具已结束时可显式使用互斥的`--start-now`；`--gyro-sum-exact-widths`仅在冻结输入副本上派生已验证的17位求和，校验来源尚未含该候选、所有RTL输入中仅`control/imu_fusion.v`改变，并写入manifest/job。旧SYN结果/内存清单不复制成新结果；50MHz预检、内存/40分钟保护、原固件和权重保持。派生来源、已完成报告及后续PNR进度统一见总计划。

`--trig-block-rom`仅用于directed布局，将`nav_ram_sdp`中INIT_FILE非空的表使用原读写/初始/保持代码加block属性实现；当前共同工程仅四份1024×16正弦表，不改变数值或读取拍。`build_g2.py`和`retry_frozen_synthesis.py`均支持；后者只修改冻结输入中的`nav/rtl/common/nav_ram_sdp.v`，原控制/固件/权重保持。`audit_trig_routing.py <SYN-build> --out <json>`记录实际四表原语及局部ROM16地址引用，不作物理拥塞或全局扇出结论。`test_trig_block_rom.py --source <frozen-SYN> --out <new-dir>`覆盖两独立端口全部地址、初始零及使能保持，再综合并用真实厂商原语验证INIT；小模块2块不能代替联合占用。若仅报告字段解析出错而RTL/SYN已完成，可显式`--resume-vendor-init`，逐字核对所有保存输入/设置后继续，不重跑已完成阶段。

`--snapshot-mux-fanout`由`control/tools/g2_snapshot_fanout.py`生成策略快照候选，仅将原捕获/移位更新提取为同拍独立过程，对数据选择信号使用`syn_maxfan=128`；CE/复位不加该属性，原控制源和125Hz保持。两个构建入口均支持，冻结派生逐字检查只有`control/rl_policy_tick_v5.v`改变。`test_snapshot_fanout.py --out <new-dir>`比较原/新快照、所有推理核心输入和发布/诊断输出，包括故障、超时、复位；核心测试替身不代表真实RL验证。`measure_snapshot_fanout.py --out <new-dir> [--only original|fanout128]`使用相同测试替身做小范围综合，只比较结构和模块内直接LUT输入负载，不报告产品时序；`--only`用于保留已完成一侧、不重复测试。`audit_global_muxes.py <PNR-build> --out <json>`追踪实际全局网络分配及其到快照D端的连接，不将高负载冒充未布通网络名单。

当前结论与原始路径见`docs/4_design/g2_joint_resources_20261022.md`，唯一当前状态表在`docs/1_plan/system_integration_plan_20261021.md`。

`audit_snapshot_fanout.py <联合SYN目录> --out <json>`用同一口径比较快照候选前后的真实联合网表：要求4032个快照Q位完整且无重复，追踪其D端直接LUT及输入负载，并单列CE/复位/时钟连接。统计局限于策略模块，子模块端口作为边界，不冒充全局扇出或实际布线状态；它不运行综合或仿真。

`package_routing_repro.py <保留的PNR目录> --out <新目录>`把冻结网表/CST/SDC、真实SFP的CSR/主机CDC、完整有效选项、原运行现场和便携有预算入口打成本地zip。Tcl输入路径相对解压目录，manifest强制diagnostic-only；没有执行工具、上传或硬件动作。当前包未按逻辑删减，不是已定位的最小故障电路或厂商bug证明；入口帮助/导入已检查，尚未独立执行整轮复现。保留原`.p/.db`在原目录，不将其当作新布局输入或拥塞证据。

`collect_pnr_result.py`仅读取实际PNR报告，保留小数DSP用量；布线失败时记录未布通网络和缺失的时序，不用SYN频率填充Fmax。例：`python integration/g2/collect_pnr_result.py <build> --out <result.json>`。

签核还检查已报告的Recovery、Removal及最小脉宽最差表；存在负裕量或缺失表时不宣布G2通过。表中负数计数仅为工具报告出的行，不当作全部违例端点数。IO/CDC与器件版本完整签核继续单列。

完整构建与冻结网表PNR会保存`g2_gate.json`，并按实际报告返回G2门槛结果；厂商工具返回0或产生位流仍可能时序失败。仅综合阶段的成功不代表G2接受。采集器区分域内与全局违例，跨域违例保留；目前332块首次布通版核心26.313MHz，G2不通过，禁止按该位流部署。

`audit_cdc_first_stages.py <flat-vo> --out <audit.json> --emit-sdc <fragment.sdc>`核对四组共26条已知同步器的实际驱动、时钟与第一级单扇出到预期第二级。`pnr_from_netlist.py --cdc-reference-audit <audit.json>`显式启用隔离的第一同步级hold-only及8/18ns最大延迟候选；原SDC保留。新报告要求26条物理连接与参考相符、52条约束全部Actived，出现其他未审例外即拒绝。小型工具实测表明第二级setup/hold及源到第一级的最大延迟仍计时；不代表完整CDC、IO、偏斜或MTBF签核。

`nav_count_feedback.v`为独立N3候选，尚未接入`g2_top`及正在运行的冻结PNR。固定名义65mm/1320CPR、10ms原始AB计数，左正/右负为前进；保留导航速度换算余数，并提供50窗计数变化的补充慢速证据、质量/超时/重叠/范围故障及粘滞连续性丢失。计数窗口不能观测门内往返、亚计数运动和打滑，不能单独确认物理停止；原PID/RL不受影响。`test_count_feedback.py`采用独立整数/分数模型，`measure_fusion_timing.py --module count-feedback`使用输入启动寄存器测量原型（测试寄存器不进产品）。接入及完整资源/时序、G3质量状态、原4500闭环和停止包络仍待完成。

`--real-sfp`将实际SFP1/Q0 Lane2 MAC/PCS/SerDes、CSR及只读CONTROL_V2引出接入共同工程。SFP2管理窗口为0x1300；使用`integration/sfp/sfp_client.py status --image joint --port <端口>`只读检查，指定`--mode`才重配。51条主机FIFO/toggle CDC与原26条分别核对，受保护厂商PCS/状态/复位仍须独立签核。

`--history-data-fanout`只对观测history initialized的数据选择负载加syn_maxfan=128，模块配对负载1000→128。`--lease-output-select`要求command-lease，只把停止/正常目标选择放在两套相同approach计算之后，保留每拍状态及优先级；24项合同63850拍逐拍输出对照通过，模块综合最坏14.481→9.007ns。生成器只改构建副本，联合效果以冻结SYN/PNR证据为准。


`retry_frozen_synthesis.py --slf-read-address-fanout`只在冻结构建副本的`slf_extract.v`读地址线上增加`/* synthesis syn_maxfan=64 */`，不改表达式、寄存器、初值或容量。`audit_slf_read_fanout.py <before-SYN> <after-SYN> --out <json>`核对源码仅此属性差异，并统计实际行缓存RAM地址脚负载；首次联合实验`sfp_slf64_clockplan_syn4`已测得负载仍275、未达到64；该方案未用于后续PNR。属性存在不构成收益，不重复相同实验。

`pnr_from_netlist.py --clock-resource-plan integration/g2/clock_resource_plan_control_priority.json --routing-evidence`增加有物理依据的全局资源分配及完整警告/BSRAM位置输出。计划将PRIMARY4..7优先用于大负载控制复位/快照，较小导航CE/地址改普通布线；四条真实时钟由工具在其余PRIMARY选择。应用前核对前轮真实负载、源网表对象存在、索引无冲突，拒绝把真实时钟改普通布线。未改网表、SDC和原IO约束，物理采集器重新核对全局分配结果；实际PRIMARY分配、未布通数、BSRAM、时序和77条CDC/154条有效约束均待新报告。`BUFG[n]`的SR/CE到PRIMARY能力已由独立138C探针确认，不能由小探针推定联合布通。

负载读取器已兼容RAM16SDP4在最后端口行闭合的语法，并计入RAD/WAD/S0/S1；旧脚本的RAM分类/读地址负载不完整。`test_physical_pin_parser.py`覆盖连续内联RAM与转义向量名；`test_clock_resource_plan.py`覆盖丢失对象和误把真实时钟改普通布线的拒绝。负载统计仍不能冒充工具的未布通网络名单。


`clock_resource_plan_control_priority_v2.json`基于前轮PRIMARY实际落实、未布通802→273的结果，保留原8条约束，仅补充观测3892-CE与遥测960-CE的泛型BUFG请求。应用器要求已有实际负载/源对象；采集器要求两条实际LW并核对四真实时钟仍PRIMARY。Arora V不使用BUFS。前轮两条未列全局而较小导航信号占LW是待检假设，不证明273条失败的身份；120分钟/无静默终止的新轮仍需完整物理报告。


两CE方案实际401未布通，较273版差；该config保留作失败复现，不作为当前较优实现。`package_routing_repro.py`现支持真实SFP：复制准确CSR/51主机CDC/全局计划与全部检查辅助，重写可迁移Tcl路径，保留原报告和p/db现场。默认120分钟总预算/静默0，强制diagnostic-only；仅本地打包，不运行工具、上传或下载硬件。`repro_sfp273_complete.zip`已验证输入保持、入口帮助和辅助导入，尚未执行完整独立复现；不能称为最小故障或工具bug证明。

`audit_routing_topology.py <frozen-vg> --library <gw5a/prim_sim.v> --out <json>`按设备原语真实输入/输出方向展开源网表，保存模块负载、数据选择和跨模块连接，拒绝漏分类原语端口。`synthesis_connectivity.py`现支持转义命名端口（例如`.\g_lane[0].we (net)`）；旧版本漏读这类CPU/SerDes层次连接。顶层外部输入和不透明IP边界的缺少显式驱动不按悬空硬件解释；显式源拓扑、物理扇出与逐网布线状态须分别记录。

`partition_routing_diagnostic.py <273-PNR-build> --out <new-dir> --prepare-only`从273版派生唯一匹配器内部移除对照，保持其外部所有显式单元/引脚、原控制参数、原时钟/SDC/CSR；只删除该模块3条失效LOCAL_CLOCK。替换输出恒零，不模拟匹配器功能，PNR可能裁剪下游、重新安排全片位置；不把通过当模块故障证明。`--run-prepared`只启动已有未实现目录，等待最多10分钟达到2GiB物理/4GiB可用提交，再按120分钟/静默0运行，结束保存门禁与诊断报告，永不下载或接受。首次输入头解析错误保留，新retry1已修正`)`与`;`可分行的语法，并强制新模块只有110个GND单元。

诊断SDC只追加`report_route_congestion -max_grids 200`和`report_high_fanout_nets -max_nets 100 -min_fanout 100`。`collect_routing_diagnostic.py <build> --out <json>`读取厂商真实拥塞网格/高扇出/未布通报告，缺失值保持未知、不授予部署接受。旧成功报告仅用于读取格式检查，不能将旧网格位置移用到新失败候选。新诊断即使布通，完整工程仍须恢复匹配器、重新通过全部物理门槛。

`prepare_bank_select_fanout.py`在银行读选择的实际寄存器驱动上加`syn_maxfan=128`，不改变语句/初值/使能/容量；不同于先前无效的SLF地址线属性。`measure_bank_select_fanout.py <frozen-SYN> --out <new-dir>`对真实64×1024×18几何配对模块SYN，每版180秒/所属工作集768MiB上限；这是小模块探针，完整SYN仍保持2/4GiB启动与40分钟预算。实测最大选择负载576→128、BSRAM64保持、FF+9/LUT+15，不承诺联合布线改善。

`verify_bank_select_replication.py <actual-vg> --out <json>`从实际LUT/寄存器INIT计算选择器的转移函数，对64状态×64银行地址×两种读使能穷举8192组；原6/复制15个寄存器各49152/122880次检查通过，确认复制后D反馈实现的保持与更新仍相同。它只证明该选择器，不宣称验证了整张RAM或物理时序。`retry_frozen_synthesis.py --bank-select-data-fanout --wait-pid <prior-PNR-wrapper>`只改冻结副本的导航RAM属性，在原PNR报告采集退出后串行联合SYN；实际源叶/宽叶、选择器负载和路径仍要重新核对，再决定下一轮完整PNR，不复用删除匹配器的诊断输入。

`continue_bank_select_once.py --synthesis <queued-SYN> --baseline <273-PNR> --wait-pid <existing-SYN-owner> --pnr-out <new-full-PNR> --state-dir <new-state-dir>`为一次性事件依赖接续，不创建定时任务或重复SYN。先核对等待进程身份，最多等3小时；SYN完成后独立进程核对精确属性差异、全部RAM参数/容量、实际选择负载、核心SYN估计、8个全局计划驱动与26＋51条源CDC连接，未完成/失败/不满足门槛就停止并保存原因。大字典随检查进程退出释放。通过后按2/4GiB启动门槛，只启动1轮完整功能120分钟/静默0 PNR；不覆盖已有目录，不下载，也不以包装退出码授予接受。`job.json`是实时阶段，`pre_pnr_checks.json`及`routing_result.json`保存后续实际证据；退出0只表示单次流程收尾，仍待审查。

`pnr_from_netlist.py --routing-evidence`现在同时追加200网格拥塞与100条高扇出只读报告请求，原时钟和时序例外保持；生成manifest记录实际追加命令。失败PNR仍可能没有tr，缺报告继续未知。


`retry_frozen_synthesis.py --sfp-lane2-from-build <proved-standalone-build> --bank-select-data-fanout`从旧完整冻结源采用已核对的SFP1 Lane2生成包装/CSR及映射注释，其他源逐字保持；只改联合SDC两条125MHz时钟源，拒绝导入独立整域异步例外。保护MAC须字节相同，配置须只启用Q0 Lane2/REFCLK1/loopback OFF。`prepare_sfp_lane2.py`在冻结和PNR前均核对来源；一次性接续还要求实际SYN的GTR Lane2 RX/TX输出分别驱动h_rx_clk/h_tx_clk。旧Lane0候选只留历史路由对照，不能用于SFP1物理验收。Lane2修正与银行复制同轮时，PNR变化不作单因素收益结论。


`revalidate_lidar_cdc.py`重新计26条实际源→首级→第二级的数据/时钟/使能及首级Q唯一负载。生成RESET编号变化仅在唯一具名驱动相同、驱动全部输入相同、包含INIT的完整复位模块定义相同时被视为别名；不跳过RESET检查，不新增时序例外。`test_lidar_reset_alias.py`对保留的实际Lane2/273冻结网表做6项正负回归，拒绝数据/时钟/负载/复位输入/INIT变化。失败接续保留原state，修复后复用既有SYN、使用新state创建尚未存在的PNR目录，不重复综合。


`prepare_slf_row_banks.py`只给实际160×98/AW8行缓存增加十个16词分布式银行，寄存选择与银行读同拍更新，原1拍延迟/read-first/re保持/初值和15680bit容量保持；其他几何原样。`measure_slf_row_banks.py <frozen-Lane2-SYN> --out <new-dir>`先做RTL对照，再各180秒/所属768MiB上限的小模块SYN及厂商真实原语对照。首轮负载686→250、RAM16 275→250、LUT−76/FF＋891、BSRAM0保持，两种仿真各9012拍通过；不证明物理拥塞根因。`retry_frozen_synthesis.py --slf-row-banks --start-now`只派生这一个RTL差异，完整SYN仍40分钟与原2/4GiB门槛。`continue_bank_select_once.py --max-runtime-minutes 150`可接续当前唯一候选；核对探针来源、104资产、BSRAM参数、全片行缓存实际负载/250个RAM16、原地图选择≤128、8驱动/26＋51 CDC及Lane2时钟。已是Lane2的输入必须保持SDC完全相同，不能重新导入独立例外。通过才运行一轮有预算PNR，任何失败留证据并停止，不自动无限重试或授予部署。


`prove_row_address_alias.py`只允许SLF的rb_ra[0:1]因综合改名：要求实际唯一LUT3、全部引脚、INIT8'hCA/8组真值及唯一上游寄存器的全部输入/时钟/使能/reset/参数一致；其他计划驱动继续严格比较。`test_row_address_alias.py`使用保留的实际网表，两项真实别名通过、四项行为/范围变化拒绝。不是删除驱动核对或给时序新增例外。初次接续失败目录保留，修复后用新state重查既有SYN，只启动尚未创建的目标PNR，避免重复综合或覆盖现场。


`prepare_bias_block_rom.py`只把冻结根fc_v4_top.v的425×26有符号bias_mem两项属性从distributed改block，原文件/数值/声明位宽/读使能/1拍延迟/MAC状态机保持。`measure_bias_block_rom.py <frozen-rowbanks-SYN> --out <new-dir>`以全部425个原始32位值为oracle，配对小模块SYN及RTL/厂商真实INIT检查：ROM16/LUT/FF 702/417/26→0、1个BIT_WIDTH32 pROM（宽拆分预测2物理块），最大负载702→7；两类检查各2968拍通过。首次有效读前原输出未定义，不强改INIT。`retry_frozen_synthesis.py --bias-block-rom --start-now`仍使用40分钟/既有内存保护；接续必须证明原BSRAM叶/参数全部保持、仅偏置新增、物理增量预测恰2且总≤336、全量ROM16消除，原行缓存/选择负载、104资产、控制合同/时钟/CDC仍重核。物理成本和收益只按完整PNR报告，不凭小模块或预测通过。

偏置映射的完整PNR实际2950条未布通/336块，较正确Lane2行缓存版本586条/334块退化，已拒绝续用，保留生成器和失败证据。下一轮`clock_resource_plan_lane2_telemetry.json`直接复用586条版本的完整SYN，不重新综合或改RTL/SDC/CSR：基于该轮物理962数据脚/960 CE脚，增加两条遥测BUFG请求，并将303/365脚的两条导航LW网转LOCAL，原8项约束保持。该配置区别于旧Lane0两CE失败对照，仍是待物理验证的分配假设。

`continue_bank_select_once.py --synthesis <已完成SYN> --baseline <原静态CDC参考> --clock-resource-plan <计划> --pnr-out <新目录> --state-dir <新目录> --max-runtime-minutes 150`在源已完成时无需提供失效PID；未完成仍须指定现有监管PID。新计划必须逐字绑定正确Lane2参考的原SYN网表，全部12条目标唯一驱动/104资产/存储参数/77条已知SYN CDC仍检查。分配器可额外审计报告中未列全局的具名目标，缺少全局分配明确记为null；CE与数据请求必须分别匹配实际CE或LUT输入，LOCAL拒绝时钟和复位脚。`test_measured_clock_plan.py --out <新目录>`用真实冻结网表检查一项有效计划及四项角色/复位/重复目标拒绝；原时钟安全回归保持。请求LW不代表实际获得LW，必须核对下一轮最终报告。

遥测分配轮实际544条未布通/334块，两条遥测确实LW，原四真时钟及四控制PRIMARY保持；仍无.tr或逐网失败名单。新准备配置`clock_resource_plan_lane2_telemetry_observation.json`基于该实际报告：观测网3892 CE＋1 D未列全局，以CE|LOGIC请求；395脚的导航LW改LOCAL，前12条约束保持。该14目标配置尚未启动，须在2GiB物理空闲/4GiB提交余量后重做完整前置核对；不把准备完成当作约束已落实。用户已关闭定时监督，准备记录不包含新的调度或后台构建。检查旧构建是否退出时同时核对PID与创建时间，避免把操作系统复用的PID当作原工具进程。


### 精确观测复用与提前准备（2026-10-24）

`prepare_observation_reuse.py`只接受既有轮速流水/initialized数据扇出属性的冻结观测源；保存原始输入并共享精确缩放，`prefetch=True`在既有状态边沿准备操作数，保持所有输出拍。它保留原64步共享除法器、轮速64位路径、舍入/饱和与28当前值/五帧历史。普通24位比例最大13的全范围证明允许精确28位表示，不是量化。

`measure_observation_reuse.py <frozen-SYN> --out <new-probe> --prefetch --reuse-baseline <verified-original-probe>`进行四符号RTL逐拍对照及配对模块SYN；`--resume`仅复用已验证且源码/包装相同的结果。`--retry-preflight`允许一次保留现场的设置回读重试，不重复已完成SYN。实际提前准备探针56000拍/主实例32帧通过，模块LUT−395、ALU＋101、FF−77、MULTALU27X18为16→4、BSRAM0，时钟估计75.317→81.542MHz。实例数65333超过本地Vivado BASIC50000许可，候选网表对原RTL仿真未启动，记录为验证缺口；不能仅凭退出状态或RTL通过宣称网表验证完成。

`retry_frozen_synthesis.py <frozen-SYN> --out <new-build> --start-now --observation-reuse-prefetch`要求该原始探针的实际RTL/资源证据，生成与已测候选逐字相同的单RTL差异，核对其他输入并进行一次40分钟联合SYN。它不自动PNR，不恢复定时监督；新综合网名/驱动、存储参数、CDC和真实Lane2时钟必须重新证明，接续器现有专门的新源检查分支；必须通过精确差异/资源/存储/驱动/CDC检查后才运行PNR。联合资源和PNR结论以实际报告为准。见[结构简化与证据](../../docs/4_design/g2_structural_simplification_20261024.md)。


`clock_resource_plan_lane2_obsreuse.json`保留正确Lane2的544条物理参考的12项分配，只将观测复位名n331_7映射到n971_7。`revalidate_observation_routing.py`证明这一个具名LUT4别名的全部输入、INIT16'hEFFF/16项真值、四个唯一上游驱动/参数；其余11个目标和四真实时钟源严格相同。应用器只在该证明通过后借用原物理网络的负载证据，不能用任意别名跳过检查。`test_observation_routing.py --out <new-folder>`的实际正例和7项负例已通过。

`continue_bank_select_once.py --synthesis <obsreuse-SYN> --baseline <fixed-CDC-reference> --pnr-out <new-dir> --state-dir <new-state> --clock-resource-plan <obsreuse-plan> --max-runtime-minutes 150`只运行一次；`validate_observation_reuse.py`重计全部26＋51已知CDC、实际Lane2 RX/TX时钟，保留原存储/104资产与所有功能。自动名变化的36个雷达RESET连接仍需已有完整驱动/模块INIT证明。完整网表仿真许可缺口和43额外跨域/PCS/复位/IO继续待签核。源检查通过不能代替PNR的真实资源、时序、分配与154条有效命令；本次不恢复周期监督，也不进行硬件操作。
