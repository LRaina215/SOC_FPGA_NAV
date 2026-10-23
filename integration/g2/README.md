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

`--route-option 0|1|2`分别为默认、更深搜索、较快布线，预检回读。当前长布线重跑采用`--place-option 1 --route-option 2 --max-runtime-minutes 60 --max-no-progress-minutes 35`，其余时钟/CDC/资源门槛不变。预算默认0表示不启用；对本任务新PNR必须显式指定。`tool_runtime.json`保存进程和预算，观察日志/中间产物变化，CPU忙或监管心跳不算进展；超限只终止本次启动的工具，保存取消说明并返回未通过。`test_bounded_tool.py`验证真实子进程生命周期，不跑FPGA或物理仿真。

`--capture-system-telemetry`可每30秒向`system_telemetry.jsonl`记录所属进程CPU/线程/工作集/私有内存/I/O及主机物理空闲/提交余量。依赖psutil，Windows提交值来自GlobalMemoryStatusEx；缺少观测记录错误，不伪造零值。系统观测文件明确排除在日志/产物进展集合之外，CPU、I/O或记录写入不会刷新静默预算；页错误累计也不能直接视为硬分页次数。当前一次延长诊断仍由总计划规定独立上限，不恢复定时监督。

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

当前结论与原始路径见`docs/4_design/g2_joint_resources_20261022.md`，唯一当前状态表在`docs/1_plan/system_integration_plan_20261021.md`。

`collect_pnr_result.py`仅读取实际PNR报告，保留小数DSP用量；布线失败时记录未布通网络和缺失的时序，不用SYN频率填充Fmax。例：`python integration/g2/collect_pnr_result.py <build> --out <result.json>`。

签核还检查已报告的Recovery、Removal及最小脉宽最差表；存在负裕量或缺失表时不宣布G2通过。表中负数计数仅为工具报告出的行，不当作全部违例端点数。IO/CDC与器件版本完整签核继续单列。

完整构建与冻结网表PNR会保存`g2_gate.json`，并按实际报告返回G2门槛结果；厂商工具返回0或产生位流仍可能时序失败。仅综合阶段的成功不代表G2接受。采集器区分域内与全局违例，跨域违例保留；目前332块首次布通版核心26.313MHz，G2不通过，禁止按该位流部署。

`audit_cdc_first_stages.py <flat-vo> --out <audit.json> --emit-sdc <fragment.sdc>`核对四组共26条已知同步器的实际驱动、时钟与第一级单扇出到预期第二级。`pnr_from_netlist.py --cdc-reference-audit <audit.json>`显式启用隔离的第一同步级hold-only及8/18ns最大延迟候选；原SDC保留。新报告要求26条物理连接与参考相符、52条约束全部Actived，出现其他未审例外即拒绝。小型工具实测表明第二级setup/hold及源到第一级的最大延迟仍计时；不代表完整CDC、IO、偏斜或MTBF签核。

`nav_count_feedback.v`为独立N3候选，尚未接入`g2_top`及正在运行的冻结PNR。固定名义65mm/1320CPR、10ms原始AB计数，左正/右负为前进；保留导航速度换算余数，并提供50窗计数变化的补充慢速证据、质量/超时/重叠/范围故障及粘滞连续性丢失。计数窗口不能观测门内往返、亚计数运动和打滑，不能单独确认物理停止；原PID/RL不受影响。`test_count_feedback.py`采用独立整数/分数模型，`measure_fusion_timing.py --module count-feedback`使用输入启动寄存器测量原型（测试寄存器不进产品）。接入及完整资源/时序、G3质量状态、原4500闭环和停止包络仍待完成。
