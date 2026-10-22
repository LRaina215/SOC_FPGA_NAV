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

`--command-lease`加入NAV唯一owner的命令年龄/有界减速/实测停稳监管原型；原4500继续维持平衡。入口RTL与24项CRC/坏帧/断线/切换等合同见`test_command_lease.py`，默认500ms停稳窗口在合同测试中缩放到10ms。外部旧UART协议没有序号/session，不能因此声称旧协议具备防重放。资源结论须归到带该标志的共同工程。

`--observation-pipeline`是隔离的轮速分子时序候选，生成器位于`control/tools/g2_observation_pipeline.py`。保持算法/数值及外部观测周期，仅写构建副本；原控制文件仍保留。`test_observation_equivalence.py`提供与原RTL四组符号、32帧逐周期比较，实际共同资源与时序另验。未指定标志时使用原观测RTL。

`--command-units-pipeline`（要求`--command-lease`）将NAV单位换算改为精确四级流水，正常接收多4拍/80ns，序号同步、停止/看门狗直接优先并清空在途命令。`test_nav_command_units.py`穷举两轴全部16位输入并验证连续输入、序号和复位/故障；不改变原4500算法、网络、权重或125Hz推理。

`--coverage-static-bounds`提前计算覆盖层每帧固定的窗口基点/边界，保留32位回绕算术、全部状态和外部周期。AS可指定相同标志用于同源码对照（覆盖层仍不实例化）。`test_coverage_collision.py --static-bounds`比较原/新模块所有外部信号与有效RAM端口每拍一致，沿用独立模型及碰撞X读值；`measure_coverage_timing.py`仅测模块综合。模块Fmax改善不等于共同工程或PNR通过，当前在跑的双流水配对没有此标志。

`--fusion-norm-pipeline`由`control/tools/g2_fusion_pipeline.py`生成BMI模长寄存边界调整版，滤波、标定参数及所有外部周期保持；`test_fusion_equivalence.py`对照完整原融合模块和24位极值。首种carry-save写法综合器内部错误已保留，当前使用显式扩展加法。原控制源不覆盖。

`--pose-predecode`在原总线捕获拍寄存位姿提交译码，替换帧接收和覆盖层的两个消费者；点流延迟、帧归属不变。`test_pose_predecode.py`在实际NAV总线状态机内与未改寄存器文件脉冲逐拍比较。

本工程统一使用器件C版、RTL核心50MHz，PNR SDC定义20ns核心与10ns接收时钟。截至directed_local，实际综合/全局提示均100MHz，旧`-frequency 50`设置未生效。脚本现改为文档支持的`-global_freq 50`，并在昂贵构建前用`saveto -all_options`回读频率和宽RAM选项，失配即停止；实际报告另读PRJ/cmd.do核对。所有PNR核心SDC约束仍为20ns，综合Fmax不能代替PNR。实板B/C对应、完整IO时序和BMI重力到导航姿态/时间适配尚待确认；旧导航IMU UART仍保留在A15，本工程不代表G3功能或实车通过。

存储优化保持原参数数值与推理调度，不量化、不跳MAC、不改训练。控制权重40个1K×16银行、ELU13bit精确常量尾、偏置26bit；导航仅改变大RAM物理分银行，容量、初始化、读前写、单拍读及保持语义不变。原始文件始终保留。

当前结论与原始路径见`docs/4_design/g2_joint_resources_20261022.md`，唯一当前状态表在`docs/1_plan/system_integration_plan_20261021.md`。

`collect_pnr_result.py`仅读取实际PNR报告，保留小数DSP用量；布线失败时记录未布通网络和缺失的时序，不用SYN频率填充Fmax。例：`python integration/g2/collect_pnr_result.py <build> --out <result.json>`。

签核还检查已报告的Recovery、Removal及最小脉宽最差表；存在负裕量或缺失表时不宣布G2通过。表中负数计数仅为工具报告出的行，不当作全部违例端点数。IO/CDC与器件版本完整签核继续单列。

完整构建与冻结网表PNR会保存`g2_gate.json`，并按实际报告返回G2门槛结果；厂商工具返回0或产生位流仍可能时序失败。仅综合阶段的成功不代表G2接受。采集器区分域内与全局违例，跨域违例保留；目前332块首次布通版核心26.313MHz，G2不通过，禁止按该位流部署。
