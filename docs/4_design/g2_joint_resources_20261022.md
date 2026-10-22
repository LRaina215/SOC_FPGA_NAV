# G2原4500与导航共同工程资源记录

2026-10-22，离线构建，未烧录。旧五组方案均未通过：四组在PNR映射时BSRAM超限，完整帧缓冲SSRAM方案在综合时逻辑超限。接续已定位开销、实现两侧无损布局和精确流水，当前仍没有成功布局布线或联合Fmax；尚无G2接受候选，文末列出实际运行状态。

源码入口：[integration/g2](../../integration/g2/build_g2.py)。固定控制为6443f56，实际RTL/ROM已保存到integration/control4500/original_src；权重数值、网络、训练与125Hz推理不变。检查点cd9b816，分支codex/integration-g2-20261022，main未移动。

统一目标为GW5AT-LV138PG484AC1/I0、版本C、核心50MHz、RGMII接收100MHz。实板B/C对应未确认。两种导航使用同一顶层、控制、器件和约束，仅覆盖层及相应固件不同；AS表示当前RTL关闭覆盖层，不冒充旧历史AS位流的逐字节重建。

保留实际BMI088采集/融合、原4500 ROM/推理、编码器、双轮PID、四舵机、OLED/红外及遥测。导航SAFE经单位适配进控制目标，实际编码器/角速度回导航。完整BMI姿态与统一时间适配仍属G3；旧导航IMU UART输入在A15保留，避免与舵机C18/C19冲突。本次不是功能整车验收。

| 运行 | 范围 | 当前结果 | 原始证据 |
|---|---|---|---|
| raw_as / raw_coverage | 初次脚本尝试 | 子进程GBK读取UTF-8失败，未综合，入口已修复 | F:/nav_integration_g2_20261022/raw_*/firmware.log |
| raw_as_v2 | AS＋原4500，覆盖关 | 综合339，PNR映射需求354，超过340拒绝布局 | F:/nav_integration_g2_20261022/raw_as_v2 |
| raw_coverage_v2 | AY覆盖＋原4500 | 综合339，PNR映射需求351，超过340拒绝布局 | F:/nav_integration_g2_20261022/raw_coverage_v2 |
| banked_both_as / coverage | 双方无损分银行 | PNR需求378 / 373，均否决 | 同证据根目录下同名目录 |
| ssram_as50 / coverage50 | 完整代价图SSRAM＋控制精确布局 | PNR需求368 / 365，均否决 | 同证据根目录下同名目录 |
| hybrid_as50 / coverage50 | 分银行＋代价图/触碰位图/完整帧缓冲SSRAM | 综合逻辑165167 / 166375，超138240；未进入PNR | 同证据根目录下同名目录 |
| planes_lease_as50 / coverage50 | 高程21位元数据SSRAM＋32位高度BSRAM，原控制布局，命令监管开启 | PNR需求368 / 362，超340，否决 | 同证据根目录下同名目录 |

首次联合综合已完成，**下表只是综合映射，尚非PNR实际占用**：

| 同一共同工程 | BSRAM综合值 | 导航u_nav | CPU/控制台u_soc | 控制u_control | SSRAM | LUT |
|---|---:|---:|---:|---:|---:|---:|
| AS、覆盖关闭 | 339 | 215 | 68 | 56 | 956 | 49345 |
| AY覆盖开启 | 339 | 221 | 64 | 54 | 1999 | 53053 |

覆盖模块本身映射9块BSRAM；同代码的控制OLED、CPU控制台等在两次全局综合中转移了BSRAM/SSRAM分配，不能将总数相同说成覆盖零成本。真实推理core在两者中均保留54块综合BSRAM，策略没有因输出不接而消失。双方完整层次见[evidence](evidence/g2_20261022/raw_as_v2_synthesis_hierarchy.json)及[覆盖层次](evidence/g2_20261022/raw_coverage_v2_synthesis_hierarchy.json)。

继续PNR后工具PA2017明确报告：AS映射需求354块、覆盖版351块，均超过器件340硬上限，拒绝布局。它们是实际映射后的资源需求，**不是成功布线后的占用或时序通过**。原始共同工程G2不通过；不能只根据综合339宣称可放下，也不能用两个总数差值推断覆盖减少资源。

时钟注意：两者核心RTL参数为50MHz，PNR的SDC为clk50/20ns及独立接收时钟100MHz。Gowin综合报告采用默认100MHz估计，综合Fmax不能作为50MHz联合PNR时序结论；最终以PNR读取约束及报告逐项核对。

按分解实现了两侧无损存储原型：导航大RAM按器件宽深模式分银行，保持容量、读前写语义、单拍读与保持；控制权重按40个1024×16银行，ELU用13bit和2307项后的精确常量尾，偏置按原数值26bit保存。未跳过MAC、未量化、未改推理周期、权重数值或训练。

`storage_equivalence`已检查全部40704权重、3073个ELU值和425偏置精确恢复；实际RTL的3组完整168维输入比较1275个神经元结果、六动作、错误/饱和/帧ID和逐周期时序一致。5种RAM几何各196608拍同输入比对，覆盖全地址、非2幂深度、同址读前写及re保持。见[证据](evidence/g2_20261022/storage_equivalence.json)。这是数值/RTL等价，不代替共同资源、时序和实车控制验证。器件手册还限制实际BSRAM同址读写，RTL比对不能作为原语碰撞已验证的证据；调用方前递和读写使用合同仍须核对。

分银行后的实际结果：AS的PNR映射需求378、覆盖版373，均超限，较原始方案更差，**已否决作为资源优化**。容量位数下降和RTL等价不能推断实际节省。失败原始目录`banked_both_as`、`banked_both_coverage`保留；层次分解确认大高程RAM53→48、结构图72→64只是局部分配减少，全工程其他映射变化抵消并反转了收益。

代价图SSRAM版保持原导航大RAM推断，只将两张128×128字节代价图转为分布式RAM，并保留控制的精确数值布局。地图容量、接口及周期不缩减。`ssram_equivalence`通过全部数值及3帧/1275神经元/5种RAM各196608拍RTL对照，但实际PNR需求AS368/覆盖365，已否决；综合仍339，不能据此记节省。

后续hybrid又将完整19200×16帧缓冲与65536位触碰图转SSRAM，其余大RAM分银行。8种RAM几何/相同196608拍及策略对照通过，但实际综合报逻辑165167/166375，均超138240，未进入PNR。实际SSRAM13072；非2幂帧缓冲的推断成本不能用净比特数简单估计。失败目录、数值检查与工具报错全部保留，见[AS证据](evidence/g2_20261022/hybrid_as50.json)及[覆盖证据](evidence/g2_20261022/hybrid_coverage50.json)。

planes仅拆53位高程格：32位完整高度仍在BSRAM，21位元数据完整存SSRAM；其他RAM及原控制布局保持原推断，容量、初值、读周期和原4500参数不变。拆平面5种RAM几何/196608拍RTL对照已过。共同工程同时接入`--command-lease`原型，实际路径保留，但实际PNR需求AS368/覆盖362，仍超限，已否决。见[AS完整层次/错误](evidence/g2_20261022/planes_lease_as50.json)及[覆盖层次/错误](evidence/g2_20261022/planes_lease_coverage50.json)。与其配套的24项命令故障注入及终点配对见[专项报告](endpoint_stop_20261022.md)，物理停止未验证。

例如覆盖版高程RAM局部综合53→32块，控制core却54→57、轨迹/网络等小RAM也改变映射；全局仍报339，实际需求反而362。仅靠局部容量差或总数差无法推断物理收益。当时尚未隔离综合到PNR的额外开销；接续已由实际网表和宽RAM拆分设置逐叶解释，见下文。旧五组均没有实际通过的50MHz时序报告。

ssram、hybrid、planes和directed_local脚本曾写综合频率提示50MHz，但接续查明`-frequency`未生效，实际PRJ及PNR全局提示仍100MHz；PNR SDC一直是相同20ns核心/10ns接收约束。历史100MHz综合报告按实际口径保留，不能称提示已生效。后续已改为文档中的`-global_freq 50`并增加有效配置回读，不能与历史混算时序收益。`collect_evidence.py`保存工具原始错误/层次/构建清单，不用宏数量代替PNR实际值。

AX独立导航334/340与综合估计309均不能当整车数值；核心时序、异步域与未标定IO约束分开。

## 2026-10-22接续：已定位宽RAM拆分选项

实际`impl/pnr/cmd.do`带有`-convert_sdp32_36_to_sdp16_18`。安装版[SUG1220-2.1E](https://cdn.gowinsemi.com.cn/SUG1220E.pdf)3.3节说明：开启后，一个32/36位SDPB/SDPX9B被拆成两个16/18位存储。对原覆盖版，净表339个BSRAM叶子中有12个宽叶子，339＋12恰好351；分银行覆盖版有34个宽叶子，对应373；元数据覆盖版有23个，对应362。

审计入口`integration/g2/audit_netlist_memories.py`展开实际网表层次，逐叶记录原语/位宽/路径，按实际PNR命令核对差额；不是按RTL容量猜测。报告见[八次构建归因](evidence/g2_20261022/wide_split_diagnosis.json)。原先“综合339不能解释额外开销”的状态由这项证据更新：已找到确定的映射设置来源，不把失败归因于权重或导航容量本身。

`build_g2.py`现显式设置`--wide-split 0`；同时`pnr_from_netlist.py`可对完全不改的旧网表、同一器件/引脚/SDC只改变该选项做PNR配对。原AS/覆盖的关闭拆分版已经越过此前的BSRAM映射拒绝。旧数据与失败构建全部保留。

接续的`directed_lease_as50/coverage50`从仓库内`control/fpga/src`加受检适配补丁构建；两侧同时显式设置大小存储类别，导航大地图按原容量分银行、小缓存固定分布式，控制权重/ELU/偏置保持数值与逐拍调度。3组168维输入/1275神经元对照及8种RAM几何各196608拍通过，实际综合/PNR进行中，不把位数预算当实测结果。

全局强制distributed_rom的第一次directed覆盖版触发工具SP00019（ROM推断内存错误）；同参数AS在推断中取消，没有伪记其资源结果。保留`directed_lease_*`，移除全局RAM/ROM风格强制，改为仅逐存储属性后，重建目录为`directed_local_as50/coverage50`。故障是工具推断失败，不能当成逻辑资源测量结论。

另用安装版gw5a原语模型实测32/36位原生存储与两块16/18位组合：8192拍，与独立按字节使能模型同时比较，读使能保持和部分写均通过。同址碰撞按器件手册排除；这是厂商功能模型验证，不是SDF或上板测试。

逐存储属性版综合已完成：AS306、覆盖314，CPU64、控制48、导航194/202。两版均被PNR接受映射并进入布局，仍待完整布线实际占用和50MHz时序；不以综合数签核。此版实际综合提示100MHz、SDC核心50MHz，后续preflight已独立验证正确参数可回读50.000与wide-split0。

### 原网表关闭拆分的实际结果

`native32_raw_coverage_v2`于16:05结束，实际BSRAM339/340、Logic74482/138240、寄存器37432、DSP165.5/298。PNR报`DesRoute failed`及PR0004：353条网络未布通，没有`.tr`时序报告。见[实际报告摘录](evidence/g2_20261022/native32_raw_coverage_v2_pnr.json)。此结果确认宽存储拆分的额外12块已消除，但仍高于336工作门槛且布线失败；没有联合Fmax，不能宣布G2通过。原AS配对在布线期间主动取消：其339块已不能通过工作门槛，且可用内存不足1GiB，释放内存给逐存储候选；[取消记录](evidence/g2_20261022/native32_raw_as_v2_cancelled.json)不冒充PNR失败或时序测量。逐存储属性AS/覆盖仍在布线。失败并未列出各未布通网络，不能据此断言某个具体模块造成拥塞。

### 保持外部周期的观测流水候选

当前SYN的长路径指向编码器速度增量到观测轮速分子的常量乘法（47.420ns数据到达，53级）；这是综合估计，不是实际PNR时序。候选生成器`control/tools/g2_observation_pipeline.py`保持原4500算法与数值，把轮增量捕获、两段部分积及精确求和分拍，在既有串行观测转换消费轮速前完成。原始控制源未覆盖，G2只在指定`--observation-pipeline`时使用构建副本。

修正后的刺激覆盖每帧交替正/负极值、零/-1和随机轮增量；四组符号配置、56000时钟、32完整帧的current/history/valid/initialized逐拍相同，同时覆盖忙时capture与中途复位。见[有限RTL等价证据](evidence/g2_20261022/observation_pipeline_equivalence.json)。它不是全域形式证明或实车验收。`directed_pipe_coverage_effective50`使用有效配置回读50MHz、宽RAM拆分0，已启动完整联合构建；结果待报告。与历史提示100MHz版不做单因素时序收益归因。

### 存储碰撞的调用方核对边界

源码核对确认：`elev_map`和`support_coverage`的点更新有三级前递；覆盖层CPU查询/滚动/老化等待点流水排空，CPU RAM是写拍读保持的单口写法。它们有助于隔离原语碰撞，但不能扩展成所有调用已证明安全。`elev_map.cpu_go`只屏蔽当拍点接收、清除和扫描，没有直接要求`!busy`，CPU读可能与末级写回并行；HAL在读前后检查busy/帧号/地图代际，但这不单独证明拍内竞争不可发生。投影LUT写入与点流读取的上层互斥也需核对。这些是源码审查边界，不是已重现的硬件错误，本轮没有为掩盖它们修改地图或调度。

覆盖层补了针对性读值毒化试验：实际16384×9逐存储实现，同址读写时强制读值X，再与既有独立CompactCoverage的251查询对照；同捕获epoch重复已收到的点不改变distinct-epoch模型计数。16183点、1306次碰撞全部通过，验证这组点更新/查询使用场景能屏蔽未知读值。最初未加强刺激时仅2次碰撞的结果仍保留，正式摘录使用加强版。见[证据](evidence/g2_20261022/coverage_collision_poison.json)与`test_coverage_collision.py`；不扩展为全部RAM调用或硬件碰撞读旧值保证。

### 导航命令单位换算流水候选

有效50MHz的观测流水版综合仍314块BSRAM；最长综合路径转为`u_nav/u_sg/out_wz`经48位单位除法到命令监管器，54级、45.705ns估计到达，SYN估计21.923MHz。见[实际SYN摘录](evidence/g2_20261022/directed_pipe_coverage_effective50_synthesis.json)。综合中的接收时钟提示也跟随全局50MHz；PNR仍使用相同SDC的100MHz接收时钟，不混用两个报告的约束或把该估计当PNR结论。

`nav_command_units.v`保持原对称最近舍入：`floor((abs(x)*512+62)/125)`再恢复符号，用25×17部分积、移位和余数修正替代组合除法。`67109*125=2^23+17`，对本范围的倒数误差小于1，商至多偏大1，比较`q*125`精确修正。四级流水每拍可接收一条；数值/序号同步，复位或SAFE停止/看门狗清空所有在途valid，原始停止/故障仍直接进监管器。正常接受相对原组合适配增加4个50MHz核心周期（80ns），200ms有效期不延长。

两轴各65536个16位输入的穷举均与原64位`(x*4096 +/-500)/1000`逐项一致，含17316次倒数修正；连续输入和65590个输出帧、8次在途清空通过。见[接口合同](evidence/g2_20261022/nav_command_units_contract.json)。这只改变接口实现，原4500观测合同/ROM/权重/125Hz调度和监管器阈值不变。构建标志`--command-units-pipeline`要求`--command-lease`；`directed_pipe2_coverage_effective50`已启动，实际资源/时序仍待报告。

### 双流水综合与同网表拆分配对

双流水覆盖版综合314块，实际回读50MHz，控制48、CPU64、导航202均保留。与同50MHz的观测流水版相比，只增加精确单位适配，最坏综合到达45.705→28.413ns、SYN估计Fmax21.923→35.566MHz；新最坏路径是覆盖层中心坐标到维护RAM写入。该收益仅归综合估计，不作为PNR签核。见[双流水SYN](evidence/g2_20261022/directed_pipe2_coverage_effective50_synthesis.json)。

双流水源网表有314个BSRAM叶子。初次预测只计了导航17块宽SDP，漏掉控制IK的一块36bit只读pROMX9，错误预计331；审计器已修正为覆盖所有宽存储物理模式。实际18块宽叶子逐对对应后PNR新增18块（7个SDPB、10个SDPX9B、1个pROMX9），全部原314个实例仍在，314＋18＝332。控制IK ROM36→两块18，未更改其表值或算法。见[逐实例实物理网表核对](evidence/g2_20261022/directed_pipe2_split1_physical_memories.json)。

固定同一网表、器件、引脚、SDC、50MHz提示及复制关闭，拆分0实际314块/502未布通，拆分1实际332块/411未布通。两者均通过≤336存储占用，却都PR0004失败，无时序报告或联合Fmax。开启后的模块分解为CPU64、控制49、导航219；与源综合的64/48/202分开记录。不能把这次未布通数下降当作布线成功或统计通过率。

为避免16GiB机器内存压力，AS逐存储版及中间单观测流水版主动取消剩余PNR，保留源码/综合/日志及取消记录。它们没有物理占用或Fmax结果，不记为布线失败；先集中完成容量更大的覆盖版，再构建可行配置的AS对照。

| 接续运行 | 实际综合 / 提示 | 本轮状态 |
|---|---|---|
| native32_raw_coverage_v2 | 339 / 100MHz | PNR实测339，353未布通，失败无时序 |
| native32_raw_as_v2 | 339 / 原冻结网表 | 因已超过工作门槛取消，未取得PNR占用/时序 |
| directed_local_as50 | 306 / 实际100MHz | 因内存与优先级取消PNR，仅保留综合值 |
| directed_local_coverage50 | 314 / 实际100MHz | 可用内存再次不足1GiB，因已被双流水配对取代而取消PNR；没有物理结果 |
| directed_pipe_coverage_effective50 | 314 / 实际50MHz | 单观测流水，中间版取消PNR |
| directed_pipe2_coverage_effective50 | 314 / 实际50MHz | 双流水拆分0，实际314，502条未布通，无时序 |
| directed_pipe2_coverage_split1_effective50 | 复用上一网表 / 实际50MHz | 双流水拆分1，实际332，411条未布通，无时序 |
| directed_static_coverage_effective50 | 314 / 实际50MHz | 加静态边界，仅综合完成，估计43.664MHz，非PNR |
| directed_static_coverage_rep1_effective50 | 复用静态边界网表 / 实际50MHz | 拆分1、复制1，实际332，布通；核心26.313MHz/5003 setup，全局8 hold，G2不通过 |

实际报告采集还要求clk50约束50MHz、rgmii_rxc约束100MHz及相应Fmax/违例检查；IO/CDC完整签核仍单列。没有报告时保留未知，不用综合或全局提示填充。

### 预计算静态窗口边界：隔离候选

针对双流水SYN长路径，`prepare_coverage_timing.py`在原有效帧切换拍预计算新窗口基点和旧窗口四边界，移出维护RAM使能前的重复常量加减。保留原32位回绕、所有状态跳转和存储操作拍；不延长维护/查询，不改变年龄、观测或支撑准入。原`rtl/map/support_coverage.v`仍保留，只在显式`--coverage-static-bounds`时生成到构建副本。

对原/新真实模块及逐存储RAM，每拍比较全部外部ready/busy/ack/数据和有效读写端口，同步用独立CompactCoverage模型核对251查询、16183点；1306次同址读值置X也通过。见[逐拍有限对照](evidence/g2_20261022/coverage_static_bounds_contract.json)。不宣称全域形式证明。

随后只做轻量模块综合，统一器件C、有效50MHz/宽RAM拆分0：

| 模块SYN | BSRAM | 寄存器 | ALU | LUT | 估计Fmax | 最坏数据到达 |
|---|---:|---:|---:|---:|---:|---:|
| 原覆盖层 | 8 | 436 | 228 | 738 | 54.836MHz | 18.532ns |
| 静态边界候选 | 8 | 546 | 502 | 301 | 112.120MHz | 9.233ns |

见[模块原始综合摘录](evidence/g2_20261022/coverage_static_bounds_synthesis.json)。增加110个寄存器和274个ALU、减少437个LUT，不称零资源代价。孤立模块的54.836MHz与联合上下文的35.566MHz本就不同，不能把112.120MHz写成联合速度。原双流水0/1配对未被此候选改写。

完整静态边界联合综合现已完成：314块（CPU64/控制48/导航202），LUT52931、ALU9714、寄存器38055；相同有效50MHz提示下，SYN估计35.566→43.664MHz、最坏到达28.413→23.216ns。新最坏路径是原BMI融合的egy→sqrt_value，尚无实际PNR时序，不能据此宣称BMI部署失败或改变其算法。见[完整联合SYN摘录](evidence/g2_20261022/directed_static_coverage_effective50_synthesis.json)。

### 布线负载诊断与工具复制候选

实际失败网表直接输入端口引用计数：控制复位9101、导航复位6528、观测复位4183、策略快照CE4032、观测提交CE3896。见[原始网表负载摘录](evidence/g2_20261022/directed_pipe2_split1_pin_loads.json)。脚本只计非时钟输入引用，不归并assign别名；这不是411条未布通网络名单。本次报告没有逐网未布通身份，不能确认这些信号就是失败根因。

官方SUG1220说明`replicate_resources`可复制高负载资源以降低扇出、改善时序；基线实际关闭。`pnr_from_netlist.py`现支持SYN-only网表及显式复制选项，先回读有效配置再运行。对静态边界联合网表的复制1候选已启动，器件/引脚/SDC/50MHz和原4500算法/权重/125Hz不变；回读为50MHz、宽存储拆分1、复制1，实际PNR命令使用二进制选项`-replicate`（不是Tcl名`-replicate_resources`）。这是一项有负载依据的试验，不是已证实修复；是否复制何种实例、BSRAM是否仍≤336、布线及50/100MHz实际时序均待完整报告。

### 首次实际布通：存储通过，时序不通过

`directed_static_coverage_rep1_effective50`于17:57:40完成PNR并生成位流，工具返回0；**不代表G2通过，未烧录**。相同器件C、引脚和20ns/10ns SDC，实际BSRAM332/340、Logic89584/138240、寄存器39957、DSP163.5/298。源314个存储叶子完整保留，18对宽叶拆分全部对齐，CPU64/控制49/导航219。见[实际PNR](evidence/g2_20261022/directed_static_coverage_rep1_effective50_pnr.json)、[物理存储](evidence/g2_20261022/directed_static_rep1_physical_memories.json)。

| 检查 | 实际结果 | 判定 |
|---|---|---|
| BSRAM工作门槛 | 332≤336，硬上限340 | 通过占用 |
| 布线 | 完成并产生实际时序报告 | 通过布线 |
| clk50 | 约束50MHz，实际26.313MHz，5003 setup违例 | 不通过 |
| rgmii_rxc域自身 | 约束100MHz，实际123.530MHz，域内setup/hold均0 | 仅该域报告通过 |
| 全局hold | 8条，最坏−0.211ns；全部为RX Gray指针到第一级同步器 | CDC及例外约束待审，整体不通过 |
| Recovery / Removal / 最小脉宽 | 最坏+3.354/+0.660/+2.843ns | 已报告表均为正 |

实际物理网表包含`gowin_replicate_reg`实例，寄存器相对源综合增加1902；但控制复位直接引用9153、导航6553、观测复位4366、策略快照CE4032、观测提交CE4001仍高，见[负载摘录](evidence/g2_20261022/directed_static_rep1_pin_loads.json)。不能声称复制降低了这些特定负载，也不能将成功独归复制：此版同时加静态边界，尚非单因素配对。

时序最坏实际路径为控制融合`egz→sqrt_value`，24级、37.915ns数据延迟、−18.004ns裕量。报告细分有绝对值进位、DSP平方和两段11.595/10.541ns长连线。导航`a_q→u_pp`另有37.028ns，路径经过位姿提交译码、帧提交和点流反压；不能只按终点DSP猜测深度参数需要再寄存（该参数原本已有寄存）。下一步同时减少实际组合链并评估时序优先布局，不能承诺单项即可过50MHz。

采集器现区分每个时钟自身违例和全局违例，8条跨域hold保留来源。整体门槛仍要求全局setup/hold为0；不因RX自身通过就宣布G2通过。`build_g2.py --stage all`与`pnr_from_netlist.py`写入`g2_gate.json`，实际时序失败时脚本返回失败，即使厂商工具已返回0。已用本次实际失败报告核验门禁拒绝。

### 控制模长与位姿译码的逐拍候选

`control/tools/g2_fusion_pipeline.py`只生成构建副本：原状态4/9寄存三个平方，下一拍由原sqrt引擎采样精确和。当前使用原绝对值/平方表达式及64位范围，最大和3×2^46，不改滤波/标定参数、状态/启动、原4500策略、权重及外部输出周期。

20万拍对照中，50MHz与100kHz参数各288完整帧，149/139次接受/拒绝加速度、64个全24位极值/随机模长向量、忙时输入和复位通过；逐拍比较全部外部输出、状态、sqrt_start及sqrt输入。测试标定样本数缩为4加速激励，生产仍原值。当前原表达式版复验见[有限RTL对照](evidence/g2_20261022/fusion_pipeline_equivalence.json)，冻结源码保存在其中标出的独立目录。

前两次有符号平方写法数学等价且RTL通过，但Gowin完整综合均在推断阶段SP00019：carry-save版报n7972、显式扩展加法版报n7429的10CSHIFT1无srcPin。第二次已没有carry-save，因此不能再将原因归结为该移位表达式；工具内部具体原因尚未证实。两次均无资源/时序数，见[第一次失败](evidence/g2_20261022/directed_fusion_carrysave_failed.json)、[第二次失败](evidence/g2_20261022/directed_fusion_signed_failed.json)，完整构建和源码全部保留。当前回退到原绝对值平方，仅保留寄存边界调整，先用`measure_fusion_timing.py`做原/新模块综合隔离，再决定是否重建联合工程。

`prepare_pose_predecode.py`只在真实总线寄存器分支捕获POSE_COMMIT译码，替换原两处消费者；原寄存器文件保留。与未改的真实寄存器文件输出在实际总线状态机内逐拍比较3091次、83个提交脉冲、541事务、2种复位通过，含不对齐别名、零数据写、读操作、特殊窗口和随机地址。见[脉冲对照](evidence/g2_20261022/pose_predecode_equivalence.json)。不插入点流延迟，也不改捕获epoch/帧归属。

组合候选`directed_fusion_pose_coverage_effective50`标志为directed＋lease＋observation/units/static-bounds/fusion-norm/pose-predecode，split1；已因上述第二次SP00019失败，未得到任何资源/时序数，不能单独据此判断位姿提前译码是否改善联合实现。

### 跨域约束审阅边界（未改SDC）

源码确认当前联合顶层：RX时钟直接来自外部rgmii_rxc，TX及未启用SFP的主机口使用clk50；RX复位经`nav_rst_sync`异步置位/两级同步释放。雷达RX FIFO两方向均是源域寄存Gray指针、目的域两级同步、第二级后解码；本次8条负hold均在`wp_g→wp_g_r1`，不是同步器第一级到第二级。另有MAC的`rdiv[5]→rsync[0]`三级链和`rx_neg→neg_s[0]`两级链，需要与FIFO一起列入CDC审查；不能仅处理报告中最坏的8条。

安装版`SUG940-2.0.5E`第37页给出例外优先级：false_path和clock_groups高于max/min_delay；第69–70页支持定向`-hold`例外及max/min_delay，没有列出`-datapath_only`或bus-skew命令。全文提取保存在`F:/nav_integration_g2_20261022/SUG940_local.txt`。因此后续应逐组定位第一同步级，保留Gray总线相对源周期的路由延迟/偏斜要求，保留第一级到第二级的正常分析，再检查实际命中对象和例外报告。不能照搬其他厂商命令，也不能用全时钟组屏蔽后误称Gray路由受max_delay约束。当前SDC仍只有两个create_clock，未加入任何掩盖现有失败的例外；这次只完成源码/手册核对，未完成CDC物理签核。

### 模长候选恢复原平方表达式后的模块测量

`fusion_retime_unsigned_synthesis`原/新模块综合均成功，避免将多模块联合综合内部崩溃继续当作算法数值问题。当前生成器保留原`abs16/abs24`平方表达式与64位上下文，只改变平方和的寄存边界；最新20万拍逐拍对照通过，冻结源码见RTL证据路径。

| 单独fusion及其sqrt/recip子模块SYN | 总寄存器 | 总ALU | 总LUT | MULTALU27X18 / MULT27X36 | 估计Fmax | 最坏到达 |
|---|---:|---:|---:|---|---:|---:|
| 原版 | 1883 | 1195 | 2495 | 12 / 18 | 81.473MHz | 12.588ns |
| 原平方寄存调整版 | 2011 | 1099 | 2632 | 12 / 18 | 105.162MHz | 9.823ns |

数字取综合XML的T_*总量，括号中的直属fusion数量另保留在JSON，不混用。器件C、有效提示50MHz、拆分0相同，DSP原语数量保持。新增128个寄存器、增加137个LUT、减少96个ALU，不称无资源代价。见[模块综合](evidence/g2_20261022/fusion_retime_unsigned_synthesis.json)。单模块原估计81MHz而联合实际26MHz说明布线/上下文影响很大，不能用105MHz承诺联合过50MHz。

最新联合SYN目录`directed_fusion_pose_unsigned_coverage_effective50`，相同directed/监管/观测/单位/边界/位姿标志，融合使用最新原表达式版，宽RAM拆分1，正在运行。前两次失败目录保留，不覆盖。

## 2026-10-23接续：动作映射的精确位宽

上次原平方表达式版联合SYN已完成，`directed_fusion_pose_unsigned_coverage_effective50`实际提示50MHz，BSRAM314（CPU64/控制48/导航202），18个宽叶保留，拆分后预测332尚非此次PNR值。SYN估计44.852MHz，最坏22.610ns为策略actions→actuator_map/rk_us。见[联合SYN](evidence/g2_20261022/directed_fusion_pose_unsigned_synthesis.json)、[逐存储清单](evidence/g2_20261022/directed_fusion_pose_unsigned_memories.json)。不能把与上一版43.664MHz的综合差异归为单个候选收益，也没有新的实际Fmax。

`control/tools/g2_action_map_widths.py`仅生成副本，保留原±4096裁剪、增益{-600,600,19,-19}、最近舍入、脉宽/轮速限幅、run/reset优先级与输出拍。裁剪后绝对值≤4096，|gain|≤600，乘积≤2457600，加2048后仍小于2^23，舍入后幅值≤600；原64位负数取绝对值/恢复符号可按该范围精确实现。生成器检查六处私有函数调用仍是原增益，失配就停止；不是量化或修改策略输出。

与完整原动作映射RTL逐拍比较：每个增益穷举8193个裁剪域值，另含24位正负极值、±4097、随机饱和输入、ff极值/随机数；8770激励帧、64个无效拍、64次run停止、复位及两组舵机零位参数通过，比较轮速/四路舵机/last_actions/have_action。见[RTL对照](evidence/g2_20261022/action_map_exact_widths_equivalence.json)。未枚举全部动作与ff组合，但ff脉宽运算本身未改，裁剪后的增益算术全域已覆盖；不计实车接受。

| 动作映射模块SYN（含194个共同输入启动寄存器） | 总寄存器 | 总ALU | 总LUT | MULTALU27X18 | 估计Fmax | 最坏到达 |
|---|---:|---:|---:|---:|---:|---:|
| 原版 | 375 | 128 | 751 | 6 | 50.137MHz | 20.259ns |
| 精确位宽候选 | 351 | 128 | 621 | 6 | 72.224MHz | 14.160ns |

见[模块测量](evidence/g2_20261022/action_map_exact_widths_synthesis.json)。最初直接将动作映射作为顶层且无输入时序约束，报告600.601MHz只量到last_actions反馈，不能代表动作输入→舵机路径；该目录保留但不采用其Fmax。有效v2测量在原/新双方都添加相同输入启动寄存器，最坏路径确为actions_q→lk_us；这些寄存器仅用于测量，不进入产品设计，也不为产品增加周期。

当前完整SYN：`F:/nav_integration_g2_20261023/directed_action_coverage50`，在上一联合源码上增加`--action-map-exact-widths`，其余directed/lease/observation/units/static-bounds/fusion/pose标志和拆分1保持。完成后按实际SYN决定是否进入place2＋复制1的PNR；目前整体仍是此前26.313MHz的实际报告，G2不通过。控制原文件和权重没有修改，未烧录。

### 26条同步器实际连接审计与定向约束候选

`audit_cdc_first_stages.py`对已经布通的静态边界物理网表检查：雷达FIFO写指针Gray到读域12位、读指针Gray到写域12位、MAC时钟监测与采样选择各1，共26条。每条直接由预期源时钟的寄存器驱动第一级D，第一/第二级为目的时钟、CE常1，第一级Q只有预期第二级D一个直接负载。Gray最高位实际合并到同值二进制指针最高位，按连接追踪而非只筛名字。见[26条逐路径审计](evidence/g2_20261023/static_cdc_first_stages.json)。这仍是四组白名单结构检查，不代表所有CDC或复位已签核。

隔离SDC片段只对这26对source-Q→first-D设置`-hold`例外，同时设置最大延迟：源RX10ns周期预算8ns，源core20ns周期预算18ns。保留setup的显式路由限制、第一级→第二级全部正常时序和复位检查；不添加clock_groups、setup false_path或multicycle。8/18ns是候选路由预算，不能据此宣称已完成Gray总线物理偏斜/MTBF签核。原`g2.sdc`仍未改，只有显式`--cdc-reference-audit`的冻结PNR副本追加片段。

先用三寄存器实际Gowin PNR验证命令语义，避免把工具不支持或未命中的约束带进昂贵联合布线。第一次误用_s0名字，实际寄存器为_s1，工具TA2003在布局前拒绝；该失败目录保留。v2按实际名字后PNR成功，3.8节两条约束均Actived；setup表仍有source→first且关系18ns，first→second的setup与hold均存在，hold表不再包含source→first。证据及极小源/网表/SDC见[工具语义检查](evidence/g2_20261023/cdc_sdc_fixture/result.json)。这个夹具仅验证约束行为，不能代表联合资源、车辆或全CDC通过；没有烧录。

`pnr_from_netlist.py --cdc-reference-audit <JSON>`仅接受上述固定分组/数量/预算，保留参考审计与片段到新构建。报告门禁要求新物理网表26条驱动/第二级/时钟与参考完全相符，52条约束全部出现在Actived表；任何缺失、额外未审例外或全局时序违例都使G2不通过。已确认原26.313MHz/8-hold报告在新门禁下仍被拒绝，未降低既有失败标准。

### 新动作映射联合综合完成，实际PNR已启动

`directed_action_coverage50`综合成功，实际全局提示50MHz、314个BSRAM叶子（CPU64/控制48/导航202），18个宽叶不变；拆分1仍预测332，不能在新PNR前冒充实际用量。核心SYN估计44.852→48.163MHz，最坏22.610→21.077ns，转到sensor/gyro_y→融合状态。源寄存器38196。见[完整SYN](evidence/g2_20261023/directed_action_coverage50_synthesis.json)、[存储逐叶](evidence/g2_20261023/directed_action_coverage50_memories.json)。RX综合仍跟随50MHz提示，PNR独立使用100MHz约束。

实际PNR目录`F:/nav_integration_g2_20261023/directed_action_place2_cdc50`，冻结上述网表，place2/复制1/拆分1均预检回读，附加参考26路径生成的定向CDC片段。源/第一/第二级所有引用寄存器已在新综合层次逐一找到，之后仍要求物理连接及52条Actived检查。已进入布局，没有TA约束错误；实际占用/布线/Fmax尚待报告，没有烧录。与旧26.313MHz的结果存在多项RTL和布局差异，未来不能按单因素收益表述。

### 16位权重的无损9位物理排布候选

原40704个权重范围−21855…20803，不能直接缩为15位。只有地址675、3055、11735的bit14不等于符号位，但这3个值也必须精确保留。新`control/tools/g2_weight_rom9.py`把低8位和符号位存入20个2048×9 ROM，把bit13:8按四权重组成24位、分成三条字节通道共15个2048×8 ROM；bit14以符号位和上述逐地址例外重建。没有量化、权重改值、附加读拍或MAC/125Hz调度变化；原控制源码/ROM保留，仅显式构建标志生成副本。

数值检查精确恢复全部40960个含填零地址。RTL检查只重测已变的控制存储，未重复原导航RAM测试：3帧、1275神经元、动作/状态/周期逐拍一致，另全扫40960地址并插入保持。见[RTL/数值证据](evidence/g2_20261023/weight9_equivalence.json)。

| 共同输入启动/输出采样夹具下的ROM综合 | BSRAM | 总寄存器 | 总LUT | 估计Fmax | 最坏到达 |
|---|---:|---:|---:|---:|---:|
| 原40×1024×16 | 40 | 669 | 393 | 230.103MHz | 4.660ns |
| 新低9位＋六位打包＋例外 | 35 | 153 | 244 | 221.337MHz | 4.832ns |

见[实际模块综合](evidence/g2_20261023/weight9_synthesis_comparison.json)。两侧均有效提示50MHz、宽拆分1，源夹具各33个寄存位，报告计数保留工具实际总量；不自行扣除优化/吸收的夹具寄存器。省5块是模块综合事实，新选择/拼接路径稍长，不能从此承诺联合时序。若其他映射保持，联合SYN314→309、拆分PNR332→327只是预测，尚未实际测量。

进一步直接加载Gowin新模块综合网表和GW5A厂商存储原语，核验INIT含第9位符号、所有40960地址、4096次随机跨银行读取及3745次禁用保持，45056次读取/48798次输出检查通过；流水夹具的延迟也与独立期望一致。见[综合后功能证据](evidence/g2_20261023/weight9_netlist_functional.json)，它不包含SDF或实板时序。新候选尚未加入当前正在运行的联合PNR。

首次新模块运行停在设置预检：有效配置已保存，但超过8分钟未返回，未启动综合；中止其测量父进程并保留目录。预检脚本现显式exit、仅此无综合预检限时60s，重新只测新模块成功，未重复已完成旧模块。不能将该中止记录写成RTL/综合失败。

联合SYN已启动：`F:/nav_integration_g2_20261023/directed_weight9_coverage50`，在前述动作映射/模长/位姿等候选上仅新增权重9位排布，器件C、有效50MHz、宽拆分1。固件镜像51712B/BSS69944B，与该覆盖候选的既有布局同口径；完整SYN/物理结果待测。新脚本在厂商run命令后同样显式exit以避免完成后滞留，不给综合或PNR设置60s时限；已启动的原冻结PNR脚本不修改。
