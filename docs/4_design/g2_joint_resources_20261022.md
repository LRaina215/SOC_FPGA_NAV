# G2原4500与导航共同工程资源记录

2026-10-22，离线构建，未烧录。首次联合综合进行中，尚无整车资源通过结论。

源码入口：[integration/g2](../../integration/g2/build_g2.py)。固定控制为6443f56，实际RTL/ROM已保存到integration/control4500/original_src；权重数值、网络、训练与125Hz推理不变。检查点cd9b816，分支codex/integration-g2-20261022，main未移动。

统一目标为GW5AT-LV138PG484AC1/I0、版本C、核心50MHz、RGMII接收100MHz。实板B/C对应未确认。两种导航使用同一顶层、控制、器件和约束，仅覆盖层及相应固件不同；AS表示当前RTL关闭覆盖层，不冒充旧历史AS位流的逐字节重建。

保留实际BMI088采集/融合、原4500 ROM/推理、编码器、双轮PID、四舵机、OLED/红外及遥测。导航SAFE经单位适配进控制目标，实际编码器/角速度回导航。完整BMI姿态与统一时间适配仍属G3；旧导航IMU UART输入在A15保留，避免与舵机C18/C19冲突。本次不是功能整车验收。

| 运行 | 范围 | 当前结果 | 原始证据 |
|---|---|---|---|
| raw_as / raw_coverage | 初次脚本尝试 | 子进程GBK读取UTF-8失败，未综合，入口已修复 | F:/nav_integration_g2_20261022/raw_*/firmware.log |
| raw_as_v2 | AS＋原4500，覆盖关 | 联合综合中 | F:/nav_integration_g2_20261022/raw_as_v2 |
| raw_coverage_v2 | AY覆盖＋原4500 | 联合综合中 | F:/nav_integration_g2_20261022/raw_coverage_v2 |

首次联合综合已完成，**下表只是综合映射，尚非PNR实际占用**：

| 同一共同工程 | BSRAM综合值 | 导航u_nav | CPU/控制台u_soc | 控制u_control | SSRAM | LUT |
|---|---:|---:|---:|---:|---:|---:|
| AS、覆盖关闭 | 339 | 215 | 68 | 56 | 956 | 49345 |
| AY覆盖开启 | 339 | 221 | 64 | 54 | 1999 | 53053 |

覆盖模块本身映射9块BSRAM；同代码的控制OLED、CPU控制台等在两次全局综合中转移了BSRAM/SSRAM分配，不能将总数相同说成覆盖零成本。真实推理core在两者中均保留54块综合BSRAM，策略没有因输出不接而消失。双方完整层次见[evidence](evidence/g2_20261022/raw_as_v2_synthesis_hierarchy.json)及[覆盖层次](evidence/g2_20261022/raw_coverage_v2_synthesis_hierarchy.json)。两份工程正继续PNR验证，339已经超过336工作门槛。

按分解实现了两侧无损存储原型：导航大RAM按器件宽深模式分银行，保持容量、读前写语义、单拍读与保持；控制权重按40个1024×16银行，ELU用13bit和2307项后的精确常量尾，偏置按原数值26bit保存。未跳过MAC、未量化、未改推理周期、权重数值或训练。

`storage_equivalence`已检查全部40704权重、3073个ELU值和425偏置精确恢复；实际RTL的3组完整168维输入比较1275个神经元结果、六动作、错误/饱和/帧ID和逐周期时序一致。5种RAM几何各196608拍同输入比对，覆盖全地址、非2幂深度、同址读前写及re保持。见[证据](evidence/g2_20261022/storage_equivalence.json)。这是数值/RTL等价，不代替共同资源、时序和实车控制验证。双方分银行后的共同工程正在重新综合及PNR。

AX独立导航334/340与综合估计309均不能当整车数值；核心时序、异步域与未标定IO约束分开。
