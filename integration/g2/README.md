# G2共同资源工程

固定输入：控制`6443f56`的真实RTL/ROM，导航检查点`cd9b816`。不会烧录。原始控制源码与数值保存在`../control4500/original_src`；`control4500_core.v`只添加导航命令入口和实际反馈，保持真实策略、BMI、编码器、PID、舵机与可观察输出。

```powershell
D:\Anaconda\python.exe -X utf8 integration/g2/build_g2.py --nav as --stage all --out F:/nav_integration_g2_20261022/example_as
D:\Anaconda\python.exe -X utf8 integration/g2/build_g2.py --nav coverage --packing planes --command-lease --stage all --out F:/nav_integration_g2_20261022/example_planes_coverage
D:\Anaconda\python.exe -X utf8 integration/g2/test_storage_equivalence.py --out F:/nav_integration_g2_20261022/example_equivalence
```

输出目录必须不存在。`--packing none|nav|control|both|ssram|hybrid|planes`比较存储布局；`ssram`转两张完整代价图，`hybrid`还转完整帧缓冲/触碰位图，均已超限否决。`planes`只把高程格21位元数据转到分布式RAM，32位高度仍为BSRAM，保留原控制布局。`--stage syn|all`区分综合与完整构建。硬上限340块BSRAM、工作门槛336；只有综合数不构成通过。

`--command-lease`加入NAV唯一owner的命令年龄/有界减速/实测停稳监管原型；原4500继续维持平衡。入口RTL与24项CRC/坏帧/断线/切换等合同见`test_command_lease.py`，默认500ms停稳窗口在合同测试中缩放到10ms。外部旧UART协议没有序号/session，不能因此声称旧协议具备防重放。资源结论须归到带该标志的共同工程。

本工程统一使用器件C版、RTL核心50MHz，PNR SDC定义20ns核心与10ns接收时钟。初次原始/分银行构建的综合报告采用默认100MHz估计；脚本已显式设置后续综合频率提示50MHz，所有PNR核心约束保持20ns。综合Fmax不能代替PNR。实板B/C对应、完整IO时序和BMI重力到导航姿态/时间适配尚待确认；旧导航IMU UART仍保留在A15，本工程不代表G3功能或实车通过。

存储优化保持原参数数值与推理调度，不量化、不跳MAC、不改训练。控制权重40个1K×16银行、ELU13bit精确常量尾、偏置26bit；导航仅改变大RAM物理分银行，容量、初始化、读前写、单拍读及保持语义不变。原始文件始终保留。

当前结论与原始路径见`docs/4_design/g2_joint_resources_20261022.md`，唯一当前状态表在`docs/1_plan/system_integration_plan_20261021.md`。
