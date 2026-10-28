# 模型轮速目标延迟与滤波

2026-10-19参数回退：main.v默认MODEL_WHEEL_FILTER_ENABLE=0，当前轮速直接进入原版10ms PID，以下为可选滤波模块说明。模型权重保持原版本。

显式启用MODEL_WHEEL_FILTER_ENABLE=1时，滤波模块顺序：7000模型原始轮速（1倍、±600mm/s）→2ms采样的五步延迟（10ms）→45ms一阶低通→当前100mm/s每10ms斜坡→PID/PWM。仅轮子模型通道使用，手动目标、舵机和模型上次动作观测保持原定义。

模块src/model_wheel_response.v保存Q12 mm/s滤波状态。系数2849/65536近似1-exp(-0.002/0.045)，每2ms更新。以首个采样为起点，延后五个样本才响应；相对异步输入有0..2ms采样相位差及3个50MHz时钟的流水延迟。乘法、状态累加、输出舍入分拍，未引入宽位组合除法。

失能、急停、模型失活或故障仍沿原路径直接切断PWM；滤波排队和状态同时清零，不等待低通自然衰减。raw target=0则正常滤波衰减，控制停机与普通零速请求不同。

V2串口left/right_target仍是原始模型目标；left/right_reference是PID斜坡后的参考（仅显式启用滤波时包含滤波影响）。现有帧未新增独立低通输出字段。看图时不可将raw target直接当作PID输入。

测试：tools/model_wheel_filter_validation/tb_model_filter.v对600组浮点参考验证误差≤1mm/s，覆盖10ms延迟、45ms响应、正反向、零输入、限幅、高频交替抑制、即时失能及流水/队列清除。tb_main_filter.v检查顶层PID接入、原始诊断保留、手动路径旁路与急停关断。

仿真演示：open_preview.sh --num-envs 1 --cpu --wheel-controller reduced_no_filter，仅去掉45ms滤波，仍保留10ms延迟及原2ms仿真速度P环；reduced为原滤波版。默认训练设置未改。此对照不是FPGA PID完整数字孪生。

本修改未重新生成.fs、未烧录；编译及实车验证前不能确认平衡改善。
