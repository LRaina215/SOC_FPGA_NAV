# SF完整资料补充审查

范围：当前references下完整资料包，重点2校准调试和5例程资料；课程目录的结论见SF_COURSE_REVIEW.md。已提取安装/调试DOCX文本和PDF文本，检查一页驱动原理图、Simulink可见主模型XML；图像型运动学PDF正文未逐页审阅，BOM工作簿未逐项核对。不代表全部文件都已验证。没有编译运行、烧录、改动现有FPGA参数或正在进行的训练。

## 模式语义已找到直接定义

[2校准调试/BLDC_Control/lib/SF_Motor/SF_Motor.h](</home/cyd/fpga/references/SF-轮足机器人资料0720/SF-轮足机器人资料0624/SF-轮足机器人资料0624/2校准调试/BLDC_Control/lib/SF_Motor/SF_Motor.h:21>)

VELOCITY_MODE=1，FORCE_ANGLE_MODE=2，VEL_ANGLE_MOED=3，TORQUE_MODE=4。setVelocity注释目标rad/s，setTorque注释“目标扭矩A”；名称与物理单位存在混用，不能把力矩模式数值当作Nm。SF_Communication.h第6行明确举例模式1下10表示10rad/s。

独立速度例程speed_S3设置setModes(1,1)、setTargets(2,2)，所以2是rad/s。力矩例程torque_S3设置setModes(4,4)、setTargets(2,2)，2是另一种模式的驱动目标。课程和整机bipedal使用mode4，因此其速度外环/倾角环数值不能直接挪给520的mm/s到PWM速度PID。

[5例程资料/主控板加小电流板/16电机速度控制/speed_S3/src/main.cpp](</home/cyd/fpga/references/SF-轮足机器人资料0720/SF-轮足机器人资料0624/SF-轮足机器人资料0624/5例程资料/主控板加小电流板/16电机速度控制/speed_S3/src/main.cpp:8>)

[5例程资料/主控板加小电流板/15电机扭矩控制/torque_S3/src/main.cpp](</home/cyd/fpga/references/SF-轮足机器人资料0720/SF-轮足机器人资料0624/SF-轮足机器人资料0624/5例程资料/主控板加小电流板/15电机扭矩控制/torque_S3/src/main.cpp:8>)

## 比简化课程更有参考价值的整机程序

[2校准调试/bipedal/src/main.cpp](</home/cyd/fpga/references/SF-轮足机器人资料0720/SF-轮足机器人资料0624/SF-轮足机器人资料0624/2校准调试/bipedal/src/main.cpp:472>)

- 高度增益调度：robotRun在高度70~100、100~110范围分别计算PID_VEL.P及姿态PD增益；I/D及横滚参数也随高度修改。支持“底盘升高后动力学改变”的处理思路，不可复制其拟合系数。大于110时部分P/Kp/Kd分支没有覆盖，不能把示例当作完整调度表。对RL可先按高度分组评估，而非部署端再叠加一个不在训练中的补偿。
- 平衡中心角校准：selfCaliCentroid第559行每约41次调用、平均轮速绝对值>1时，用负轮速修正中心角，每次限制±0.5度，中心角限制3±7度。该机制是找平衡点，不是陀螺零偏校准；其调用条件仅是前后遥控通道连续相同，并不等于零命令，因此不能直接复制到正常行驶时，否则可能把运动学成偏置。若研究此项应限于零命令、低角速度、可靠接地的稳定窗口，并保持仿真与部署一致。
- 腿与轮的前后命令协同：rampedForward每loop变化0.05；controlTarget.forward依据前进命令改变腿端X；coordTarget.x还加wheel_X_off=2.5。支持检查模型的中立姿态/质心与腿部配合，但loop周期未固定，系数不能直接变成FPGA时间常数。
- 角速度阻尼：姿态控制显式减PID_Stb.Kd*GyroY。这是角速度到平衡驱动的阻尼，不能与轮速PID的D混为一谈。RL已有角速度观测，应先检查策略反应，而不是任意额外加一条实车专用阻尼。

## 滤波没有找到可直接复用的速度时间常数

main声明LPFPitch{0.03}但未查到其有效调用；LPFRoll{0.05}实际滤的是横滚姿态误差。不能据“速度低通滤波”的注释认定本程序对轮速使用30ms滤波。SF_Motor头文件有velFlt/currentFlt成员，但内部实现仍在libSF_Motor.a，未找到可读的SF_Motor.cpp或编码器测速实现，无法确认速度滤波时间常数、内环频率或完整制动逻辑。

## 文档、原理图和模型的边界

调试文档V1.3给了舵机零位偏置、反馈方向SpdDir和控制方向Dir分别标定的完整流程。它要求配套舵机板7~7.5V，不能据此给不同型号舵机改电源。它的极对数7与无刷换相校准不适用于520有刷减速电机。

驱动原理图显示两套三相桥、三路相电流采样放大器和分流电阻。当前520方案是PWM/方向+AB测速，硬件接口不是同一种。完整资料能借鉴环路组织，不能自动带来相同电机响应。

18simlink的可见slx主模型以UDP收发、增益和Scope为主，配套还有受保护slxp模块；未发现可直接用于我们轮足RL训练的完整机器人动力学环境，不能将其等同IsaacGym仿真资产。

## 对当前项目的建议

当前10ms/P3训练继续保持不变。近期优先借鉴IMU启动真实重力初始化和按高度分组验证；后续才研究零指令平衡偏置自校准。课程PID参数、mode4力矩控制与其他舵机供电不直接移植。现有RL仍输出四路舵机动作+左右轮速度目标，轮速内环PID/PWM方案不变。
