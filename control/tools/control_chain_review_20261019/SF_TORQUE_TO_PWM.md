# SF力矩模式到PWM的可确认路径

本次只读检查公开源码、头文件和静态库符号/重定位，不修改驱动、训练或FPGA代码。没有反编译出完整函数，不能声称已得到生产驱动全部计算公式。

## 整机驱动调用关系

SF_Motor.h明确TORQUE_MODE=4。整机bipedal调用motors.setModes(4,4)，目标由速度外环→目标倾角→姿态PD得到，之后叠加差速项并限幅±5.7，通过setTargets传给驱动板。变量targetVoltage和接口注释“目标扭矩A”存在单位混用；不是神经网络直接输出经标定的Nm。

libSF_Motor.a中readelf重定位可确认：

SF_Motor::run → SF_Motor::setTorque(float) → getElecAngle + SF_FOC::setTorque(float,float,float) → sinf/cosf + SF_FOC::setPWM(float,float,float) →三次浮点除法 + _mcpwm_pwm_output(float,float,float,void*)。

本库setTorque函数未显示getCurrent/currentPID/滤波器等调用，不能把通用的“目标扭矩→Iq电流闭环→Uq”流程当作此模式已启用的事实。函数存在内联/算术分支等可能；未反汇编完整函数，因此不据符号独断它的所有缩放与限幅。literal中0.5常数本身也不能单独证明具体代码语义。力矩标量可能是电压近似或电流侧设定，不能宣称其单位为Nm。

## 公开教学例程里的电压调制

源码位置：/home/cyd/fpga/references/SF-轮足机器人资料0720/SF-轮足机器人资料0624/SF-轮足机器人资料0624/5例程资料/主控板加小电流板/14电流检测/current_S1/src/main.cpp:54，setPhaseVoltage在68行，PWM配置在30~32行。

示例给出：

Ualpha = -Uq*sin(theta)
Ubeta = Uq*cos(theta)
Ua = Ualpha + Vbus/2
Ub = (sqrt(3)*Ubeta-Ualpha)/2 + Vbus/2
Uc = (-sqrt(3)*Ubeta-Ualpha)/2 + Vbus/2
Da/Db/Dc = clamp(Ua/Ub/Uc / Vbus,0,1)
ledcWrite(channel,duty*255)

这是Ud=0的简化逆Park/逆Clarke及带中点偏置的三相PWM。该示例配置30kHz/8bit；整机静态库使用MCPWM，不能由示例推定整机也正好30kHz/8bit。

例如Vbus=12V、Uq=3V、theta=0时，Ua=6V、Ub约8.598V、Uc约3.402V，占空比分别约50%、71.65%、28.35%。同一个Uq在不同电角度会得到不同的三相占空比；不是“力矩数值乘固定倍率得到一个PWM”。

此电流检测教学例程DFOC_M0_Current把第三相电流传入了cal_Iq_Id的角度参数，不能当作已经验证正确的Iq检测代码整体复制。

## 与520方案的区别

SF是三相无刷FOC，并有磁编码器、电流采样硬件接口。520有刷电机依赖机械换相，我们当前输出一组方向+PWM并由AB测速形成速度环。可以参考电压归一化、限幅和闭环组织，不能直接搬三相调制，也不能把真实扭矩Nm固定倍乘成520 PWM。

若以后要真正控制520轮端力矩，需要明确电流测量/控制能力以及电机转矩常数、减速比、效率等；这是另一个执行器接口。当前新训练的轮子动作仍是目标线速度，未因本次审查改变。
