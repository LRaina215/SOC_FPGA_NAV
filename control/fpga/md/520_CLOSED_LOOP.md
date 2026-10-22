# 520左右轮线速度PID闭环

默认左右目标+1.00m/s。M2左轮、M1右轮。OLED首行显示实际轮速m/s，下面为加速度g和角速度dps。

## main中可修改的接口

```verilog
parameter real TARGET_LEFT_MPS=1.00,
parameter real TARGET_RIGHT_MPS=1.00,
parameter MOTOR_ENABLE=1'b1,
parameter real LEFT_KP=2.50,LEFT_KI=0.35,LEFT_KD=2.00,LEFT_KFF=0.80,
parameter real RIGHT_KP=2.50,RIGHT_KI=0.35,RIGHT_KD=2.00,RIGHT_KFF=0.80,
parameter integer MOTOR_DUTY_MAX=2400,
parameter integer MOTOR_RAMP_STEP_MMPS=100
```

速度参数直接填写m/s，负数后退、0停止；修改后重新编译烧录。MOTOR_ENABLE=0关闭两路。实时外部目标可直接连接线程的signed32 target_left_mmps/target_right_mmps，1000=1m/s；real参数仅在编译时换算成整数，不在FPGA中实现浮点。

## 本次响应调整

用户反馈旧版最大约0.5m/s；旧版PWM上限是1250/2500=50%，在目标1m/s、反馈0.5m/s时原PI会被限幅，继续加PID不能突破占空限制。新版恢复原驱动允许的2400/2500=96%上限；参考速度每10ms增加100mm/s，到1m/s参考值约0.1秒，旧版为0.5秒。

PID+前馈运算：

```text
e = v_ref-v_measured
I候选 = I + KI*e
filtered_dv += (v_measured-v_previous-filtered_dv)/4
D = -KD*filtered_dv
u = KFF*v_ref + KP*e + I + D
PWM占空比 = abs(u)/2500
```

速度用mm/s。KP/KFF单位是PWM计数/mm/s；KI是每10ms样本的积分增量；KD作用于每个样本的测速变化量，不是直接对目标误差差分。D使用Q8小数保留、alpha=1/4低通，限幅±250PWM计数，避免编码器量化噪声放大。首次采样不产生D跳变，目标改变不产生微分冲击；停止/禁用/复位清空历史。增益非负；KD=0可退回PI。

积分和输出限幅；输出饱和且误差还推向饱和时暂停积分。目标限幅±2m/s。按样本缓升，按载波边界改变PWM，反向前空档一个载波周期。

这些增益是初值。96%占空下能否达到1m/s取决于供电、负载与电机；仿真中的电机模型不是实测辨识，需实物整定。

## 时间尺度

- 测速/闭环更新周期10ms（100Hz）。新目标通常在下一个测速样本开始运算。
- PID流水线8个50MHz时钟，约0.16µs；随后在下一PWM载波边界装载，最多再等50µs。
- PWM载波20kHz，每50µs一周期。
- 1m/s参考缓升约0.1秒，这不是电机保证达速时间。
- OLED100kHz I2C整屏刷新约0.3~0.35秒。看屏幕的延迟比控制环大，机械达速时间也不同于控制周期。

## 引脚与方向

65mm轮径、1320四倍频计数/轮圈，沿用已验证原工程。正驱动DIR1=1/DIR2=0，两路均向前；编码器左正、右负，右反馈取反。

| 通道 | PWM | DIR1 | DIR2 | 编码器A/B |
|---|---|---|---|---|
| M1右 | D17/P11-23 | B20/P11-19 | B17/P11-21 | D14/F13，P11-15/17 |
| M2左 | C17/P11-24 | B18/P11-22 | A20/P11-20 | D15/F14，P11-16/18 |

## 停机与验证

main的left_motor_fault/right_motor_fault：1缺测速样本超过200ms；2参考至少0.2m/s、PWM至少20%、测速不超过0.02m/s持续100样本；3参考至少0.2m/s、测量反向至少0.05m/s持续10样本。故障锁存并停该路；设目标0、禁用或复位可清除。

验证包括：默认两路1m/s模型收敛、负载变化恢复、独立目标、反转、零速清积分、占空限幅与故障停机；D低通、目标无微分冲击、正负D限幅、停止历史清除；PWM逐周期脉宽、DIR方向、完整换向空档及禁用/复位。

测试在 /home/cyd/fpga/tools/bmi088_oled_validation/tb_motor520_control.v、tb_motor520_derivative.v、tb_motor520_pwm.v。尚未完成本版实物调参，也未将新版位流下载到开发板。编译命令bash build_oled.sh；烧录文件impl/pnr/fpga.fs。

## 停车往复振荡调参（当前版）

按用户要求将左右轮比例增益P由3.00小幅降低为2.50（Q12由12288变为10240），顶层与驱动默认值同步修改。I=0.35、D=2.00、前馈=0.80保持不变。遥控前后目标为±0.3m/s，高度目标130.5mm；模型轮速输出上限仍为±0.6m/s、PWM上限仍为2400。调整效果需实物对比，仿真不能证明停车振荡已消除。
