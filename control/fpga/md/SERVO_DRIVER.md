# 四路舵机PWM驱动

文件 src/servo_driver.v。已加入fpga.gprj并接入main，四路默认为1500µs中位脉宽，不自动扫动。

## main控制接口

直接修改main顶部的参数（单位µs），重新编译烧录：

```verilog
parameter [15:0] SERVO_LH_US=16'd1500, // 舵机1左髋
parameter [15:0] SERVO_LK_US=16'd1500, // 舵机2左膝
parameter [15:0] SERVO_RH_US=16'd1500, // 舵机3右髋
parameter [15:0] SERVO_RK_US=16'd1500, // 舵机4右膝
parameter SERVO_ENABLE=1'b1,
parameter SERVO_APPLY_DIRECTION=1'b1
```

设置1600、1400等值就能改变高电平时间。默认方向映射以1500µs为中心，沿用标定[+1,-1,+1,+1]：左膝输入1600时实际输出1400；另外三路输入1600实际输出1600。1500是信号中位，不自动等同于机械90°或机器人默认站姿。

## 动态控制接口

驱动的pulse_lh_us/pulse_lk_us/pulse_rh_us/pulse_rk_us都是16位输入，可以接寄存器或模型输出，实现实时控制。main已经预留servo_lh_command_us等四根线，当前接上述参数，后续直接换这些线的来源。

```verilog
servo_driver #(.APPLY_CALIBRATED_DIRECTION(1'b1)) servo_thread(
    .clk(clk),.reset_n(reset_n),.enable(1'b1),
    .pulse_lh_us(lh_command),.pulse_lk_us(lk_command),
    .pulse_rh_us(rh_command),.pulse_rk_us(rk_command),
    .SERVO1(SERVO1),.SERVO2(SERVO2),.SERVO3(SERVO3),.SERVO4(SERVO4),
    .frame_tick(),.active_lh_us(),.active_lk_us(),.active_rh_us(),.active_rk_us());
```

如果上层已经把动作方向、IK和零位转换成物理脉宽，例如直接接原rl_action_exec的pulse_lh/pulse_lk等输出，设APPLY_CALIBRATED_DIRECTION=0（main对应SERVO_APPLY_DIRECTION=0），使输入1600直接输出1600，避免反向两次。原12位脉宽连接到本16位输入时可写{4'd0,pulse_lh}。

## 时序和范围

默认50MHz时钟，每1µs使能一次；周期20000µs=20ms=50Hz，高电平范围500~2500µs，越界钳位。四路输入在帧起点一起锁存，帧中修改不会改变当前脉冲，最迟下一帧生效（约20ms）。输出为时钟寄存器，不直接组合译码多位计数器。

reset_n低或enable低时在下一个时钟沿输出0并复位帧；重新使能立即开始完整新帧。pulse输入0会按范围和方向钳位，不代表关闭；关闭用enable。frame_tick在每帧开始高1个时钟，active_*_us给出当前实际物理脉宽。

CLK_HZ须为1MHz的整数倍；MAX_US应小于FRAME_US。MIN_US/MAX_US和CENTER_*_US可作为实例参数定制。角度换算应按实际舵机规格和机械安装确定，当前接口直接控制脉宽。

## 引脚

| 输出 | 部位 | FPGA引脚 | P11 | 方向 |
|---|---|---|---|---|
| SERVO1 | 左髋 | C18 | 1 | +1 |
| SERVO2 | 左膝 | C19 | 2 | -1 |
| SERVO3 | 右髋 | E19 | 3 | +1 |
| SERVO4 | 右膝 | D19 | 4 | +1 |

约束照原工程复制，main.cst已加入。电源和地沿用已有舵机接线。

## 验证

验证四路独立脉宽、50Hz帧、方向映射、直接物理模式、宽输入钳位、帧原子更新、帧中改值无毛刺、禁用/复位/重启；另外在默认50MHz下核对1500µs及其他通道脉宽、20ms周期。

测试源 /home/cyd/fpga/tools/bmi088_oled_validation/tb_servo_driver.v、tb_servo50.v。工程烧录文件仍为impl/pnr/fpga.fs。本节验证记录覆盖仿真，实物动作按对应板级测试记录。

当前main仍包含左右1m/s电机闭环与BMI088/OLED功能，舵机新增接口默认中位；如只测试舵机，可将main的MOTOR_ENABLE设为0后编译。
