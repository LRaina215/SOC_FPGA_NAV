# 520 AB相测速与OLED第一行 m/s 显示

当前 main 同时运行双轮AB编码器测速、BMI088六轴采集及OLED显示。

```text
L+000.98 R+001.02 m/s
AX:+0.98g  GX:+0000.1
AY:-0.01g  GY:-0012.3
AZ:+1.00g  GZ:+0023.4
```

全部12号字。首行L/R是左右轮轮缘线速度，保留2位小数，显式m/s单位；加速度是g，陀螺仪是dps（°/s）。首行宽126像素，轴值左右各60像素；轴行Y=16/32/48。

## 原工程参数与引脚

参数来自 /home/cyd/fpga/Documents/fpga_embedded_competition/src/520.v、src/main.v 和 /home/cyd/fpga/README.md 中已确认的规格。轮胎直径65mm，每相11脉冲、减速比30:1，四倍频CPR=11×4×30=1320。右轮原始前进计数为负，左轮为正，显示右轮取反后统一前进为正。

| 通道 | A相 | B相 | P11 | 显示 |
|---|---|---|---|---|
| M1右轮 | D14 | F13 | 15/17 | R |
| M2左轮 | D15 | F14 | 16/18 | L |

main.cst已加入这四个引脚及原工程的上拉配置。

## 测速实现

src/motor520_speed.v 是独立测速模块，包含三级同步、初始AB电平稳定等待、四倍频查表、非法双位跳变统计、10ms计数窗口和单位转换。只接编码器测量，未包含PWM或电机驱动输出。

轮周长按65mm和π计算，取整为204204µm。默认参数下：

v(m/s) = delta × 0.204204 / (1320 × 0.01)
v(cm/s) = delta × 1547 / 1000

硬件用整数比例约分和逐周期除法，正负对称四舍五入；避免近似固定点增益在半值处舍入错误。speed_cms 是signed16，1单位=0.01m/s，饱和到±32767；position和speed_delta是signed32。每个窗口结束后发布一次新速度，未出现跳变的下一个窗口自动归零。窗口最后一拍的编码器边沿计入本窗口且不重复计数。

## main的接口

ENCODER_CPR 和 WHEEL_DIAMETER_UM 常量在 main 中。若换轮胎或电机，改这两个参数再编译。SPEED_GATE_CYCLES和CLK_HZ也可在测速模块实例中覆盖；默认50MHz、500000周期=10ms。

OLED参数 `.SIX_AXIS_MODE(1),.WHEEL_HEADER_MODE(1)`，输入 `.SPEED_L(left_speed_cms),.SPEED_R(right_speed_cms)`。速度每帧锁存，渲染时不会混入半帧新数值。WHEEL_HEADER_MODE=0仍可显示之前的BMI状态标题；新模式下BMI故障由六轴区域的无效值占位表示。

## 验证和烧录

测试文件位于 /home/cyd/fpga/tools/bmi088_oled_validation：

- tb_motor520_speed.v：正反转、右轮取反、停止归零、500计数半值舍入、初始电平无虚假脉冲、非法跳变、窗口边界、限幅。
- tb_wheel_screen.v：轮速m/s首行与g/dps六轴显示，与全部1024字节黄金显存对比；检查每帧快照、负速度及BMI故障/恢复。
- tb_bmi_oled_system.v：完整main的BMI与OLED实际I2C模型端到端整屏对比，停止时轮速为0。

打开 fpga.gprj，顶层main；运行 bash build_oled.sh，输出 impl/pnr/fpga.fs。此次实物测速和点亮验证尚未执行。

## 已加入速度闭环

当前main同时启用motor520_closed_loop，默认左右1m/s，20kHz PWM，50%占空上限。测速模块仍负责反馈，实际驱动输出与调参方法见520_CLOSED_LOOP.md。
