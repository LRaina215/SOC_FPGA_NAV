# BMI088 六轴采集 + OLED 12号字双列显示

打开本目录 fpga.gprj，顶层 main。工程新增 src/bmi088.v，复用 oled_driver.v 内的 iic 模块定义，不另加 iic.v。旧 Documents/fpga_embedded_competition 工程未修改。IDE 若一直打开着，请关闭后重新打开工程，加载新增源文件。

## 屏幕

```text
BMI088: OK g/dps
AX:+0.98g  GX:+0000.1
AY:-0.01g  GY:-0012.3
AZ:+1.00g  GZ:+0023.4
```

字体是原包 asc2_1206 的 6×12。顶部21个字符，Y=0。左列X=0，右列X=64；轴行Y=16/32/48。加速度显示带符号的g值、两位小数，如+0.98g；陀螺仪显示带符号的角速度、一位小数，如-0012.3，单位dps=°/s。左右每项最多60像素，不重叠。每帧锁存同一个完整六轴样本，渲染过程中输入变化不会混合进该帧。约3帧/秒刷新；传感器定时100Hz采集，OLED显示最新完整样本。

## main 中的接口

```verilog
oled #(
    .SIX_AXIS_MODE(1),
    .AXIS0_TEXT("AX:"), .AXIS1_TEXT("AY:"), .AXIS2_TEXT("AZ:"),
    .AXIS3_TEXT("GX:"), .AXIS4_TEXT("GY:"), .AXIS5_TEXT("GZ:")
) display (
    .VAL0(accel_x), .VAL1(accel_y), .VAL2(accel_z),
    .VAL3(gyro_x), .VAL4(gyro_y), .VAL5(gyro_z),
    .sample_valid(sample_valid), .sensor_ready(sensor_ready),
    .sensor_error(sensor_error), .sensor_error_code(sensor_error_code),
    // 时钟、复位、IIC_SCL/IIC_SDA 等连接参见 main.v
);
```

VAL0..5 输入仍是传感器的 signed16 ADC值，显示线程负责单位换算；无需在main重复缩放。AXISn_TEXT 是三个ASCII字符，可自定义标签。sample_valid 高一个周期时保存完整输入，sensor_ready 表示初始化成功，sensor_error/error_code 用于顶部状态。SIX_AXIS_MODE=0 仍保留之前的 TEXT/TEXT2/TEXT3/TEXT4 和 LED_NUM 接口。

## 数据含义和配置

本版显示实际单位：加速度为g，陀螺仪为角速度dps（°/s），不积分成姿态角。按Bosch当前配置的标称灵敏度换算：

- 加速度：g = raw / 10920；显示0.01g。硬件计算 `(abs(raw)*100 + 5460)/10920` 后还原符号。
- 陀螺仪：dps = raw / 16.384；显示0.1dps。硬件计算 `(abs(raw)*10000 + 8192)/16384` 后还原符号。

使用逐周期整数除法、四舍五入（半值远离0），无浮点运算；显示舍入为0时去掉负号。10702显示+0.98g；10920显示+1.00g；raw=-32768显示-3.00g/-2000.0dps。单位换算未做零偏校准；如果之后修改BMI088量程，需要同步调整 oled_axes_thread 的灵敏度常数。
配置沿用参考 bmi088_iic.v：加速度 ACC_PWR_CONF(7C)=00、ACC_PWR_CTRL(7D)=04、ACC_CONF(40)=A8、ACC_RANGE(41)=00；陀螺 LPM1(11)=00、RANGE(0F)=00、BANDWIDTH(10)=01。加速度±3g、100Hz；陀螺±2000°/s、2000Hz ODR/230Hz带宽。轮询每10ms读一组，两颗内部传感器不是硬件同步采样。参考驱动中某些中文注释把陀螺 0F 范围和 10 带宽混淆，这里已修正文案，寄存器值保持原配置。

Bosch官方资料：https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf

## 接线

| 信号 | FPGA引脚 | P11 |
|---|---|---|
| OLED SCL（sck） | F19 | 5 |
| OLED SDA（sda） | F20 | 6 |
| BMI088 SDA | F18 | 7 |
| BMI088 SCL | E18 | 8 |

时钟W19，复位D21。BMI088引脚约束照抄已验证的原工程；SCL/SDA需要模块已有或外部3.3V上拉和公共地。OLED与BMI088是两组独立总线，复用同一份 iic RTL，不把它们合到一组引脚上。

## 状态和故障

- WAIT：未初始化完成或还没有完整数据，数值显示 -.--g/----.-。
- OK g/dps：初始化成功并收到完整样本。
- ERR n：传感器故障，数值显示 -.--g/----.-；驱动100ms后自动重新探测、配置、读回校验并恢复采集。

错误码1/2/3为地址/寄存器/读地址NACK，4为SCL超时，5为SDA空闲超时，6为内部事务异常，7为芯片ID不符，8为配置回读不符。读取陀螺后才同时更新所有轴；失败时保留上一次样本但OLED会显示故障占位符。OLED的通信故障由自己的error独立表示，初始化失败需复位重试。

## 编译与验证

运行 bash build_oled.sh，生成 impl/pnr/fpga.fs。当前综合布局布线成功，50MHz时钟下Fmax见当前impl/pnr/fpga.tr报告，setup/hold违例均为0。

验证文件保存在 /home/cyd/fpga/tools/bmi088_oled_validation：

- tb_bmi088.v：实际I2C从设备模型，验证备用地址、芯片ID、配置回读和bit7屏蔽、6字节读取及大小端、失败不发布半帧、NACK/回读异常/SCL超时与自动恢复。
- tb_oled_axes.v：用原始oledfont.h生成黄金显存，逐字节比较整帧；验证12号字双列、真实单位及小数点、最小负数、帧快照、故障占位和恢复。
- tb_bmi_oled_system.v：完整main端到端仿真，BMI088 I2C模型的六轴值经OLED总线输出，与全部1024字节黄金显存比较。

另有tb_units.v，用独立Decimal四舍五入参考核对25组正负数、接近0、0.98g、1g、进位和满量程，覆盖全部六轴。

RTL仿真和编译验证已完成，尚未执行此次位流的实物点亮验证。

## 当前顶部已改为轮速

当前main设置WHEEL_HEADER_MODE=1，第一行显示左右520轮缘线速度m/s。原来的BMI状态标题在WHEEL_HEADER_MODE=0时仍可使用。双列加速度g与角速度dps显示保持；详见520_SPEED_OLED.md。
