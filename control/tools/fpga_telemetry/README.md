# FPGA真实传感器遥测

后台持续保存（不打开曲线窗口）：`bash /home/cyd/fpga/tools/fpga_telemetry/open_recorder.sh --port auto`。自动等待CH340串口，插入后记录，断开后自动重连；从第一帧CRC有效的数据开始写CSV，`record_time_s`从0开始，`host_timestamp`保留电脑接收时间，原FPGA时间和原始值也保留。电机停转、模型停机和传感器故障帧仍然记录，不会只保存正常运行段。文件逐行刷新到logs目录，状态见monitor_status.json。

记录器和图形监控共用同一接收逻辑，串口独占打开；使用其中一种即可，两者都保存CSV。

当前源码已恢复遥控前后指令：`FIXED_FORWARD_ENABLE=0`、`COMMAND_FORWARD_MPS=0`，取消固定 −0.05m/s。`+`/`−` 连续调节高度，`5` 锁定急停直到硬件复位。需要重新综合、布线并下载后才会在板上生效；当前`fpga.fs`已于2026-10-18生成，包含7000轮模型、18位权重、轮速参数及V2遥测，UserCode为000074BE，50MHz时序通过；尚未烧录。`bitstream_manifest.json` 的身份检查会阻止用新源码配旧文件烧录。

当前源码串口：230400、8N1、100Hz（V2）；旧位流仍为115200/50Hz，读取旧位流需加`--baud 115200`。FPGA TX=P11-40/A16→模块RX，FPGA RX=P11-39/A15←模块TX，共地，3.3V IO。RX暂不解析，不接受串口控制命令。

启动电脑曲线：

```bash
bash /home/cyd/fpga/tools/fpga_telemetry/open_monitor.sh --port /dev/ttyUSB0
```

显示融合姿态和角速度、左右轮目标与编码器速度、车身速度指令与轮速均值估计、转向角速度、带方向PWM、原始加速度。CSV自动保存到logs/。模型状态、故障、序号、CRC错误显示在窗口顶部。无有效数据时显示等待，断流超过1秒标记STALE。

轮速均值仅为编码器估计，不是真实车身速度测量；无高度传感器，顶部仅显示HEIGHT TARGET。融合姿态由FPGA投影重力换算，不能估计绝对偏航角。模型运行/校准/故障状态保留，不因串口显示而改变。

## 帧协议v1

- 魔数A5 5A，版本01，负载长度64(hex，即100字节)。
- 负载小端，Python格式`<IH6B6h6i3i2i2h2h4HII2i`。
- CRC16/CCITT-FALSE：初始FFFF，多项式1021，不反射，xorout=0；覆盖版本、长度和100字节负载，CRC低字节先发。总长106字节。
- 50帧每秒约5300字节/秒；FPGA端每20ms锁存整帧，发送过程中数据变化不改变已锁存帧。原始BMI和融合数据各自在valid信号时锁存完整样本，来自不同采样时刻的最新值，不代表全部传感器严格同时采样。

| 负载偏移 | 数据 | 格式/单位 |
|---:|---|---|
|0|上电时间|uint32，ms|
|4|帧序号|uint16|
|6|状态位|uint8，见mem_policy.status_flags|
|7|策略故障|uint8|
|8|BMI故障|uint8|
|9、10|左右电机故障（保留证据）|各uint8|
|11|融合标志|bit0校准、bit1零偏可疑、bit2加速度拒收|
|12|AX AY AZ GX GY GZ原始值|6×int16|
|24|机身角速度WX WY WZ及重力方向XYZ|6×int32，Q12|
|48|前进、偏航、高度指令|3×int32，Q12|
|60|模型左右轮目标|2×int32，mm/s|
|68|编码器左右轮速度|2×int16，cm/s|
|72|左右PWM带方向占空计数|2×int16，满周期2500|
|76|LH LK RH RK舵机脉宽|4×uint16，µs|
|84|完成推理帧数、推理周期数|2×uint32|
|92|左右编码器原始增量|2×int32，每10ms计数，尚未做物理方向转换|

注意：传输中的轮速反馈本身已有1cm/s量化，不能靠上位机插值当成更高精度测量。

## 验证

`verification.json`记录8N1线级仿真、CRC、快照隔离、序号/时间戳、分段输入、坏帧恢复、带符号物理量及顶层编译检查。`tb_uart_telemetry.v`捕获实际RTL串口输出。

FPGA工程：`/home/cyd/fpga/fpga/fpga.gprj`；构建日志`build_telemetry.log`；生成位流`impl/pnr/fpga.fs`。下载前需确认位流是本次新生成文件。

## V2追加PID诊断

本次源码使用V2：版本02，负载长度78(hex，即120字节)，100Hz，230400 8N1。前100字节布局与V1相同；偏移100/104为左/右PID内部参考int32 mm/s，108/112为左/右误差int32 mm/s，116/118为左/右控制完成计数uint16。CRC覆盖版本、长度和全部120字节负载，总帧长126字节。解码兼容V1，旧固件缺失的新字段为空/NaN；监控曲线新增PID内部参考。协议参数和待实车验证的调参依据见[轮速响应调整](../motor_response_tuning/README.md)。
