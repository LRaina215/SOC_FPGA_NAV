# 整车联合镜像：时序收敛与架空测试（2026-10-27 夜）

## 结论

- 导航 + 原 4500 控制的联合镜像（SLF128、存储迁移、SYN45 时序候选，加本次融合模块两处寄存）在 place2/route0 下布通且时序收敛：clk50 实测 50.080 MHz，rgmii_rxc 110.318 MHz，两个时钟 setup/hold 违例都为 0；recovery/removal/最小脉宽为正（最差 +5.067/+0.329/+2.200 ns）；SDC 61 条约束全部 Actived。
- 资源：BSRAM 280/340（83%），Logic 60436/138240（44%），CLS 42760/69120（62%），寄存器 31491。
- 位流已下载到 SRAM（UserCode 0x248F，未写 Flash），只读自检和导航侧 SAFE 指令链测试通过。
- 电机、BMI088、雷达相关测试没有做成：BMI088 的 I2C 报错 5（SDA 被拉低），下载前控制组原镜像同样报错 5；雷达网口 PHY 无链路。判断是车上传感器和执行器没有供电，需要整车上电后补测。

## 构建

| 目录（F:/nav_integration_g2_20261023/） | 内容 | 结果 |
|---|---|---|
| SLF128_measured_timing_place2_pnr56 | Codex 的 SYN45 网表，place2 | 布通，clk50 49.695 MHz，setup 9 个端点 TNS −0.662 ns，RX −0.047 ns |
| K_timing_syn46 | SYN45 + 融合状态 6 的陀螺仪静止门控提前一拍寄存 | 综合 |
| K_timing_syn47 | 再加：状态 9 的 \|eg\| 先寄存，新状态 17 再求平方 | 综合，部署用 |
| K_pnr57_p2r0 | SYN47 网表，place2/route0 | **布通，时序全部满足** |
| K_pnr58_p2r1、K_pnr60_p1r0 | 同一网表 place2/route1、place1/route0 | 稳定性复核，见下文 |
| K_timing_syn48 | SYN47 + pc_filter 输出加 skid、RGMII 接收选沿后打一拍 | 只综合，未部署（备用） |

部署位流备份：`D:\tools\g2_joint_slf128_pnr57_20261027.fs`，哈希见 `pnr57_hashes.txt`。

## 融合模块改动与验证

补丁 `rtl_patches/syn47_imu_fusion.patch`：

1. 状态 6 的"旋转过快不信任加速度"判断改用提前一拍寄存的 `gyro_still_q`。bmi088 的六轴输出只随 data_valid 同拍更新，状态 6 在采样后 90 拍以上，所以晚一拍的值与原组合式相同。
2. 状态 9 先寄存 |egx|、|egy|、|egz|，新状态 17 再求平方并启动 sqrt，处理时间多 1 拍（20 ns），数值不变。

`fusion_equivalence/tb_fusion_eq.v` 用相同随机激励对照原模块和新模块（BIAS_SAMPLES=4，含偏置标定、±满量程、1 g 窗口内外、陀螺仪门限附近、处理中途来新样本），按每次发布逐项比较重力向量、角速度、accel_rejected、bias_suspect：3874 次发布全部一致，信任/拒绝两支各 1696/2178 次。ModelSim AE 运行，许可证 `C:/Users/Li/Downloads/LR-204362_License.dat`。

## 架空测试（RK3588 上的 rk_bench/bench.py）

串口：`/dev/ttyACM0` = 板载 CH9102（导航总线桥 921600），`/dev/ttyUSB0` = P11-40 USB-TTL（控制遥测 230400）。

| 项目 | 结果 |
|---|---|
| SYS_ID / 软核 | 0x4E415630；128 KB；固件运行，主循环约 17.6k 次/秒，雷达方向表就绪 |
| 控制遥测 | 100 Hz，CRC 错 0，序号连续 |
| SAFE 指令（软核保持复位，主机直写） | 输出跟随 50 mm/s、300 mrad/s；停止发送约 200 ms 后看门狗置位、输出归零 |
| SAFE 急停 | 置 ESTOP 后输出立即为 0，释放后恢复 |
| 指令有效期（lease） | 没有有效里程计反馈时不授权，控制收到的指令保持 0（BMI088 未就绪导致 feedback_valid=0），符合设计 |
| BMI088 | status 0x0A（传感器超时、标定中），error_code 5（I2C 线忙超时、SCL 高，即 SDA 被拉低），六轴全 0 |
| 雷达 | 固件每约 7 s 重试 TCP 启动；NETL_PHY_STAT 0x00270006，网口无链路 |
| RK3588 ↔ 导航邮箱（CH9102，tools/nav_host.py） | 状态报文 5 Hz；IDLE 解锁后 GOAL 被接受，模式进入"行进" |
| 失联停止（rk_bench/link_loss_test.py） | RK 以 10 Hz 持续发 GOAL 时保持"行进"；停发后第一帧在 0.304 s 显示"急停"（固件超时 250 ms，状态 200 ms 一帧）。恢复要先发一次 IDLE |

## 同一网表的其他布局

| 运行 | 选项 | 结果 |
|---|---|---|
| K_pnr57_p2r0 | place2 / route0 | 布通，时序满足（部署） |
| K_pnr60_p1r0 | place1 / route0 | 布通，clk50 41.803 MHz，1217 个端点 TNS −1368 ns |
| K_pnr58_p2r1 | place2 / route1 | 2719 条未布通 |

所以 SYN47 网表在 place2 下能收敛，但余量为零，换布局就不过。K_timing_syn48（pf 流 skid + RGMII 接收打拍）用来加余量，见 K_pnr59_s48p2r0。

## 导航：128 结构图的长时漂移（330 s 巡检，field3x5_v1，种子 1）

| 配置 | 误差 mm（60/120/200/270/330 s） | 最大 | P95 |
|---|---|---|---|
| 256 对照 | 2 / 11 / 2 / 6 / 15 | 117 | 60 |
| 128 前移 1.5 m（当前固件缺省） | 19 / 68 / 147 / 305 / 445 | 611 | 491 |
| 128 前移 1.0 m | 35 / 85 / 49 / 50 / 40 | 193 | 101 |
| 128 居中 | 37 / 96 / 1 / 5 / 246 | 268 | 105 |
| **128 窗口固定在场地中心** | 51 / 55 / 5 / 12 / 24 | 134 | 88 |
| 256 存储、128 窗口前移 1.5 m | 16 / 12 / 8 / 58 / 209 | 371 | 301 |

256 存储配 128 前移窗口同样发散，说明漂移来自窗口位置，不是存储尺寸。窗口固定罩住整个场地时，误差不再累积，水平接近 256。结果目录 `F:/nav_integration_g2_20261023/K_patrol_20261027/`。

固件因此增加固定窗口选项（不改 RTL）：`MSG_SLF_CONFIG` 扩展到 24 字节，带 fixed、x_mm、y_mm（里程计系）。主机命令 `python3 tools/nav_host.py --link uart:/dev/ttyACM0 slf fixed 2050 950 350`，其中 (2050, 950) 是 field3x5_v1 出发位姿 (0.45, −0.95, 0°) 下的场地中心；真实场地按"场地中心相对出发位姿"换算。已在板上用调试口装载新固件验证：查询、设固定、切回前移的应答都正确（profile 0x534C0108）。只有车在 IDLE 且未加载先验图时允许切换。

## G3：BMI088 姿态接入导航（候选，未部署）

联合镜像里导航的姿态输入原来是 A15 上的 WIT 串口，车上没有这个 IMU。K_timing_syn49 改为片内连接：

- `g3_attitude/bmi_att_uart.v`：控制侧融合每次发布（约 100 Hz）时，把重力单位向量（Q12）和角速度（Q9 rad/s，控制机身系）打成两帧 11 字节，按 115200（DIV 434）送进导航的 IMU 接收口。ModelSim 用参考 UART 接收器对照 40 个样本、饱和值和忙时丢弃，0 错误。
- 固件新增 `IMU_PROTO_BMI_INTERNAL`：轴变换 nav = (ctrl_y, −ctrl_x, ctrl_z)，由重力向量算俯仰/横滚（与 WIT 角度帧同约定，pitch>0 低头），角速度换算成 WIT 原始单位。主机单元测试 289 个姿态（±40°），最大误差 0.015°。启动时写 IMU.BAUD_DIV=434。板级构建用 `g3_attitude/build_rv_k.py`（加 -DAPP_IMU_PROTO=1 -DAPP_IMU_BAUD_DIV=434）。
- 轴向只按坐标定义推导，**装车后必须用手倾斜核对**：低头时 pitch 应为正，左侧抬起时 roll 应为正；错了改 imu_parse.c 里 `IMU_BMI_NAV_*` 三行，经调试口重新装载固件即可，不用重综合。
- 固件镜像 54164 B，.bss 70020 B，6 KB 栈预留后余 744 B。

整车上电后补测：

```bash
cd ~/fpga_bench
python3 bench.py selftest 4                # BMI088 就绪、策略 active、雷达帧率
python3 bench.py drive 50 0 2 --hold-cpu   # 前进 50 mm/s 2 s，看 forward_command 与轮速
python3 bench.py drive -50 0 2 --hold-cpu
python3 bench.py drive 0 300 2 --hold-cpu  # 转向
python3 bench.py estoptest 50              # 急停
```
