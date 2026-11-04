# 云拓项目阶段收尾与交接（2026-11-04）

本文汇总项目交付组合、启动和演示入口、验证结果与后续维护事项。版本依据真实实现报告及已确认的部署记录，设备当时是否在线不作为项目功能说明；本次文档收尾没有重新运行构建或实车测试。

## 1. 交付版本

| 项目 | 最后确认的版本 / 状态 | 依据 |
| --- | --- | --- |
| FPGA Flash | **PNR100**，`g2_swingunlock_ultrasonic_pnr100_20261104.fs`，已写入并校验 | [Flash部署记录](../../integration/g2/field_nav_20261102/evidence/swing_unlock_pnr100_20261104/deployment.json) |
| FPGA配置 | GW5AT-138C构建、136KiB PicoRV32 RAM、SLF128、原4500/125Hz、OV5640/LCD、超声波遥控保护 | [实现记录](../../integration/g2/field_nav_20261102/evidence/swing_unlock_pnr100_20261104/report.json) |
| Flash内置固件 | `nav_fw_swingunlock_yaw3_20261104.bin`，58,688B，启动标识`Nov 4 2026 08:41:55` | 同上；重启后keeper可覆盖此固件 |
| RK最后选择的ACTIVE | **`nav_fw_map2demo_radius35_yaw3_20261104.bin`**，61,712B，启动标识`Nov 4 2026 13:21:29` | [13:22:59加载记录](../../integration/g2/field_nav_20261102/evidence/demo_radius_20261104/deployment.json) |
| 最新加载后状态 | 13:23：两条链路在线、地图已加载、导航idle/零指令；仍记录policy_fault1、BMI088错误4/过期 | [加载后快照](../../integration/g2/field_nav_20261102/evidence/demo_radius_20261104/post_load.json)；不当作现在的实时故障状态 |
| 先验地图 | 家庭场地地图2，`field_home2`，9,880B NVMP v3；第一张图仍可作为独立配置保留 | [地图2几何](../../config/field_home2_20261102.json)、[场地记录](../../integration/g2/field_nav_20261102/README.md) |
| RK与FPGA通信 | CH串口921600负责导航任务/地图，P11-40的独立USB-TTL串口230400负责控制V2遥测 | [引脚](../2_interface/pinout.md)、[RK接口](../5_teams/rk3588_design.md) |

Flash默认固件与RK的ACTIVE是两个独立版本。联网启动时，`yuntuo-prior-keeper`可经UART把ACTIVE载入软核RAM，随后恢复选定地图；所以“Flash已更新”不等于“当前应用一定运行Flash内置固件”。PNR94/85及128KiB镜像是历史或回退材料，136KiB固件不能直接装到旧128KiB配置上。UserCode AA1E也不能单独区分PNR99与PNR100，应结合所用FS、硬件容量和实际固件启动标识。

## 2. 实现与资源

PNR100自然完成布局布线，完整TR与位流已生成，无未布通报告。全局及四个时钟域setup/hold端点违例与TNS均为0，319条原有SDC命令均为Actived。

| 时钟 | 目标MHz | 实际Fmax MHz |
| --- | ---: | ---: |
| clk50 | 50 | 50.145 |
| rgmii_rxc | 100 | 157.374 |
| camera_pclk | 50 | 120.892 |
| lcd9 | 8.967 | 66.875 |

实际BSRAM **322/340**、DSP **178/298**、Logic **65,003**、FF **33,821**、CLS **45,117/69,120**。最差已列setup裕量为+0.058ns。CPU的136KiB为原128KiB主体加独立8KiB尾区；实际68块CPU存储的初始化已和冻结镜像核对。

相机PR1014普通路由段告警仍保留；现有SDC没有外部器件的板级IO时序窗口。这些限制没有因为内部STA通过而自动完成。当前实现可以作为已有调试与演示的工程基线，完整实物能力按实际测试记录判断。

## 3. 当前导航、地图2与清障

最后加载的radius35沿用PNR100硬件和原4500平衡控制，正常遥控前进档为0.3m/s。临时0.20m/s、纯先验/关闭实时障碍等版本已经被后续加载取代，不再作为当前配置描述。

地图2演示路线仅在地图包身份匹配时启用：结构包CRC `0xd9a34e4a`，粗图原点(-8,-21)、尺寸38×34。搜索模式按以下毫米坐标巡回；出现新鲜视觉目标时转入原跟踪流程。

| 顺序 | x/mm | y/mm |
| --- | ---: | ---: |
| 1 | -110 | -1220 |
| 2 | 830 | 580 |
| 3 | 2290 | 550 |
| 4 | 2300 | -930 |
| 5 | 1150 | -930 |

**350mm是这条演示搜索路线的经过半径**，允许经过区域时切换下一段，也处理前一段仍在观察的状态；它不是全部GOTO任务的统一到达容差，更不代表已量测的定位误差。地图2入口连接、可达观察点及350mm修改的范围见[入口补丁](../../integration/g2/field_nav_20261102/evidence/map2_demo_entry_20261104/firmware.patch)与[路线补丁](../../integration/g2/field_nav_20261102/evidence/demo_radius_20261104/firmware.patch)。

当前完整链保留实时障碍、最终足迹检查、观察/射线清障及恢复原目标逻辑。清障硬件使用保留深度帧和版本比较，作废的是旧高程观测，不能直接把未知格写成自由；需要新鲜地面和代价图确认。12:06那次实测确实启动两次、完成一次局部地面复核，但随后仍被邻格拦住，没有恢复持续前进。因此清障实车有效性仍列为维护项，不能只凭状态文字或模块检查宣布完成。

地图制作、NVMP v3格式、上传和SET_POSE见[建图指南](ground_debug_and_mapping_20261027.md)。地图READY只说明接收完成，定位是否可信要另看姿态、初始位置和SLF结果。两套巡航点在RK/Web按地图配置管理；FPGA同时装载的是选中的一套地图，删除RK上的旧地图文件不会缩减固定片上RAM。

## 4. 超声波、遥控与两路相机

- 超声波保护默认关闭，遥控**0键**切换，**LED7/N17**亮表示开启；仅限制遥控前进，导航旁路。
- 原遥控前进0.3m/s保留；600mm开始按距离连续减速，250mm及以内进入停车轮速保持，原4500继续平衡。障碍移远后自然恢复，没有新增故障锁存或松键重武装。
- SG90固定朝前，1500µs/20ms，不扫描。P11-36/B13为PWM、37/A13为Echo、38/A14为Trig；电平和供电见引脚表。
- 数据通路使用上电初值，模式位保留复位。热复位关闭超声波模式，但测距状态等待下一次有效测量更新。无效测量不更新目标，没有额外的失效停车策略。
- 运行停止/网页急停撤销运动目标并保留平衡；IR5是原控制锁停路径，两者不能混称全部轮子断电。
- **Astra RGB-D接RK**，供YOLO、目标三维位置和Web视频；**OV5640接FPGA**，供RGB565本地LCD预览，两条链路独立。实际LCD为4.3英寸480×272，160×120双缓冲图像使用38块BSRAM；不要沿用早期5英寸/800×480待合入描述。

## 5. 日常启动、网页与模型展示

1. 按现有整车接线供电。通过服务状态和实际数据判断传感器恢复，不能用串口在线代替BMI/雷达健康。
2. RK在`~/fpga_bench/fw/ACTIVE`选择固件，`loaded.json`记录加载标识；`yuntuo-prior-keeper`和`yuntuo-fpga-bridge`负责加载/通信。桥接独占两个串口，维护工具直接访问串口时按现有流程暂停所有者，操作结束再恢复。
3. `~/QHXD`的`qhxd-backend`提供**网页前端与API同端口8000**。RK桌面打开`http://127.0.0.1:8000/`，同局域网电脑打开`http://<RK局域网IP>:8000/`。源码开发才使用`./scripts/start_frontend.sh`及5173端口。离线前先安装依赖、准备模型和构建`frontend/dist`。
4. 公网入口仍为`https://lingxunrobot.cn/`；手机小程序通过已配置的API域名访问。公网/微信能力依赖网络，RK本地模型和局域网网页可独立演示；不要把这两种条件混为全程离线。
5. RK图形桌面执行`cd ~/QHXD && ./scripts/open_ai_demo.sh`，打开真实请求处理的ASR、LLM、TTS等模块观察窗口。SSH可用`python3 scripts/observe_ai_pipeline.py --console all`。这些窗口观察现有流程，不需要另起一套推理链。

SenseVoice负责本地ASR，Qwen3-1.7B W8A8通过RKLLM/NPU运行，Matcha+Vocos负责本地TTS；YOLO26n运行在RKNN/NPU。Qwen与YOLO共享NPU，模型文件和私有环境配置不进Git。具体安装、端口及模型路径见[RK/Web部署文档](https://github.com/LRaina215/QHXD_Web/blob/yuntuo-fpga/docs/YUNTUO.md)。页面“停止已请求”和真实停稳有不同含义，演示配置采用`YUNTUO_ARRIVE_NEED_WHEEL_STOP=false`，轮速静止到达门可配置；任务完成和实际车体停稳的含义分别保留。

## 6. 下载、升级与回退

当前Flash已是PNR100，普通演示不需要重复烧录。导航应用更新走RK的ACTIVE与既有UART加载流程；接口和RAM容量必须匹配。先备份当前ACTIVE/loaded和选定地图，再按既有加载器核对实际启动标识、地图READY与服务恢复，不手工假写“加载成功”。

临时替换硬件只下载SRAM：先执行`programmer_cli --scan-cables`和`--scan --location <实际位置>`，正确ID为`0x0011081B`；再选择`--device GW5AT-138C --run 2 --fsFile <已验收FS> --location <实际位置>`。USB位置35/36会变化。全F读码表示目标尚未识别，不能把USB下载器存在当器件已就绪。

Flash固化使用不同的操作54，会改变断电后启动镜像，不能与SRAM操作2混用。PNR100现有Flash校验记录已归档。本次收尾没有重新执行下载。回退时明确回退硬件还是仅回退软件；136KiB应用不能载入128KiB旧硬件，keeper还可能再次按ACTIVE覆盖Flash默认应用。

## 7. 已完成、保留问题与归档位置

| 范围 | 本阶段结论 |
| --- | --- |
| 单芯片联合实现 | 导航、原4500、双串口、本地视频和超声波共同工程布通，四时钟约束STA通过 |
| 发布与持久化 | PNR100 Flash写入/校验及内置swingunlock启动已确认；最新radius35经RK加载、启动标识和地图恢复有记录 |
| 历史落地导航 | PNR82/live12的单点往返有用户认可的实测记录；不移签为所有后来候选的完整验收 |
| 清障与定位 | 实车仍有假障碍/邻格反复拦截、漂移和重新对准问题；原始高程成因并未全部定位 |
| 站立与停止 | 零运动指令不保证车体完全静止；受控停车、失联后地面回稳和新固件稳定性仍按[待测清单](../1_plan/car_test_backlog_20261102.md)保留 |
| 超声波实物 | 开关、数值和构建已实现；本轮没有新增实物停止距离验收 |
| 移动端 | 小程序开发者工具已有播放记录；新版手机播放、前后台和发布状态仍以小程序维护记录为准 |

本机冻结硬件在`F:/nav_integration_g2_20261023/K_swingunlock_syn11_20261104`，实现报告在`K_pnr100_swingunlock_ultrasonicp2r0/impl/pnr`；最新应用源码在`D:/tools/demo_radius_20261104/fw`。仓库根`fw/`及通用`build_g2.py`不能自动视为这些外部冻结工作区的完整替身。此次Git交付归档版本、证据、关键源码补丁和文档；大位流、模型及外部完整工作区仍按本机路径保留，不宣称仅靠一次旧通用构建就能复现当前发布。

后续维护从本组合及其明确差异继续。失败候选保留历史，不恢复参数轮换；如继续优化，按实际路径或实车日志选定问题。当前文档收尾不要求重新运行已有仿真或测试矩阵。

- [本次收尾版本清单](../4_design/evidence/project_closeout_20261104/manifest.json)
- [整车当前状态](../1_plan/system_integration_plan_20261021.md)
- [场地实测与软件演进](../../integration/g2/field_nav_20261102/README.md)
- [超声波实现记录](../../integration/g2/ultrasonic_guard_20261103/README.md)
