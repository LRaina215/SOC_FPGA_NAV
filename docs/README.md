# 文档目录

按用途分五类。第一次接手先读 [1_plan/roadmap.md](1_plan/roadmap.md)，要改代码先读 [3_guide/dev_guide.md](3_guide/dev_guide.md)。

| 目录 | 文档 | 内容 |
|---|---|---|
| `1_plan/` 进度与计划 | [roadmap.md](1_plan/roadmap.md) | 当前状态、本机环境、踩过的坑、关键参数、BSRAM 预算、已知问题、后续任务（接手先读） |
| | [integration_plan.md](1_plan/integration_plan.md) | 整车联调：系统组成、接口矩阵、资源预算、里程碑 I0–I9、安全链、风险 |
| | [mapping_plan.md](1_plan/mapping_plan.md) | 先验地图：量测建图与 SLAM 扫图两条路线 |
| `2_interface/` 接口（唯一真值） | [icd.md](2_interface/icd.md) | 雷达、主机报文、底盘、IMU、软核总线的接口定义 |
| | [regmap.md](2_interface/regmap.md) | 寄存器表（由 `config/regmap.yaml` 生成，勿手改） |
| | [pinout.md](2_interface/pinout.md) | 引脚、电平、P11 分配、BMI088 小板接线、安全电平 |
| `3_guide/` 操作手册 | [hardware_guide.md](3_guide/hardware_guide.md) | 实机搭建与测试：环境、烧录、初始化、逐个外设接入、实地测试、排障 |
| | [board_bringup.md](3_guide/board_bringup.md) | 开发侧上板要点、IP 生成、有图模式与 PC 单独操作 |
| | [dev_guide.md](3_guide/dev_guide.md) | 开发规范：唯一真值、改动后必跑、新模块接入、地址空间、协作 |
| `4_design/` 设计与参数 | [algorithms.md](4_design/algorithms.md) | 闭环仿真平台、算法改进记录、批量评测结果 |
| | [platform.md](4_design/platform.md) | 机器人 / 传感器 / 场地参数（已知 / 假设 / 待测） |
| `5_teams/` 与其他组对接 | [chassis_integration.md](5_teams/chassis_integration.md) | 底盘 / RL：接口契约、代码分层、安全边界、联调顺序 |
| | [chassis_review.md](5_teams/chassis_review.md) | 对底盘组技术路线的审阅意见、调整要求、指令分层、问题单 |
| | [rk3588_design.md](5_teams/rk3588_design.md) | RK3588：职责边界、链路方案、目标报文、联调步骤 |
| | `reference/` | 其他组的原始资料（只存不改） |
