# UART 主机失联停止：冻结来源软件候选（2026-10-25）

当前可审阅版本为 `F:/nav_integration_g2_20261023/uart_host_loss_stop_candidate2`，尚未合入 FPGA 镜像、PNR27/28 输入或生产 RK。已完成冻结来源审计、RV32镜像/静态栈、原完整导航回归及正常RK任务握手定向验证；没有启动Gowin构建、下载、重启服务、打开串口或发送字节。补丁为该目录 `candidate.patch`，入口记录为 `manifest.json`。机器证据见 [uart_host_loss_candidate_20261025.json](evidence/g2_20261023/uart_host_loss_candidate_20261025.json)。

## 来源纠正与实际镜像绑定

早期 `uart_host_loss_stop_candidate1` 错用了工作树FW，缺少实际冻结版的 `capture_us`、support coverage/epoch等行为。其源码、失败和通过结果均保留，**不能算作SYN22冻结固件验证**。此前51332B镜像、73536B BSS、2672B静态栈及56B余量仅属于该旧来源，由下述冻结来源结果替代。

candidate2从 `sfp_lane2_private_snapshot_syn11_retry1/fw` 完整复制。原59个C/头文件/启动/链接/构建输入逐字保持，只在 `nav_app.c` 与 `hal_fpga.c` 插入任务锁与提交门控，新增两个guard文件。审计逆向去除这些明确插入后，两个文件全文均恢复到冻结原文，保留采集时间、coverage及epoch逻辑。无SFP的SYN22来源仍包含OLED和console，不能套用其它裁剪档的特性。

独立重编译**未改动冻结FW**后，原51712B BIN与重编译结果只有7字节不同，全部位于同一启动诊断打印的 `__DATE__/__TIME__` 字符串，旧值为 `Oct 23 2026 15:38:47`。该确切字符串范围以外逐字相同。24份原预处理源中，23份除行号路径外相同；main唯一差异也是这两个时间宏。严格原BIN字节相同检查仍记录false，来源审计明确记录这一有限差异，未覆盖原失败。

原104项资产集合已列出；根目录与 `firmware/` 的五份CPU HEX逐字相同，四字节银行与32位词文件独立重建的128KiB初始化均等于原BIN加零填充。新候选五份HEX也重建为新BIN加零填充，均留在独立 `rv32/`，未替换物理候选资产。原CPU硬件、128KiB容量和控制权重不变。详细证据为 `prepared_origin_binding.json`、`baseline_reproduction_result.json` 和 `frozen_origin_and_image_binding.json`。

## 最小合同与正常业务接续

复用现有NV报文、MBX到达时间戳及 `NAV_SYS_TIME_US` 的32位微秒时间。**1s超时只是候选参数**。RK的UART分支每200ms发送原24字节 `TARGET(found=0)`，等待MAP_ACK时也保活。原 `target_update()` 对found=0直接返回，不改变目标位置、ID、类别或年龄，也不延长视觉目标原1.5s有效期。R0 STATUS/MMIO读轮询不是固件入站NV活性，因此需要这一主机配套。

上电保持软件任务锁；失联后清旧目标、路径、航点及主动转向。过期、未来或新鲜窗口内倒退的RX时间不能续命。锁止时保存设备时间epoch，**锁前已排队的idle不可恢复，锁后的保活或GOAL也不可自动解锁**。只有明确的新idle才清锁，随后必须重新发起任务。

原RK新goto/search入口直接发送任务，没有先发idle。独立RK候选现已接入显式新任务流程：清旧发送队列，只发新idle；记录实际发送后的epoch，清原停止证据；要求推进的新STATUS为idle/零需求，并且反馈从本次idle之后连续健康、静止至少原0.5s门槛，才调用原TaskRunner入口。ACK或写入成功不能替代反馈。首次goto/search及暂停恢复已验证。

重连不自动发送idle、恢复旧任务或重放待授权任务。mode4/state6无法区分手动急停与主机超时，普通新任务拒绝；先明确请求解除导航急停，再另发新任务。断连、新急停、取消、握手超时和状态时钟回退会丢弃待启动意图。RK内存HTTP队列记录入队时间，故障epoch以前的旧任务/解除请求不能在重连后充当新授权。原UDP业务入口的立即处理行为保留。

| 候选文件 | 修改边界 |
|---|---|
| `fw/nav/host_link_guard.h/.c` | 活性、锁止、原始RX时间和锁止epoch检查 |
| `fw/app/nav_app.c` | 初始化及主循环三处检查；仅合法且通过年龄/锁检查的报文进入原分派；清旧任务 |
| `fw/drv/hal_fpga.c::hal_send_cmd()` | 真正提交前重新读设备时间，锁止/过期时将vx/vy/wz强制归零 |
| `rk/yuntuo/bridge/core.py` | UART保活、断连清队列、显式新任务零需求与新反馈握手、内存请求epoch |

## 实际轻量验证

全部在IDLE优先级、CPU16单核运行，不使用板级串口或生产RK服务。

| 验证 | 结果 |
|---|---|
| guard + 原冻结实际HAL的普通内存MMIO夹具 | 3258项通过；含延迟规划跨超时提交过滤、32位回绕、旧idle/GOAL、500组found=0目标全文不变 |
| 原冻结完整导航回归 | test_main全部断言、hal_sim原文不改；仅测试夹具启动注入新idle、每200ms保活；失败0，原90条轨迹候选逐字不一致0 |
| 原app + 原接收队列 | 10项通过；锁前idle/GOAL不恢复，新idle后新GOAL可恢复 |
| 原实际RK方法保活假链路 | 9项通过；等待地图ACK保活、UART断连丢旧队列、UDP旧行为保持 |
| 实际候选core + 原TaskRunner/StopEvidence | 31项通过；正常goto/search/resume、急停拒绝、显式恢复、旧HTTP队列、旧STATUS/不健康反馈拒绝和握手中断 |
| RV32链接/完整静态栈 | 镜像52164B（原51712，+452）；BSS69968B（原69944，+24）；静态界2736B，原6KiB栈预留与≤3072B门槛保持；IRQ关闭 |
| 实际ELF容量 | `_bss_end=122136`、`_stack_top=131072`；扣6144B栈预留剩2792B，原余量3272B；CPU128KiB保持 |

原冻结导航模型绕障终点误差98mm、最近离障324mm只是该仿真结果，不是实车精度/制动验收。RK的0.5s门槛来自原StopEvidence连续新鲜健康反馈，也不是独立物理停稳证明。早期有符号epoch比较在长离线回绕定例中的失败和修正历史留在candidate1，没有删除反例。

复现脚本位于candidate2：`run_contracts.py`、`run_rv32_validation.py`、`run_navigation_regression.py`、`test_rk_mission_handshake.py`、`audit_frozen_origin.py`。结果与来源在 `manifest.json` 逐项列出，不替换物理镜像。

## STOP、时限及未证明边界

本候选提交**数值为零的速度需求**，不写 `SAFE_CTRL.ESTOP`，不关闭模型、CONTROL_ENABLE、轮或舵机使能。超时可报告软件 `NAV_MODE_ESTOP`，但顶层没有解码该mode来物理停机；真正动作来自零vx/vy/wz。通信停仍执行策略/腿部控制，只说明RTL没有禁用控制链，不证明实车已平衡或刹停。

零需求先经SAFE默认1ms节拍、DV=2mm/s、DW=3mrad/s斜坡，再进入lease。SAFE_CTRL[2]是另一入口，在SAFE节拍形成ch_stop，继而lease清导航需求；也不同于影响control_run/模型故障链的IR/编译急停。本候选不自动设置或清除该位。

原 `hal_traj_eval()` DONE轮询及接收/雷达排空存在无界循环，未证明完整软件硬WCET。提交前门控只证明：该次读取设备时间已锁止/过期时，不把非零需求写入SAFE寄存器。CPU停止提交时原始命令watchdog默认200ms仍是退路；继续写零则走原减速链。lease最大前进需求2458Q12、1ms减41Q12时约60ms归零只是需求斜坡界限，不是整条链或车体停车时间。

**USB迟到旧授权仍未解决。** 旧字节若在主机/USB内部滞留，锁止后才到FPGA，会获得新的RX时间。NV没有session/nonce/命令序号，不能与新授权区分；清outbox或HTTP内存队列不等于撤回USB内部字节。严格防重放需另立会话/挑战合同或明确外部复位/清停策略。多圈计数、单一可信R0所有权及完整复位恢复也不由本次定例自动证明。

当前 `ready_for_integration=false`、`safe_for_deployment=false`。仍待根审阅确认超时/恢复合同及USB迟到边界，再决定合入；真实双UART、当前镜像保活/断连/反馈、软件循环与物理停车测量尚待执行，布线与时序签核门槛保持。
