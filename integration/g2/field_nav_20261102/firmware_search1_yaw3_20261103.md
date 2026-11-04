# search1＋3° 固件与位流更新（2026-11-03）

## 内容与依据

ZCode 的 search1 包含两项已获用户批准的固件修改：边界搜索考虑车宽可达性；允许从起始外接圆已经重叠的致命格向外退出。原 search1 与 PNR83 的内置固件为 10°。

本轮将这些修改合入 `F:\nav_integration_g2_20261023\K_fw_terrain1\fw`，并将 `APP_MOUNT_YAW_CDEG` 改为 300。采用的是本会话最新固定坐标静态粗估约 +3°，不宣称已完成动态导航或高精度六自由度标定。相机外参没有随雷达角度改动。

未调整近区格子不过期半径；未合入 last-seen、缩短搜索视距等未采用的仿真试验。

## 源码与产物

- 冻结源码：`F:\nav_integration_g2_20261023\K_fw_terrain1\fw_search1_yaw3_20261103`。
- 固件镜像：`D:\tools\nav_fw_search1_yaw3_20261103.bin`，51,136 字节。
- 启动版本标识：`Nov  3 2026 07:49:47`。配套 ELF 与链接映射位于 `D:\tools\nav_fw_search1_yaw3_20261103.elf`、同名 `.map`；RAM 调试工具应使用本版 ELF。
- 构建目录：`F:\nav_integration_g2_20261023\K_fw_terrain1\out_terrain1_ground_search1_yaw3`。
- 综合工程：`F:\nav_integration_g2_20261023\K_terrain18_prepared`。
- 布线工程：`F:\nav_integration_g2_20261023\K_pnr84_terrain18p2r0`。
- 位流文件：`D:\tools\g2_terrain18_search1_yaw3_pnr84_20261103.fs`，42,340,870 字节，已完成实际时序报告验收。
- 构建状态：`D:\tools\firmware_update_search1_yaw3_20261103\pnr84_state.json`。

## 已做检查

固件编译成功，text 51,136 字节、BSS 70,200 字节、栈预留 6 KiB。原构建器的自动栈分析不完整，采用 1,280 字节直接调用界；不将其表述为完整运行期栈证明。

77 个 RTL 源文件及综合／布线约束沿用 PNR83，内容保持不变，Tcl 只改新工程路径。四路 RAM 初始化文件按字节通道还原后与新固件镜像一致；原 PNR83 输入也可还原为旧 search1 镜像。版本依据是初始化内容和构建链，不能仅凭两个位流文件不同就证明具体嵌入版本。

本轮不重跑已暂停的找物批量仿真。沿用 search1 已有验证记录，新增检查限于固件编译、初始化对应关系和实际综合布线报告。烧录与实机导航另行记录。

## 当前状态

- 新固件已编译；PNR84 于 08:09 完成综合／布线（均退出码 0，无 ERROR），实际时序和资源报告已验收。clk50 Fmax 50.052 MHz；rgmii_rxc 142.001 MHz；camera_pclk 109.069 MHz；lcd9 68.993 MHz。四个时钟的 setup／hold TNS 均为 0，setup／hold 违例端点均为 0，BSRAM 318/340。
- RK 当前连接超时，本轮尚未确认新 ACTIVE 已加载。最近已确认运行的是 live12＋3°，该版本没有 search1 的两项修改。
- Flash 既有记录为 PNR82。本轮没有烧录 PNR83／PNR84，没有下发运动目标。
- ZCode 的找物完成判定网桥补丁已在本地修改并测试；最新 ZCode 记录仍为未部署，本轮等待连接后核对实际状态。

已同步首页表格。构建与验收记录为 `D:\tools\firmware_update_search1_yaw3_20261103\manifest.json`、`pnr84_state.json`、`pnr84_review.json`。RTL 与约束沿用原版本，时序数字与 PNR83 相同；RAM 初始化已更新为新编译固件。保留原有相机时钟 generic-routing 警告，本轮未新增此项约束。

截至本轮结束，hky 的 Tailscale 状态仍为离线（最后在线 UTC 2026-11-02 20:42:43，东八区 11-03 04:42:43）。本轮没有选择新 ACTIVE、没有给车下发运动目标、没有烧录 Flash；待 RK 联网后，先复读空闲状态，再更新 ACTIVE 并核对 keeper 加载的新启动版本。


## 后续代码检查：暂缓实机加载

PNR84 的编译／时序验收结论保持有效，但本轮发现两个原有公共控制问题，不能视为任务逻辑已验收。对冻结源码使用现有模拟 HAL 做了三个定向 C 案例，无实车运动、无 MuJoCo 批量仿真：

- SEARCH 切到 GOAL(1130,230) 正常：进入 GOTO，并清除旧探索目标有效标志与路径。
- TRACK 已到站位点、目标在左侧、旧前视点在正前方时，原地朝向指令被公共速度处理覆盖，输出前进 30 mm/s、转速 0。
- 前一周期速度 300 mm/s、前方 400 mm 有障碍时，足迹检查返回 MOTION_HAZARD；其后的减速斜坡仍输出前进 250 mm/s，未再次检查。

对应 `nav_task.c` 的公共指令改写段（约 869–893 行）和检查后的减速段（约 932–938 行）。这两个问题在 search1 合入前已存在；search1 自身另有一项全模式影响：起始外接圆内致命格的退出规则位于共用 `motion_guard.c`，同样作用于定点导航。它是之前获批的行为变更，不属于探索目标抢占。

复现程序、结果和明细：`D:\tools\search_goto_review_20261103`。固件 BIN／FS 未改动；先修复公共控制逻辑并验证，再生成替代版本上车。部署脚本已加已知问题阻断；这不是要求用户重新标定或上电。


## 13:23 用户要求先烧现有版本：已完成

用户明确要求“直接烧进去先，其他的你自己慢慢改”，并确认已支撑小车、电机驱动关闭。按此要求，保留原 PNR84 的 BIN／FS，未将本地修复混入本次烧录。

- 下载器 location 36，设备 ID 0x0011081B，沿用 GW5AT-138C、操作 54（外置 Flash 擦除／编程／校验）。
- `D:\tools\flash_pnr84.log` 返回 `Program and Verify Flash successfully.`，退出码 0，用时 154.4 秒。
- 旧 wp_c 定点任务已取消，烧录前导航急停、输出为零；暂停 keeper／网桥后将 ACTIVE 原子切换为 `nav_fw_search1_yaw3_20261103.bin`，旧选择已备份。
- 写入后网桥与 keeper 已恢复，ACTIVE 与启动标识 `Nov  3 2026 07:49:47` 核对成功。状态 idle、vx=0、wz=0，未下发新目标。IMU 当前仍报错 4／数据过期，后续使用前需整车断电重启。
- 提权密码恢复服务的命令曾被自动审批拦截；随后通过既有免交互权限启动网桥，核实网桥和 keeper 均 active，恢复步骤已完成。
- 原有两处公共控制问题仍在已部署镜像中。主 `fw` 的本地 cmdfix1 修改已通过 8 项定向 C 检查，冻结源码 `fw_search1_yaw3_cmdfix1_20261103`；它尚未部署，不影响本次已烧入镜像。

实际部署记录：`D:\tools\firmware_update_search1_yaw3_20261103\flash_deployment_20261103.json`。本地修复记录与复现：`D:\tools\command_finalization_fix_20261103`。本节取代前文“尚未烧录／部署待联网／暂缓”的历史状态。

本地 cmdfix1 后续完成目标平台编译：BIN `D:\tools\nav_fw_search1_yaw3_cmdfix1_20261103.bin`，51,140 字节，text 51,138 字节，BSS 70,200 字节；启动标识 `Nov  3 2026 13:25:23`。8 项定向 C 检查通过；未部署、未替换当前 Flash／ACTIVE，不宣称已完成实车验证。
