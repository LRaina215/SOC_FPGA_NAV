# 导航模块进度与后续计划（接续工作请先读本文件）

详细实验记录与数据见 [algorithms.md](algorithms.md)，实物参数（机器人 / 传感器 / 场地，已知与假设）见 [platform.md](platform.md)。Python 用 `D:/Anaconda/python.exe`（PATH 里的 msys python 没有 numpy）。

## 已完成

- RTL 全链路（XT-M60 接收 → 点云 → 高程图 → **地形 v3** → 轨迹评估 → 限幅）+ **SLF 匹配定位硬件**（在线 / 有图两种模式）
  + 高程读口 + 主机邮箱串口注入 / 发送回读，全部与 Python 模型逐位一致，xsim 回归 11/11；
  nav_subsystem 综合 102.5 MHz、BSRAM 280/340；板级构建（50 MHz）布线 0 违例、BSRAM 68%，已出比特流。
- **两种模式**（选题 3 方案）：
  - 无图：局部高程图 / 可通行性 + 记忆栅格 A* + 边界探索；SLF 匹配自建局部结构图作激光里程计。
  - 有图：先验地图包（`tools/map_tool.py`：SLAM 点云 → 定位图 + 20 cm 代价图）经 PC 或 RK3588 下发；
    板上只定位不建图：SET_POSE 后 ±0.5 m / ±10° 对准、跟踪质量判定、丢失自动重定位；先验代价图 + 动态障碍层规划；
    有图搜索（找相机没看过的区域）。
- 主机工具 `tools/nav_host.py`（PC / RK3588 通用，UDP 或 USB 串口），导航不依赖 RK3588。
- 车体参数按实车更新：180 × 213 mm，外接圆 140 mm，膨胀 3 格（圆形）；两轮自平衡仿真模型、5 m 场地 field5（algorithms.md 第 9 节）：
  10 个用例 × 5 种子 49/50 到达，定位 xy RMS 1.0–4.2 cm。
- **软核上板**：PicoRV32 SoC（rtl/soc）+ 导航固件在 ACX750 上运行（主循环 5.7 万次/秒），PC 经 USB 串口装载程序、看控制台（board_bringup.md 第 4 节）。
- 闭环仿真 + 批量评测：无图 4 用例 20/20（xy RMS 0.20–0.34 m）；有图 6 用例 30/30（xy RMS 1.3–4.9 cm，
  含扫图后新增障碍、行驶中被撞），全部 0 碰撞。数据见 algorithms.md 第 2、8 节。

## 已知问题 / 待定

1. 场地已确认 ≤ 5 m × 5 m（室内自建），远小于片上结构图 12.8 m 的覆盖范围，不需要 DDR3。
2. 无图模式探索新区域时的打滑漂移（0.2–0.5 m / 10 m）：无图时固有。
3. 有图对准在"视野里只有平行结构"时要原地转 1–3 次（每次约 3.4 s），初始摆放时车头尽量对着有转角的地方。
4. **BSRAM 预算**：软核 RAM 160 KB 时板级 306/340（90%，此时雷达 MAC 未接、点云链路被优化掉）。接上雷达后点云链路（方向表 48 块、
   帧缓冲 19 块）恢复会超出。候选压缩：结构图槽字 23 -> 18 位（约 -28 块）、方向表只存两个分量（-16 块，待确认深度定义）、
   固件数据 106 -> 约 80 KB（代价图副本直接读窗口、A* 与 BFS 共用缓冲，RAM 降到 128 KB，-16 块）。
5. 底盘：两轮自平衡轮腿、轮径 65 mm；可跨台阶与坡度未测，暂按 20 mm / 12°（仿真仍是可跨 10 cm、坡度 20° 的四点着地模型，待改）。
6. 车身持续摆动：已改为陀螺辅助姿态滤波 + 帧时刻插值（需要 IMU 输出角速度，≥ 100 Hz）；IMU088 手册到后核对轴向与刻度。
7. PC 经 USB 串口注入报文、读回状态的链路只在 RTL 仿真（`tb_nav_top`）中验证，未上板；网口链路与 RK3588 相同。

8. 被撞检测偶尔漏检（pf5_kidnap 1/5），高速贴近箱角偶有擦碰：下一步按离障距离限速、用内点率突变辅助检测被撞。

## 后续计划（按顺序）

1. 实地扫图：录 XT-M60 + IMU088（+ 轮速），Linux SLAM 出点云，`map_tool.py` 转包，检查图核对后下发。
2. 软核（GoRV32 / PicoRV32）：总线桥、固件交叉编译、实测 RAM 与主循环耗时。
3. 实机接入：雷达内参写方向表、IMU088 协议、底盘速度接口（RL 运动控制）联调；PC 经串口走通地图 / 位姿 / 目标下发。
4. 算法侧（不需要实物，可在仿真里完成）：5 m 场地适配（规划栅格 10 cm、有图定位图 2.5 cm 或缩小）、自平衡车身摆动模型与姿态插值、多帧对准 / 全局重定位、有图模式动态障碍过期、终点朝向对准。

## 常用命令

```bash
PY=/d/Anaconda/python.exe
$PY sim/run.py                                   # RTL 回归
$PY fw/test/build.py                             # 固件主机测试
$PY sim/closed_loop/test_nav_fast.py             # 向量化模型一致性
$PY sim/closed_loop/eval_trav.py --trav v1 v3    # 地形单独评测
$PY sim/closed_loop/batch.py --tag x                        # 无图闭环评测（约 5 分钟）
$PY sim/closed_loop/batch.py --tag y --cases p_terrain_a p_terrain_b p_search p_loop p_perturb p_kidnap   # 有图（约 25 分钟）
$PY sim/closed_loop/test_prior.py                           # 有图固件单元测试
$PY sim/closed_loop/run_cl.py --scene terrain --goal 9.3,2.8 --seed 2 --gif --out sim/build/cl/demo
$PY sim/closed_loop/run_cl.py --scene terrain --search 8.0,-3.0 --prior auto --init-err rand --seed 2 --gif --out sim/build/cl/demo_prior_search
$PY tools/map_tool.py build map.pcd -o field.nvmp            # 地图点云 -> 先验地图包 + 检查图
$PY tools/nav_host.py --link uart:COM5 map field.nvmp        # PC 下发；pose / goal / wp / mode / listen
$PY gowin/syn/syn.py && $PY gowin/syn/syn.py --board          # 综合 / 板级构建
```
