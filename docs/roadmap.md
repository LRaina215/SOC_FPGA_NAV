# 导航模块进度与后续计划（接续工作请先读本文件）

详细实验记录与数据见 [algorithms.md](algorithms.md)。Python 用 `D:/Anaconda/python.exe`（PATH 里的 msys python 没有 numpy）。

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
- 车体参数按实车更新：180 × 213 mm，外接圆 140 mm，膨胀 3 格。
- 闭环仿真 + 批量评测：无图 4 用例 20/20（xy RMS 0.20–0.41 m）；有图 6 用例 30/30（xy RMS 1.3–4.9 cm，
  含扫图后新增障碍、行驶中被撞），全部 0 碰撞。

## 已知问题 / 待定

1. 场地（含墙）须 ≤ 12.8 m × 12.8 m（片上结构图 256×256×5 cm）；更大需 DDR3 存整张定位图、片上窗口随机器人换页（固件已按窗口原点设计）。
2. 无图模式探索新区域时的打滑漂移（0.2–0.5 m / 10 m）：无图时固有。
3. 有图对准在"视野里只有平行结构"时要原地转 1–3 次（每次约 3.4 s），初始摆放时车头尽量对着有转角的地方。
4. 软核片上 RAM：板级 BSRAM 余量约 112 块（≈ 250 KB），GoRV32 / PicoRV32 选定后核算（固件新增：先验层 9 KB、相机已看过 9 KB）。
5. 底盘能力（轮径、可跨台阶、坡度）未给，仿真沿用可跨 10 cm、坡度 20°；实车给出后改 `world.Robot` 与地形阈值。

## 后续计划（按顺序）

1. 实地扫图：录 XT-M60 + IMU088（+ 轮速），Linux SLAM 出点云，`map_tool.py` 转包，检查图核对后下发。
2. 软核（GoRV32 / PicoRV32）：总线桥、固件交叉编译、实测 RAM 与主循环耗时。
3. 实机接入：雷达内参写方向表、IMU088 协议、底盘速度接口（RL 运动控制）联调；PC 经串口走通地图 / 位姿 / 目标下发。
4. 算法侧可继续：DDR3 大场地、有图模式下动态障碍过期、多帧对准（转一圈拼接后再搜索）。

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
```
