# 导航模块进度与后续计划（接续工作请先读本文件）

详细实验记录与数据见 [algorithms.md](algorithms.md)。Python 用 `D:/Anaconda/python.exe`（PATH 里的 msys python 没有 numpy）。

## 已完成

- RTL 全链路（XT-M60 接收 → 点云 → 高程图 → **地形 v3** → 轨迹评估 → 限幅）+ **SLF 匹配定位硬件** + 高程读口，
  全部与 Python 模型逐位一致，xsim 回归 11/11；nav_subsystem 综合 105.6 MHz；板级构建（50 MHz）布线通过、
  无时序违例（布线后 68 MHz），BSRAM 66%，比特流已生成。
- 固件：全局记忆栅格 + A*、局部规划（含脱困遍）、卡住恢复、边界探索、跟踪、协议、位姿（三维航位推算、ZUPT、
  姿态 α-β + 突变跳帧、高度锚定）、SLF 匹配（异步打分、Hessian 退化判定）。上板 HAL 已实现高程读与匹配。
- 闭环仿真 + 批量评测：4 用例（定点 ×2、搜索、回环）× 5 种子 20/20 到达、0 碰撞、误报 ≤ 0.2%、漏报 2–4%、定位 xy RMS 0.23–0.37 m
  （仿真中 FPGA 侧与 RTL 用同一模型；打分延迟按硬件 40 ms 建模）。

## 已知问题 / 待定

1. 探索新区域时的打滑漂移（0.2–0.5 m / 10 m）：SLAM 固有，要么更好的里程计，要么先验地图/回环。
2. 软核片上 RAM：BSRAM 余量约 116 块（≈ 260 KB），软核选定后核算；不够时按 algorithms.md 第 7 节压缩。

## 后续计划（按顺序）

1. 软核选型后：固件交叉编译、实测 RAM 占用，打分耗时（约 40 ms/帧）是否需要并行化（如 3 行交织读，降到约 15 ms）。
2. 实机接入：雷达内参写方向表、IMU 协议确认、底盘速度接口（RL 运动控制）联调。
3. 算法侧可继续：更多场景（窄通道、动态障碍）、RK3588 目标跟踪闭环、轮速-IMU 融合检测打滑。

## 常用命令

```bash
PY=/d/Anaconda/python.exe
$PY sim/run.py                                   # RTL 回归
$PY fw/test/build.py                             # 固件主机测试
$PY sim/closed_loop/test_nav_fast.py             # 向量化模型一致性
$PY sim/closed_loop/eval_trav.py --trav v1 v3    # 地形单独评测
$PY sim/closed_loop/batch.py --tag x                        # 标准闭环评测（约 5 分钟）
$PY sim/closed_loop/run_cl.py --scene terrain --goal 9.3,2.8 --seed 2 --gif --out sim/build/cl/demo
```
