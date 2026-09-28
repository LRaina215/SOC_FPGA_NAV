# 导航模块进度与后续计划（接续工作请先读本文件）

详细实验记录与数据见 [algorithms.md](algorithms.md)。Python 用 `D:/Anaconda/python.exe`（PATH 里的 msys python 没有 numpy）。

## 已完成

- RTL 全链路（XT-M60 接收 → 点云 → 高程图 → 可通行性 v1 → 轨迹评估 → 限幅），与 Python 逐位模型一致，xsim 回归 10/10。
- 固件：全局记忆栅格 + A*、局部规划（含脱困遍）、卡住恢复、探索、跟踪、协议、位姿（三维航位推算、ZUPT、姿态 α-β + 突变跳帧、高度锚定）、SLF 匹配定位。
- 模型：地形 v3（`model/trav_v3.py`）、SLF 匹配（`model/slf_match.py`）。
- 闭环仿真 + 批量评测：3 用例 × 5 种子 15/15 到达、0 碰撞、误报 ≤ 0.2%、定位 xy RMS 0.21–0.38 m（见 algorithms.md 第 2 节）。

## 已知问题

1. 探索新区域时的打滑漂移（0.2–0.5 m / 10 m）：SLAM 固有，要么更好的里程计，要么先验地图/回环。
2. v3 地形与 SLF 匹配只在模型里，RTL 仍是 v1、无匹配硬件；闭环仿真里 FPGA 侧用的是模型（v1 部分与 RTL 逐位一致，v3/SLF 尚无 RTL）。
3. 上板 HAL：`hal_map_height`、`hal_match_eval` 返回不支持（需要寄存器表的高程读窗口与匹配硬件）。

## 后续计划（按顺序）

1. **v3 移植 RTL**：`trav_analysis.v` 增加 4 行行缓存做基线坡度、落差扫描（左/上用已扫描数据，右/下用反向扫描遍或前瞻缓冲）、近区保持；
   更新 `nav_fixed.py`（逐位模型）与 `nav_fast.py`，xsim 回归逐位一致；参数进 `config/nav_params.yaml` 与寄存器表。
2. **高程读窗口**：寄存器表加 MAP 高程读（按世界格坐标读 zmin/zmax/有效），实现上板 `hal_map_height`（区间中点）。
3. **SLF 硬件**：结构点提取（lidar 链路旁路）、结构图 RAM、法向建图遍、打分阵列；先做资源估算再定并行度；实现 `hal_match_eval`。
4. 重新综合（BSRAM 当前 47%、Fmax 100 MHz），检查资源与时序。
5. 算法侧可继续：更多场景（窄通道、动态障碍）、RK3588 目标跟踪闭环、打滑漂移（轮速-IMU 融合检测打滑）。

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
