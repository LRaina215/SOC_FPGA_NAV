# 导航模块进度与后续计划（接续工作请先读本文件）

最后更新：闭环仿真 + 可视化加入后（提交见 git log）。详细实验记录见 [algorithms.md](algorithms.md)。

## 已完成

- RTL 全链路（XT-M60 接收 → 点云 → 高程图 → 可通行性 v1 → 轨迹评估 → 限幅），与 Python 逐位模型一致，xsim 回归 10/10。
- 固件：A*、局部规划、探索、跟踪、协议、位姿（三维航位推算、零速陀螺零偏、姿态低通、足迹高度锚定）。
- 闭环仿真 `sim/closed_loop/run_cl.py`：场景真值、机器人误差模型、FPGA 模型、固件动态库；`--gif` 输出动画。
- 地形 v2（仅模型 `model/trav_v2.py`）：跳变门控差分、遮挡落差、空洞填补、按底盘半径膨胀。

## 已知问题（按严重程度）

1. **定位发散（可视化发现）**：`run_cl.py --scene terrain --goal 9.3,2.8 --trav v2 --seed 2 --calib 4 --time 100 --gif`
   中估计轨迹在第一次转向后就与真值分离（xy RMS 4.3 m，航向误差约 10°），结果可复现。
   同样参数的早先一次运行曾在 53 s 到达，说明其后的某次改动（姿态低通 / 零偏累计平均 / 足迹锚定 / 膨胀半径 5）
   引入了问题，需逐项回退二分定位。先看 `sim/build/cl/demo/run.gif`。
2. EICM 匹配定位有约 1 个候选步长的系统偏差，默认关闭。
3. 带噪声闭环中地形误报 10–18%（真值位姿下 0.2%）：多帧融合的位姿不一致。
4. v2 未移植 RTL；足迹锚定需要高程读窗口寄存器。

## 后续计划（按顺序）

1. **排查定位发散（问题 1）**：二分最近的定位改动，给闭环加"估计 vs 真值"的逐帧曲线（航向、xy、z）。
2. **多种子统计**：修复后跑 ≥5 个种子，报告到达率、耗时、定位误差、误报/漏报；动画逐个检查。
3. **改进匹配定位**：只用竖直结构点 + 关键帧结构二值图距离变换（似然场）打分；先模型，再定硬件结构。
4. **v2 移植 RTL**：`trav_analysis.v` 改跳变门控差分、加遮挡落差扫描寄存器、空洞填补并入膨胀遍；更新 `nav_fixed.py`，回归逐位一致。
5. **寄存器表加高程读窗口**，上板 HAL 实现 `hal_map_height`；匹配打分硬件 `match_eval` 与 `hal_match_eval`。
6. 以上稳定后重新综合，检查 BSRAM（当前 47%）与时序。

## 常用命令

```bash
python sim/run.py                          # RTL 回归
python fw/test/build.py                    # 固件主机测试
python sim/closed_loop/test_nav_fast.py    # 向量化模型一致性
python sim/closed_loop/eval_trav.py --trav v1 v2
python sim/closed_loop/run_cl.py --scene terrain --goal 9.3,2.8 --trav v2 --seed 2 --calib 4 --gif --out sim/build/cl/demo
python sim/closed_loop/run_cl.py ... --ideal | --noise att      # 理想传感器 / 单一误差源
```
