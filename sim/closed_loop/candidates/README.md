# 导航隔离试验

这些文件不进入生产固件；接受基线仍为N1c，工作区生产C仍为AS。所有候选保留原4500权重、runtime和控制代码。

`build_support_candidate.py`先复制工作区`fw/`，应用冻结的`n2_support_coverage.patch`（AY），然后按`--extra-patch`顺序应用实验补丁。上下文不匹配立即失败。默认不加实验补丁仍重建AY。

当前评审入口：

- `n2_small_scan_exit_anchor.patch`：BJ。显式保留轮中心锚定、双侧±20°观察、在原±45°目标窗口查验退出走廊；包括与新策略配套的反馈观察测试。主机通过不代表坡道/深沟任务通过。
- `n3_stop_envelope_v2.patch`：BH2。锚定、实际速度响应管道、受检线性/航向保持与停止确认；将原车体与新增响应余量分开，余量未知不能借用为当前占用，停止确认同时约束启动观察。33项C合同通过。`stop_guard.c/.h`是相同保持模块的直接阅读入口；编译时以补丁内文件为准。离线参数未标定，地形停止尚未验收，不能部署。
- `n2_anchor_contacts.patch`：只有锚定逻辑的早期阅读入口；`n2_side_scan.patch`是BA的±60°诊断；`n2_route_scan_anchor.patch`是BE；`n3_stop_envelope.patch`是BC的仅线性保持，`n3_stop_envelope_xyaw.patch`是BH。它们均为AY上的独立分支，不能串行叠加。

例如，从仓库根目录构建BJ：

```powershell
$env:Path='C:\msys64\mingw64\bin;'+$env:Path
D:\Anaconda\python.exe -X utf8 sim/closed_loop/build_support_candidate.py --out sim/build/review_bj --extra-patch sim/closed_loop/candidates/n2_small_scan_exit_anchor.patch --host-tests
```

输出目录必须不存在。实验数据、失败记录、资源与未完成项见`docs/1_plan/nav_optimization_plan.md`的AZ章节及`sim/build/terrain4500_20261022/n2av/n2az/INDEX.md`。原94例筛查不包含本轮诊断；重复同种子不计独立验证。
