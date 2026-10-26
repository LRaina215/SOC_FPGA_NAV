当前只完成包装器准备和纯 Python 替身检查，没有真实求解观测结果。

最终准备目录：`F:/nav_integration_g2_20261023/final128_public_observer_prepared3`。`profile.json` 的 d/ZHI 为 null、root_approved 为 false，入口不能启动闭环。现有72格、原330秒、场景first-hit等待器均未改。

根选定profile后，另写 approved-profile.json，包含原入口路径、原 argv（从入口文件名起）、原 environment、新的独立输出路径、选定 d/ZHI 和 root_approved=true。然后用生成器 `prepare_final128_public_observer.py --prepare --profile <approved-profile.json> --out <new-directory>` 冻结新版本，不修改 prepared3。stage1b入口还要求其真实 best-distance bound_manifest 已存在并绑定；prior 比较保留原固定窗口和环境 d=0。

包装器在原入口完成 Python 配置后安装，再调用原 R.main。它只转发原 py_set_time/py_poll，读每个原轮询后的 py_get_match，并观察原 match_eval/match_update。DLL句柄的 argtypes/restype 仍转发给原函数，因此冷配置通过 self.d 发出的原调用也可观察。它不新增轮询、不调用额外 py_get_commit、不替换数学函数或打分结果、不重建 DLL。

`events.jsonl` 的打分回调返回是同步 Python 打分完成；稍后 `HAL_match_update` 回调表示更新/释放路径到达。后者还可能来自 alignment、stale/cancel 或没有发起打分的路径，因此不能称作每次 C solver 完成。能明确确认的是每个原轮询后 n_applied 的变化；零修正、退化轴也照实记录，不据此自动判丢失或通过。

时间使用原 py_set_time 的 uint32 微秒值和调用序号。`error_curve.csv` 保留原 timeseries 的时间轴；不按数组下标把提交采样和轨迹强行对齐。公开接口没有内部 n_runs、所有拒绝原因、n_in_best、at_edge；本包装器不会补造这些字段，也没有新设“丢定位”阈值。
