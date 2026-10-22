# 本仓库内的控制副本

此目录是原4500控制仓库的完整受跟踪文件副本，来源为
`D:/Desktop/SOC FPGA/control_repo_snapshot_20261021`的`main@6443f56`。
原本地快照只有这一层可见提交（浅克隆），此次按Git树导入845个受跟踪条目，保留文件类型和原内容，不建立嵌套Git仓库。没有在GitHub另建远程fork。

后续直接修改`control/`并在导航这个仓库提交，控制侧历史可用：

```powershell
git log --oneline -- control/
git diff -- control/
```

原4500策略、权重、训练、控制参数未因导入而改变。原始只读快照保留；`integration/control4500/original_src`继续作为6443f56的基线对照。845项包括上游本来已跟踪的模型、验证输入及旧FPGA构建文件；这些旧位流不是当前联合工程的通过证据，不能直接据此部署。新的共同工程仍复制源文件到独立构建目录，避免覆盖这里的历史输出。

共同构建、适配和当前资源结论见[总计划](../docs/1_plan/system_integration_plan_20261021.md)与[G2入口](../integration/g2/README.md)。以后控制修改、适配修改和对应验证结果应一起提交，禁止只改仓库外快照。

G2已改为读取`control/fpga/src`，通过受检补丁生成NAV接口，策略原入口仍保留。闭环仿真默认读取本目录；历史只读快照仍可显式传入用于配对。导入后的16帧/128ms短迁移检查中，两份来源的qpos/qvel/动作逐项相同；这个检查不代替整车或地形验收。

新增控制候选也在这里留历史：`tools/g2_observation_pipeline.py`仅生成独立构建目录中的轮速观测流水版，未覆盖原`fpga/src/rl_observation_v5.v`。有限RTL逐拍对照记录在G2报告；只有共同资源/时序通过后才能讨论接受，不能将候选生成器视为原4500已改版部署。

`tools/g2_fusion_pipeline.py`同样只生成构建副本，把模长平方与求和调整到原sqrt采样边界两侧；原`fpga/src/imu_fusion.v`、滤波/标定参数及4500权重保留。20万拍有限对照通过；首种写法触发Gowin内部综合错误的记录也保留，改写后的联合综合/PNR结果见G2报告。两种候选均需显式构建标志，未作为实机部署版本接受。
