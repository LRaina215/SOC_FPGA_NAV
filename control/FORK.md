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
