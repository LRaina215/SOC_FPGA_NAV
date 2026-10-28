# 原4500轮速模型

本目录只保留最初4500版本及其配套文件，不是2026-10-19重新训练的4500，也不是6500/7000版本。

| 文件/目录 | 用途 |
| --- | --- |
| `model_4500.pt` | 唯一实体模型检查点；本地预览目录中的同名文件为指向它的软链接 |
| `model_4500_config.json` | 对应训练参数 |
| `model_4500_evaluation.json` | 原有评估记录；不代表实车验证 |
| `model_4500_manifest.json` | 模型来源及SHA256 |
| `model_4500_training_archive.tgz` | 对应训练代码归档 |
| `mem_4500/` | FPGA定点权重、配置、误差报告和测试向量 |
| `wheel_velocity_original/` | 该4500模型的本地预览及代码依赖，已移除3000检查点和冗余日志 |

SHA256：`a0c0024e132b71d10fc42cc92081129e2738e8148b0191f56acf7eecfc73d99b`。轮子动作为独立目标速度，满幅±0.6m/s；腿部为高度IK加±0.03rad修正。该模型先训练3000轮，再停车微调1500轮。

本地预览入口：

```bash
bash /home/cyd/fpga/model/wheel_velocity_original/open_parking_preview.sh
```

预览程序依赖 `rl_ws` 中的运行环境。FPGA 工程的 ROM 副本位于 `control/fpga/src/mem`。

其他模型、试验目录与历史日志已移到可恢复备份：`/home/cyd/fpga/_model_backups/20261021_110907`。备份中的`cleanup_manifest.json`逐项记录原位置，未永久删除。
