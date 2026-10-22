#!/bin/bash
# 服务器训练启动脚本（后台 nohup 运行）
export PATH=/root/miniconda3/bin:$PATH
cd /root/autodl-tmp
export PYTHONPATH=/root/autodl-tmp/isaacgym/python:/root/autodl-tmp

mkdir -p logs

# 启动训练（4096 环境）
nohup python -u Wheel-Legged-Gym-SF_small/wheel_legged_gym/scripts/train.py \
    --task=car_robot \
    --headless \
    --num_envs=4096 \
    --max_iterations=5000 \
    > logs/train_$(date +%Y%m%d_%H%M%S).log 2>&1 &

echo "训练已启动 PID=$!"
sleep 3
echo "日志文件:"
ls -t logs/*.log 2>/dev/null | head -1
