#!/bin/bash
# 2026-10-11 奖励重设计版（reward_v3）部署：同步 3 个改动文件到服务器，起新一轮训练。
# 用法：AutoDL 实例开机后，在本目录执行  ./deploy_reward_v3.sh
# 改动内容见 _env_backup/2026-10-11_reward_v3/ 的对照与代码内 2026-10-11 注释。
set -e
cd "$(dirname "$0")"
. ./.sshcred
LOCAL=../Wheel-Legged-Gym-SF_small/wheel_legged_gym

echo "[1/4] 探活……"
./rc "echo alive"

echo "[2/4] 备份服务器原文件（只备份一次，已存在则跳过）"
./rc "[ -e /root/autodl-tmp/backup_20261011/car_robot_config.py ] || (mkdir -p /root/autodl-tmp/backup_20261011 && cp -a /root/autodl-tmp/wheel_legged_gym/envs/car_robot/car_robot_config.py /root/autodl-tmp/wheel_legged_gym/envs/car_robot/car_robot.py /root/autodl-tmp/wheel_legged_gym/utils/terrain.py /root/autodl-tmp/backup_20261011/) && ls /root/autodl-tmp/backup_20261011/"

echo "[3/4] 上传 3 个文件"
/home/cyd/miniforge3/bin/sshpass -p "$PASS" scp -P "$PORT" -o StrictHostKeyChecking=no \
  "$LOCAL/envs/car_robot/car_robot_config.py" "$LOCAL/envs/car_robot/car_robot.py" \
  root@"$HOST":/root/autodl-tmp/wheel_legged_gym/envs/car_robot/
/home/cyd/miniforge3/bin/sshpass -p "$PASS" scp -P "$PORT" -o StrictHostKeyChecking=no \
  "$LOCAL/utils/terrain.py" \
  root@"$HOST":/root/autodl-tmp/wheel_legged_gym/utils/

echo "[4/4] 校验关键改动已在服务器上，然后起训练（全新一轮，不 resume）"
./rc "grep -c 'upright = 0.0' /root/autodl-tmp/wheel_legged_gym/envs/car_robot/car_robot_config.py && grep -c 'curriculum = True' /root/autodl-tmp/wheel_legged_gym/envs/car_robot/car_robot_config.py"
./rc "cd /root/autodl-tmp && export PATH=/root/miniconda3/bin:\$PATH && export PYTHONPATH=/root/autodl-tmp/isaacgym/python:/root/autodl-tmp && setsid nohup python3 -u wheel_legged_gym/scripts/train.py --task=car_robot --headless --num_envs=4096 --max_iterations=3000 > logs/train_\$(date +%Y%m%d_%H%M%S).log 2>&1 < /dev/null & echo '训练已启动'"
echo "完成。看进度：./rc \"tail -25 /root/autodl-tmp/logs/train_*.log\""
