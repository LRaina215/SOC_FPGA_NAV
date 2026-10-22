#!/usr/bin/env bash
set -euo pipefail
cd /root/autodl-tmp/car_original_velocity
export PATH=/root/miniconda3/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PYTHONPATH="$PWD/vendor:/root/autodl-tmp/isaacgym/python"
export LD_LIBRARY_PATH=/root/miniconda3/lib
if pgrep -f '^/root/miniconda3/bin/python -u deploy/run.py train' >/dev/null; then
    echo 'Original velocity training is already running';exit 0
fi
mkdir -p logs/preflight logs/original_velocity_v1
/root/miniconda3/bin/python -u deploy/run.py smoke --output logs/preflight > logs/preflight/console.log 2>&1
tail -3 logs/preflight/console.log
nohup /root/miniconda3/bin/python -u deploy/run.py train --iterations 3000 --output logs/original_velocity_v1 > logs/original_velocity_v1/console.log 2>&1 < /dev/null &
echo "$!" > training.pid
echo "Started original velocity training PID=$! target=3000 from scratch"
