#!/usr/bin/env bash
set -euo pipefail
cd /root/autodl-tmp/car_original_velocity
export PATH=/root/miniconda3/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PYTHONPATH="$PWD/vendor:/root/autodl-tmp/isaacgym/python"
export LD_LIBRARY_PATH=/root/miniconda3/lib
mkdir -p logs/parking_resume logs/parking_final
/root/miniconda3/bin/python -u deploy/parking.py train --checkpoint logs/parking_finetune/model_4300_resume.pt --iterations 200 --output logs/parking_resume > logs/parking_resume/console.log 2>&1
/root/miniconda3/bin/python -u deploy/parking.py eval --checkpoint logs/parking_resume/model_4500.pt --output logs/parking_final --all-commands > logs/parking_final/console.log 2>&1
cat logs/parking_final/evaluation.json
