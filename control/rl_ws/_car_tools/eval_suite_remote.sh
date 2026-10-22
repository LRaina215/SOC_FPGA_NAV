#!/usr/bin/env bash
set -euo pipefail
cd /root/autodl-tmp
export PATH=/root/miniconda3/bin:$PATH
export PYTHONPATH=/root/autodl-tmp/isaacgym/python:/root/autodl-tmp
checkpoint="$1"
output="$2"
wheel_torque="${3:-0.5}"
min_height="${4:-0.09}"
velocity_origin="${5:-cad}"
for case in 0,0,0.1305 0.6,0,0.1305 -0.6,0,0.1305 0,2.9,0.1305 0,-2.9,0.1305 "0,0,$min_height" 0,0,0.19; do
    python -u balance_experiment.py eval --profile full --num-envs 64 --checkpoint "$checkpoint" --output "$output" --seconds 20 --case="$case" --wheel-torque "$wheel_torque" --min-height "$min_height" --velocity-origin "$velocity_origin"
done
python -u balance_experiment.py eval --profile full --num-envs 64 --checkpoint "$checkpoint" --output "$output/sweep" --seconds 20 --height-sweep --wheel-torque "$wheel_torque" --min-height "$min_height" --velocity-origin "$velocity_origin"
