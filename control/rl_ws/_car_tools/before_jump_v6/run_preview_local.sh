#!/usr/bin/env bash
set -euo pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$workspace/IsaacGym_Preview_4_Package/isaacgym/python:$workspace/Wheel-Legged-Gym-SF_small${PYTHONPATH:+:$PYTHONPATH}"
export PATH="/home/cyd/miniforge3/envs/sfgym/bin:/usr/bin:/bin:$PATH"
cd "$workspace"
exec /home/cyd/miniforge3/envs/sfgym/bin/python -u _car_tools/preview_car.py \
    _car_tools/results/logs/car_balance/full_center_v4/model_1000.pt \
    --config _car_tools/results/logs/car_balance/full_center_v4/config.json "$@"
