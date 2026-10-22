#!/usr/bin/env bash
set -euo pipefail
cd /home/cyd/fpga/model/wheel_velocity_original
export PATH=/home/cyd/miniforge3/envs/sfgym/bin:/usr/bin:/bin:$PATH
export LD_LIBRARY_PATH=/home/cyd/miniforge3/envs/sfgym/lib:${LD_LIBRARY_PATH:-}
export PYTHONPATH="$PWD/vendor:/home/cyd/fpga/rl_ws/IsaacGym_Preview_4_Package/isaacgym/python"
exec /home/cyd/miniforge3/envs/sfgym/bin/python -u preview_parking.py \
    "$PWD/model_4500.pt" --config "$PWD/parking_config.json" \
    --snapshot "$PWD/parking_preview.png" "$@"
