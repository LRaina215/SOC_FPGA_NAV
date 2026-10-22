#!/usr/bin/env bash
set -euo pipefail
cd /home/cyd/fpga/rl_ws2
export PATH=/home/cyd/miniforge3/envs/sfgym/bin:/usr/bin:/bin:$PATH
export LD_LIBRARY_PATH=/home/cyd/miniforge3/envs/sfgym/lib:${LD_LIBRARY_PATH:-}
export PYTHONPATH=/home/cyd/fpga/model/wheel_velocity_original/vendor:/home/cyd/fpga/rl_ws2/IsaacGym_Preview_4_Package/isaacgym/python
exec /home/cyd/miniforge3/envs/sfgym/bin/python -u _car_tools/preview_car.py \
    /home/cyd/fpga/model/wheel_velocity_original/model_3000.pt \
    --config /home/cyd/fpga/model/wheel_velocity_original/config.json \
    --snapshot /home/cyd/fpga/model/wheel_velocity_original/preview.png "$@"
