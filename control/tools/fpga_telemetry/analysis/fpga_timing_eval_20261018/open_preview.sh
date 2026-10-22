#!/usr/bin/env bash
set -euo pipefail
cd /home/cyd/fpga/model/wheel_velocity_original
export PATH=/home/cyd/miniforge3/envs/sfgym/bin:/usr/bin:/bin:${PATH:-}
export LD_LIBRARY_PATH=/home/cyd/miniforge3/envs/sfgym/lib:${LD_LIBRARY_PATH:-}
export PYTHONPATH=/home/cyd/fpga/model/wheel_velocity_original/vendor:/home/cyd/fpga/rl_ws/IsaacGym_Preview_4_Package/isaacgym/python
exec python -u /home/cyd/fpga/tools/fpga_telemetry/analysis/fpga_timing_eval_20261018/preview_hardware.py
