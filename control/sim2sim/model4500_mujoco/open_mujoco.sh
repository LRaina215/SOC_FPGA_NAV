#!/usr/bin/env bash
set -euo pipefail
cd /home/cyd/fpga/sim2sim/model4500_mujoco
export LD_LIBRARY_PATH=/home/cyd/miniforge3/envs/mujoco/lib:${LD_LIBRARY_PATH:-}
exec /home/cyd/miniforge3/envs/mujoco/bin/python -u run.py "$@"
