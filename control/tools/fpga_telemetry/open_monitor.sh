#!/usr/bin/env bash
set -euo pipefail
cd /home/cyd/fpga/tools/fpga_telemetry
export LD_LIBRARY_PATH=/home/cyd/miniforge3/envs/sfgym/lib:${LD_LIBRARY_PATH:-}
exec /home/cyd/miniforge3/envs/sfgym/bin/python -u monitor.py "$@"
