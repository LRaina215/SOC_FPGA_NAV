#!/usr/bin/env bash
set -euo pipefail
cd /home/cyd/fpga/pt_to_fpga/parking_speed_4500
# Wait only for the collector launched for this export; keep results local.
COLLECTOR_PID=${1:-}
if [[ -n "$COLLECTOR_PID" ]]; then
    while kill -0 "$COLLECTOR_PID" 2>/dev/null; do sleep 5; done
fi
PYTHON=/home/cyd/miniforge3/envs/mujoco/bin/python
"$PYTHON" -c 'import json; s=json.load(open("reports/trajectory_collection.json")); assert len(s)==7; assert all(x["samples"]==400 for x in s)'
"$PYTHON" reference/validate.py --skip-export > reports/full_fixed_validation.log 2>&1
"$PYTHON" summarize_errors.py > reports/error_summary.log 2>&1
"$PYTHON" prepare_motion_rtl.py > reports/motion_vectors.log 2>&1
bash run_rtl.sh --vector-dir assets/vectors/rtl_motion --samples 56 --label trajectory_4500 --skip-units > reports/rtl_motion_launch.log 2>&1
echo ALL_NUMERICAL_AND_RTL_CHECKS_COMPLETE
