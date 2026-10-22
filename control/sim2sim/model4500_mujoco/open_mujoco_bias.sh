#!/usr/bin/env bash
set -euo pipefail
exec bash /home/cyd/fpga/sim2sim/model4500_mujoco/open_mujoco.sh --forward-bias -0.01 "$@"
