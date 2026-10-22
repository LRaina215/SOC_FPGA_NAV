#!/usr/bin/env bash
set -euo pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$workspace/IsaacGym_Preview_4_Package/isaacgym/python:$workspace/Wheel-Legged-Gym-SF_small${PYTHONPATH:+:$PYTHONPATH}"
export PATH="/home/cyd/miniforge3/envs/sfgym/bin:$PATH"
cd "$workspace"
exec /home/cyd/miniforge3/envs/sfgym/bin/python -u "$workspace/_car_tools/balance_experiment.py" "$@"
