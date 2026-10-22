#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/cyd/fpga/fpga
TESTS=$(cd -- "$(dirname -- "$0")" && pwd)
IVL_ROOT=/home/cyd/fpga/pt_to_fpga/parking_speed_4500/tools/iverilog/usr
cd "$ROOT"
python3 /home/cyd/fpga/tools/bmi088_oled_validation/make_axes_golden.py
for name in observation action_map policy_faults model_status_screen policy_system; do
 "$IVL_ROOT/bin/iverilog" -B "$IVL_ROOT/lib/x86_64-linux-gnu/ivl" -g2012 \
  -s "tb_$name" -o "/tmp/mem_policy_test_$name" src/*.v "$TESTS/tb_$name.v"
 "$IVL_ROOT/bin/vvp" "/tmp/mem_policy_test_$name"
done
python3 "$TESTS/check_system.py"
