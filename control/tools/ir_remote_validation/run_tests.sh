#!/usr/bin/env bash
set -euo pipefail
TESTS=$(cd -- "$(dirname -- "$0")" && pwd)
IVL_ROOT=/home/cyd/fpga/pt_to_fpga/parking_speed_4500/tools/iverilog/usr
cd /home/cyd/fpga/fpga
for name in ir_remote ir_system; do
 "$IVL_ROOT/bin/iverilog" -B "$IVL_ROOT/lib/x86_64-linux-gnu/ivl" -g2012 \
  -s "tb_$name" -o "/tmp/test_$name" src/*.v "$TESTS/tb_$name.v"
 "$IVL_ROOT/bin/vvp" "/tmp/test_$name"
done
