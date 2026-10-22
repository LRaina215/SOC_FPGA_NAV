#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "$0")/../.." && pwd)
IVL=/home/cyd/.iverilog/usr
OUT=$(mktemp -d /tmp/motor-response-check.XXXXXX)
echo "Check outputs: $OUT"
cd "$ROOT/fpga"
for entry in 'tb_control tb_motor520_control' 'tb_derivative tb_motor520_derivative' 'tb_pwm tb_motor520_pwm' 'tb_speed tb_motor520_speed'; do
 read -r top file <<< "$entry"
 "$IVL/bin/iverilog" -B "$IVL/lib/x86_64-linux-gnu/ivl" -g2012 -s "$top" -o "$OUT/$top" src/motor520*.v "$ROOT/tools/bmi088_oled_validation/$file.v"
 "$IVL/bin/vvp" "$OUT/$top"
done
for name in zero_feedback response_profile tuned_response uart_v2; do
 "$IVL/bin/iverilog" -B "$IVL/lib/x86_64-linux-gnu/ivl" -g2012 -s "tb_$name" -o "$OUT/$name" src/*.v "$ROOT/tools/motor_response_tuning/tb_$name.v"
 if [[ "$name" == uart_v2 ]]; then (cd "$OUT"; "$IVL/bin/vvp" "$OUT/$name"); else "$IVL/bin/vvp" "$OUT/$name"; fi
done
(cd "$OUT"; python3 "$ROOT/tools/motor_response_tuning/check_protocol.py")
# Existing IR integration also exercises continued model inference on key release/emergency stop.
for name in ir_remote ir_system; do
 "$IVL/bin/iverilog" -B "$IVL/lib/x86_64-linux-gnu/ivl" -g2012 -s "tb_$name" -o "$OUT/$name" src/*.v "$ROOT/tools/ir_remote_validation/tb_$name.v"
 "$IVL/bin/vvp" "$OUT/$name"
done
echo ALL_RESPONSE_CHECKS_PASSED
