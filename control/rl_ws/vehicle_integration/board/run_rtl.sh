#!/usr/bin/env bash
set -euo pipefail
BOARD=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(cd -- "$BOARD/.." && pwd)
PRIVATE="$ROOT/tools/iverilog/usr"
IVL_ARGS=()
if [[ -z ${IVERILOG:-} && -x "$PRIVATE/bin/iverilog" ]]; then
    IVERILOG="$PRIVATE/bin/iverilog"
    IVL_ARGS=(-B "$PRIVATE/lib/x86_64-linux-gnu/ivl")
fi
IVERILOG=${IVERILOG:-iverilog}
VVP=${VVP:-"$PRIVATE/bin/vvp"}
if [[ ! -x "$VVP" ]]; then VVP=vvp; fi
BUILD="$BOARD/tb/build"
mkdir -p "$BUILD" "$BOARD/reports"
exec > >(tee "$BOARD/reports/rtl_board.log") 2>&1
printf 'Isolated board RTL regression root=%s\n' "$ROOT"
printf 'Digest is checkpoint identifier only; baseline source hashes bind build provenance.\n'
sha256sum "$ROOT/rtl/"*.v | tee "$BOARD/reports/rtl_core_before.sha256"
compile_run() {
    local top=$1
    shift
    "$IVERILOG" "${IVL_ARGS[@]}" -g2012 -Wall -s "$top" -o "$BUILD/$top.vvp" "$@" "$BOARD/tb/$top.v"
    "$VVP" "$BUILD/$top.vvp" "+ROOT=$ROOT"
}
compile_run tb_bmi088 "$BOARD/rtl/bmi088.v" "$BOARD/rtl/spi.v"
compile_run tb_board_uart "$BOARD/rtl/board_uart.v"
compile_run tb_board_rl "$BOARD/rtl/board_uart.v" "$BOARD/rtl/board_rl_bridge.v" "$BOARD/rtl/board_rl_top.v" \
    "$ROOT/rtl/fc_v4_top.v" "$ROOT/rtl/fc_v4_round_sat.v" "$BOARD/rtl/bmi088.v" "$BOARD/rtl/spi.v" "$BOARD/rtl/mt6701_ssi.v"
sha256sum "$ROOT/rtl/"*.v > "$BOARD/reports/rtl_core_after.sha256"
cmp "$BOARD/reports/rtl_core_before.sha256" "$BOARD/reports/rtl_core_after.sha256"
printf 'PASS BOARD_RTL_SUITE original_core_unchanged; captures rtl_reply_nominal.hex(id1/sample0) rtl_reply_stress.hex(id2/sample1)\n'
