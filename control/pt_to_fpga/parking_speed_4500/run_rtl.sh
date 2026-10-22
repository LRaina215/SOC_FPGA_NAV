#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
VECTOR_DIR="$ROOT/assets/vectors"
SAMPLES=256
LABEL=synthetic
RUN_UNITS=1
while (($#)); do
    case "$1" in
        --vector-dir) VECTOR_DIR=$2; shift 2 ;;
        --samples) SAMPLES=$2; shift 2 ;;
        --label) LABEL=$2; shift 2 ;;
        --skip-units) RUN_UNITS=0; shift ;;
        -h|--help)
            printf '%s\n' 'Usage: run_rtl.sh [--vector-dir PATH] [--samples N] [--label NAME] [--skip-units]'
            exit 0 ;;
        *) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
    esac
done
[[ "$SAMPLES" =~ ^[0-9]+$ && "$SAMPLES" -ge 1 && "$SAMPLES" -le 4096 ]] || { printf 'Invalid samples\n' >&2; exit 2; }
[[ "$LABEL" =~ ^[a-zA-Z0-9_.-]+$ ]] || { printf 'Invalid label\n' >&2; exit 2; }
PRIVATE="$ROOT/tools/iverilog/usr"
IVL_ARGS=()
if [[ -z ${IVERILOG:-} && -x "$PRIVATE/bin/iverilog" ]]; then
    IVERILOG="$PRIVATE/bin/iverilog"
    IVL_ARGS=(-B "$PRIVATE/lib/x86_64-linux-gnu/ivl")
fi
IVERILOG=${IVERILOG:-iverilog}
VVP=${VVP:-"$PRIVATE/bin/vvp"}
if [[ ! -x "$VVP" ]]; then VVP=vvp; fi
BUILD="$ROOT/tb/build/$LABEL"
mkdir -p "$BUILD" "$ROOT/reports"
LOG="$ROOT/reports/rtl_${LABEL}.log"
exec > >(tee "$LOG") 2>&1
printf 'RTL verification vector_dir=%s samples=%s label=%s\n' "$VECTOR_DIR" "$SAMPLES" "$LABEL"
python3 "$ROOT/tb/check_vectors.py" --root "$ROOT" --vector-dir "$VECTOR_DIR" --samples "$SAMPLES"
compile_run() {
    local name=$1
    shift
    "$IVERILOG" "${IVL_ARGS[@]}" -g2012 -Wall -s "$name" -o "$BUILD/$name.vvp" "$@" "$ROOT/tb/$name.v"
    "$VVP" "$BUILD/$name.vvp"
}
if ((RUN_UNITS)); then
    compile_run tb_fc_v4_round_sat "$ROOT/rtl/fc_v4_round_sat.v"
    compile_run tb_fc_v4_datapath "$ROOT/rtl/fc_v4_top.v" "$ROOT/rtl/fc_v4_round_sat.v"
    compile_run tb_fc_v4_history "$ROOT/rtl/fc_v4_history_buffer.v"
fi
"$IVERILOG" "${IVL_ARGS[@]}" -g2012 -Wall -s tb_fc_v4_top -o "$BUILD/tb_fc_v4_top.vvp" \
    "$ROOT/rtl/fc_v4_top.v" "$ROOT/rtl/fc_v4_round_sat.v" "$ROOT/tb/tb_fc_v4_top.v"
"$VVP" "$BUILD/tb_fc_v4_top.vvp" "+VECTOR_DIR=$VECTOR_DIR" "+SAMPLES=$SAMPLES"
printf 'PASS RTL_SUITE label=%s samples=%s\n' "$LABEL" "$SAMPLES"
