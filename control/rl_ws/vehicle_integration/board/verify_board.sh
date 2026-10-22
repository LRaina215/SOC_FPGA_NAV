#!/usr/bin/env bash
set -euo pipefail
board="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
root="$(dirname -- "$board")"
python_bin="${BOARD_PYTHON:-python3}"
export PYTHONDONTWRITEBYTECODE=1
fail() {
    local status=$1
    trap - ERR INT TERM
    set +e
    if ! "$python_bin" "$board/acceptance_state.py" failed; then
        printf 'Acceptance invalidation failed; removing stale reports.\n' >&2
        rm -f -- "$board/reports/acceptance.json" "$board/reports/ACCEPTANCE.md" \
            "$board/reports/acceptance.json.tmp" "$board/reports/ACCEPTANCE.md.tmp"
    fi
    exit "$status"
}
trap 'fail "$?"' ERR
trap 'fail 130' INT
trap 'fail 143' TERM
mkdir -p -- "$board/reports"
"$python_bin" "$board/acceptance_state.py" pending
"$python_bin" "$board/check_baseline.py" > "$board/reports/baseline_before.json"
"$python_bin" "$board/manifest_inputs.py" --output "$board/reports/test_inputs_before.json"
"$python_bin" -m unittest discover -s "$board/host/tests" -v 2>&1 | tee "$board/reports/host_tests.log"
bash "$board/run_rtl.sh"
"$python_bin" "$board/host/client.py" --offline "$board/reports/rtl_reply_nominal.hex" \
    --samples 1 --start-index 0 --initial-sticky-saturation 0 --no-report > "$board/reports/host_capture_nominal.json"
"$python_bin" "$board/host/client.py" --offline "$board/reports/rtl_reply_stress.hex" \
    --samples 1 --vector-dir "$root/assets/vectors/stress" --start-index 1 \
    --initial-sticky-saturation 0 --no-report > "$board/reports/host_capture_stress.json"
"$python_bin" "$board/manifest_inputs.py" --check "$board/reports/test_inputs_before.json" \
    --output "$board/reports/test_inputs_after.json"
bash "$board/build_board.sh" --build --rtl-ready
"$python_bin" "$board/manifest_inputs.py" --check "$board/reports/test_inputs_before.json" \
    --output "$board/reports/test_inputs_after_build.json"
"$python_bin" "$board/check_baseline.py" > "$board/reports/baseline_after.json"
"$python_bin" "$board/summarize_board.py"
trap - ERR INT TERM
printf 'Board preparation verified. No programming or physical serial access.\n'
