#!/usr/bin/env bash
set -euo pipefail
cd /home/cyd/fpga/tools/fpga_telemetry
exec bash open_monitor.sh --record-only "$@"
