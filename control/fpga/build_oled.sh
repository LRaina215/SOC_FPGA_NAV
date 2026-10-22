#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
export LD_LIBRARY_PATH=/home/cyd/GW_fpga/Gowin_V1.9.12.04_linux/IDE/lib
export QT_QPA_PLATFORM=offscreen QT_QPA_OFFSCREEN_NO_GLX=1 QT_OPENGL=software LIBGL_ALWAYS_SOFTWARE=1
exec /home/cyd/GW_fpga/Gowin_V1.9.12.04_linux/IDE/bin/gw_sh build_oled.tcl
