#!/usr/bin/env bash
set -euo pipefail
# Run only with the robot supported, still, and its wheels clear of the ground.
cd /home/cyd/fpga/fpga
python3 - <<'CHECK'
import json,hashlib
from pathlib import Path
manifest=json.loads(Path('/home/cyd/fpga/tools/fpga_telemetry/bitstream_manifest.json').read_text())
for path,expected in manifest['sha256'].items():
    assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==expected, 'Rebuild required: '+path
print('Verified bitstream and source identity')
CHECK
exec /home/cyd/GW_fpga/Gowin_V1.9.12.04_linux/Programmer/bin/programmer_cli \
    --device GW5AT-138C --operation_index 2 --cable-index 5 --location 785 \
    --fsFile /home/cyd/fpga/fpga/impl/pnr/fpga.fs
