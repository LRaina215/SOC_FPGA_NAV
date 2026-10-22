#!/usr/bin/env python3
"""Invalidate published board acceptance before verification or after failure."""
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

REPORTS = Path(__file__).resolve().parent / 'reports'


def atomic_text(path, text):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(text)
    temporary.replace(path)


def publish(status):
    REPORTS.mkdir(exist_ok=True)
    data = {'status': status, 'timestamp_utc': datetime.now(timezone.utc).isoformat(),
            'levels': {'routed_bitstream_at_50mhz': 'not_currently_accepted',
                       'physical_board_inference': 'not_verified', 'real_vehicle_balance': 'not_verified'},
            'programming_performed': False}
    outputs = {'acceptance.json': json.dumps(data, indent=2) + '\n',
               'ACCEPTANCE.md': '# Board Preparation Acceptance\n\nStatus: ' + status
               + '\n\nNo current PASS is published. Inspect current verification/build logs.\n'
               'No FPGA programming or physical-board acceptance is claimed.\n'}
    error = None
    for name, text in outputs.items():
        path = REPORTS / name
        try:
            atomic_text(path, text)
        except OSError as exc:
            # Still invalidate the other report, and remove any stale PASS here.
            error = exc
            for candidate in (path, path.with_suffix(path.suffix + '.tmp')):
                try:
                    candidate.unlink(missing_ok=True)
                except OSError:
                    pass
    if error is not None:
        raise error


if __name__ == '__main__':
    publish(sys.argv[1] if len(sys.argv) > 1 else 'pending')
