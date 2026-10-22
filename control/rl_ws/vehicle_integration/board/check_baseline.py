#!/usr/bin/env python3
"""Check immutable inference inputs and report unrelated live-project drift."""
import hashlib
import json
from pathlib import Path
import sys

BOARD = Path(__file__).resolve().parent
ROOT = BOARD.parent


def main():
    baseline = json.loads((BOARD / 'baseline.json').read_text())
    checked = []
    mismatches = []
    live_drift = []
    for group in ('protected_inputs', 'live_project'):
        for name, expected in baseline[group].items():
            path = Path(name) if group == 'live_project' else ROOT / name
            try:
                actual = hashlib.sha256(path.read_bytes()).hexdigest()
            except OSError as exc:
                actual = None
                error = str(exc)
            else:
                error = None
            item = {'path': str(path), 'expected_sha256': expected,
                    'sha256': actual, 'matches_original_baseline': actual == expected}
            if error:
                item['error'] = error
            checked.append(item)
            if actual != expected:
                (live_drift if group == 'live_project' else mismatches).append(item)
    report = {'status': 'fail' if mismatches else 'pass', 'checked': checked,
              'protected_input_mismatches': mismatches, 'live_project_drift': live_drift,
              'scope': 'Core/ROM identity gate; live project is not compiled or modified. '
                       'External drift is retained, not restored or rebaselined. '
                       'This is not hardware or timing evidence.'}
    print(json.dumps(report, indent=2))
    if mismatches:
        sys.exit(1)


if __name__ == '__main__':
    main()
