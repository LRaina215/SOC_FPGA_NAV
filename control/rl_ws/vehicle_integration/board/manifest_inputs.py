#!/usr/bin/env python3
"""Hash the exact board regression and implementation inputs."""
import argparse
import hashlib
import json
from pathlib import Path

BOARD = Path(__file__).resolve().parent
ROOT = BOARD.parent


def files():
    result = [ROOT / 'rtl/fc_v4_top.v', ROOT / 'rtl/fc_v4_round_sat.v',
              ROOT / 'assets/weights.mem', ROOT / 'assets/bias.mem', ROOT / 'assets/elu.mem',
              BOARD / 'baseline.json']
    result.extend(BOARD / 'gowin' / name
                  for name in ('board.cst', 'board.sdc', 'build.tcl', 'build.py'))
    for directory in (BOARD / 'rtl', BOARD / 'tb', BOARD / 'host', BOARD / 'tests'):
        result.extend(p for p in directory.rglob('*') if is_input(p))
    result.extend(p for p in BOARD.glob('*.py') if p.is_file())
    result.extend(p for p in BOARD.glob('*.sh') if p.is_file())
    for directory in (ROOT / 'assets/vectors',):
        result.extend(p for p in directory.rglob('*') if is_input(p))
    return sorted(set(result))


def is_input(path):
    # tb/build is run_rtl.sh's output directory, never a regression source tree.
    return (path.is_file() and BOARD / 'tb/build' not in path.parents
            and '__pycache__' not in path.parts
            and path.suffix.lower() not in ('.pyc', '.pyo', '.vvp', '.vcd', '.fst'))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--check', type=Path)
    args = parser.parse_args()
    current = {str(path): digest(path) for path in files()}
    payload = {'schema_version': 1, 'files': current}
    if args.check:
        expected = json.loads(args.check.read_text())['files']
        if current != expected:
            missing = sorted(set(expected) - set(current))
            added = sorted(set(current) - set(expected))
            changed = sorted(name for name in set(current) & set(expected)
                             if current[name] != expected[name])
            raise SystemExit('Regression/build inputs changed: '
                             + json.dumps({'missing': missing, 'added': added, 'changed': changed}))
    args.output.write_text(json.dumps(payload, indent=2) + '\n')
    print(json.dumps({'status': 'pass', 'files': len(current), 'output': str(args.output)}))


if __name__ == '__main__':
    main()
