#!/usr/bin/env python3
"""Publish board-preparation evidence, never physical-board acceptance."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import sys

from acceptance_state import atomic_text, publish
from manifest_inputs import files, digest

BOARD = Path(__file__).resolve().parent
ROOT = BOARD.parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def load(path):
    return json.loads(path.read_text())


def main():
    try:
        publish('summarizing')
        summarize()
    except BaseException:
        try:
            publish('failed')
        except OSError as exc:
            print('Acceptance invalidation failed: ' + str(exc), file=sys.stderr)
        raise


def summarize():
    reports = BOARD / 'reports'
    tested = load(reports / 'test_inputs_before.json')['files']
    current = {str(path): digest(path) for path in files()}
    require(tested == current, 'Current regression inputs differ from tested source/input set')
    for name in ('test_inputs_after.json', 'test_inputs_after_build.json'):
        require(load(reports / name)['files'] == tested, 'Regression inputs changed: ' + name)
    baseline = load(BOARD / 'baseline.json')
    for name, expected in baseline['protected_inputs'].items():
        require(sha(ROOT / name) == expected, 'Core/ROM identity changed: ' + name)
    build = load(BOARD / 'gowin/build_status.json')
    require(build.get('status') == 'routed_50mhz_pass' and build.get('accepted') is True,
            'No accepted routed implementation')
    require(build.get('timing_gate') is True and build.get('sources_unchanged') is True,
            'Timing or source stability gate failed')
    require(build.get('programming_performed') is False, 'Unexpected programming claim')
    require(build['pins']['verified'] and build['pins']['actual_pad_count'] == 26,
            'Routed physical pads are not verified')
    require(build['routed_timing']['actual_fmax_mhz'] >= 50, 'Routed Fmax below 50 MHz')
    expected_build_paths = {
        ROOT / 'rtl/fc_v4_top.v', ROOT / 'rtl/fc_v4_round_sat.v',
        ROOT / 'assets/weights.mem', ROOT / 'assets/bias.mem', ROOT / 'assets/elu.mem',
        BOARD / 'baseline.json', BOARD / 'gowin/board.cst', BOARD / 'gowin/board.sdc',
        BOARD / 'gowin/build.tcl', BOARD / 'gowin/build.py', BOARD / 'build_board.sh'}
    expected_build_paths.update((BOARD / 'rtl').glob('*.v'))
    require(set(build['input_hashes']) == {str(path) for path in expected_build_paths},
            'Current implementation input set differs from accepted build manifest')
    require(set(build['input_hashes']) <= set(tested),
            'Implementation inputs are missing from tested source/input set')
    for name, expected in build['input_hashes'].items():
        require(sha(Path(name)) == expected, 'Build input changed since accepted run: ' + name)
        require(tested[name] == expected, 'Tested inputs differ from accepted implementation: ' + name)
    bitstream = Path(build['bitstream']['path'])
    require(bitstream.is_file() and sha(bitstream) == build['bitstream']['sha256'],
            'Bitstream missing or changed')
    rtl_log = (reports / 'rtl_board.log').read_text()
    require('PASS SENSOR_TELEMETRY atomic/frozen/64bytes/CRC' in rtl_log, 'Sensor snapshot regression not passed')
    require('PASS BOARD_UART clk50MHz baud115200 divisor434' in rtl_log,
            'Default baud UART regression not passed')
    require('PASS BOARD_RTL_SUITE original_core_unchanged' in rtl_log,
            'Board RTL suite not passed')
    require('PASS simultaneous core8/RX5 core-priority first-error-wins drop_count1' in rtl_log,
            'Same-cycle error arbitration not passed')
    inference_rows = re.findall(r'^PASS INFER frame_id=(\d+) dataset=(\w+) sample_index=(\d+) '
                               r'actions6 latent3 debug425 raw6 cycles81998 flags=([0-9a-f]+) '
                               r'saturated=([01]) capture=([01])$', rtl_log, re.M)
    require(len(inference_rows) == 14, 'Expected 14 complete successful UART inference comparisons')
    require({int(row[2]) for row in inference_rows if row[1] == 'stress'} == set(range(8)),
            'Not all eight stress samples were tested')
    require({int(x) for x in re.findall(r'^PASS error code=(\d+)', rtl_log, re.M)} == set(range(1, 9)),
            'Not all eight protocol errors tested')
    host_log = (reports / 'host_tests.log').read_text()
    require(re.search(r'Ran 30 tests in .*\n\nOK\s*$', host_log) is not None,
            'Host 30-test suite not passed')
    captures = {}
    for name, sample, frame_id, flags, saturated in [('nominal', 0, 1, 0, 0), ('stress', 1, 2, 16, 1)]:
        report = load(reports / ('host_capture_' + name + '.json'))
        require(report['status'] == 'validated' and report['mode'] == 'offline'
                and report['physical_board_claim'] is False, 'Invalid capture mode/status')
        require(report['checkpoint_sha256'] == baseline['checkpoint_sha256'], 'Capture checkpoint differs')
        require(len(report['results']) == 1, 'Capture does not contain exactly one result')
        result = report['results'][0]
        for field, expected in [('sample_index', sample), ('frame_id', frame_id), ('cycles', 81998),
                                ('flags', flags), ('saturated', saturated), ('layers', 425),
                                ('latent', 3), ('actions', 6)]:
            require(result[field] == expected, 'Capture field mismatch: ' + name + '/' + field)
        captures[name] = result
    before = load(reports / 'baseline_before.json')
    after = load(reports / 'baseline_after.json')
    require(before['status'] == after['status'] == 'pass', 'Baseline verification failed')
    connection = load(reports / 'connection_status.json')
    require(not any(connection.get(key) for key in ('programmer_executed', 'serial_opened',
                                                   'serial_transmitted', 'programming_performed')),
            'Preparation evidence includes unexpected hardware operations')
    run_dir = Path(build['run_directory'])
    paths = set()
    for directory in [BOARD / 'rtl', BOARD / 'tb', BOARD / 'host', reports, run_dir]:
        paths.update(p for p in directory.rglob('*') if p.is_file()
                     and '__pycache__' not in p.parts and p.name not in ('acceptance.json', 'ACCEPTANCE.md'))
    paths.update(p for p in (BOARD / 'gowin').glob('*') if p.is_file())
    paths.update(p for p in BOARD.glob('*') if p.is_file())
    paths.update(ROOT / name for name in baseline['protected_inputs'])
    paths.update(Path(name) for name in tested)
    evidence = {str(path): sha(path) for path in sorted(paths)
                if path.name not in ('acceptance.json', 'ACCEPTANCE.md', 'acceptance.json.tmp', 'ACCEPTANCE.md.tmp')}
    summary = {'schema_version': 1, 'timestamp_utc': datetime.now(timezone.utc).isoformat(),
               'status': 'board_preparation_pass', 'checkpoint_sha256': baseline['checkpoint_sha256'],
               'levels': {'communication_and_inference_rtl': 'pass',
                          'routed_bitstream_at_50mhz': 'pass', 'physical_board_inference': 'not_verified',
                          'real_vehicle_balance': 'not_verified'},
               'programming_performed': False, 'actuator_operation_performed': False,
               'bitstream': build['bitstream'], 'routed_timing': build['routed_timing'],
               'resources': build.get('resource_summary'), 'rom_inference': build['rom_inference'],
               'pins': build['pins'], 'warnings_and_coverage': build['warnings_and_coverage'],
               'successful_uart_inferences': 14, 'stress_samples_checked': 8, 'host_tests': 30,
               'core_cycles': 81998, 'core_seconds_at_50mhz': 81998 / 50000000,
               'diagnostic_uart_line_seconds': 18500 / 115200, 'captures': captures,
               'live_project_drift': after['live_project_drift'],
               'live_project_hashes_changed_during_verifier': before['checked'] != after['checked'],
               'connection_report': str(reports / 'connection_status.json'),
               'build_run_directory': str(run_dir), 'evidence_sha256': evidence}
    fmax = build['routed_timing']['actual_fmax_mhz']
    text = f'''# Board Preparation Acceptance

Generated: {summary['timestamp_utc']}

## Result

- Communication and complete network RTL simulation: PASS.
- Independent synthesis, placement, routing and 50 MHz bitstream: PASS.
- Physical FPGA programming and inference: NOT PERFORMED / NOT VERIFIED.
- Real-vehicle balance: NOT VERIFIED.
- Motor and servo operation: NOT PERFORMED. All 12 actuator outputs are constant zero.

## Evidence

- Checkpoint SHA256: `{baseline['checkpoint_sha256']}`.
- Routed Fmax: {fmax:.3f} MHz, 20.000 ns requested clock; setup/hold gate passed.
- Exactly 26 physical pads verified against independent CST.
- 30 host tests, 14 successful complete UART inference comparisons, all 8 stress samples.
- Per successful inference: all 425 layers, 3 latent, 6 raw and 6 clipped actions checked.
- Core latency: 81998 clocks, 1.63996 ms at 50 MHz, excluding input loading and UART.
- Host separately decoded nominal and saturated RTL captures exactly.
- Diagnostic UART line time: 160.590 ms at 115200 baud, not a 125 Hz control interface.
- ROM initialization and inferred memory/DSP evidence are in the build report.

Bitstream: `{bitstream}`

Bitstream SHA256: `{build['bitstream']['sha256']}`

Build run: `{run_dir}`

`acceptance.json` contains routed timing, resources, warning/coverage details and
SHA256 identities for source files, evidence and artifacts.

## Isolation And Hardware Boundary

Core/ROM identities match the original baseline. The unrelated live vehicle
project changed externally after the original snapshot; its original and current
hashes are preserved in `baseline.json`, verifier reports and `acceptance.json`.
It was not restored, overwritten or compiled into this bitstream. Do not claim
that it remained historically unchanged.

No programmer or serial device was opened by this preparation. FTDI is only a
candidate download cable; the CH9102 board UART and target identity are unverified.
Later programming requires separate authorization, verified board/cable identity,
physical bank2 voltage verification, actuator power isolation and mechanical
support. Removing servo pulses can release loads, and configuration-time pad
levels are not guaranteed by constant-zero RTL. Flash is separately unauthorized.
'''
    atomic_text(reports / 'ACCEPTANCE.md', text)
    atomic_text(reports / 'acceptance.json', json.dumps(summary, indent=2) + '\n')
    print(json.dumps({'status': summary['status'], 'bitstream': str(bitstream),
                      'fmax_mhz': fmax, 'evidence_files': len(evidence),
                      'physical_board_inference': 'not_verified'}, indent=2))


if __name__ == '__main__':
    try:
        main()
    except (OSError, KeyError, ValueError, RuntimeError) as exc:
        print('Board acceptance failed: ' + str(exc), file=sys.stderr)
        sys.exit(1)
