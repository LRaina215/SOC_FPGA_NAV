#!/usr/bin/env python3
"""Audited build-only Gowin implementation. Never opens hardware devices."""
import argparse
import datetime
import hashlib
import html
from html.parser import HTMLParser
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
BOARD = HERE.parent
ROOT = BOARD.parent
GW_SH = Path('/home/cyd/fpga/GW_fpga/Gowin_V1.9.12.04_linux/IDE/bin/gw_sh')
CORE = [ROOT / 'rtl/fc_v4_round_sat.v', ROOT / 'rtl/fc_v4_top.v']
ROM = [ROOT / 'assets/weights.mem', ROOT / 'assets/bias.mem', ROOT / 'assets/elu.mem']
PINS = {'clk': ('W19', '4', 'in'), 'UART_RXD': ('L21', '3', 'in'),
        'UART_TXD': ('M21', '3', 'out'), 'MOTOR_EN': ('A20', '2', 'out'),
        'PWM_U': ('E17', '2', 'out'), 'PWM_V': ('D15', '2', 'out'),
        'PWM_W': ('F14', '2', 'out'), 'MOTOR_EN2': ('B20', '2', 'out'),
        'PWM_U2': ('F16', '2', 'out'), 'PWM_V2': ('D14', '2', 'out'),
        'PWM_W2': ('F13', '2', 'out'), 'SERVO1': ('C18', '2', 'out'),
        'SERVO2': ('C19', '2', 'out'), 'SERVO3': ('E19', '2', 'out'),
        'SERVO4': ('D19', '2', 'out')}

PINS.update({'SPI_MISO': ('E16', '2', 'in'), 'SPI_SCLK': ('E18', '2', 'out'), 'SPI_MOSI': ('F18', '2', 'out'), 'SPI_CSB1': ('F19', '2', 'out'), 'SPI_CSB2': ('F20', '2', 'out'), 'MT6701_A': ('C13', '2', 'in'), 'MT6701_B': ('A13', '2', 'out'), 'MT6701_I': ('A15', '2', 'out')})


PINS.update({'MT6701_A_R': ('B13', '2', 'in'), 'MT6701_B_R': ('A14', '2', 'out'), 'MT6701_I_R': ('A16', '2', 'out')})

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + '\n')


def observed_sha(path):
    try:
        return sha(path)
    except OSError as exc:
        return {'error': str(exc)}


def protected_snapshot():
    baseline = json.loads((BOARD / 'baseline.json').read_text())
    checked, mismatches = {}, []
    for group in ('protected_inputs', 'live_project'):
        for name, expected in baseline[group].items():
            path = Path(name) if group == 'live_project' else ROOT / name
            actual = observed_sha(path)
            checked[str(path)] = actual
            if actual != expected:
                mismatches.append({'group': group, 'path': str(path), 'expected_sha256': expected,
                                   'observed_sha256': actual})
    # Preserve independent original synthesis inputs and outputs as well.
    for path in sorted((ROOT / 'gowin').rglob('*')):
        if path.is_file():
            checked[str(path)] = observed_sha(path)
    return checked, mismatches


def baseline_check():
    checked, mismatches = protected_snapshot()
    fatal = [item for item in mismatches if item['group'] == 'protected_inputs']
    if fatal:
        raise RuntimeError('Refusing build: protected core/ROM inputs changed: ' +
                           ', '.join(item['path'] for item in fatal))
    return checked


def build_protected(snapshot):
    live_paths = set(json.loads((BOARD / 'baseline.json').read_text())['live_project'])
    return {path: value for path, value in snapshot.items() if path not in live_paths}


def inputs():
    board_rtl = sorted((BOARD / 'rtl').glob('*.v'))
    paths = CORE + ROM + board_rtl + [BOARD / 'baseline.json', HERE / 'board.cst',
                                    HERE / 'board.sdc', HERE / 'build.tcl', HERE / 'build.py',
                                    BOARD / 'build_board.sh']
    return {str(p): observed_sha(p) for p in paths}


def preflight():
    protected = baseline_check()
    current = inputs()
    top = BOARD / 'rtl/board_rl_top.v'
    if not top.is_file() or not re.search(r'\bmodule\s+fc_v4_board_top\b', top.read_text()):
        raise RuntimeError('Awaiting board/rtl/board_rl_top.v module fc_v4_board_top')
    cst = (HERE / 'board.cst').read_text()
    actual = dict(re.findall(r'IO_LOC\s+"([^"]+)"\s+(\w+)\s*;', cst))
    if actual != {name: value[0] for name, value in PINS.items()}:
        raise RuntimeError('CST does not match the 15 verified physical pads')
    if len(set(actual.values())) != len(PINS):
        raise RuntimeError('CST has a conflicting pin assignment')
    live = Path('/home/cyd/fpga/Documents/fpga_embedded_competition/src/fpga_embedded_competition.cst').read_text()
    live_pins = dict(re.findall(r'IO_LOC\s+"([^"]+)"\s+(\w+)\s*;', live))
    for name, pin in actual.items():
        # Right-wheel pads are newly assigned from user-confirmed P11 36/38/40,
        # cross-checked against CB/core schematics (B13/A14/A16).
        if name not in ('UART_RXD', 'UART_TXD','MT6701_A_R','MT6701_B_R','MT6701_I_R') and live_pins.get(name) != pin:
            raise RuntimeError('Active CST differs for ' + name)
    rom_stats = {}
    for path, expected in zip(ROM, (39936, 425, 3073)):
        words = [int(x, 16) for x in path.read_text().split()]
        if len(words) != expected or not any(words):
            raise RuntimeError('ROM depth/nonzero check failed: ' + str(path))
        rom_stats[path.name] = {'words': len(words), 'nonzero_words': sum(v != 0 for v in words), 'sha256': sha(path)}
    if not GW_SH.is_file() or not os.access(GW_SH, os.X_OK):
        raise RuntimeError('Installed gw_sh executable unavailable')
    return protected, current, rom_stats


class TimingHTML(HTMLParser):
    def __init__(self):
        super().__init__()
        self.section = ''
        self.tables = {}
        self.row = None
        self.cell = None
        self.text = []

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if tag == 'a' and attrs.get('name'):
            self.section = attrs['name']
        elif tag == 'tr':
            self.row = []
        elif tag in ('td', 'th'):
            if self.cell is not None:
                self.finish_cell()
            self.cell = []

    def finish_cell(self):
        if self.cell is not None and self.row is not None:
            self.row.append(' '.join(''.join(self.cell).split()))
        self.cell = None

    def handle_endtag(self, tag):
        if tag in ('td', 'th'):
            self.finish_cell()
        elif tag == 'tr':
            self.finish_cell()
            if self.row is not None:
                self.tables.setdefault(self.section, []).append(self.row)
            self.row = None

    def handle_data(self, data):
        self.text.append(data)
        if self.cell is not None:
            self.cell.append(data)


def analyze(run_dir, status):
    pnr = run_dir / 'fc_v4_board/impl/pnr'
    # create_project may use the requested directory directly on some versions.
    if not pnr.is_dir():
        pnr = run_dir / 'impl/pnr'
    status['implementation_directory'] = str(pnr)
    report = pnr / 'fc_v4_board.rpt.txt'
    timing = pnr / 'fc_v4_board_tr_content.html'
    fs = pnr / 'fc_v4_board.fs'
    status['bitstream'] = {'path': str(fs), 'exists': fs.is_file(), 'sha256': sha(fs) if fs.is_file() else None}
    if not report.is_file() or not timing.is_file():
        raise RuntimeError('Missing routed PnR/timing report; synthesis alone is not acceptance')
    text = report.read_text(errors='replace')
    parser = TimingHTML()
    parser.feed(timing.read_text(errors='replace'))
    tables = parser.tables
    status['routed_timing'] = {'report': str(timing), 'tables': tables}
    (run_dir / 'routed_timing.txt').write_text('\n'.join(parser.text))
    pin_rows = {}
    pin_section = re.search(r'7\. Pinout by Port Name(.*?)(?:\n8\.|\Z)', text, re.S)
    for line in (pin_section.group(1) if pin_section else '').splitlines():
        fields = [x.strip() for x in line.split('|')]
        if len(fields) > 16 and fields[4] in ('in', 'out', 'inout'):
            if fields[0] in pin_rows:
                raise RuntimeError('Duplicate routed port: ' + fields[0])
            pin_rows[fields[0]] = fields
    io_usage = re.search(r'^\s*I/O Port\s*\|\s*(\d+)/', text, re.M)
    pin_ok = set(pin_rows) == set(PINS) and bool(io_usage and int(io_usage[1]) == 26)
    for name, (pad, bank, direction) in PINS.items():
        row = pin_rows.get(name, [])
        pin_ok &= bool(row and row[2] == pad + '/' + bank and row[3] == 'Y' and
                       row[4] == direction and row[7] == 'LVCMOS33' and row[16] == '3.3' and
                       row[9] == ('UP' if name in ('UART_RXD','SPI_MISO','MT6701_A','MT6701_A_R') else 'NONE') and
                       (direction != 'out' or row[8] == '8'))
    status['pins'] = {'expected_pad_count': 26, 'actual_pad_count': len(pin_rows), 'verified': pin_ok, 'rows': pin_rows,
                      'report': str(report)}
    resource = re.search(r'3\. Resource Usage Summary(.*?)4\. I/O Bank', text, re.S)
    status['resource_summary'] = resource.group(1).strip() if resource else None
    vg_paths = list(run_dir.rglob('fc_v4_board.vg'))
    vg = vg_paths[0].read_text(errors='replace') if len(vg_paths) == 1 else ''
    init_values = re.findall(r'INIT_RAM_\w+\s*=\s*\d+\s*\x27h([0-9a-fA-F]+)', vg)
    init_by_rom = {}
    for name in ('weight_mem', 'bias_mem', 'elu_mem'):
        values = re.findall(r'defparam\s+[^;\s]*' + name + r'[^;\s]*\.INIT_RAM_\w+\s*=\s*\d+\s*\x27h([0-9a-fA-F]+)', vg)
        init_by_rom[name] = {'init_parameters': len(values), 'nonzero_parameters': sum(int(v, 16) != 0 for v in values)}
    status['rom_inference'] = {'netlist': str(vg_paths[0]) if len(vg_paths) == 1 else None,
                               'init_parameter_count': len(init_values),
                               'nonzero_init_parameter_count': sum(int(x, 16) != 0 for x in init_values),
                               'by_rom': init_by_rom, 'bsram_init_enabled': True,
                               'limitation': 'Nonzero per-ROM initialization evidence; not full netlist-to-ROM equivalence'}
    logs = '\n'.join(p.read_text(errors='replace') for p in run_dir.rglob('*.log'))
    warnings = sorted(set(line for line in logs.splitlines() if re.search(r'WARN|ERROR|unconstrain|no clock|not.*constrain', line, re.I)))
    status['warnings_and_coverage'] = warnings
    (run_dir / 'warnings_and_coverage.txt').write_text('\n'.join(warnings) + '\n')
    summary = dict(tables.get('STA_Tool_Run_Summary', []))
    status['routed_timing']['summary'] = summary
    fmax_rows = [r for r in tables.get('Max_Frequency_Report', []) if len(r) > 3 and r[1] == 'clk']
    fmax = float(re.sub(r'[^0-9.]', '', fmax_rows[0][3])) if len(fmax_rows) == 1 else None
    status['routed_timing']['actual_fmax_mhz'] = fmax
    clocks = [r for r in tables.get('Clock_Report', []) if len(r) > 4 and r[1] == 'clk']
    clock_ok = len(clocks) == 1 and clocks[0][3] == '20.000'
    violated = (summary.get('Numbers of Setup Violated Endpoints') == '0' and
                summary.get('Numbers of Hold Violated Endpoints') == '0')
    # Warning allowlist is intentionally empty until actual diagnostics are reviewed.
    # Unknown warnings, including unmatched SDC objects, fail closed.
    coverage_bad = bool(warnings)
    status['warning_classification'] = {'unreviewed': warnings, 'allowed': []}
    status['timing_gate'] = bool(clock_ok and violated and fmax is not None and fmax >= 50.0 and not coverage_bad)
    status['io_timing_scope'] = {'UART_RXD': 'asynchronous input; only first synchronizer D exception',
                                'UART_TXD': 'synthetic max/min 0 ns relative clk budget; not external synchronous interface',
                                'actuators': '12 constant-zero outputs; no clocked paths',
                                'unconstrained_internal_allowed': False}
    status['accepted'] = bool(status['timing_gate'] and pin_ok and fs.is_file() and
                              all(item['nonzero_parameters'] > 0 for item in init_by_rom.values()) and
                              re.search(r'BSRAM\s*\|\s*[1-9]', text) and
                              re.search(r'DSP\s*\|\s*[1-9]', text))
    if not status['accepted']:
        raise RuntimeError('Implementation acceptance failed; inspect routed timing, pins, ROM init and full logs')


def connection_status():
    result = subprocess.run(['lsusb'], text=True, capture_output=True)
    serial = []
    for directory in ('/dev/serial/by-id', '/dev/serial/by-path'):
        for path in sorted(Path(directory).glob('*')):
            serial.append({'path': str(path), 'target': str(path.resolve()), 'opened': False})
    data = {'timestamp': datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'inspection': 'read-only lsusb and filesystem enumeration only',
            'usb_enumeration_exit_code': result.returncode, 'usb_stdout': result.stdout, 'usb_stderr': result.stderr,
            'serial_links': serial, 'ftdi_0403_6014_present': '0403:6014' in result.stdout,
            'ftdi_classification': 'JTAG/download-cable candidate only, NOT a verified usable board UART',
            'ch9102_detected': bool(re.search(r'CH9102|1a86:55d[34]', result.stdout, re.I)),
            'board_uart_usable_verified': False,
            'board_connection_verified': False, 'programmer_executed': False,
            'serial_opened': False, 'serial_transmitted': False, 'programming_performed': False,
            'programmer_executable': '/home/cyd/fpga/GW_fpga/Gowin_V1.9.12.04_linux/Programmer/bin/programmer_cli',
            'programmer_document': '/home/cyd/fpga/GW_fpga/Gowin_V1.9.12.04_linux/Programmer/doc/EN/SUG502-2.5.2E_Gowin Programmer User Guide.pdf',
            'later_authorized_download_template': 'programmer_cli --device <verified GW5AT-138C CLI name> --run 2 --fsFile <approved bitstream.fs> --cable-index <verified> --location <verified> --uid <verified>',
            'restriction': 'Template only, never executed. SRAM mode 2 per local guide. Exact target/cable requires later human authorization; flash is separately unauthorized.'}
    write_json(BOARD / 'reports/connection_status.json', data)
    print(json.dumps(data, indent=2))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--build', action='store_true', help='Run independent synthesis + routing + bitstream generation only')
    ap.add_argument('--rtl-ready', action='store_true', help='Explicit coordinator confirmation that board RTL suite is stable')
    ap.add_argument('--connection-status', action='store_true', help='Read-only USB/serial availability; never invoke programmer')
    args = ap.parse_args()
    if args.connection_status:
        connection_status()
        return
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    run_dir = HERE / 'runs' / stamp
    run_dir.mkdir(parents=True)
    status = {'schema_version': 1, 'run_directory': str(run_dir), 'status': 'preflight',
              'programming_performed': False, 'accepted': False, 'timing_gate': False}
    try:
        before = inputs()
        protected, mismatches = protected_snapshot()
        write_json(run_dir / 'inputs_before.json', before)
        write_json(run_dir / 'protected_before.json', protected)
        status['protected_mismatches_before'] = mismatches
        status['input_hashes'] = before
        _, _, rom_stats = preflight()
        status['rom_inputs'] = rom_stats
        status['input_manifest_sha256'] = sha(run_dir / 'inputs_before.json')
        status['input_hashes'] = before
        status['source_list'] = [str(p) for p in CORE + sorted((BOARD / 'rtl').glob('*.v'))]
        if not args.build:
            status['status'] = 'preflight_pass_waiting_for_authorized_build'
        else:
            if not args.rtl_ready:
                raise RuntimeError('Build held: require --rtl-ready after coordinator stable-test notice')
            time.sleep(2)
            if before != inputs() or build_protected(protected) != build_protected(baseline_check()):
                raise RuntimeError('Input hashes changed during stability check; refusing build')
            env = dict(os.environ, BOARD_RUN_DIR=str(run_dir),
                       LD_LIBRARY_PATH=str(GW_SH.parent.parent / 'lib'),
                       QT_AUTO_SCREEN_SCALE_FACTOR='0', QT_ENABLE_HIGHDPI_SCALING='0',
                       QT_SCALE_FACTOR='1', QT_FONT_DPI='96', QT_QPA_PLATFORM='offscreen',
                       QT_QPA_OFFSCREEN_NO_GLX='1', QT_OPENGL='software', LIBGL_ALWAYS_SOFTWARE='1')
            for key in ('QT_PLUGIN_PATH', 'QT_QPA_PLATFORM_PLUGIN_PATH', 'QT_SCREEN_SCALE_FACTORS',
                        'XDG_SESSION_TYPE'):
                env.pop(key, None)
            with (run_dir / 'build.log').open('w') as log:
                result = subprocess.run([str(GW_SH), str(HERE / 'build.tcl')], cwd=run_dir,
                                        env=env, stdout=log, stderr=subprocess.STDOUT)
            status['tool_exit_code'] = result.returncode
            after = inputs()
            protected_after, mismatches_after = protected_snapshot()
            write_json(run_dir / 'inputs_after.json', after)
            write_json(run_dir / 'protected_after.json', protected_after)
            status['protected_mismatches_after'] = mismatches_after
            baseline_check()
            status['sources_unchanged'] = before == after and build_protected(protected) == build_protected(protected_after)
            if not status['sources_unchanged']:
                raise RuntimeError('Input edit detected during build; artifacts rejected')
            if result.returncode:
                raise RuntimeError('Gowin build failed, full output retained in build.log')
            analyze(run_dir, status)
            status['status'] = 'routed_50mhz_pass'
    except Exception as exc:
        status['status'] = 'failed_or_blocked'
        status['error'] = str(exc)
        print(str(exc), file=sys.stderr)
    finally:
        try:
            after = inputs()
            protected_after, mismatches_after = protected_snapshot()
            write_json(run_dir / 'inputs_after.json', after)
            write_json(run_dir / 'protected_after.json', protected_after)
            status['protected_mismatches_after'] = mismatches_after
            status['live_project_baseline_drift'] = [item for item in mismatches_after if item['group'] == 'live_project']
            status['live_project_changes_during_run'] = {
                path: {'before': protected.get(path), 'after': value}
                for path, value in protected_after.items()
                if path in json.loads((BOARD / 'baseline.json').read_text())['live_project']
                and protected.get(path) != value}
            status['sources_unchanged'] = before == after and build_protected(protected) == build_protected(protected_after)
            if not status['sources_unchanged'] or any(item['group'] == 'protected_inputs' for item in mismatches_after):
                status.update(status='failed_or_blocked', accepted=False, timing_gate=False,
                              error='Protected/build input changed; artifacts rejected')
        except Exception as exc:
            status.update(status='failed_or_blocked', accepted=False, timing_gate=False,
                          snapshot_error=str(exc))
    write_json(run_dir / 'build_status.json', status)
    write_json(HERE / 'build_status.json', status)
    print(json.dumps(status, indent=2))
    if status['status'] == 'failed_or_blocked':
        sys.exit(1)


if __name__ == '__main__':
    main()
