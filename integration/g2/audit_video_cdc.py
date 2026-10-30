"""Inventory every register-to-register crossing between clock domains in a flat netlist.

For each flip-flop D pin, walk back through combinational cells to the launching flip-flops
and report pairs whose clocks differ. Crossings that match the reviewed video structures are
classified; anything unclassified is reported as an error so no crossing is silently waived.

Usage: python audit_video_cdc.py <netlist.vg> [--sdc out.sdc] [--json out.json]
"""
import argparse
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from audit_pin_loads import CELLS, PORTS, NETS  # noqa: E402

CLOCKS = {'clk50_d': 'clk50', 'rgmii_rxc_d': 'rgmii_rxc', 'CAMERA_PCLK_d': 'camera_pclk', 'video_clock': 'lcd'}
PERIOD = {'clk50': 20.0, 'rgmii_rxc': 10.0, 'camera_pclk': 20.0, 'lcd': 1000.0 * 184 / (50.0 * 33)}
OUTPUT_PINS = {'Q', 'F', 'O', 'DO', 'DOUT', 'COUT', 'SUM', 'CO', 'Q0', 'Q1'}


def bits(expression):
    result = []
    for net, hi, lo in NETS.findall(expression):
        net = net.lstrip('\\')
        if net in ('GND', 'VCC'):
            continue
        if hi:
            a, b = int(hi), int(lo or hi)
            result += [f'{net}[{i}]' for i in range(min(a, b), max(a, b) + 1)]
        else:
            result.append(net)
    return result


def load(netlist):
    text = Path(netlist).read_text(encoding='utf-8')
    cells = {}
    driver = {}
    for match in CELLS.finditer(text):
        kind, name, body = match.groups()
        name = name.lstrip('\\')
        ports = dict(PORTS.findall(body))
        cells[name] = (kind, ports)
        for pin, expr in ports.items():
            if pin in OUTPUT_PINS or pin.startswith('DO') or (pin.startswith('Q') and kind.startswith('DFF')):
                for net in bits(expr):
                    driver[net] = name
    return cells, driver


def clock_of(cell):
    kind, ports = cell
    clk = bits(ports.get('CLK', ''))
    return CLOCKS.get(clk[0]) if len(clk) == 1 else None


def is_seq(cell):
    return cell[0].startswith('DFF')


def sources(cells, driver, start_nets, limit=4000):
    """Launching flip-flops (and BSRAM outputs) reachable backwards through combinational logic."""
    found = set()
    seen = set()
    stack = list(start_nets)
    while stack and len(seen) < limit:
        net = stack.pop()
        if net in seen:
            continue
        seen.add(net)
        name = driver.get(net)
        if name is None:
            continue
        cell = cells[name]
        if is_seq(cell) or cell[0] in ('SDPB', 'SDPX9B', 'SP', 'SPX9', 'DPB', 'DPX9B', 'pROM', 'pROMX9'):
            found.add(name)
            continue
        for pin, expr in cell[1].items():
            if pin in OUTPUT_PINS or pin.startswith('DO'):
                continue
            stack.extend(bits(expr))
    return found


def classify(src, dst, sclk, dclk):
    pairs = [
        ('hud_payload', r'u_video/hud/held_\d+_s\d*', r'u_video/hud/page_\d+_s\d*', 'clk50', 'lcd'),
        ('hud_request', r'u_video/hud/req_s\d*', r'u_video/hud/req_meta_s\d*', 'clk50', 'lcd'),
        ('hud_ack', r'u_video/hud/ack_s\d*', r'u_video/hud/ack_meta_s\d*', 'lcd', 'clk50'),
        ('stale_flag', r'u_video/stale_s\d*', r'u_video/stale_meta_s\d*', 'clk50', 'lcd'),
        ('frame_request_core', r'u_video/capture/frame_req_s\d*', r'u_video/request_meta_s\d*', 'camera_pclk', 'clk50'),
        ('frame_request_lcd', r'u_video/capture/frame_req_s\d*', r'u_video/store/req_meta_s\d*', 'camera_pclk', 'lcd'),
        ('frame_ack', r'u_video/store/frame_ack_s\d*', r'u_video/capture/ack_sync_0_s\d*', 'lcd', 'camera_pclk'),
        ('camera_ready', r'u_video/capture/.*', r'u_video/capture/ready_sync_0_s\d*', 'clk50', 'camera_pclk'),
        ('lcd_reset', r'.*', r'u_video/lcd_reset_pipe_\d+_s\d*', None, 'lcd'),
    ]
    for name, sp, dp, sc, dc in pairs:
        if re.fullmatch(sp, src) and re.fullmatch(dp, dst) and (sc is None or sc == sclk) and dc == dclk:
            return name
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('netlist')
    ap.add_argument('--sdc')
    ap.add_argument('--json')
    a = ap.parse_args()
    cells, driver = load(a.netlist)
    crossings = []
    for name, cell in cells.items():
        if not is_seq(cell):
            continue
        dclk = clock_of(cell)
        if dclk is None:
            continue
        for pin in ('D', 'CE', 'RESET', 'SET', 'CLEAR', 'PRESET'):
            if pin not in cell[1]:
                continue
            for src in sources(cells, driver, bits(cell[1][pin])):
                sclk = clock_of(cells[src])
                if sclk is None:
                    sk = bits(cells[src][1].get('CLKB', cells[src][1].get('CLK', '')))
                    sclk = CLOCKS.get(sk[0]) if sk else None
                if sclk and sclk != dclk:
                    crossings.append(dict(source=src, dest=name, pin=pin, source_clock=sclk, dest_clock=dclk))
    known_nav = re.compile(r'u_nav/u_netl/u_rxf/(wp_g_r1|rp_g_w1)_\d+_s\d*|u_nav/u_lmac/(rsync_0|neg_s_0)_s\d*|u_lmac/(rsync_0|neg_s_0)_s\d*')
    # Asynchronous-assert reset synchronizers: only CLEAR/PRESET crosses; checked by recovery/removal.
    reset_sync = re.compile(r'u_lrr/(r1|r2)_s\d*|u_lrr/gowin_replicate_reg_r2_\d+_\d+|u_video/capture/pclk_reset_[01]_s\d*')
    rows, unknown = [], []
    for c in crossings:
        c['group'] = classify(c['source'], c['dest'], c['source_clock'], c['dest_clock'])
        if c['group'] is None and known_nav.fullmatch(c['dest']):
            c['group'] = 'existing_nav_cdc'
        if c['group'] is None and c['pin'] in ('CLEAR', 'PRESET') and reset_sync.fullmatch(c['dest']):
            c['group'] = 'reset_synchronizer'
        (rows if c['group'] else unknown).append(c)
    loads = defaultdict(list)
    for name, cell in cells.items():
        for pin, expr in cell[1].items():
            if pin in OUTPUT_PINS or pin.startswith('DO'):
                continue
            for net in bits(expr):
                loads[net].append((name, pin))
    single_bit = {'hud_request', 'hud_ack', 'stale_flag', 'frame_request_core', 'frame_request_lcd',
                  'frame_ack', 'camera_ready'}
    for r in rows:
        if r['group'] not in single_bit:
            continue
        q = bits(cells[r['dest']][1].get('Q', ''))
        fan = loads.get(q[0], []) if len(q) == 1 else []
        ok = len(fan) == 1 and fan[0][1] == 'D' and is_seq(cells[fan[0][0]]) and clock_of(cells[fan[0][0]]) == r['dest_clock']
        r['second_stage'] = fan[0][0] if len(fan) == 1 else None
        if not ok:
            unknown.append(dict(r, problem='first stage does not drive exactly one same-clock second register'))
    by_group = defaultdict(int)
    for r in rows:
        by_group[r['group']] += 1
    result = dict(netlist=str(Path(a.netlist).resolve()), crossings=len(crossings), by_group=dict(by_group),
                  unclassified=unknown, passed=not unknown)
    print(json.dumps({k: v for k, v in result.items() if k != 'unclassified'}, indent=1))
    for u in unknown[:80]:
        print('UNCLASSIFIED', u['source_clock'], '->', u['dest_clock'], u['pin'], u['source'], '->', u['dest'])
    if a.json:
        Path(a.json).write_text(json.dumps(dict(result, rows=rows), indent=1))
    if a.sdc and not unknown:
        budget = {'hud_payload': None, 'hud_request': None, 'hud_ack': None, 'stale_flag': None,
                  'frame_request_core': None, 'frame_request_lcd': None, 'frame_ack': None, 'camera_ready': None}
        lines = ['', '# Video clock-domain crossings (audit_video_cdc.py on this netlist).',
                 '# Single-bit synchronizer first stages and the HUD page captured on a synchronized',
                 '# request toggle: hold waived, setup bounded by one destination period.']
        for r in rows:
            if r['group'] in ('existing_nav_cdc', 'lcd_reset', 'reset_synchronizer') or r['pin'] != 'D':
                continue
            limit = min(PERIOD[r['source_clock']], PERIOD[r['dest_clock']])
            s = '[get_pins {' + r['source'] + '/Q}]'
            d = '[get_pins {' + r['dest'] + '/D}]'
            lines.append(f'set_false_path -from {s} -to {d} -hold')
            lines.append(f'set_max_delay -from {s} -to {d} {limit:.3f}')
        Path(a.sdc).write_text('\n'.join(lines) + '\n')
        print('wrote', a.sdc, (len(lines) - 4) // 2, 'paths')


if __name__ == '__main__':
    main()
