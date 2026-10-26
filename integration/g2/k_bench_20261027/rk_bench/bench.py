"""RK3588-side FPGA bench tool.

Nav bus bridge on the on-board CH9102 (921600 8N1, uart_bus_bridge protocol):
    W <addr:u16 BE> <val:u32 BE>  -> 'K'
    R <addr:u16 BE>               -> 'D' <val:u32 BE>
Control telemetry on P11-40 through a USB-TTL adapter (230400 8N1, protocol.py V1/V2 frames).

Usage (on the RK3588):
    python3 bench.py id
    python3 bench.py rd SYS_TIME_US | rd 0x5028
    python3 bench.py wr SAFE_CTRL 1
    python3 bench.py stat
    python3 bench.py soc
    python3 bench.py con [seconds]
    python3 bench.py telem [seconds]
    python3 bench.py drive <vx_mmps> <wz_mradps> <seconds> [--hold-cpu]
"""
import json
import struct
import sys
import time

import nav_regs_map as rm
import protocol

BRIDGE_PORT = '/dev/ttyACM0'
TELEM_PORT = '/dev/ttyUSB0'
SOC_CTRL, SOC_RAM_ADDR, SOC_RAM_DATA, SOC_CON_WPTR, SOC_STATUS, SOC_INFO, SOC_CON = (
    0xF000, 0xF004, 0xF008, 0xF00C, 0xF010, 0xF014, 0xE000)


class Bridge:
    def __init__(self, port=BRIDGE_PORT, baud=921600):
        import serial
        self.s = serial.Serial(port, baud, timeout=0.3)
        self.s.reset_input_buffer()

    def wr(self, addr, val):
        self.s.write(b'W' + struct.pack('>HI', addr, val & 0xFFFFFFFF))
        if self.s.read(1) != b'K':
            raise IOError(f'write 0x{addr:04X}: no ack')

    def rd(self, addr):
        self.s.write(b'R' + struct.pack('>H', addr))
        r = self.s.read(5)
        if len(r) != 5 or r[0:1] != b'D':
            raise IOError(f'read 0x{addr:04X}: no reply ({r!r})')
        return struct.unpack('>I', r[1:])[0]


def addr_of(name):
    return int(name, 0) if name.lower().startswith('0x') else rm.REGS[name]


def s16(v):
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def cmd_id(b):
    out = {n: b.rd(rm.REGS[n]) for n in ('SYS_ID', 'SYS_VERSION', 'SYS_TIME_US')}
    t1 = b.rd(rm.REGS['SYS_TIME_US'])
    out['time_advancing'] = t1 != out['SYS_TIME_US']
    print(json.dumps({k: (f'0x{v:08X}' if isinstance(v, int) and k != 'SYS_TIME_US' else v) for k, v in out.items()}))


STAT_NAMES = ('NETL_PHY_STAT', 'NETL_STAT_RX_FRAMES', 'NETL_STAT_RX_UDP', 'NETL_STAT_RX_ERR',
              'LIDAR_STAT_FRAMES', 'LIDAR_STAT_BAD', 'LIDAR_STAT_DROP', 'LIDAR_STAT_POINTS',
              'LIDAR_LAST_WH', 'MAP_FRAME_CNT', 'SAFE_CTRL', 'SAFE_STAT', 'SAFE_OUT_V', 'SAFE_OUT_W',
              'SAFE_ODO_V', 'SAFE_ODO_W', 'SAFE_ODO_CNT', 'SAFE_LIM_V', 'SAFE_LIM_W', 'SAFE_WD_US')


def cmd_stat(b):
    for n in STAT_NAMES:
        if n in rm.REGS:
            v = b.rd(rm.REGS[n])
            print(f'{n:22s} 0x{v:08X} {v}')


def cmd_soc(b):
    info = b.rd(SOC_INFO)
    print(f'RAM {info >> 16} KB ver {info & 0xFFFF} status 0x{b.rd(SOC_STATUS):08X} '
          f'console {b.rd(SOC_CON_WPTR)} B hold_reset {b.rd(SOC_CTRL) & 1}')


def con_read(b, since):
    wp = b.rd(SOC_CON_WPTR)
    if wp - since > 4096:
        since = wp - 4096
    out = bytearray()
    i = since
    while i < wp:
        w = b.rd(SOC_CON + ((i & 4095) & ~3))
        for k in range(i & 3, 4):
            if i >= wp:
                break
            out.append((w >> (8 * k)) & 0xFF)
            i += 1
    return bytes(out), wp


def cmd_con(b, secs):
    wp = max(0, b.rd(SOC_CON_WPTR) - 3000)
    t_end = time.time() + secs
    while time.time() < t_end:
        data, wp = con_read(b, wp)
        if data:
            sys.stdout.write(data.decode('utf-8', 'replace'))
            sys.stdout.flush()
        time.sleep(0.1)


KEYS = ('time_s', 'sequence', 'status', 'policy_fault', 'sensor_error', 'left_fault', 'right_fault',
        'fusion_flags', 'gyro_x', 'gyro_y', 'gyro_z', 'accel_x_g', 'accel_y_g', 'accel_z_g',
        'pitch_deg', 'roll_deg', 'forward_command', 'yaw_command', 'height_command',
        'left_target', 'right_target', 'left_speed', 'right_speed', 'left_duty', 'right_duty',
        'lh_us', 'lk_us', 'rh_us', 'rk_us', 'completed_frames', 'inference_cycles',
        'left_delta', 'right_delta', 'active')


class Telemetry:
    def __init__(self, port=TELEM_PORT, baud=230400):
        import serial
        self.s = serial.Serial(port, baud, timeout=0.05)
        self.s.reset_input_buffer()
        self.dec = protocol.Decoder()

    def poll(self):
        data = self.s.read(4096)
        return self.dec.feed(data) if data else []


def summarize(frames, dec, secs):
    res = {'frames': len(frames), 'rate_hz': round(len(frames) / secs, 1),
           'bad_crc': dec.bad_crc, 'skipped': dec.skipped}
    if frames:
        last = frames[-1]
        res['last'] = {k: (round(last[k], 4) if isinstance(last.get(k), float) else last.get(k)) for k in KEYS}
        for k in ('left_speed', 'right_speed', 'left_duty', 'right_duty', 'pitch_deg'):
            vals = [f[k] for f in frames if isinstance(f.get(k), (int, float)) and f[k] == f[k]]
            if vals:
                res[k + '_minmax'] = [round(min(vals), 4), round(max(vals), 4)]
    return res


def cmd_telem(secs):
    t = Telemetry()
    frames = []
    t_end = time.time() + secs
    while time.time() < t_end:
        frames += t.poll()
    print(json.dumps(summarize(frames, t.dec, secs), ensure_ascii=False))


def cmd_selftest(b, secs=3.0):
    """Read-only check of the joint image: identity, CPU, lidar rate, SAFE/odometry, telemetry."""
    res = {}
    res['SYS_ID'] = f"0x{b.rd(rm.REGS['SYS_ID']):08X}"
    res['SYS_VERSION'] = f"0x{b.rd(rm.REGS['SYS_VERSION']):08X}"
    t0 = b.rd(rm.REGS['SYS_TIME_US'])
    info = b.rd(SOC_INFO)
    res['soc'] = {'ram_kb': info >> 16, 'status': f'0x{b.rd(SOC_STATUS):08X}',
                  'hold_reset': b.rd(SOC_CTRL) & 1, 'console_bytes': b.rd(SOC_CON_WPTR)}
    f0, bad0 = b.rd(rm.REGS['LIDAR_STAT_FRAMES']), b.rd(rm.REGS['LIDAR_STAT_BAD'])
    o0 = b.rd(rm.REGS['SAFE_ODO_CNT'])
    tm = Telemetry()
    frames = []
    t_end = time.time() + secs
    while time.time() < t_end:
        frames += tm.poll()
    t1 = b.rd(rm.REGS['SYS_TIME_US'])
    dt = ((t1 - t0) & 0xFFFFFFFF) / 1e6
    f1, bad1 = b.rd(rm.REGS['LIDAR_STAT_FRAMES']), b.rd(rm.REGS['LIDAR_STAT_BAD'])
    o1 = b.rd(rm.REGS['SAFE_ODO_CNT'])
    res['sys_time_dt_s'] = round(dt, 3)
    res['lidar_fps'] = round((f1 - f0) / dt, 2) if dt > 0 else None
    res['lidar_bad_delta'] = bad1 - bad0
    res['lidar_phy'] = f"0x{b.rd(rm.REGS['NETL_PHY_STAT']):08X}"
    res['odo_rate_hz'] = round((o1 - o0) / dt, 1) if dt > 0 else None
    res['safe'] = {n: f'0x{b.rd(rm.REGS[n]):08X}' for n in ('SAFE_CTRL', 'SAFE_STAT', 'SAFE_OUT_V', 'SAFE_OUT_W')}
    res['telemetry'] = summarize(frames, tm.dec, secs)
    print(json.dumps(res, ensure_ascii=False))


def cmd_drive(b, vx, wz, secs, hold_cpu):
    """Send SAFE commands at 20 Hz for secs, then stop sending and watch the watchdog stop."""
    t = Telemetry()
    if hold_cpu:
        b.wr(SOC_CTRL, 1)
    lim_v, lim_w = b.rd(rm.REGS['SAFE_LIM_V']), b.rd(rm.REGS['SAFE_LIM_W'])
    b.wr(rm.REGS['SAFE_LIM_V'], (lim_v & 0xFFFF0000) | min(abs(vx) + 1, 200))
    b.wr(rm.REGS['SAFE_LIM_W'], min(abs(wz) + 1, 800))
    b.wr(rm.REGS['SAFE_CTRL'], 1)
    log = []
    safe_out = []
    t0 = time.time()
    phase_end = t0 + secs
    while time.time() < phase_end:
        b.wr(rm.REGS['SAFE_CMD_V'], vx & 0xFFFF)
        b.wr(rm.REGS['SAFE_CMD_W'], wz & 0xFFFF)
        safe_out.append((round(time.time() - t0, 2), s16(b.rd(rm.REGS['SAFE_OUT_V'])),
                         s16(b.rd(rm.REGS['SAFE_OUT_W'])), f"0x{b.rd(rm.REGS['SAFE_STAT']):08X}"))
        for f in t.poll():
            log.append(('drive', time.time() - t0, f))
        time.sleep(0.05)
    t_stop = time.time()
    for _ in range(12):
        safe_out.append((round(time.time() - t0, 2), s16(b.rd(rm.REGS['SAFE_OUT_V'])),
                         s16(b.rd(rm.REGS['SAFE_OUT_W'])), f"0x{b.rd(rm.REGS['SAFE_STAT']):08X}"))
        time.sleep(0.05)
    while time.time() < t_stop + 2.0:
        for f in t.poll():
            log.append(('stopped', time.time() - t0, f))
        time.sleep(0.02)
    stat = b.rd(rm.REGS['SAFE_STAT'])
    out_v = b.rd(rm.REGS['SAFE_OUT_V'])
    b.wr(rm.REGS['SAFE_LIM_V'], lim_v)
    b.wr(rm.REGS['SAFE_LIM_W'], lim_w)
    if hold_cpu:
        b.wr(SOC_CTRL, 0)
    rows = [(ph, round(tt, 2), round(f['forward_command'], 4), round(f['yaw_command'], 4),
             round(f['left_target'], 3), round(f['right_target'], 3),
             round(f['left_speed'], 3), round(f['right_speed'], 3), f['left_duty'], f['right_duty'])
            for ph, tt, f in log[::10]]
    print(json.dumps({'safe_stat_after': f'0x{stat:08X}', 'safe_out_v_after': s16(out_v),
                      'safe_out_trace': safe_out[::3], 'samples': rows[::4], 'bad_crc': t.dec.bad_crc},
                     ensure_ascii=False))


def cmd_estoptest(b, vx):
    """Keep sending vx at 20 Hz; assert SAFE ESTOP for the middle second; report forward_command per phase."""
    t = Telemetry()
    b.wr(SOC_CTRL, 1)
    lim_v = b.rd(rm.REGS['SAFE_LIM_V'])
    b.wr(rm.REGS['SAFE_LIM_V'], (lim_v & 0xFFFF0000) | min(abs(vx) + 1, 200))
    b.wr(rm.REGS['SAFE_CTRL'], 1)
    phases = [('run', 1.0, 1), ('estop', 1.0, 5), ('released', 1.0, 1)]
    out = {}
    for name, dur, ctrl in phases:
        b.wr(rm.REGS['SAFE_CTRL'], ctrl)
        vals = []
        outs = []
        t_end = time.time() + dur
        while time.time() < t_end:
            b.wr(rm.REGS['SAFE_CMD_V'], vx & 0xFFFF)
            b.wr(rm.REGS['SAFE_CMD_W'], 0)
            outs.append(s16(b.rd(rm.REGS['SAFE_OUT_V'])))
            vals += [round(f['forward_command'], 4) for f in t.poll()]
            time.sleep(0.05)
        out[name] = {'n': len(vals), 'forward_cmd_last': vals[-3:], 'safe_out_v': outs[::3],
                     'safe_stat': f"0x{b.rd(rm.REGS['SAFE_STAT']):08X}"}
    b.wr(rm.REGS['SAFE_CTRL'], 1)
    b.wr(rm.REGS['SAFE_LIM_V'], lim_v)
    b.wr(SOC_CTRL, 0)
    print(json.dumps(out, ensure_ascii=False))


def cmd_poweredtest(b):
    """After the whole car is powered and elevated: check BMI088/policy readiness, then short
    low-speed SAFE commands (CPU held in reset) and report whether the control saw them."""
    t = Telemetry()
    frames = []
    t_end = time.time() + 3
    while time.time() < t_end:
        frames += t.poll()
    if not frames:
        print(json.dumps({'ok': False, 'reason': 'no control telemetry on P11-40'}))
        return 1
    last = frames[-1]
    ready = (last['status'] & 1) == 1 and last['sensor_error'] == 0 and not (last['status'] & 2)
    if not ready:
        print(json.dumps({'ok': False, 'reason': 'BMI088 not ready / still calibrating',
                          'status': last['status'], 'sensor_error': last['sensor_error'],
                          'active': last['active']}, ensure_ascii=False))
        return 1
    b.wr(SOC_CTRL, 1)
    lim_v, lim_w = b.rd(rm.REGS['SAFE_LIM_V']), b.rd(rm.REGS['SAFE_LIM_W'])
    b.wr(rm.REGS['SAFE_LIM_V'], (lim_v & 0xFFFF0000) | 60)
    b.wr(rm.REGS['SAFE_LIM_W'], 400)
    b.wr(rm.REGS['SAFE_CTRL'], 1)
    res = {'ok': True, 'start': {k: last[k] for k in ('status', 'active', 'left_speed', 'right_speed')}}
    try:
        for name, vx, wz in (('zero', 0, 0), ('fwd50', 50, 0), ('rev50', -50, 0),
                             ('left300', 0, 300), ('right300', 0, -300), ('zero_end', 0, 0)):
            seen = []
            t_end = time.time() + 2.0
            while time.time() < t_end:
                b.wr(rm.REGS['SAFE_CMD_V'], vx & 0xFFFF)
                b.wr(rm.REGS['SAFE_CMD_W'], wz & 0xFFFF)
                seen += t.poll()
                time.sleep(0.05)
            tail = seen[-50:] or seen
            avg = lambda k: round(sum(f[k] for f in tail) / max(1, len(tail)), 4)
            res[name] = {'forward_cmd': avg('forward_command'), 'yaw_cmd': avg('yaw_command'),
                         'left_target': avg('left_target'), 'right_target': avg('right_target'),
                         'left_speed': avg('left_speed'), 'right_speed': avg('right_speed'),
                         'active': tail[-1]['active'] if tail else None}
        t_stop = time.time()
        after = []
        while time.time() < t_stop + 1.0:
            after += [(round(time.time() - t_stop, 2), round(f['forward_command'], 4)) for f in t.poll()]
            time.sleep(0.02)
        res['after_stop'] = after[::10]
    finally:
        b.wr(rm.REGS['SAFE_LIM_V'], lim_v)
        b.wr(rm.REGS['SAFE_LIM_W'], lim_w)
        b.wr(SOC_CTRL, 0)
    print(json.dumps(res, ensure_ascii=False))
    return 0


def main():
    a = sys.argv[1:]
    if not a:
        print(__doc__)
        return 1
    c = a[0]
    if c == 'telem':
        cmd_telem(float(a[1]) if len(a) > 1 else 5)
        return 0
    b = Bridge()
    if c == 'id':
        cmd_id(b)
    elif c == 'rd':
        v = b.rd(addr_of(a[1]))
        print(f'0x{addr_of(a[1]):04X} = 0x{v:08X} ({v})')
    elif c == 'wr':
        b.wr(addr_of(a[1]), int(a[2], 0))
    elif c == 'stat':
        cmd_stat(b)
    elif c == 'soc':
        cmd_soc(b)
    elif c == 'con':
        cmd_con(b, float(a[1]) if len(a) > 1 else 3)
    elif c == 'poweredtest':
        return cmd_poweredtest(b)
    elif c == 'estoptest':
        cmd_estoptest(b, int(a[1]) if len(a) > 1 else 50)
    elif c == 'selftest':
        cmd_selftest(b, float(a[1]) if len(a) > 1 else 3)
    elif c == 'drive':
        cmd_drive(b, int(a[1]), int(a[2]), float(a[3]), '--hold-cpu' in a)
    else:
        print(__doc__)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
