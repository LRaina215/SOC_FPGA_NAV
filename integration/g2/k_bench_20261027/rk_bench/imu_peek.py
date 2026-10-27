"""Snapshot nav firmware g_app.imu (imu_parser_t) through the debug RAM window.

The RAM window is only readable while the CPU is held in reset, so this holds the CPU,
reads 16 words, and releases it; the firmware then restarts from address 0 (same as load).
Usage: python3 imu_peek.py [base_hex]   (default 0x17F18 = g_app.imu of the forward-1000 build)
"""
import json
import math
import struct
import sys
import time

from bench import Bridge, SOC_CTRL, SOC_RAM_ADDR, SOC_RAM_DATA


def s16(v):
    return v - 0x10000 if v & 0x8000 else v


def ang(v):
    return round(s16(v) * 360.0 / 65536.0, 3)


def main():
    base = int(sys.argv[1], 16) if len(sys.argv) > 1 else 0x17F18
    b = Bridge()
    b.wr(SOC_CTRL, 1)
    words = []
    try:
        for i in range(16):
            b.wr(SOC_RAM_ADDR, base // 4 + i)
            words.append(b.rd(SOC_RAM_DATA))
    finally:
        b.wr(SOC_CTRL, 0)
    raw = b''.join(struct.pack('<I', w) for w in words)
    u16 = lambda o: struct.unpack_from('<H', raw, o)[0]
    u32 = lambda o: struct.unpack_from('<I', raw, o)[0]
    out = {
        'proto': u32(0), 'att_new': raw[28], 'roll_deg': ang(u16(30)), 'pitch_deg': ang(u16(32)),
        'yaw_deg': ang(u16(34)), 'att_t_us': u32(36), 'gyro_new': raw[40],
        'gx_raw': s16(u16(42)), 'gy_raw': s16(u16(44)), 'gz_raw': s16(u16(46)),
        'gyro_t_us': u32(48), 'n_ok': u32(52), 'n_bad': u32(56), 'bmi_seq': u16(60),
        'released_at': time.strftime('%H:%M:%S'),
    }
    print(json.dumps(out))


if __name__ == '__main__':
    main()
