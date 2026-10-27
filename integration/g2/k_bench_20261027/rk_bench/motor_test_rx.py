"""Receive motor_test_top records on /dev/ttyUSB0 (230400) and summarize each test segment.
Record: A5 5A phase u8, duty s16, lspd s16 (cm/s), rspd s16, lpos s32, rpos s32, sum u8 (bytes 2..16).
Usage: python3 motor_test_rx.py <seconds> <out.jsonl>
"""
import json
import struct
import sys
import time

import serial


def main():
    secs, path = float(sys.argv[1]), sys.argv[2]
    s = serial.Serial('/dev/ttyUSB0', 230400, timeout=0.05)
    s.reset_input_buffer()
    buf = bytearray()
    rows, bad = [], 0
    t0 = time.time()
    while time.time() - t0 < secs:
        buf += s.read(4096)
        while True:
            i = buf.find(b'\xa5\x5a')
            if i < 0:
                del buf[:-1]
                break
            if len(buf) - i < 18:
                del buf[:i]
                break
            rec = buf[i:i + 18]
            if sum(rec[2:17]) & 0xFF != rec[17]:
                bad += 1
                del buf[:i + 1]
                continue
            del buf[:i + 18]
            phase, duty, ls, rs, lp, rp = struct.unpack('<BhhhiI', rec[2:17])
            rp = rp - (1 << 32) if rp & 0x80000000 else rp
            rows.append({'t': round(time.time() - t0, 3), 'phase': phase, 'duty': duty,
                         'lspd': ls, 'rspd': rs, 'lpos': lp, 'rpos': rp})
    with open(path, 'w') as f:
        for r in rows:
            f.write(json.dumps(r) + '\n')
    print(json.dumps({'records': len(rows), 'bad': bad}))
    segs = {}
    for r in rows:
        segs.setdefault(r['phase'], []).append(r)
    for ph in sorted(segs, key=lambda p: (p == 0xFE, p != 0xFF, p)):
        g = segs[ph]
        k = len(g)
        tail = g[k // 2:] if ph < 32 else g
        side = '-' if ph >= 32 else ('R' if ph & 16 else 'L')
        print(f"phase {ph:3d} side {side} duty {g[0]['duty']:6d} n {k:4d}  "
              f"L spd mean(2nd half) {sum(x['lspd'] for x in tail)/len(tail):7.1f} cm/s dpos {g[-1]['lpos']-g[0]['lpos']:6d}  "
              f"R spd {sum(x['rspd'] for x in tail)/len(tail):7.1f} cm/s dpos {g[-1]['rpos']-g[0]['rpos']:6d}")


if __name__ == '__main__':
    main()
