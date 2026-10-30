"""Dump every elevation slot plus the hardware cost map (map keeps running; the scene is static).
Slots are read in pipelined batches over the bus bridge. The frame counter is sampled per row so
the offline model can age cells the way the hardware does.
Output JSON: zmin, zmax (raw signed 16-bit fields; with MAP_CTRL.AVG they hold the Q3 deviation / Q3 mean), cnt, tag, stamp, stale (16384 each), fc_row (128), cost (16384 bytes),
lat_cx/lat_cy, centre_x/centre_y and the terrain registers.
Usage: python3 elev_dump.py <out.json>
"""
import json
import struct
import sys
import time

import nav_regs_map as rm
from bench import Bridge


def s16(v):
    v &= 0xFFFF
    return v - 65536 if v & 0x8000 else v


def s32(v):
    return v - (1 << 32) if v >> 31 else v


def main():
    b = Bridge()
    R = rm.REGS
    t0 = time.time()
    regs = {k: b.rd(R[k]) for k in ('MAP_CTRL', 'MAP_THR0', 'MAP_THR1', 'MAP_WEIGHT', 'MAP_INFL', 'MAP_VALID_CFG',
                                   'MAP_DROP', 'MAP_FILL', 'MAP_INV_RES', 'MAP_FRAME_CNT', 'MAP_STAT')}
    out = dict(regs=regs, lat_cx=s32(b.rd(R['MAP_LAT_CX'])), lat_cy=s32(b.rd(R['MAP_LAT_CY'])),
               centre_x=s32(b.rd(R['MAP_CENTER_X'])), centre_y=s32(b.rd(R['MAP_CENTER_Y'])))
    cost = bytearray()
    for w in range(4096):
        cost += struct.pack('<I', b.rd(R['WIN_COSTMAP'] + 4 * w))
    A, C, Q0, Q1 = R['MAP_ELEV_ADDR'], R['MAP_ELEV_CMD'], R['MAP_ELEV_Q0'], R['MAP_ELEV_Q1']
    W = lambda a, v: b'W' + struct.pack('>HI', a, v)
    RD = lambda a: b'R' + struct.pack('>H', a)
    q0s, q1s, fc_row = [0] * 16384, [0] * 16384, []
    retry = 0
    for row in range(128):
        fc_row.append(b.rd(R['MAP_FRAME_CNT']))
        resp = b''
        for i in range(128):
            b.wr(A, row * 128 + i); b.wr(C, 1)
            resp += b'KKD' + struct.pack('>I', b.rd(Q1)) + b'D' + struct.pack('>I', b.rd(Q0))
        for i in range(128):
            r = resp[i * 12:(i + 1) * 12]
            if r[0:2] != b'KK' or r[2:3] != b'D' or r[7:8] != b'D':
                raise IOError('bad batch reply at slot %d: %r' % (row * 128 + i, r))
            q1 = struct.unpack('>I', r[3:7])[0]
            q0 = struct.unpack('>I', r[8:12])[0]
            if not q1 >> 31:
                retry += 1
                b.wr(A, row * 128 + i)
                b.wr(C, 1)
                for _ in range(20):
                    q1 = b.rd(Q1)
                    if q1 >> 31:
                        break
                q0 = b.rd(Q0)
            q0s[row * 128 + i], q1s[row * 128 + i] = q0, q1
    out.update(zmin=[s16(q >> 16) for q in q0s], zmax=[s16(q) for q in q0s], cnt=[q & 0xF for q in q1s],
               tag=[(q >> 4) & 0xFF for q in q1s], stamp=[(q >> 12) & 0xFF for q in q1s],
               stale=[(q >> 20) & 1 for q in q1s], fc_row=fc_row, cost=list(cost), retries=retry)
    out['dump_s'] = round(time.time() - t0, 1)
    json.dump(out, open(sys.argv[1], 'w'))
    print('dumped in %.1f s (%d retries); centre %s/%s lat %s/%s fc %d..%d, used slots %d'
          % (out['dump_s'], retry, out['centre_x'], out['centre_y'], out['lat_cx'], out['lat_cy'], fc_row[0],
             fc_row[-1], sum(1 for c in out['cnt'] if c)))


if __name__ == '__main__':
    main()
