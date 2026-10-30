"""Read elevation-map cells (zmin/zmax/count) along rays in front of the robot (world cells, 5 cm).
Usage: python3 elev_probe.py
"""
import time

import nav_regs_map as rm
from bench import Bridge


def s16(v):
    v &= 0xFFFF
    return v - 65536 if v & 0x8000 else v


def decode(q0, avg):
    """-> (lo, hi) mm. Extreme mode: per-frame zmin/zmax. Average mode (MAP_CTRL.AVG): mean -/+ mean abs deviation."""
    if avg:
        mean, mad = s16(q0) >> 3, (q0 >> 19) & 0x1FFF
        return mean - mad, mean + mad
    return s16(q0 >> 16), s16(q0)


def read_cell(b, ix, iy, avg=False):
    b.wr(rm.REGS['MAP_ELEV_ADDR'], ((iy & 127) << 7) | (ix & 127))
    b.wr(rm.REGS['MAP_ELEV_CMD'], 1)
    for _ in range(50):
        q1 = b.rd(rm.REGS['MAP_ELEV_Q1'])
        if q1 >> 31:
            q0 = b.rd(rm.REGS['MAP_ELEV_Q0'])
            return decode(q0, avg) + (q1 & 0xF,)
        time.sleep(0.002)
    return None


def main():
    b = Bridge()
    avg = bool(b.rd(rm.REGS['MAP_CTRL']) & 0x10)
    print('averaged map: columns are mean-dev / mean+dev / points' if avg else 'per-frame map: zmin / zmax / points')
    print('cell x (m) |  y=-0.3 m             |  y=0                   |  y=+0.3 m')
    print('           |  zmin  zmax  n        |  zmin  zmax  n         |  zmin  zmax  n')
    for ix in range(4, 48, 2):
        row = []
        for iy in (-6, 0, 6):
            c = read_cell(b, ix, iy, avg)
            row.append('  ----  ----  -' if c is None or c[2] == 0 else '%6d%6d%3d' % c)
        print('%5.2f      | %s     | %s      | %s' % (ix * 0.05 + 0.025, row[0], row[1], row[2]))


if __name__ == '__main__':
    main()
