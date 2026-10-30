"""Read the elevation map in front of the robot, fit the floor plane and locate tall objects.
Region: x 0.30..2.30 m, y -1.00..+1.00 m (world cells of 5 cm; robot assumed at the origin, yaw ~0).
Usage: python3 elev_grid.py <out.json>
"""
import json
import math
import sys
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
        time.sleep(0.001)
    return None


def solve3(m, v):
    """Solve 3x3 linear system by Cramer's rule."""
    def det(a):
        return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
                + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
    d = det(m)
    out = []
    for k in range(3):
        mk = [row[:] for row in m]
        for i in range(3):
            mk[i][k] = v[i]
        out.append(det(mk) / d)
    return out


def main():
    b = Bridge()
    avg = bool(b.rd(rm.REGS['MAP_CTRL']) & 0x10)
    cells = {}
    for ix in range(6, 46):
        for iy in range(-20, 21):
            c = read_cell(b, ix, iy, avg)
            if c and c[2] > 0:
                cells[(ix, iy)] = c
    json.dump({'%d,%d' % k: v for k, v in cells.items()}, open(sys.argv[1], 'w'))
    pts = [((ix + 0.5) * 0.05, (iy + 0.5) * 0.05, (zmin + zmax) / 2.0) for (ix, iy), (zmin, zmax, n) in cells.items()
           if zmax - zmin < 60 and zmax < 150]
    print('observed cells', len(cells), 'floor candidates', len(pts))
    for it in range(3):
        sx = sum(1 for _ in pts)
        m = [[sx, sum(p[0] for p in pts), sum(p[1] for p in pts)],
             [sum(p[0] for p in pts), sum(p[0] ** 2 for p in pts), sum(p[0] * p[1] for p in pts)],
             [sum(p[1] for p in pts), sum(p[0] * p[1] for p in pts), sum(p[1] ** 2 for p in pts)]]
        v = [sum(p[2] for p in pts), sum(p[0] * p[2] for p in pts), sum(p[1] * p[2] for p in pts)]
        a, bx, by = solve3(m, v)
        res = [p[2] - (a + bx * p[0] + by * p[1]) for p in pts]
        sd = math.sqrt(sum(r * r for r in res) / len(res))
        pts = [p for p, r in zip(pts, res) if abs(r) < 3 * sd]
    print('floor plane: z = %.0f mm %+.1f mm/m * x %+.1f mm/m * y   (n=%d, rms %.0f mm)' % (a, bx, by, len(pts), sd))
    print('  -> slope along x %.2f deg (negative = floor falls away ahead), along y %.2f deg (positive = left side higher)'
          % (math.degrees(math.atan(bx / 1000)), math.degrees(math.atan(by / 1000))))
    tall = sorted(((ix + 0.5) * 0.05, (iy + 0.5) * 0.05, zmax) for (ix, iy), (zmin, zmax, n) in cells.items()
                  if zmax - (a + bx * (ix + 0.5) * 0.05 + by * (iy + 0.5) * 0.05) > 200)
    print('cells rising > 200 mm above the floor plane:', len(tall))
    if tall:
        xs = [t[0] for t in tall]
        ys = [t[1] for t in tall]
        print('  x %.2f..%.2f m, y %.2f..%.2f m (y>0 = left), max height %d mm' % (min(xs), max(xs), min(ys), max(ys), max(t[2] for t in tall)))
        print('  rows (y from +1.0 left to -1.0 right, 5 cm; # = tall):')
        for ix in range(6, 46):
            line = ''.join('#' if any(abs(t[0] - (ix + 0.5) * 0.05) < 0.01 and abs(t[1] - (iy + 0.5) * 0.05) < 0.01 for t in tall)
                           else ('.' if (ix, iy) in cells else ' ') for iy in range(20, -21, -1))
            if '#' in line:
                print('  x=%.2f %s' % ((ix + 0.5) * 0.05, line))


if __name__ == '__main__':
    main()
