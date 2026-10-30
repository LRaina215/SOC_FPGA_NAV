"""Read the foreground cost map window and the nav pose, and list obstacles in the robot frame.

Cost map: 128x128 bytes at WIN_COSTMAP, slot (sy*128+sx) with sx/sy = world cell & 127,
5 cm cells; 0..200 traversable cost, 253 inflated, 254 lethal, 255 unknown.
LAT_CX/LAT_CY = world cell of the window centre. The window is world (odometry) aligned, so the
robot heading from STATUS is used to express cells in the robot frame (x forward, y left).
Usage: python3 costmap_dump.py <out_prefix>
"""
import json
import math
import struct
import sys
import time

import nav_regs_map as rm
from bench import Bridge


def nav_status():
    """One STATUS message through the mailbox read-back (nav_host UartLink)."""
    sys.path.insert(0, 'tools')
    import nav_host
    link = nav_host.UartLink('/dev/ttyACM0')
    try:
        t0 = time.time()
        while time.time() - t0 < 3.0:
            msg = nav_host.parse(link.recv(0.5) or b'')
            if msg and msg.get('type') == 'status':
                return msg
    finally:
        link.u.s.close()
    return None


def main():
    prefix = sys.argv[1]
    b = Bridge()
    t0 = time.time()
    raw = bytearray()
    for w in range(4096):
        raw += struct.pack('<I', b.rd(rm.REGS['WIN_COSTMAP'] + 4 * w))
    cx, cy = b.rd(rm.REGS['MAP_LAT_CX']), b.rd(rm.REGS['MAP_LAT_CY'])
    cx = cx - (1 << 32) if cx >> 31 else cx
    cy = cy - (1 << 32) if cy >> 31 else cy
    read_s = time.time() - t0
    b.s.close()
    st = nav_status()
    grid = [[raw[(sy * 128 + sx)] for sx in range(128)] for sy in range(128)]
    # window coordinates: world cell (cx-64+i, cy-64+j) lives in slot ((cx-64+i)&127, (cy-64+j)&127)
    win = [[grid[(cy - 64 + j) & 127][(cx - 64 + i) & 127] for i in range(128)] for j in range(128)]
    out = {'read_s': round(read_s, 1), 'centre_cell': [cx, cy], 'status': st}
    yaw = None
    px = py = None
    if st:
        yaw = math.radians(st['yaw'])
        px, py = st['x'], st['y']
    out['yaw_deg'] = None if yaw is None else round(math.degrees(yaw), 1)
    counts = {}
    for j in range(128):
        for i in range(128):
            v = win[j][i]
            k = 'unknown' if v == 255 else 'lethal' if v == 254 else 'inflated' if v == 253 else 'free' if v == 0 else 'cost'
            counts[k] = counts.get(k, 0) + 1
    out['counts'] = counts
    # lethal cells in robot frame (relative to window centre; robot cell = centre)
    obst = []
    # The cost map is computed for the window centre at its last sweep; the robot may have moved
    # since, so cells are expressed relative to the current STATUS pose, not the window centre.
    rx = px / 1000.0 if px is not None else cx * 0.05
    ry = py / 1000.0 if py is not None else cy * 0.05
    out['robot_offset_from_centre_m'] = [round(rx - cx * 0.05, 2), round(ry - cy * 0.05, 2)]
    for j in range(128):
        for i in range(128):
            if win[j][i] == 254:
                dxw = (cx - 64 + i + 0.5) * 0.05 - rx
                dyw = (cy - 64 + j + 0.5) * 0.05 - ry
                if yaw is not None:
                    fx = dxw * math.cos(-yaw) - dyw * math.sin(-yaw)
                    fy = dxw * math.sin(-yaw) + dyw * math.cos(-yaw)
                else:
                    fx, fy = dxw, dyw
                obst.append((round(fx, 2), round(fy, 2)))
    out['lethal_robot_frame'] = obst
    with open(prefix + '.json', 'w') as f:
        json.dump(dict(out, window=win), f)
    print(json.dumps({k: v for k, v in out.items() if k != 'lethal_robot_frame'}, ensure_ascii=False))
    near = [o for o in obst if 0 < o[0] < 3.0 and abs(o[1]) < 1.5]
    print('lethal cells in front 0..3 m, |y|<1.5 m:', len(near))
    # ASCII view: rows = forward distance (top = far), cols = left..right, robot frame, 10 cm bins
    if yaw is not None:
        bins = {}
        for fx, fy in obst:
            if -0.5 < fx < 3.0 and abs(fy) < 1.5:
                bins[(int(fx / 0.1), int(round(fy / 0.1)))] = 1
        print('ASCII (each char 10 cm; top = 3 m ahead, bottom = robot; left side of the car on the LEFT):')
        for r in range(29, -6, -1):
            line = ''.join('#' if bins.get((r, c)) else ('R' if r == 0 and c == 0 else '.') for c in range(15, -16, -1))
            print('%5.1f %s' % (r * 0.1, line))


if __name__ == '__main__':
    main()
