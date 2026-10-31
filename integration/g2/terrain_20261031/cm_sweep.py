"""Live sweep of map registers: for each setting clear the map, wait, then sample the cost map 5 times
and report lethal / inflated / free / unknown fractions on the open floor in front (robot fixed at origin)."""
import struct, sys, time
import nav_regs_map as rm
from bench import Bridge
b = Bridge(); R = rm.REGS
def costmap():
    raw = bytearray()
    for w in range(4096):
        raw += struct.pack('<I', b.rd(R['WIN_COSTMAP'] + 4 * w))
    return raw
cells = [((iy & 127) << 7) | (ix & 127) for ix in range(8, 27) for iy in range(-12, 12)]
settings = [tuple(int(v, 0) for v in s.split(',')) for s in sys.argv[1:]]
for ctrl, thr0 in settings:
    b.wr(R['MAP_CTRL'], ctrl); b.wr(R['MAP_THR0'], thr0); b.wr(R['MAP_CMD'], 2)
    time.sleep(20)
    acc = [0, 0, 0, 0]
    for k in range(4):
        cm = costmap()
        for a in cells:
            v = cm[a]
            acc[0 if v == 254 else 1 if v == 253 else 3 if v == 255 else 2] += 1
        time.sleep(1.5)
    n = sum(acc)
    print('MAP_CTRL 0x%02X (AVG_SHIFT %d) step %d mm: lethal %.1f%% inflated %.1f%% free %.1f%% unknown %.1f%%'
          % (ctrl, (ctrl >> 5) & 7, thr0 & 0xFFFF, 100 * acc[0] / n, 100 * acc[1] / n, 100 * acc[2] / n, 100 * acc[3] / n), flush=True)
b.wr(R['MAP_CTRL'], 0x7D); b.wr(R['MAP_THR0'], 0x00280014); b.wr(R['MAP_CMD'], 2)
