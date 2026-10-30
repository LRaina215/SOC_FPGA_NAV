"""Read 32-bit words from nav CPU RAM through the debug window (holds the CPU, which then restarts).
Usage: python3 ram_peek.py name=0xADDR[:size] ...   size in bytes 1/2/4 (default 4)
"""
import struct
import sys

from bench import Bridge, SOC_CTRL, SOC_RAM_ADDR, SOC_RAM_DATA


def main():
    req = []
    for arg in sys.argv[1:]:
        name, spec = arg.split('=')
        addr, _, size = spec.partition(':')
        req.append((name, int(addr, 16), int(size or 4)))
    b = Bridge()
    b.wr(SOC_CTRL, 1)
    words = {}
    try:
        for _, addr, size in req:
            for w in {addr // 4, (addr + size - 1) // 4}:
                if w not in words:
                    b.wr(SOC_RAM_ADDR, w)
                    words[w] = b.rd(SOC_RAM_DATA)
    finally:
        b.wr(SOC_CTRL, 0)
    for name, addr, size in req:
        raw = b''.join(struct.pack('<I', words[w]) for w in sorted({addr // 4, (addr + size - 1) // 4}))
        off = addr % 4
        val = int.from_bytes(raw[off:off + size], 'little')
        print(f'{name} = {val} (0x{val:X})')


if __name__ == '__main__':
    main()
