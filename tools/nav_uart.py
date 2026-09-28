"""PC 调试工具：经板载 USB 串口（uart_bus_bridge，921600 8N1）读写导航子系统寄存器。

用法:
  python tools/nav_uart.py COM5 id                    读 SYS.ID / VERSION / TIME_US
  python tools/nav_uart.py COM5 rd MAP_STAT           按寄存器名读（名字见 docs/regmap.md，BLOCK_REG）
  python tools/nav_uart.py COM5 wr SAFE_CTRL 1        写
  python tools/nav_uart.py COM5 stat                  打印网口/雷达/地图统计
  python tools/nav_uart.py COM5 map out.png           读出前台代价图并保存为图片（需 matplotlib）
  python tools/nav_uart.py COM5 lut rtl/lidar/lut_default.hex   经 LUT_ADDR/LUT_XY/LUT_Z 写入方向查找表
依赖: pyserial
"""
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "model"))
import nav_regs_map as rm  # noqa: E402


class NavUart:
    def __init__(self, port, baud=921600):
        import serial
        self.s = serial.Serial(port, baud, timeout=0.5)

    def wr(self, addr, val):
        self.s.write(b"W" + struct.pack(">HI", addr, val & 0xFFFFFFFF))
        if self.s.read(1) != b"K":
            raise IOError(f"写 0x{addr:04X} 无应答")

    def rd(self, addr):
        self.s.write(b"R" + struct.pack(">H", addr))
        r = self.s.read(5)
        if len(r) != 5 or r[0:1] != b"D":
            raise IOError(f"读 0x{addr:04X} 无应答")
        return struct.unpack(">I", r[1:])[0]

    def reg(self, name):
        return rm.REGS[name]


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    u = NavUart(sys.argv[1])
    cmd = sys.argv[2]
    if cmd == "id":
        print(f"ID=0x{u.rd(rm.REGS['SYS_ID']):08X} VERSION=0x{u.rd(rm.REGS['SYS_VERSION']):08X} "
              f"TIME_US={u.rd(rm.REGS['SYS_TIME_US'])}")
    elif cmd == "rd":
        a = rm.REGS[sys.argv[3]] if not sys.argv[3].startswith("0x") else int(sys.argv[3], 16)
        v = u.rd(a)
        print(f"0x{a:04X} = 0x{v:08X} ({v})")
    elif cmd == "wr":
        a = rm.REGS[sys.argv[3]] if not sys.argv[3].startswith("0x") else int(sys.argv[3], 16)
        u.wr(a, int(sys.argv[4], 0))
    elif cmd == "stat":
        for n in ("NETL_STAT_RX_FRAMES", "NETL_STAT_RX_UDP", "NETL_STAT_RX_ERR", "NETL_STAT_ARP",
                  "NETH_STAT_RX_FRAMES", "NETH_STAT_RX_UDP", "LIDAR_STAT_FRAMES", "LIDAR_STAT_BAD",
                  "LIDAR_STAT_DROP", "LIDAR_STAT_POINTS", "LIDAR_LAST_FRAME", "LIDAR_LAST_WH",
                  "MAP_STAT", "MAP_STAT_UPD", "MAP_STAT_DROP", "MAP_FRAME_CNT", "SAFE_STAT", "SAFE_OUT_V"):
            v = u.rd(rm.REGS[n])
            print(f"{n:22s} 0x{v:08X} {v}")
    elif cmd == "map":
        buf = bytearray()
        t0 = time.time()
        for w in range(4096):
            buf += struct.pack("<I", u.rd(rm.REGS["WIN_COSTMAP"] + 4 * w))
        cx, cy = u.rd(rm.REGS["MAP_LAT_CX"]), u.rd(rm.REGS["MAP_LAT_CY"])
        print(f"读出 16 KB，用时 {time.time() - t0:.1f}s，中心 ({cx}, {cy})")
        import numpy as np
        m = np.frombuffer(bytes(buf), dtype=np.uint8).reshape(128, 128)
        cx = cx - (1 << 32) if cx >> 31 else cx
        cy = cy - (1 << 32) if cy >> 31 else cy
        sx, sy = (cx - 64) & 127, (cy - 64) & 127
        m = np.roll(np.roll(m, -sy, axis=0), -sx, axis=1)     # 槽坐标 -> 窗口坐标
        out = sys.argv[3] if len(sys.argv) > 3 else "costmap.png"
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.imshow(m, origin="lower", cmap="viridis")
        plt.colorbar(label="cost")
        plt.savefig(out, dpi=120)
        print("保存", out)
    elif cmd == "lut":
        lines = Path(sys.argv[3]).read_text().split()
        u.wr(rm.REGS["LIDAR_LUT_ADDR"], 0)
        for ln in lines:
            v = int(ln, 16)
            u.wr(rm.REGS["LIDAR_LUT_XY"], v & 0xFFFFFFFF)
            u.wr(rm.REGS["LIDAR_LUT_Z"], (v >> 32) & 0xFFFF)
        print(f"写入 {len(lines)} 项")
    else:
        print(__doc__)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
