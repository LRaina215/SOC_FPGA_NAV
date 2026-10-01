"""PC 调试工具：经板载 USB 串口（uart_bus_bridge，921600 8N1）读写导航子系统寄存器。

用法:
  python tools/nav_uart.py COM5 id                    读 SYS.ID / VERSION / TIME_US
  python tools/nav_uart.py COM5 rd MAP_STAT           按寄存器名读（名字见 docs/2_interface/regmap.md，BLOCK_REG）
  python tools/nav_uart.py COM5 wr SAFE_CTRL 1        写
  python tools/nav_uart.py COM5 stat                  打印网口/雷达/地图统计
  python tools/nav_uart.py COM5 map out.png           读出前台代价图并保存为图片（需 matplotlib）
  python tools/nav_uart.py COM5 lut lut.hex     经 LUT_ADDR/LUT_XY/LUT_Z 写入方向查找表（model/lidar_lut.py 生成；
                                                固件上电后会按雷达内参自己写表，这条只用于调试）
  软核（nav_soc 调试口，地址 0xE000–0xFFFF）：
  python tools/nav_uart.py COM5 soc                  软核信息（RAM 大小、状态字、控制台字节数、是否保持复位）
  python tools/nav_uart.py COM5 con                  持续打印软核控制台输出（Ctrl-C 退出）
  python tools/nav_uart.py COM5 load fw/build/rv/nav/nav_fw.bin   保持复位 -> 写入 RAM -> 抽查回读 -> 放开运行
  python tools/nav_uart.py COM5 reset                软核复位重启（程序不变）
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

    def wr_burst(self, items, depth=64):
        """连续写（不逐条等应答，最多 depth 条在途）：串口往返延时远大于命令本身，逐条等待会慢 10 倍以上"""
        pend = 0
        for addr, val in items:
            self.s.write(b"W" + struct.pack(">HI", addr, val & 0xFFFFFFFF))
            pend += 1
            if pend >= depth:
                if self.s.read(pend) != b"K" * pend:
                    raise IOError("连续写应答缺失")
                pend = 0
        if pend and self.s.read(pend) != b"K" * pend:
            raise IOError("连续写应答缺失")

    def rd(self, addr):
        self.s.write(b"R" + struct.pack(">H", addr))
        r = self.s.read(5)
        if len(r) != 5 or r[0:1] != b"D":
            raise IOError(f"读 0x{addr:04X} 无应答")
        return struct.unpack(">I", r[1:])[0]

    def reg(self, name):
        return rm.REGS[name]

    # ---------------- 软核调试口
    SOC_CTRL, SOC_RAM_ADDR, SOC_RAM_DATA, SOC_CON_WPTR, SOC_STATUS, SOC_INFO, SOC_CON =         0xF000, 0xF004, 0xF008, 0xF00C, 0xF010, 0xF014, 0xE000

    def soc_load(self, img, verify_every=97):
        self.wr(self.SOC_CTRL, 1)
        self.wr(self.SOC_RAM_ADDR, 0)
        words = [struct.unpack_from("<I", img + bytes(-len(img) % 4), i)[0] for i in range(0, len(img) + (-len(img) % 4), 4)]
        self.wr_burst([(self.SOC_RAM_DATA, w) for w in words])
        bad = 0
        for i in range(0, len(words), verify_every):
            self.wr(self.SOC_RAM_ADDR, i)
            if self.rd(self.SOC_RAM_DATA) != words[i]:
                bad += 1
        self.wr(self.SOC_CTRL, 0)
        return len(words), bad

    def con_read(self, since):
        """读出 since 之后的控制台字节，返回 (bytes, 新指针)"""
        wp = self.rd(self.SOC_CON_WPTR)
        if wp - since > 4096:
            since = wp - 4096
        out = bytearray()
        i = since
        while i < wp:
            w = self.rd(self.SOC_CON + ((i & 4095) & ~3))
            for b in range(i & 3, 4):
                if i >= wp:
                    break
                out.append((w >> (8 * b)) & 0xFF)
                i += 1
        return bytes(out), wp


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
                  "NETL_PHY_STAT", "NETL_RAW_STAT", "NETL_LIDAR_IP", "LIDAR_STAT_SKIP", "LIDAR_INFO_SEQ", "LIDAR_LENS0",
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
    elif cmd == "soc":
        info = u.rd(u.SOC_INFO)
        print(f"RAM {info >> 16} KB  版本 {info & 0xFFFF}  状态字 0x{u.rd(u.SOC_STATUS):08X}  "
              f"控制台 {u.rd(u.SOC_CON_WPTR)} 字节  保持复位 {u.rd(u.SOC_CTRL) & 1}")
    elif cmd == "con":
        wp = max(0, u.rd(u.SOC_CON_WPTR) - 2048)
        try:
            while True:
                b, wp = u.con_read(wp)
                if b:
                    sys.stdout.write(b.decode("utf-8", "replace"))
                    sys.stdout.flush()
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass
    elif cmd == "load":
        img = Path(sys.argv[3]).read_bytes()
        t0 = time.time()
        n, bad = u.soc_load(img)
        print(f"写入 {n} 字（{len(img)} 字节），{time.time() - t0:.1f} s，抽查回读错误 {bad}")
        return 1 if bad else 0
    elif cmd == "reset":
        u.wr(u.SOC_CTRL, 1)
        u.wr(u.SOC_CTRL, 0)
    elif cmd == "lut":
        lines = Path(sys.argv[3]).read_text().split()
        u.wr(rm.REGS["LIDAR_LUT_ADDR"], 0)
        for ln in lines:
            v = int(ln, 16)                                      # 36 位 {rz, ry, rx}，各 12 位
            u.wr(rm.REGS["LIDAR_LUT_XY"], v & 0xFFFFFF)
            u.wr(rm.REGS["LIDAR_LUT_Z"], (v >> 24) & 0xFFF)
        print(f"写入 {len(lines)} 项")
    else:
        print(__doc__)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
