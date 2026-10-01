"""主机工具（PC / RK3588 通用）：下发目标点、路径点、模式、初始位姿、先验地图，接收状态。
报文格式见 docs/icd.md，与 fw/nav/proto.c 一一对应。导航不依赖 RK3588：PC 用下面任一链路即可完成全部操作。

链路（--link）：
  udp:IP[:端口]   网口（默认 192.168.2.100:9000，SFP 主机口；需要先生成 1G Serial Ethernet IP，目前不可用）
  uart:COM5       板载 USB 串口（uart_bus_bridge）：报文写入 MBX.INJ_* 注入接收队列，状态/应答从发送缓冲回读；
                  RK3588 经 USB 接这个口时写成 uart:/dev/ttyACM0（docs/rk3588_design.md 方案 R0）

用法:
  python tools/nav_host.py --link uart:COM5 map  sim/build/prior/terrain.nvmp    下发先验地图（.nvmp）
  python tools/nav_host.py --link uart:COM5 pose -1000 -300 0                   初始位姿（地图系 mm / 度），在先验图上对准
  python tools/nav_host.py --link uart:COM5 goal 9300 2800                      定点
  python tools/nav_host.py --link uart:COM5 wp 6600,-2600 6800,2300 --loop      路径点
  python tools/nav_host.py --link uart:COM5 mode search                         探索（有图：未看过的区域；无图：边界探索）
  python tools/nav_host.py --link udp:192.168.2.100 listen                      打印状态
  python tools/nav_host.py --link udp:192.168.2.100 target 1500 200 45          目标（机身系 mm，采图到发送的延时 ms）
"""
import argparse
import math
import socket
import struct
import sys
import time
from pathlib import Path

FPGA_PORT = 9000
LOCAL_PORT = 9001
MODES = {"idle": 0, "goto": 1, "track": 2, "search": 3, "estop": 4}
STATES = {0: "空闲", 1: "行进", 2: "到达", 3: "受阻", 4: "跟踪", 5: "探索", 6: "急停"}
LOC_STATES = {0: "无图", 1: "对准中", 2: "正常", 3: "退化", 4: "丢失"}
MAP_STATES = {0: "无", 1: "接收中", 2: "已加载", 3: "校验错", 4: "格式错"}
CHUNK = 1024


# ---------------------------------------------------------------- 报文
def hdr(t):
    return b"NV" + bytes([1, t])


def goal(x, y, yaw_cdeg=0x7FFFFFFF):
    return hdr(0x10) + struct.pack("<iii", x, y, yaw_cdeg)


def waypoints(pts, loop=False):
    return hdr(0x11) + struct.pack("<BBH", len(pts), int(loop), 0) + b"".join(struct.pack("<ii", *p) for p in pts)


def set_pose(x, y, yaw_cdeg, align=True, range_mm=0, range_cdeg=0):
    """初始位姿（有图：地图系；align=True 时在先验图上搜索 ±range 对准，0 = 固件缺省 ±0.5 m / ±10°）"""
    return hdr(0x12) + struct.pack("<iiiHHHH", x, y, yaw_cdeg, 1 if align else 0, range_mm, range_cdeg, 0)


def target(x, y, latency_ms=0, found=1, cls=0, conf=1000, z=0, track_id=0):
    return hdr(0x20) + struct.pack("<BBHiiiHH", found, cls, conf, x, y, z, latency_ms, track_id)


def mode(m):
    return hdr(0x30) + bytes([m, 0, 0, 0])


def map_begin(total):
    return hdr(0x40) + struct.pack("<I", total)


def map_data(offset, data):
    return hdr(0x41) + struct.pack("<I", offset) + data


def map_end():
    return hdr(0x42)


def map_clear():
    return hdr(0x43)


def map_messages(pkg):
    """先验地图包 -> 报文序列（BEGIN、按 1 KB 分块的 DATA、END）"""
    msgs = [map_begin(len(pkg))]
    for off in range(0, len(pkg), CHUNK):
        msgs.append(map_data(off, pkg[off:off + CHUNK]))
    msgs.append(map_end())
    return msgs


def parse(b):
    """FPGA -> 主机报文 -> dict（STATUS / MAP_ACK），其他返回 None"""
    if len(b) < 4 or b[:2] != b"NV":
        return None
    t = b[3]
    if t == 0x80 and len(b) >= 40:
        m, st, seq, x, y, yaw, gx, gy, vx, wz, flags, _, tu = struct.unpack("<BBHiiiiihhHHI", b[4:40])
        d = dict(type="status", mode=m, state=STATES.get(st, st), map_seq=seq, x=x, y=y, yaw=yaw / 100,
                 goal=(gx, gy), vx=vx, wz=wz, flags=flags, t_us=tu)
        if len(b) >= 44:
            d["loc"] = LOC_STATES.get(b[40], b[40])
            d["loc_q"] = b[41]
        return d
    if t == 0x82 and len(b) >= 12:
        nxt, st = struct.unpack("<IB", b[4:9])
        return dict(type="map_ack", next=nxt, state=MAP_STATES.get(st, st), state_code=st)
    return None


parse_status = parse


# ---------------------------------------------------------------- 链路
class UdpLink:
    def __init__(self, spec):
        host, _, port = spec.partition(":")
        self.addr = (host or "192.168.2.100", int(port) if port else FPGA_PORT)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind(("", LOCAL_PORT))
        self.s.settimeout(0.2)

    def send(self, b):
        self.s.sendto(b, self.addr)

    def recv(self, timeout=0.2):
        self.s.settimeout(timeout)
        try:
            b, _ = self.s.recvfrom(2048)
            return b
        except socket.timeout:
            return None


class UartLink:
    """板载 USB 串口：寄存器级注入（MBX.INJ_*）与发送缓冲回读（MBX.TXB_*）"""

    def __init__(self, port):
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        import nav_uart
        import nav_regs_map as rm
        self.u = nav_uart.NavUart(port)
        self.R = rm.REGS
        self.last_seq = self.u.rd(self.R["MBX_TX_SEQ"])

    def send(self, b):
        # 接收队列满时等待（描述符队列 16 项）
        for _ in range(200):
            if ((self.u.rd(self.R["MBX_RX_STAT"]) >> 16) & 0xFF) < 12:
                break
            time.sleep(0.005)
        pad = b + bytes((-len(b)) % 4)
        self.u.wr_burst([(self.R["MBX_INJ_DATA"], struct.unpack_from("<I", pad, i)[0]) for i in range(0, len(pad), 4)] +
                        [(self.R["MBX_INJ_LEN"], len(b)), (self.R["MBX_INJ_GO"], 1)])

    def recv(self, timeout=0.2):
        t0 = time.time()
        while time.time() - t0 < timeout:
            seq = self.u.rd(self.R["MBX_TX_SEQ"])
            if seq != self.last_seq:
                n = self.u.rd(self.R["MBX_TX_LEN"]) & 0xFFFF
                words = []
                for i in range((n + 3) // 4):
                    self.u.wr(self.R["MBX_TXB_ADDR"], i)
                    words.append(self.u.rd(self.R["MBX_TXB_Q"]))
                if self.u.rd(self.R["MBX_TX_SEQ"]) == seq:          # 读取期间没有被覆盖
                    self.last_seq = seq
                    return b"".join(struct.pack("<I", w) for w in words)[:n]
            time.sleep(0.01)
        return None


def open_link(spec):
    kind, _, rest = spec.partition(":")
    if kind == "udp":
        return UdpLink(rest)
    if kind == "uart":
        return UartLink(rest)
    raise ValueError(f"未知链路 {spec}")


def upload_map(link, pkg, reliable=False, log=print):
    """下发先验地图。网口：按 MAP_ACK 的"下一个期望偏移"重发丢失的块；串口：注入不会丢，只在最后确认状态"""
    msgs = map_messages(pkg)
    link.send(msgs[0])
    off, tries, t0 = 0, 0, time.time()
    while off < len(pkg):
        link.send(map_data(off, pkg[off:off + CHUNK]))
        if reliable:
            off += CHUNK
            continue
        a = None
        for _ in range(5):
            r = link.recv(0.3)
            a = parse(r) if r else None
            if a and a["type"] == "map_ack":
                break
        if a and a["type"] == "map_ack" and a["next"] > off:
            off = a["next"]
            tries = 0
        else:
            tries += 1
            if tries > 20:
                raise IOError(f"地图下发在偏移 {off} 失败")
    link.send(map_end())
    for _ in range(20):
        r = link.recv(0.3)
        a = parse(r) if r else None
        if a and a["type"] == "map_ack":
            log(f"地图 {len(pkg)} 字节，{time.time() - t0:.1f} s，板上状态：{a['state']}")
            return a["state_code"] == 2
    log("未收到地图应答")
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--link", default="udp:192.168.2.100")
    ap.add_argument("cmd")
    ap.add_argument("args", nargs="*")
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--no-align", action="store_true", help="pose：位姿准确，不做对准搜索")
    ap.add_argument("--range", default="0,0", help="pose：对准搜索半宽 mm,度（0 = 缺省）")
    a = ap.parse_args()
    ln = open_link(a.link)
    c, v = a.cmd, a.args
    if c == "map":
        pkg = Path(v[0]).read_bytes()
        ok = upload_map(ln, pkg, reliable=a.link.startswith("uart"))
        return 0 if ok else 1
    if c == "unmap":
        ln.send(map_clear())
    elif c == "pose":
        rm_, rd_ = (float(t) for t in a.range.split(","))
        ln.send(set_pose(int(v[0]), int(v[1]), int(round(float(v[2]) * 100)), align=not a.no_align,
                         range_mm=int(rm_), range_cdeg=int(rd_ * 100)))
    elif c == "goal":
        yaw = int(round(float(v[2]) * 100)) if len(v) > 2 else 0x7FFFFFFF
        ln.send(goal(int(v[0]), int(v[1]), yaw))
    elif c == "wp":
        pts = [tuple(int(t) for t in p.split(",")) for p in v]
        ln.send(waypoints(pts, a.loop))
    elif c == "mode":
        ln.send(mode(MODES[v[0]]))
    elif c == "target":
        ln.send(target(int(v[0]), int(v[1]), int(v[2]) if len(v) > 2 else 0))
    elif c == "listen":
        while True:
            r = ln.recv(1.0)
            d = parse(r) if r else None
            if d:
                print(d)
    else:
        print(__doc__)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
