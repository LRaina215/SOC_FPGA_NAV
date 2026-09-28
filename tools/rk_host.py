"""RK3588 侧协议参考实现（Python）：下发目标点 / 路径点 / 目标坐标 / 模式，接收 FPGA 状态。
报文格式见 docs/icd.md，与 fw/nav/proto.c 一一对应。RK3588 上的 C++/ROS 节点可直接照此移植。

用法:
  python tools/rk_host.py 192.168.2.100 goal 2500 -800        去 (2.5 m, -0.8 m)
  python tools/rk_host.py 192.168.2.100 mode search           进入探索
  python tools/rk_host.py 192.168.2.100 target 1500 200 45    目标在机身前 1.5 m、左 0.2 m，采图到发送延时 45 ms
  python tools/rk_host.py 192.168.2.100 listen                打印状态报文
"""
import socket
import struct
import sys

FPGA_PORT = 9000
LOCAL_PORT = 9001
MODES = {"idle": 0, "goto": 1, "track": 2, "search": 3, "estop": 4}
STATES = {0: "空闲", 1: "行进", 2: "到达", 3: "受阻", 4: "跟踪", 5: "探索", 6: "急停"}


def hdr(t):
    return b"NV" + bytes([1, t])


def goal(x, y, yaw_cdeg=0x7FFFFFFF):
    return hdr(0x10) + struct.pack("<iii", x, y, yaw_cdeg)


def waypoints(pts, loop=False):
    return hdr(0x11) + struct.pack("<BBH", len(pts), int(loop), 0) + b"".join(struct.pack("<ii", *p) for p in pts)


def target(x, y, latency_ms=0, found=1, cls=0, conf=1000, z=0, track_id=0):
    return hdr(0x20) + struct.pack("<BBHiiiHH", found, cls, conf, x, y, z, latency_ms, track_id)


def mode(m):
    return hdr(0x30) + bytes([m, 0, 0, 0])


def parse_status(b):
    if len(b) < 40 or b[:2] != b"NV" or b[3] != 0x80:
        return None
    m, st, seq, x, y, yaw, gx, gy, vx, wz, flags, _, t = struct.unpack("<BBHiiiiihhHHI", b[4:40])
    return dict(mode=m, state=STATES.get(st, st), map_seq=seq, x=x, y=y, yaw=yaw / 100, goal=(gx, gy),
                vx=vx, wz=wz, flags=flags, t_us=t)


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    ip, cmd = sys.argv[1], sys.argv[2]
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("", LOCAL_PORT))
    if cmd == "goal":
        s.sendto(goal(int(sys.argv[3]), int(sys.argv[4])), (ip, FPGA_PORT))
    elif cmd == "mode":
        s.sendto(mode(MODES[sys.argv[3]]), (ip, FPGA_PORT))
    elif cmd == "target":
        s.sendto(target(int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5]) if len(sys.argv) > 5 else 0), (ip, FPGA_PORT))
    elif cmd == "listen":
        while True:
            b, _ = s.recvfrom(2048)
            st = parse_status(b)
            if st:
                print(st)
    else:
        print(__doc__)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
