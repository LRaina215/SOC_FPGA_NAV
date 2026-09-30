"""伪 XT-M60：在 PC 上按厂家 SDK 的格式（model/xt_proto.py）发送 UDP 深度帧，或回放真雷达的录制 / 抓包。
雷达到货前用它驱动板级半实物测试；到货后换成真雷达，FPGA 侧不用改。

用法:
  python tools/fake_lidar.py 192.168.0.100 [--fps 10] [--frames 100] [--walk] [--amp] [--imu] [--per 2]
      虚拟场景 + 虚拟雷达（与闭环仿真相同的方向模型：SDK 缺省内参、径向距离、列号翻转）
      --amp 深度+幅值；--imu 每帧中间夹一条 IMU 消息（应被 FPGA 跳过）；--per 每个 UDP 报文装几个分包
  python tools/fake_lidar.py 127.0.0.1 --frames 50
      发给本机：打开厂家上位机（它在 TCP 连不上时照样收 UDP 7687 出图）检查我们的格式能否被厂家解析器正确显示
  python tools/fake_lidar.py 192.168.0.100 --xbin C:/xt-pcds/某次录制 [--fps 10] [--loop]
      回放厂家上位机录制的 .xbin（V3：4 字节版本号 + 原始帧），按 SDK 分包重新发送
  python tools/fake_lidar.py 192.168.0.100 --pcap xt.pcapng [--speed 1.0]
      回放 Wireshark 抓包（目的端口 7687 的 UDP 负载原样发送，按原时间间隔；真雷达的源端口不是 7687）
  python tools/fake_lidar.py 192.168.0.100 --hex dump.txt      回放十六进制文本（每行一个 UDP 负载）
  --dump out.txt   另存本次发送的全部 UDP 负载（十六进制文本，可用 --hex 回放或做测试向量）
  python tools/fake_lidar.py --tcp-server [--stream-for 20] [--tcp-log] [上面任一种数据源选项]
      连雷达的方式也冒充：监听 TCP 7787，收到"开始测量"（命令 1）才出流，发给这个 TCP 客户端的 IP（与实测雷达一致），
      TCP 断开后继续出流；命令 2 停流。用来测 FPGA 的雷达启动客户端（板子 NETL_LIDAR_IP 写成电脑的 IP）。
      --stream-for 每次启动后出流多少秒就停（模拟雷达掉电重启，看 FPGA 会不会重新启动它）；--tcp-log 应答前先发一条上报
注意：PC 有线网卡须与 FPGA 雷达口同网段（缺省 192.168.0.x，FPGA 为 192.168.0.100）；源端口按雷达缺省 7687。
"""
import argparse
import socket
import struct
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "model"))
import sim_world as sw  # noqa: E402
import xt_proto as xt  # noqa: E402


def read_pcap(path):
    """经典 pcap 与 pcapng（以太网链路）-> [(时间 s, UDP 负载)]，只取目的端口 7687 的 IPv4/UDP"""
    data = Path(path).read_bytes()
    pkts = []

    def eth_udp(fr):
        if len(fr) < 42 or fr[12:14] != b"\x08\x00" or fr[23] != 17:
            return None
        ihl = (fr[14] & 0x0F) * 4
        u = 14 + ihl
        sport, dport, ulen = struct.unpack_from(">HHH", fr, u)
        return fr[u + 8:u + ulen] if dport == xt.UDP_PORT else None

    magic = struct.unpack_from("<I", data, 0)[0]
    if magic in (0xA1B2C3D4, 0xA1B23C4D):
        ns = magic == 0xA1B23C4D
        pos = 24
        while pos + 16 <= len(data):
            ts, tf, incl, _ = struct.unpack_from("<IIII", data, pos)
            fr = data[pos + 16:pos + 16 + incl]
            pos += 16 + incl
            p = eth_udp(fr)
            if p is not None:
                pkts.append((ts + tf * (1e-9 if ns else 1e-6), p))
    elif magic == 0x0A0D0D0A:
        pos = 0
        while pos + 12 <= len(data):
            btype, blen = struct.unpack_from("<II", data, pos)
            if btype == 6:                                   # Enhanced Packet Block（时间戳按微秒）
                _, th, tl, cap, _ = struct.unpack_from("<IIIII", data, pos + 8)
                p = eth_udp(data[pos + 28:pos + 28 + cap])
                if p is not None:
                    pkts.append((((th << 32) | tl) * 1e-6, p))
            pos += blen
    else:
        raise SystemExit(f"{path} 不是 pcap / pcapng 文件")
    return pkts


def read_xbin(path):
    """目录或文件 -> 按文件名序号排序的原始帧列表（START..END）。只支持 V3。"""
    p = Path(path)
    files = sorted(p.glob("*.xbin"), key=lambda f: (len(f.stem), f.stem)) if p.is_dir() else [p]
    frames = []
    for f in files:
        b = f.read_bytes()
        ver = struct.unpack_from("<I", b, 0)[0]
        if ver != 3:
            print(f"{f.name}: xbin 版本 {ver}，只支持 V3（新固件录制），跳过")
            continue
        fr = b[4:]
        if fr[:4] != xt.START:                                # 文件里可能不带 START/len：补上
            body = fr[:-4] if fr[-4:] == xt.END else fr
            fr = xt.START + struct.pack("<I", len(body)) + body + xt.END
        frames.append(fr)
    return frames


def xt_msg(body):
    return xt.START + struct.pack("<I", len(body)) + bytes(body) + xt.END


def take_msgs(buf):
    """TCP 字节流 -> ([消息体], 剩余字节)；消息 = START | len u32 小端 | 体 | END"""
    out = []
    while True:
        i = buf.find(xt.START)
        if i < 0:
            return out, buf[-3:]
        if len(buf) < i + 8:
            return out, buf[i:]
        n = struct.unpack_from("<I", buf, i + 4)[0]
        if len(buf) < i + 12 + n:
            return out, buf[i:]
        if buf[i + 8 + n:i + 12 + n] == xt.END:
            out.append(buf[i + 8:i + 8 + n])
        buf = buf[i + 12 + n:]


class TcpServer(threading.Thread):
    """冒充雷达 TCP 7787：命令 1 -> 应答 OK、出流目标 = 这个客户端的 IP；命令 2 -> 停流；其他命令 -> "命令不支持"。
    一次只服务一个连接（与雷达一致），断开后继续出流。"""

    def __init__(self, bind, port, log_first):
        super().__init__(daemon=True)
        self.bind, self.port, self.log_first = bind, port, log_first
        self.dst, self.running, self.t_start = None, False, 0.0
        self.n_conn = self.n_start = 0

    def run(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.bind, self.port))
        srv.listen(1)
        print(f"TCP 服务端 {self.bind}:{self.port} 等待连接（收到开始命令才出流）")
        while True:
            c, addr = srv.accept()
            self.n_conn += 1
            t0 = time.time()
            print(f"TCP 连接 #{self.n_conn} 来自 {addr[0]}:{addr[1]}")
            c.settimeout(10.0)
            buf = b""
            try:
                while True:
                    d = c.recv(4096)
                    if not d:
                        break
                    bodies, buf = take_msgs(buf + d)
                    for b in bodies:
                        self.command(c, b, addr[0])
            except (socket.timeout, OSError) as e:
                print(f"  连接异常结束：{e}")
            finally:
                c.close()
            print(f"  TCP 断开（{(time.time() - t0) * 1000:.0f} ms）")

    def command(self, c, body, peer):
        cid, rc, state = body[0], 0, 2
        if cid == 1:
            self.dst, self.running, self.t_start = peer, True, time.time()
            self.n_start += 1
            state = 3
            print(f"  开始测量（数据 {body[1:-2].hex()}），出流 -> {peer}:7687（第 {self.n_start} 次）")
        elif cid == 2:
            self.running = False
            print("  停止测量")
        else:
            rc = 1
            print(f"  命令 {cid} 不支持")
        if self.log_first:
            c.sendall(xt_msg(bytes([0x2A]) + b"fake XT-M60 log" + bytes([0, state, 3])))
        c.sendall(xt_msg(bytes([cid, rc, state, 3])))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dst", nargs="?", help="UDP 目标 IP（--tcp-server 时不用：发给连 TCP 的主机）")
    ap.add_argument("--tcp-server", action="store_true")
    ap.add_argument("--tcp-bind", default="0.0.0.0")
    ap.add_argument("--tcp-port", type=int, default=7787)
    ap.add_argument("--tcp-log", action="store_true")
    ap.add_argument("--stream-for", type=float, default=0, help="--tcp-server：每次启动后出流多少秒就停（0 = 不停）")
    ap.add_argument("--port", type=int, default=xt.UDP_PORT)
    ap.add_argument("--src-port", type=int, default=xt.UDP_PORT)
    ap.add_argument("--fps", type=float, default=10)
    ap.add_argument("--frames", type=int, default=0, help="发送帧数，0 = 不停")
    ap.add_argument("--walk", action="store_true", help="虚拟机器人沿 x 缓慢前进")
    ap.add_argument("--amp", action="store_true", help="深度+幅值（缺省只发深度）")
    ap.add_argument("--imu", action="store_true", help="每帧中间夹一条 IMU 消息")
    ap.add_argument("--per", type=int, default=1, help="每个 UDP 报文装几个分包")
    ap.add_argument("--chunk", type=int, default=xt.CHUNK, help="每个分包的帧数据字节数")
    ap.add_argument("--xbin")
    ap.add_argument("--pcap")
    ap.add_argument("--hex")
    ap.add_argument("--speed", type=float, default=1.0, help="--pcap 回放速度倍数")
    ap.add_argument("--loop", action="store_true", help="回放循环")
    ap.add_argument("--dump")
    a = ap.parse_args()
    if not a.dst and not a.tcp_server:
        ap.error("需要 UDP 目标 IP（或用 --tcp-server）")

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("", a.src_port))
    dump = open(a.dump, "w") if a.dump else None
    n_dg = 0
    srv = None
    if a.tcp_server:
        srv = TcpServer(a.tcp_bind, a.tcp_port, a.tcp_log)
        srv.start()

    def live():
        """TCP 模式：有人发过开始命令才出流；--stream-for 到时停流"""
        if srv is None:
            return True
        if srv.running and a.stream_for and time.time() - srv.t_start > a.stream_for:
            srv.running = False
            print(f"已出流 {a.stream_for:g} s，停流（模拟雷达重启），等 FPGA 重新发开始命令")
        return srv.running

    def wait_live():
        while not live():
            time.sleep(0.02)

    def send(d):
        nonlocal n_dg
        if not live():
            return
        s.sendto(d, (srv.dst if srv else a.dst, a.port))
        n_dg += 1
        if dump:
            dump.write(d.hex() + "\n")
        time.sleep(0.00015)                                   # 百兆链路约 1500 字节 / 120 us，留余量

    def send_frame(fr, sn, k):
        sub = xt.packetize(fr, sn=sn & 0xFFFF, chunk=a.chunk)
        if a.imu:
            sub.insert(len(sub) // 2, xt.imu_subpacket(k & 0xFFFF))
        for d in xt.datagrams(sub, per=a.per):
            send(d)

    if a.pcap or a.hex:
        if a.pcap:
            pk = read_pcap(a.pcap)
        else:
            pk = [(i * 0.0002, bytes.fromhex(ln)) for i, ln in enumerate(Path(a.hex).read_text().split())]
        print(f"回放 {len(pk)} 个 UDP 负载")
        while True:
            wait_live()
            t0, w0 = pk[0][0], time.time()
            for t, d in pk:
                dt = (t - t0) / a.speed - (time.time() - w0)
                if dt > 0:
                    time.sleep(dt)
                send(d)
            if not a.loop:
                break
    elif a.xbin:
        frames = read_xbin(a.xbin)
        print(f"回放 {len(frames)} 帧 xbin")
        k = 0
        t_next = time.time()
        while frames:
            for fr in frames:
                if not live():
                    wait_live()
                    t_next = time.time()
                send_frame(fr, k, k)
                k += 1
                t_next += 1.0 / a.fps
                time.sleep(max(0.0, t_next - time.time()))
            if not a.loop:
                break
    else:
        world = sw.default_scene()
        lid = sw.VirtualLidar(noise_mm=3.0)
        Rbs, tbs = sw.mount_pose()
        k = 0
        t_next = time.time()
        while a.frames == 0 or k < a.frames:
            if not live():
                wait_live()
                t_next = time.time()
            x = 0.05 * k if a.walk else 0.0
            Rws, tws = sw.compose(sw.rotz(0.0), np.array([x, 0.0, 0.0]), Rbs, tbs)
            depth, amp = lid.capture(world, Rws, tws)
            depth = np.where(depth == 0, 64001, depth)          # 设备的无效码：没有回波按"幅值过低"
            info = xt.frame_info(160, 60, imageflags=3 if a.amp else 1, ts_ms=int(time.time() * 1000) & 0xFFFFFFFF)
            fr = xt.build_frame(depth.tolist(), amp.tolist() if a.amp else None, sn=k & 0xFFFF, info=info)
            send_frame(fr, k, k)
            k += 1
            t_next += 1.0 / a.fps
            time.sleep(max(0.0, t_next - time.time()))
            if k % 10 == 0:
                print(f"已发送 {k} 帧（{n_dg} 个 UDP 报文）")
    if dump:
        dump.close()
    print(f"共发送 {n_dg} 个 UDP 报文")
    return 0


if __name__ == "__main__":
    sys.exit(main())
