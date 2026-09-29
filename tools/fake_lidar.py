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
      回放 Wireshark 抓包（源端口 7687 的 UDP 负载原样发送，按原时间间隔）
  python tools/fake_lidar.py 192.168.0.100 --hex dump.txt      回放十六进制文本（每行一个 UDP 负载）
  --dump out.txt   另存本次发送的全部 UDP 负载（十六进制文本，可用 --hex 回放或做测试向量）
注意：PC 有线网卡须与 FPGA 雷达口同网段（缺省 192.168.0.x，FPGA 为 192.168.0.100）；源端口按雷达缺省 7687。
"""
import argparse
import socket
import struct
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "model"))
import sim_world as sw  # noqa: E402
import xt_proto as xt  # noqa: E402


def read_pcap(path):
    """经典 pcap 与 pcapng（以太网链路）-> [(时间 s, UDP 负载)]，只取源端口 7687 的 IPv4/UDP。"""
    data = Path(path).read_bytes()
    pkts = []

    def eth_udp(fr):
        if len(fr) < 42 or fr[12:14] != b"\x08\x00" or fr[23] != 17:
            return None
        ihl = (fr[14] & 0x0F) * 4
        u = 14 + ihl
        sport, dport, ulen = struct.unpack_from(">HHH", fr, u)
        return fr[u + 8:u + ulen] if sport == xt.UDP_PORT else None

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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dst")
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

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("", a.src_port))
    dump = open(a.dump, "w") if a.dump else None
    n_dg = 0

    def send(d):
        nonlocal n_dg
        s.sendto(d, (a.dst, a.port))
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
