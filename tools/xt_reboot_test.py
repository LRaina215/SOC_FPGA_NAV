"""雷达冷启动复测：不碰电源，经板子让 XT-M60 重启，检查软核的雷达启动客户端（fw/nav/xt_link.c）。

做法：经 FPGA 雷达口的原始帧通道（LRAW_TX / LRAW_RX）给雷达发厂家重启命令 13（数据 "XINTAN"），同时让软核重启，
相当于"雷达和板上固件同时冷启动"；之后记录控制台里的 lidar start / ok 与帧计数，打印启动尝试次数、第一帧时刻与帧率。
发命令期间软核保持复位：软核空闲时会把原始帧接收格清空，会和本脚本抢同一批帧；保持复位期间到达的雷达帧会被丢（LIDAR_STAT_DROP 增加）。
前提：导航比特流已下载、雷达直连板子 RJ45 且正在出流（雷达 IP 用缺省 192.168.0.101）。

用法: python tools/xt_reboot_test.py COM11 [次数]
通过标准（docs/3_guide/hardware_guide.md 6.4 节第四步）：每次只出现一次 lidar start ok，之后 10 帧/秒、坏帧 0；
雷达还在启动时第一次尝试会因 ARP 没有应答而失败，属正常。
"""
import struct
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "tools"), str(ROOT / "model")]
import nav_uart  # noqa: E402
import nav_regs_map as m  # noqa: E402

R = m.REGS
MY_MAC = bytes.fromhex("024E56000001")
MY_IP = bytes([192, 168, 0, 100])
LIDAR_IP = bytes([192, 168, 0, 101])
START, END = bytes.fromhex("7EFFAA55"), bytes.fromhex("FF7E55AA")
CMD_RESET = START + struct.pack("<I", 1 + 6 + 2) + bytes([13]) + b"XINTAN" + b"\x00\x01" + END


def csum(b):
    if len(b) % 2:
        b += b"\x00"
    s = sum(struct.unpack(f"!{len(b) // 2}H", b))
    while s >> 16:
        s = (s & 0xFFFF) + (s >> 16)
    return (~s) & 0xFFFF


def raw_send(u, frame):
    assert len(frame) <= 128
    for _ in range(100):
        if not (u.rd(R["NETL_RAW_STAT"]) & 1):
            break
    pad = frame + bytes(-len(frame) % 4)
    u.wr_burst([(R["WIN_LRAW_TX"] + i, struct.unpack_from("<I", pad, i)[0]) for i in range(0, len(pad), 4)]
               + [(R["NETL_RAW_TX"], len(frame))])


def raw_recv(u, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        st = u.rd(R["NETL_RAW_STAT"])
        if (st >> 1) & 7:
            n = min((st >> 8) & 0x7FF, 128)
            words = [u.rd(R["WIN_LRAW_RX"] + 4 * k) for k in range((n + 3) // 4)]
            u.wr(R["NETL_RAW_POP"], 1)
            return b"".join(struct.pack("<I", w) for w in words)[:n]
        time.sleep(0.002)
    return None


def drain(u):
    while (u.rd(R["NETL_RAW_STAT"]) >> 1) & 7:
        u.wr(R["NETL_RAW_POP"], 1)


def arp(u):
    req = (b"\xff" * 6 + MY_MAC + b"\x08\x06" + struct.pack("!HHBBH", 1, 0x0800, 6, 4, 1)
           + MY_MAC + MY_IP + bytes(6) + LIDAR_IP)
    for _ in range(10):
        raw_send(u, req)
        t0 = time.time()
        while time.time() - t0 < 0.5:
            f = raw_recv(u, 0.5)
            if f and f[12:14] == b"\x08\x06" and struct.unpack("!H", f[20:22])[0] == 2 and f[28:32] == LIDAR_IP:
                return f[22:28]
    return None


def tcp_frame(dst_mac, sport, seq, ack, flags, payload=b"", opts=b""):
    thl = 20 + len(opts)
    tcp = struct.pack("!HHIIBBHHH", sport, 7787, seq, ack, (thl // 4) << 4, flags, 4096, 0, 0) + opts
    ph = MY_IP + LIDAR_IP + struct.pack("!BBH", 0, 6, thl + len(payload))
    c = csum(ph + tcp + payload)
    tcp = tcp[:16] + struct.pack("!H", c) + tcp[18:]
    ip = struct.pack("!BBHHHBBH4s4s", 0x45, 0, 20 + thl + len(payload), 0x1234, 0x4000, 64, 6, 0, MY_IP, LIDAR_IP)
    ip = ip[:10] + struct.pack("!H", csum(ip)) + ip[12:]
    return dst_mac + MY_MAC + b"\x08\x00" + ip + tcp + payload


def tcp_seg(f):
    if not f or f[12:14] != b"\x08\x00" or f[23] != 6 or f[26:30] != LIDAR_IP:
        return None
    ihl = (f[14] & 15) * 4
    t = f[14 + ihl:]
    sport, dport, seq, ack, off, flags = struct.unpack("!HHIIBB", t[:14])
    tot = struct.unpack("!H", f[16:18])[0]
    data = t[(off >> 4) * 4: tot - ihl]
    return dict(seq=seq, ack=ack, flags=flags, data=data, dport=dport)


def send_reset(u, log):
    mac = arp(u)
    if mac is None:
        log("ARP 无应答")
        return False
    sport, iss = 40000 + int(time.time()) % 20000, 0x1000_0000
    raw_send(u, tcp_frame(mac, sport, iss, 0, 0x02, opts=b"\x02\x04\x05\xb4"))   # SYN，MSS 1460
    t0 = time.time()
    s = None
    while time.time() - t0 < 1.5:
        s = tcp_seg(raw_recv(u, 0.3))
        if s and s["dport"] == sport and s["flags"] & 0x12 == 0x12:
            break
        s = None
    if s is None:
        log("没有收到 SYN-ACK")
        return False
    rcv = (s["seq"] + 1) & 0xFFFFFFFF
    raw_send(u, tcp_frame(mac, sport, iss + 1, rcv, 0x18, CMD_RESET))              # ACK | PSH + 命令 13
    t0 = time.time()
    got = []
    while time.time() - t0 < 0.8:
        s = tcp_seg(raw_recv(u, 0.2))
        if s and s["dport"] == sport:
            got.append((s["flags"], len(s["data"])))
    raw_send(u, tcp_frame(mac, sport, iss + 1 + len(CMD_RESET), rcv, 0x14))        # RST | ACK
    log(f"重启命令已发（雷达 MAC {mac.hex(':')}，之后收到的段 {got}）")
    return True


def main():
    port = sys.argv[1]
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    u = nav_uart.NavUart(port)
    for k in range(1, n + 1):
        t_ref = time.time()
        log = lambda s: print(f"[第 {k} 次] {time.time() - t_ref:6.2f}s  {s}", flush=True)
        f_before = u.rd(R["LIDAR_STAT_FRAMES"])
        u.wr(u.SOC_CTRL, 1)                       # 软核保持复位
        drain(u)
        ok = send_reset(u, log)
        u.wr(u.SOC_CTRL, 0)                       # 放开：固件从头启动
        t_rel = time.time()
        wp = u.rd(u.SOC_CON_WPTR)
        log(f"软核放开（固件重启），控制台指针 {wp}")
        if not ok:
            return 1
        buf, oks, starts, first = "", 0, 0, None
        f_last = u.rd(R["LIDAR_STAT_FRAMES"])
        while time.time() - t_rel < 45:
            cur = u.rd(u.SOC_CON_WPTR)
            if cur < wp:
                wp = 0
            b, wp = u.con_read(wp)
            buf += b.decode("utf-8", "replace")
            lines = buf.split("\n")
            buf = lines.pop()
            for ln in lines:
                if "lidar" in ln:
                    log("CON " + ln.strip())
                    oks += "start ok" in ln
                    starts += "lidar start #" in ln
            f = u.rd(R["LIDAR_STAT_FRAMES"])
            if first is None and f != f_last and time.time() - t_rel > 0.5:
                first = time.time() - t_rel
                log(f"第一帧（帧计数 {f_last} -> {f}）")
            f_last = f
            time.sleep(0.2)
        a = u.rd(R["LIDAR_STAT_FRAMES"])
        time.sleep(5)
        rate = (u.rd(R["LIDAR_STAT_FRAMES"]) - a) / 5
        print(f"[第 {k} 次] 结果：启动尝试 {starts}，成功 {oks}，放开软核到第一帧 {first} s，之后帧率 {rate:.1f}/s，"
              f"BAD {u.rd(R['LIDAR_STAT_BAD'])}，RX_ERR {u.rd(R['NETL_STAT_RX_ERR'])}，重启前帧计数 {f_before}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
