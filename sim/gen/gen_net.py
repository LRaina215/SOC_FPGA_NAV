"""网口协议栈测试：生成 MAC 接收帧序列与期望邮箱内容；post 模式检查发送帧。

用法: gen_net.py <输出目录>          生成激励
      gen_net.py post <输出目录>     检查 RTL 发出的帧（tx_frames.hex）
"""
import random
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "model"))
import netpkt as npk  # noqa: E402

MY_MAC = 0x024E56000002
MY_IP = "192.168.2.100"
MY_PORT = 9000
PEER_MAC = 0x0A1B2C3D4E5F
PEER_IP = "192.168.2.10"
TX_PAYLOAD = bytes(range(37)) + b"hello-rk3588"
OTHER_IP = "192.168.2.77"
# 原始帧发送：TCP SYN（54 字节，硬件补到 60）与一个 100 字节的带数据段
RAW_TX = [npk.tcp_frame(PEER_MAC, MY_MAC, MY_IP, PEER_IP, 49153, 7787, 1000, 0, npk.TCP_SYN),
          npk.tcp_frame(PEER_MAC, MY_MAC, MY_IP, PEER_IP, 49153, 7787, 1001, 555, npk.TCP_ACK | npk.TCP_PSH, bytes(range(46)))]


def words(b):
    b = bytes(b) + bytes(-len(b) % 4)
    return [struct.unpack_from("<I", b, i)[0] for i in range(0, len(b), 4)]


def gen(out):
    rng = random.Random(3)
    frames, exp = [], []      # frames: (bytes, err)

    def good(payload, sport=5555, src_ip=PEER_IP, fcs=False):
        frames.append((npk.udp_frame(MY_MAC, PEER_MAC, src_ip, MY_IP, sport, MY_PORT, payload, fcs=fcs), 0))
        exp.append(payload)

    frames.append((npk.arp_request_frame(PEER_MAC, PEER_IP, MY_IP), 0))                 # 应答
    frames.append((npk.arp_request_frame(PEER_MAC, PEER_IP, "192.168.2.99"), 0))         # 不是本机
    good(b"\x01goal")                                                                     # 短报文（帧有填充）
    good(bytes(rng.randrange(256) for _ in range(900)))
    good(bytes(rng.randrange(256) for _ in range(18)))                                    # 恰好 60 字节帧
    good(bytes(rng.randrange(256) for _ in range(200)), fcs=True)                         # 带 FCS
    f = bytearray(npk.udp_frame(MY_MAC, PEER_MAC, PEER_IP, MY_IP, 5555, MY_PORT, b"badcsum!"))
    f[24] ^= 0x55                                                                         # IP 校验和错
    frames.append((bytes(f), 0))
    frames.append((npk.udp_frame(MY_MAC, PEER_MAC, PEER_IP, MY_IP, 5555, 1234, b"wrongport"), 0))
    frames.append((npk.udp_frame(0x111111111111, PEER_MAC, PEER_IP, MY_IP, 5555, MY_PORT, b"notmine"), 0))
    frames.append((npk.udp_frame(MY_MAC, PEER_MAC, PEER_IP, MY_IP, 5555, MY_PORT, b"fcs error"), 1))  # MAC 报错
    trunc = npk.udp_frame(MY_MAC, PEER_MAC, PEER_IP, MY_IP, 5555, MY_PORT, bytes(300))
    frames.append((trunc[:200], 0))                                                       # 截断
    for _ in range(12):
        good(bytes(rng.randrange(256) for _ in range(rng.randint(1, 200))), sport=6000)
    frames.append((npk.arp_request_frame(PEER_MAC, PEER_IP, MY_IP), 0))                 # 第二次应答
    # 原始帧接收：发给本机的 ARP 应答、发给本机 IP 的 TCP 段进 4 格缓冲（第 5 个因格子满被丢弃），其余不收
    raw = []

    def rawf(f, keep, err=0):
        frames.append((f, err))
        if keep:
            raw.append(f)
    rawf(npk.arp_reply_frame(PEER_MAC, PEER_IP, MY_MAC, MY_IP), True)
    rawf(npk.arp_reply_frame(PEER_MAC, PEER_IP, MY_MAC, OTHER_IP), False)               # 不是给本机的
    rawf(npk.tcp_frame(MY_MAC, PEER_MAC, PEER_IP, MY_IP, 7787, 49153, 5000, 1001, npk.TCP_SYN | npk.TCP_ACK,
                       options=bytes([2, 4, 5, 0xB4])), True)                            # SYN-ACK（带 MSS 选项）
    rawf(npk.tcp_frame(MY_MAC, PEER_MAC, PEER_IP, OTHER_IP, 7787, 49153, 1, 1, npk.TCP_ACK), False)
    rawf(npk.tcp_frame(MY_MAC, PEER_MAC, PEER_IP, MY_IP, 7787, 49153, 2, 2, npk.TCP_ACK), False, err=1)   # FCS 错
    rawf(npk.tcp_frame(MY_MAC, PEER_MAC, PEER_IP, MY_IP, 7787, 49153, 5001, 1018, npk.TCP_ACK | npk.TCP_PSH,
                       bytes(rng.randrange(256) for _ in range(150))), True)             # 长于 128 字节：只存前 128
    rawf(npk.tcp_frame(MY_MAC, PEER_MAC, PEER_IP, MY_IP, 7787, 49153, 5151, 1018, npk.TCP_FIN | npk.TCP_ACK), True)
    rawf(npk.tcp_frame(MY_MAC, PEER_MAC, PEER_IP, MY_IP, 7787, 49153, 5152, 1019, npk.TCP_ACK), False)      # 格子满

    # 测试台在全部帧注入后才读邮箱，总量需小于接收缓冲（4096 字节）
    assert sum(len(p) for p in exp) < 4000, sum(len(p) for p in exp)
    lines = []
    for fr, err in frames:
        for i, b in enumerate(fr):
            last = int(i == len(fr) - 1)
            lines.append(f"{((err & last) << 9) | (last << 8) | b:03x}")
    (out / "rx.hex").write_text("\n".join(lines) + "\n")
    el = []
    for p in exp:
        el.append(f"{len(p):08x}")
        for i in range(0, len(p), 4):
            w = p[i:i + 4] + b"\x00" * (4 - len(p[i:i + 4]))
            el.append(f"{struct.unpack('<I', w)[0]:08x}")
    (out / "mbx_exp.hex").write_text("\n".join(el) + "\n")
    tl = [f"{len(TX_PAYLOAD):08x}"]
    for i in range(0, len(TX_PAYLOAD), 4):
        w = TX_PAYLOAD[i:i + 4] + b"\x00" * (4 - len(TX_PAYLOAD[i:i + 4]))
        tl.append(f"{struct.unpack('<I', w)[0]:08x}")
    (out / "tx_payload.hex").write_text("\n".join(tl) + "\n")
    rl = [len(raw)]
    for f in raw:
        rl += [len(f), (min(len(f), 128) + 3) // 4] + words(f[:128])
    (out / "raw_exp.hex").write_text("".join(f"{v:08x}\n" for v in rl))
    xl = [len(RAW_TX)]
    for f in RAW_TX:
        xl += [len(f)] + words(f)
    (out / "raw_tx.hex").write_text("".join(f"{v:08x}\n" for v in xl))
    (out / "counts.txt").write_text(f"{len(lines)} {len(exp)} {len(el)} {len(tl)} {len(rl)} {len(xl)}\n")
    print(f"帧 {len(frames)}，期望报文 {len(exp)}")


def post(out):
    text = (out / "tx_frames.hex").read_text().split()
    frames, cur = [], bytearray()
    for t in text:
        v = int(t, 16)
        cur.append(v & 0xFF)
        if v >> 8:
            frames.append(bytes(cur))
            cur = bytearray()
    kinds = []
    for f in frames:
        assert len(f) >= 60, f"帧过短 {len(f)}"
        d = npk.parse(f)
        assert d["src"] == npk.mac_bytes(MY_MAC), "源 MAC 错"
        if d["type"] == "arp":
            assert d["oper"] == 2 and d["dst"] == npk.mac_bytes(PEER_MAC)
            assert d["tha"] == npk.mac_bytes(PEER_MAC) and d["tpa"] == npk.ip_bytes(PEER_IP)
            assert d["spa"] == npk.ip_bytes(MY_IP) and d["sha"] == npk.mac_bytes(MY_MAC)
            kinds.append("arp")
        elif d["type"] == "udp":
            assert d["ip_ok"], "IP 校验和错"
            assert d["dst"] == npk.mac_bytes(PEER_MAC) and d["dst_ip"] == npk.ip_bytes(PEER_IP)
            assert d["dport"] == 6000 and d["sport"] == MY_PORT, (d["dport"], d["sport"])
            assert d["payload"] == TX_PAYLOAD, "负载不一致"
            kinds.append("udp")
        elif d["type"] == "tcp":
            k = kinds.count("tcp")
            ref = RAW_TX[k] + bytes(max(0, 60 - len(RAW_TX[k])))
            assert f == ref, f"原始帧 {k} 不一致"
            assert d["ip_ok"] and d["tcp_ok"], "原始帧校验和错"
            kinds.append("tcp")
        else:
            raise AssertionError("未知帧")
    assert kinds.count("arp") == 2 and kinds.count("udp") == 1 and kinds.count("tcp") == len(RAW_TX), kinds
    print("发送帧检查通过:", kinds)


if __name__ == "__main__":
    if sys.argv[1] == "post":
        post(Path(sys.argv[2]))
    else:
        gen(Path(sys.argv[1]))
