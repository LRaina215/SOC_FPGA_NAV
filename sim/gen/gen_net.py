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
    (out / "counts.txt").write_text(f"{len(lines)} {len(exp)} {len(el)} {len(tl)}\n")
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
        else:
            raise AssertionError("未知帧")
    assert kinds.count("arp") == 2 and kinds.count("udp") == 1, kinds
    print("发送帧检查通过:", kinds)


if __name__ == "__main__":
    if sys.argv[1] == "post":
        post(Path(sys.argv[2]))
    else:
        gen(Path(sys.argv[1]))
