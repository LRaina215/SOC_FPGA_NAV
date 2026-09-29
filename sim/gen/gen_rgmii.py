"""rgmii_mac100 测试激励：接收帧（含坏 FCS、RX_ER、半字节数为奇数）、发送帧（含不足 60 字节的补零）。
FCS 用 zlib.crc32 独立计算，不借用 RTL 的 CRC 实现。

用法: gen_rgmii.py <输出目录>
  rx.hex      接收激励：每行 {er, dv, nibble}（25 MHz 一拍一个半字节）
  rx_exp.hex  期望接收输出：每行 {error, last, byte}
  tx.hex      发送激励（客户端）：每行 {last, byte}
  tx_exp.hex  期望线上字节（前导码 + SFD + 数据 + 补零 + FCS）
"""
import random
import sys
import zlib
from pathlib import Path


def fcs(b):
    return zlib.crc32(bytes(b)).to_bytes(4, "little")


def main(out):
    out = Path(out)
    rng = random.Random(7)
    rx, rx_exp = [], []

    def idle(n):
        rx.extend([(0, 0, 0)] * n)

    def rx_frame(body, bad_fcs=False, er_at=None, odd=False):
        f = bytes(body) + fcs(body)
        if bad_fcs:
            f = f[:-1] + bytes([f[-1] ^ 0x5A])
        nib = [5] * 15 + [0xD]
        rx.extend((0, 1, n) for n in nib)
        k = 0
        for i, b in enumerate(f):
            for n in (b & 0xF, b >> 4):
                rx.append((1 if er_at == i else 0, 1, n))
                k += 1
        if odd:
            rx.append((0, 1, 0x3))
        idle(24)
        err = bad_fcs or er_at is not None or odd
        for i, b in enumerate(f):
            last = i == len(f) - 1
            rx_exp.append((int(err and last), int(last), b))

    idle(10)
    rx_frame([rng.randrange(256) for _ in range(60)])
    rx_frame([rng.randrange(256) for _ in range(1400)])
    rx_frame([rng.randrange(256) for _ in range(80)], bad_fcs=True)
    rx_frame([rng.randrange(256) for _ in range(70)], er_at=20)
    rx_frame([rng.randrange(256) for _ in range(64)], odd=True)
    rx_frame([rng.randrange(256) for _ in range(100)])

    tx, tx_exp = [], []
    for n in (42, 60, 333):                   # ARP 应答长度（需补零）、恰好 60、较长
        body = [rng.randrange(256) for _ in range(n)]
        for i, b in enumerate(body):
            tx.append((int(i == n - 1), b))
        padded = bytes(body) + bytes(max(0, 60 - n))
        tx_exp.extend(list(bytes([0x55] * 7 + [0xD5]) + padded + fcs(padded)))

    (out / "rx.hex").write_text("".join(f"{(e << 5) | (d << 4) | n:02x}\n" for e, d, n in rx))
    (out / "rx_exp.hex").write_text("".join(f"{(e << 9) | (l << 8) | b:03x}\n" for e, l, b in rx_exp))
    (out / "tx.hex").write_text("".join(f"{(l << 8) | b:03x}\n" for l, b in tx))
    (out / "tx_exp.hex").write_text("".join(f"{b:02x}\n" for b in tx_exp))
    (out / "counts.txt").write_text(f"{len(rx)} {len(rx_exp)} {len(tx)} {len(tx_exp)}\n")
    print(f"rx 半字节 {len(rx)}，期望字节 {len(rx_exp)}；tx 字节 {len(tx)}，线上字节 {len(tx_exp)}")


if __name__ == "__main__":
    main(sys.argv[1])
