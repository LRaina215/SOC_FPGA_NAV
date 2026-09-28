"""xt_frame_rx 测试激励与期望输出。

用法: gen_xt_rx.py <场景> <输出目录>
场景: be_depth   大端、仅深度；含丢包坏帧、灰度帧、垃圾字节
      le_inter   小端像素、深度+幅值逐像素交织，幅值阈值 + 原始无效值
      planar     深度+幅值平面布局，幅值在后
"""
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "model"))
import xt_proto as xt  # noqa: E402

W, H = 160, 60


def rand_frame(rng, w, h):
    d = [[rng.choice([0, rng.randint(1, 60000)]) if rng.random() < 0.05 else rng.randint(100, 20000)
          for _ in range(w)] for _ in range(h)]
    a = [[rng.randint(0, 3000) for _ in range(w)] for _ in range(h)]
    return d, a


def main(scn, out):
    out = Path(out)
    rng = random.Random(hash(scn) & 0xFFFF)
    cfg = dict(hdr_le=0, pix_le=0, layout=0, amp_first=0, udp_skip=20, size_adj=0, amp_min=0, raw_inv=0)
    stim = []   # (data, last, err)
    exp = []    # (u, v, depth, sof, eof)

    def push_packets(pkts, drop=None, err=None):
        for i, p in enumerate(pkts):
            if i == drop:
                continue
            for j, b in enumerate(p):
                stim.append((b, int(j == len(p) - 1), int(i == err and j == len(p) - 1)))

    def expect(d, a, w, h):
        for v in range(h):
            for u in range(w):
                x = d[v][u]
                ok = x != 0 and (cfg["raw_inv"] == 0 or x < cfg["raw_inv"])
                if a is not None and a[v][u] < cfg["amp_min"]:
                    ok = False
                exp.append((u, v, x if ok else 0, int(u == 0 and v == 0), int(u == w - 1 and v == h - 1)))

    if scn == "be_depth":
        frames = [(W, H, "ok"), (W, H, "drop"), (80, 30, "ok"), (W, H, "gray"), (W, H, "err"), (W, H, "ok")]
        for sn, (w, h, kind) in enumerate(frames):
            d, a = rand_frame(rng, w, h)
            it = 2 if kind == "gray" else 1
            fr = xt.build_frame(d, sn=sn, img_type=it)
            pk = xt.packetize(fr, udp_skip=cfg["udp_skip"], sn=sn)
            # 帧间插入一个不含 START 的无关 UDP 报文
            push_packets([bytes(rng.randint(0, 255) for _ in range(64))])
            push_packets(pk, drop=1 if kind == "drop" else None, err=3 if kind == "err" else None)
            if kind == "ok":
                expect(d, None, w, h)
    elif scn == "le_inter":
        cfg.update(pix_le=1, amp_min=500, raw_inv=0xFFF0)
        for sn in range(3):
            d, a = rand_frame(rng, W, H)
            for _ in range(50):
                d[rng.randrange(H)][rng.randrange(W)] = 0xFFF5
            fr = xt.build_frame(d, a, sn=sn, img_type=0, layout="interleaved", pix_le=True)
            push_packets(xt.packetize(fr, udp_skip=20, sn=sn))
            expect(d, a, W, H)
    elif scn == "planar":
        cfg.update(layout=1, amp_min=800, hdr_le=1)
        for sn in range(2):
            d, a = rand_frame(rng, W, H)
            fr = xt.build_frame(d, a, sn=sn, img_type=0, layout="planar", hdr_le=True)
            push_packets(xt.packetize(fr, udp_skip=20, sn=sn))
            expect(d, a, W, H)
    else:
        raise SystemExit(f"未知场景 {scn}")

    with open(out / "stim.hex", "w") as f:
        for b, l, e in stim:
            f.write(f"{(e << 9) | (l << 8) | b:03x}\n")
    with open(out / "exp.hex", "w") as f:
        for u, v, dd, s, e in exp:
            f.write(f"{(s << 33) | (e << 32) | (u << 24) | (v << 16) | dd:09x}\n")
    with open(out / "cfg.hex", "w") as f:
        for k in ("hdr_le", "pix_le", "layout", "amp_first", "udp_skip", "size_adj", "amp_min", "raw_inv"):
            f.write(f"{cfg[k] & 0xFFFF:04x}\n")
    with open(out / "counts.txt", "w") as f:
        f.write(f"{len(stim)} {len(exp)}\n")
    print(f"{scn}: stim={len(stim)} exp={len(exp)}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
