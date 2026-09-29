"""xt_frame_rx 测试激励与期望输出（期望由 model/xt_proto.py RxModel 给出，与 RTL 判定点一一对应）。

用法: gen_xt_rx.py <场景> <输出目录>
场景: sdk_depth  SDK 格式、仅深度：正常帧、丢包、乱序、重复、IMU 消息夹在帧中（单独报文 / 与帧分包同报文）、
                 灰度帧、错误报文、80×30、坏结束标记、两个分包一个报文、迟到的零散分包、报文截断、垃圾报文
      sdk_amp    SDK 格式、深度+幅值交织：幅值阈值、深度/幅值无效码；FrameInfo 锁存（单位、合并、ROI、温度）
      be_v2      旧版：大端头部、52 字节帧尾（不锁存 FrameInfo）
"""
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "model"))
import xt_proto as xt  # noqa: E402

W, H = 160, 60


def rand_img(rng, w, h, lo=100, hi=20000, p_inv=0.05):
    codes = [0, 64000, 64001, 64002, 64003, 64007, 64010]
    return [[rng.choice(codes) if rng.random() < p_inv else rng.randint(lo, hi) for _ in range(w)] for _ in range(h)]


def main(scn, out):
    out = Path(out)
    rng = random.Random(sum(map(ord, scn)))
    cfg = dict(hdr_le=1, pix_le=1, subhdr=20, amp_min=0, raw_inv=xt.INVALID_MIN)
    dgrams = []          # (字节, err)

    def send(subs, per=1, err_at=None, cut=None):
        """分包 -> 报文；err_at: 第几个报文带错误标志；cut: (第几个报文, 截掉的字节数)。"""
        for i, d in enumerate(xt.datagrams(subs, per=per)):
            if cut and cut[0] == i:
                d = d[:-cut[1]]
            dgrams.append((d, i == err_at))

    if scn == "sdk_depth":
        sn = 0

        def frame(w=W, h=H, **kw):
            nonlocal sn
            sn += 1
            return sn, xt.build_frame(rand_img(rng, w, h), sn=sn, **kw)

        s, f = frame(); send(xt.packetize(f, s))                                   # 正常
        dgrams.append((bytes(rng.randint(0, 255) for _ in range(64)), False))      # 垃圾报文
        s, f = frame(); p = xt.packetize(f, s); del p[3]; send(p)                  # 丢包
        s, f = frame(); p = xt.packetize(f, s)                                     # IMU 夹在帧中
        p.insert(2, xt.imu_subpacket(900)); p.insert(7, xt.imu_subpacket(901)); send(p)
        s, f = frame(); p = xt.packetize(f, s); p[5], p[6] = p[6], p[5]; send(p)   # 乱序
        s, f = frame(); p = xt.packetize(f, s); p.insert(3, p[2]); send(p)         # 重复
        s, f = frame(img_type=xt.TYPE_GRAY); send(xt.packetize(f, s))              # 灰度：忽略
        s, f = frame(); send(xt.packetize(f, s), err_at=4)                        # 错误报文
        s, f = frame(80, 30); send(xt.packetize(f, s))                             # 80×30
        s, f = frame(); f = f[:-1] + b"\x00"; send(xt.packetize(f, s))             # 坏结束标记
        s, f = frame(); p = xt.packetize(f, s, chunk=700)                          # 两个分包一个报文
        p.insert(9, xt.imu_subpacket(902)); send(p, per=2)
        send([p[4]])                                                               # 迟到的零散分包
        s, f = frame(); send(xt.packetize(f, s), cut=(6, 100))                     # 报文截断
        s, f = frame(); send(xt.packetize(f, s))                                   # 正常
        s, f = frame(); send(xt.packetize(f, s)[:-1])                              # 缺最后一包，被下一帧顶掉
        s, f = frame(); send(xt.packetize(f, s))                                   # 正常
    elif scn == "sdk_amp":
        cfg.update(amp_min=500)
        for k in range(3):
            d, a = rand_img(rng, W, H), rand_img(rng, W, H, 0, 3000, 0.03)
            info = None
            if k == 2:
                info = xt.frame_info(W, H, imageflags=3, unit_div=2, binning=1, roi=(0, 0), temps=(4321, -123))
            fr = xt.build_frame(d, a, sn=10 + k, info=info)
            send(xt.packetize(fr, 10 + k))
    elif scn == "be_v2":
        cfg.update(hdr_le=0)
        for k in range(2):
            fr = xt.build_frame(rand_img(rng, W, H), sn=20 + k, proto=2, hdr_le=False)
            send(xt.packetize(fr, 20 + k, hdr_le=False))
    else:
        raise SystemExit(f"未知场景 {scn}")

    m = xt.RxModel(hdr_le=bool(cfg["hdr_le"]), pix_le=bool(cfg["pix_le"]), subhdr=cfg["subhdr"],
                   amp_min=cfg["amp_min"], raw_inv=cfg["raw_inv"])
    stim = []
    for d, e in dgrams:
        m.feed(d, e)
        for j, b in enumerate(d):
            last = j == len(d) - 1
            stim.append((b, int(last), int(e and last)))
    exp = []
    for fr in m.frames:
        for v in range(fr["h"]):
            for u in range(fr["w"]):
                exp.append((u, v, fr["depth"][v][u], int(u == 0 and v == 0), int(u == fr["w"] - 1 and v == fr["h"] - 1)))
    infos = [fr["info"] for fr in m.frames if fr["info"]]
    li = infos[-1] if infos else None
    chk = [len(m.frames), m.bad, m.skip, len(infos)]
    if li:
        t0, t1 = li["temps"]
        chk += [li["lens_raw"][0], li["lens_raw"][8],
                (li["roiy0"] << 24) | (li["roix0"] << 16) | (li["binning"] << 8) | li["unit_div"],
                (li["hdrmode"] << 24) | (li["imageflags"] << 16) | li["infosize"],
                ((t1 & 0xFFFF) << 16) | (t0 & 0xFFFF)]
    else:
        chk += [0] * 5

    with open(out / "stim.hex", "w") as f:
        for b, l, e in stim:
            f.write(f"{(e << 9) | (l << 8) | b:03x}\n")
    with open(out / "exp.hex", "w") as f:
        for u, v, dd, s, e in exp:
            f.write(f"{(s << 33) | (e << 32) | (u << 24) | (v << 16) | dd:09x}\n")
    with open(out / "cfg.hex", "w") as f:
        for k in ("hdr_le", "pix_le", "subhdr", "amp_min", "raw_inv"):
            f.write(f"{cfg[k] & 0xFFFF:04x}\n")
    with open(out / "chk.hex", "w") as f:
        for v in chk:
            f.write(f"{v & 0xFFFFFFFF:08x}\n")
    with open(out / "counts.txt", "w") as f:
        f.write(f"{len(stim)} {len(exp)}\n")
    print(f"{scn}: stim={len(stim)} exp={len(exp)} frames={len(m.frames)} bad={m.bad} skip={m.skip} info={len(infos)}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
