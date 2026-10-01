"""雷达启动 TCP 客户端（fw/nav/xt_link.c）对假雷达的主机测试。

假雷达按实测行为建模（docs/2_interface/icd.md 第 2 节）：7787 端口一次只服务一个连接；收到命令 1 {1,1} 回应答并开始出流，
UDP 发给最近一次连它的主机；TCP 断开后继续出流。报文用 model/netpkt.py 独立构造和校验（校验和、端口、序号）。
固件发出的每一帧都检查：源 MAC/IP、目的 MAC（ARP 学到的）、IP/TCP 校验和、长度与 IP 总长一致。
场景：正常启动、已在出流、开关关闭、雷达晚启动、丢 SYN、SYN-ACK 校验和错、丢数据段、SYN 被 RST、应答前有上报/应答拆段、
应答码失败、流中断（雷达重启）、RST 关闭、雷达先关、雷达不应答但出流、走 nav_app_poll 整体接入。
用法: python sim/closed_loop/test_xt_link.py [--no-build]
"""
import ctypes
import struct
import sys
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "model"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import nav_regs_map as rm  # noqa: E402
import netpkt as np_  # noqa: E402
import run_cl  # noqa: E402
import xt_proto as xt  # noqa: E402

# 固件一侧用寄存器复位值（本机地址、雷达 IP），假雷达用雷达出厂缺省：两边对不上测试就不通过
R = rm.RESET
FW_MAC = ((R["NETL_MAC_HI"] & 0xFFFF) << 32 | R["NETL_MAC_LO"]).to_bytes(6, "big")
FW_IP = ".".join(str(b) for b in R["NETL_IP"].to_bytes(4, "big"))
FW_LIDAR_IP = R["NETL_LIDAR_IP"]
L_MAC = bytes.fromhex("7822F633C735")
L_IP = xt.DEFAULT_LIDAR_IP
START = bytes([0x7E, 0xFF, 0xAA, 0x55])
END = bytes([0xFF, 0x7E, 0x55, 0xAA])
CMD1 = START + struct.pack("<I", 5) + bytes([1, 1, 1, 0, 1]) + END
MSS_OPT = bytes([2, 4, 0x05, 0xB4])
ST = ["WATCH", "ARP", "SYN", "CMD", "FIN", "BACKOFF"]


def ip2int(s):
    return int.from_bytes(np_.ip_bytes(s), "big")


def xt_msg(body):
    return START + struct.pack("<I", len(body)) + body + END


class FakeLidar:
    def __init__(self, t_boot=0.0, streaming=False, drop_syn=0, rst_syn=0, drop_data=0, resp="normal", rc=0,
                 fin_after=False, bad_synack=0, warmup=0.0):
        self.t_boot, self.streaming = t_boot, streaming
        self.warmup, self.stream_at = warmup, None       # 应答 OK 后过 warmup 秒才出第一帧（实测刚上电约 3 s）
        self.drop_syn, self.rst_syn, self.drop_data, self.bad_synack = drop_syn, rst_syn, drop_data, bad_synack
        self.resp, self.rc, self.fin_after = resp, rc, fin_after
        self.conn = None
        self.out = []
        self.err = []
        self.cmds = []
        self.n_arp = self.n_syn = self.n_rst_rx = self.n_closed = self.n_stray = 0
        self.t_stream = 0.0 if streaming else None

    def up(self, t):
        return t >= self.t_boot

    def reboot(self, t, boot_s):
        self.streaming, self.conn, self.t_boot, self.t_stream, self.stream_at = False, None, t + boot_s, None, None

    def seg(self, c, flags, payload=b"", options=b"", bad=False):
        self.out.append(np_.tcp_frame(FW_MAC, L_MAC, L_IP, FW_IP, 7787, c["cport"], c["snd"], c["rcv"], flags,
                                      payload, options=options, bad_csum=bad))
        c["snd"] += len(payload) + (1 if flags & (np_.TCP_SYN | np_.TCP_FIN) else 0)

    def rst(self, sport, seq, ack):
        self.out.append(np_.tcp_frame(FW_MAC, L_MAC, L_IP, FW_IP, 7787, sport, seq, ack, np_.TCP_RST | np_.TCP_ACK))

    def check(self, f):
        """固件发出帧的独立检查"""
        e = self.err
        if len(f) > 128:
            e.append(f"帧长 {len(f)} > 128")
        if f[6:12] != FW_MAC:
            e.append("源 MAC 不对")
        p = np_.parse(f)
        if p["type"] == "arp":
            if not (p["dst"] == np_.BCAST and p["oper"] == 1 and p["sha"] == FW_MAC and
                    p["spa"] == np_.ip_bytes(FW_IP) and p["tpa"] == np_.ip_bytes(L_IP) and len(f) == 42):
                e.append("ARP 请求格式不对")
            return p
        if p["type"] != "tcp":
            e.append(f"不认识的帧 {p['type']}")
            return None
        tot = struct.unpack(">H", f[16:18])[0]
        if not (p["ip_ok"] and p["tcp_ok"]):
            e.append("IP/TCP 校验和错")
            return None                                  # 真协议栈会丢掉
        if tot != len(f) - 14:
            e.append(f"IP 总长 {tot} 与帧长 {len(f)} 不符")
        if p["dst"] != L_MAC or p["src_ip"] != np_.ip_bytes(FW_IP) or p["dst_ip"] != np_.ip_bytes(L_IP):
            e.append("TCP 地址不对")
        if p["dport"] != 7787 or not 49152 <= p["sport"] <= 65535:
            e.append(f"端口不对 {p['sport']}->{p['dport']}")
        if p["flags"] & np_.TCP_SYN and f[54:58] != MSS_OPT:
            e.append("SYN 没带 MSS")
        return p

    def on_frame(self, f, t):
        p = self.check(f)
        if p is None or not self.up(t):
            return
        if p["type"] == "arp":
            self.n_arp += 1
            self.out.append(np_.arp_reply_frame(L_MAC, L_IP, FW_MAC, FW_IP))
            return
        fl, sport, seq, ack, pl = p["flags"], p["sport"], p["seq"], p["ack"], p["payload"]
        c = self.conn
        if fl & np_.TCP_RST:
            self.n_rst_rx += 1
            if c and c["cport"] == sport:
                self.conn = None
            return
        if fl & np_.TCP_SYN:
            self.n_syn += 1
            if self.rst_syn > 0:
                self.rst_syn -= 1
                self.rst(sport, 0, seq + 1)
                return
            if self.drop_syn > 0:
                self.drop_syn -= 1
                return
            c = self.conn = dict(cport=sport, st="SYN_RCVD", iss=0x13570000 + self.n_syn * 7919, rcv=seq + 1,
                                 buf=b"", fin_sent=False, fin_acked=False, peer_fin=False)
            c["snd"] = c["iss"]
            self.seg(c, np_.TCP_SYN | np_.TCP_ACK, options=MSS_OPT, bad=self.bad_synack > 0)
            self.bad_synack -= self.bad_synack > 0
            return
        if c is None or c["cport"] != sport:
            self.n_stray += 1
            if not fl & np_.TCP_RST:
                self.rst(sport, ack, seq + len(pl))
            return
        if fl & np_.TCP_ACK:
            if c["st"] == "SYN_RCVD" and ack == c["iss"] + 1:
                c["st"] = "EST"
            if c["fin_sent"] and ack == c["snd"]:
                c["fin_acked"] = True
        if c["st"] != "EST":
            return
        if pl:
            if self.drop_data > 0:
                self.drop_data -= 1
                return
            if seq == c["rcv"]:
                c["buf"] += pl
                c["rcv"] += len(pl)
                self.seg(c, np_.TCP_ACK)
                self.commands(c, t)
            else:
                self.seg(c, np_.TCP_ACK)
        if fl & np_.TCP_FIN and seq + len(pl) == c["rcv"]:
            c["rcv"] += 1
            c["peer_fin"] = True
            if c["fin_sent"]:
                self.seg(c, np_.TCP_ACK)
            else:
                self.seg(c, np_.TCP_FIN | np_.TCP_ACK)
                c["fin_sent"] = True
        if c["peer_fin"] and c["fin_acked"]:
            self.conn = None
            self.n_closed += 1

    def commands(self, c, t):
        b = c["buf"]
        while True:
            i = b.find(START)
            if i < 0 or len(b) < i + 8:
                break
            n = struct.unpack_from("<I", b, i + 4)[0]
            if len(b) < i + 12 + n:
                break
            body, ok = b[i + 8:i + 8 + n], b[i + 8 + n:i + 12 + n] == END
            b = b[i + 12 + n:]
            self.cmds.append(body)
            if not ok or body != bytes([1, 1, 1, 0, 1]):
                self.err.append(f"命令不对 {body.hex()}")
                continue
            if self.rc == 0 and not self.streaming and self.stream_at is None:
                self.stream_at = t + self.warmup
            resp = xt_msg(bytes([1, self.rc, 2, 3]))
            log = xt_msg(bytes([0x2A]) + b"XT-M60 log: measure start" + bytes([0, 2, 3]))
            if self.resp == "normal":
                self.seg(c, np_.TCP_ACK | np_.TCP_PSH, resp)
            elif self.resp == "log_first":
                self.seg(c, np_.TCP_ACK | np_.TCP_PSH, log)
                self.seg(c, np_.TCP_ACK | np_.TCP_PSH, resp)
            elif self.resp == "split":
                self.seg(c, np_.TCP_ACK | np_.TCP_PSH, log + resp[:7])
                self.seg(c, np_.TCP_ACK | np_.TCP_PSH, resp[7:])
            if self.fin_after and self.resp != "none":
                self.seg(c, np_.TCP_FIN | np_.TCP_ACK)
                c["fin_sent"] = True
        c["buf"] = b


def run(d, lid, T, tcp_en=True, close_rst=False, events=(), app=False, dt_us=1000):
    """1 ms 一步：假雷达的输出下一步送进固件（按硬件 4 槽，放不下的等下一步），固件发出的帧当步交给假雷达。"""
    d.py_set_netl(FW_MAC, ip2int(FW_IP), FW_LIDAR_IP, int(tcp_en))
    d.py_set_lidar_frames(0)
    d.py_init()
    d.py_xl_close_rst(int(close_rst))
    buf = (ctypes.c_uint8 * 256)()
    stat = (ctypes.c_int32 * 10)()
    ev = sorted(events, key=lambda e: e[0])
    r = dict(tx=[], t_ok=[], trans=[], frames=0)
    t_next, n_ok, st = 0.0, 0, None
    t_us = 1000
    while t_us < T * 1e6:
        t = t_us / 1e6
        while ev and ev[0][0] <= t:
            ev.pop(0)[1](lid, t)
        while lid.out:
            f = lid.out[0]
            if not d.py_lraw_push_rx(f + struct.pack("<I", zlib.crc32(f) & 0xFFFFFFFF), len(f) + 4):
                break
            lid.out.pop(0)
        if lid.stream_at is not None and t >= lid.stream_at:
            lid.streaming, lid.stream_at = True, None
            if lid.t_stream is None:
                lid.t_stream = t
        if lid.streaming and lid.up(t) and t >= t_next:
            r["frames"] += 1
            t_next = t + 0.1
            d.py_set_lidar_frames(r["frames"])
        d.py_set_time(t_us & 0xFFFFFFFF)                   # 板上 TIME_US 是 32 位微秒，约 71.6 分钟回绕
        d.py_poll() if app else d.py_xl_poll()
        while True:
            n = d.py_lraw_pop_tx(buf)
            if n <= 0:
                break
            f = bytes(buf[:n])
            r["tx"].append((t, f))
            lid.on_frame(f, t)
        d.py_xl_stat(stat)
        if stat[0] != st:
            st = stat[0]
            r["trans"].append((round(t, 3), ST[st]))
        if stat[2] != n_ok:
            n_ok = stat[2]
            r["t_ok"].append(t)
        t_us += dt_us
    r["stat"] = dict(zip(("st", "n_start", "n_ok", "n_fail", "n_rx", "n_tx", "last_resp", "lport", "tries", "rx_drop"),
                         list(stat)))
    return r


def kinds(r):
    """固件发出帧的种类序列：ARP / S / D（带数据）/ A / F / R"""
    out = []
    for _, f in r["tx"]:
        p = np_.parse(f)
        if p["type"] == "arp":
            out.append("ARP")
        else:
            fl = p["flags"]
            out.append("S" if fl & np_.TCP_SYN else "R" if fl & np_.TCP_RST else "F" if fl & np_.TCP_FIN
                       else "D" if p["payload"] else "A")
    return out


def near(v, lo, hi):
    return v is not None and lo <= v <= hi


def scenarios():
    """(名称, 假雷达参数, run 参数, 检查函数 (r, lid) -> 问题列表)"""
    def ok_once(t_lo, t_hi, starts=1, fails=0, closed=1):
        def chk(r, lid):
            s, bad = r["stat"], []
            want = (s["n_start"] if starts is None else starts, 1, s["n_fail"] if fails is None else fails)
            if (s["n_start"], s["n_ok"], s["n_fail"]) != want:
                bad.append(f"启动/成功/失败 = {s['n_start']}/{s['n_ok']}/{s['n_fail']}，应为 {want[0]}/1/{want[2]}")
            if not r["t_ok"] or not near(r["t_ok"][0], t_lo, t_hi):
                bad.append(f"成功时刻 {r['t_ok'][:1]} 不在 [{t_lo}, {t_hi}] s")
            if not lid.streaming or len(lid.cmds) != 1:
                bad.append(f"雷达出流 {lid.streaming}，收到命令 {len(lid.cmds)} 条")
            if closed is not None and (lid.n_closed != closed or lid.conn is not None):
                bad.append(f"连接没正常关闭（关闭 {lid.n_closed}，残留 {lid.conn is not None}）")
            if s["st"] != 0:
                bad.append(f"结束状态 {ST[s['st']]}")
            return bad
        return chk

    def seq_is(want):
        def chk(r, lid):
            k = kinds(r)
            return [] if k == want else [f"发出帧序列 {k}，应为 {want}"]
        return chk

    def both(*fs):
        return lambda r, lid: sum((f(r, lid) for f in fs), [])

    def silent(r, lid):
        s = r["stat"]
        return [] if s["n_start"] == 0 and not r["tx"] else [f"不该动作：启动 {s['n_start']}，发出 {len(r['tx'])} 帧"]

    def rc_recover(r, lid):
        s, bad = r["stat"], []
        if s["n_ok"] != 1 or s["n_fail"] < 2 or not lid.streaming:
            bad.append(f"成功 {s['n_ok']} 失败 {s['n_fail']} 出流 {lid.streaming}")
        if not r["t_ok"] or not near(r["t_ok"][0], 10.0, 14.0):
            bad.append(f"恢复时刻 {r['t_ok'][:1]}")
        if lid.n_closed != s["n_start"]:
            bad.append(f"每次连接都应正常关闭：启动 {s['n_start']}，关闭 {lid.n_closed}")
        return bad

    def reboot(r, lid):
        s, bad = r["stat"], []
        if s["n_ok"] != 2 or len(r["t_ok"]) != 2 or not near(r["t_ok"][1], 16.0, 20.0) or not lid.streaming:
            bad.append(f"成功 {s['n_ok']} 次，时刻 {[round(x, 2) for x in r['t_ok']]}，出流 {lid.streaming}")
        t1 = [t for t, f in r["tx"] if 5.0 < t < 11.5]
        if t1:
            bad.append(f"出流正常期间不该发帧（{len(t1)} 帧）")
        return bad

    def no_resp(r, lid):
        s, k = r["stat"], kinds(r)
        bad = [] if (s["n_start"], s["n_ok"], s["n_fail"]) == (1, 0, 1) else [f"启动/成功/失败 {s['n_start']}/{s['n_ok']}/{s['n_fail']}"]
        if k[-1:] != ["R"] or lid.conn is not None or not lid.streaming:
            bad.append(f"超时后应 RST 关闭：{k}，残留连接 {lid.conn is not None}")
        return bad

    def absent(r, lid):
        k = kinds(r)
        bad = [] if set(k) <= {"ARP"} and 30 <= len(k) <= 45 else [f"雷达不在时应只发 ARP 且限速：{len(k)} 帧 {set(k)}"]
        s = r["stat"]
        if s["n_fail"] < 3:
            bad.append(f"失败次数 {s['n_fail']}")
        return bad

    base = ["ARP", "S", "D", "A", "F", "A"]
    return [
        ("正常启动", {}, dict(T=20), both(ok_once(2.0, 2.1), seq_is(base))),
        ("已经在出流", dict(streaming=True), dict(T=15), silent),
        ("开关关闭", {}, dict(T=10, tcp_en=False), silent),
        ("雷达晚启动 8 s", dict(t_boot=8.0), dict(T=25), ok_once(9.0, 9.1, starts=2, fails=1)),
        ("丢第一个 SYN", dict(drop_syn=1), dict(T=15), both(ok_once(3.0, 3.1), seq_is(["ARP", "S", "S", "D", "A", "F", "A"]))),
        ("SYN-ACK 校验和错", dict(bad_synack=1), dict(T=15), both(ok_once(3.0, 3.1), seq_is(["ARP", "S", "S", "D", "A", "F", "A"]))),
        ("丢命令数据段", dict(drop_data=1), dict(T=15), ok_once(2.6, 2.7)),
        ("SYN 两次被 RST", dict(rst_syn=2), dict(T=20), ok_once(6.0, 6.1, starts=3, fails=2)),
        ("应答前有上报", dict(resp="log_first"), dict(T=15), ok_once(2.0, 2.1)),
        ("上报 + 应答拆段", dict(resp="split"), dict(T=15), ok_once(2.0, 2.1)),
        ("应答码失败 10 s 后恢复", dict(rc=1), dict(T=25, events=[(10.0, lambda l, t: setattr(l, "rc", 0))]), rc_recover),
        ("出流中断（雷达重启 6 s）", {}, dict(T=30, events=[(10.0, lambda l, t: l.reboot(t, 6.0))]), reboot),
        # 实测（2026-09-30）：雷达刚上电时应答 OK 后约 3 s 才出第一帧；不该因此再启动一次
        ("应答后 3.5 s 才出流", dict(warmup=3.5), dict(T=20), both(ok_once(2.0, 2.1), seq_is(base))),
        ("RST 关闭", {}, dict(T=15, close_rst=True),
         both(ok_once(2.0, 2.1, closed=0), seq_is(["ARP", "S", "D", "A", "R"]))),
        ("雷达先关连接", dict(fin_after=True), dict(T=15), ok_once(2.0, 2.1)),
        ("不应答但已出流", dict(resp="none"), dict(T=15), no_resp),
        ("雷达不在", dict(t_boot=1e9), dict(T=30), absent),
        # 长时间没雷达后才接上：时间差超过 int32（约 36 min）、跨 32 位微秒回绕（约 71.6 min）后仍要能启动
        ("40 分钟后才接雷达", dict(t_boot=2400.0), dict(T=2420, dt_us=10000), ok_once(2400.0, 2408.0, starts=None, fails=None)),
        ("75 分钟后才接雷达", dict(t_boot=4500.0), dict(T=4520, dt_us=10000), ok_once(4500.0, 4508.0, starts=None, fails=None)),
        ("走 nav_app_poll", {}, dict(T=12, app=True), both(ok_once(2.0, 2.1), seq_is(base))),
    ]


def main():
    if "--no-build" not in sys.argv:
        run_cl.build_dll()
    d = ctypes.CDLL(str(run_cl.DLL))
    d.py_set_netl.argtypes = [ctypes.c_char_p, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_int]
    d.py_set_lidar_frames.argtypes = [ctypes.c_uint32]
    d.py_set_time.argtypes = [ctypes.c_uint32]
    fails = 0
    for name, lk, rk, chk in scenarios():
        lid = FakeLidar(**lk)
        r = run(d, lid, **rk)
        bad = lid.err + chk(r, lid)
        fails += bool(bad)
        s = r["stat"]
        print(f"{name:22s} 启动 {s['n_start']} 成功 {s['n_ok']} 失败 {s['n_fail']}  发 {len(r['tx']):3d} 帧  "
              f"首次成功 {r['t_ok'][0] if r['t_ok'] else '-':>6}  {'OK' if not bad else '不通过'}")
        for b in bad[:6]:
            print(f"    {b}")
        if bad:
            print(f"    状态变化 {r['trans'][:16]}")
    print("XT_LINK PASSED" if fails == 0 else f"XT_LINK FAILED（{fails} 项）")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
