"""以太网 / IPv4 / UDP / ARP 报文构造与解析（测试激励与主机侧参考实现共用）。"""
import struct
import zlib

BCAST = b"\xff" * 6


def mac_bytes(m):
    if isinstance(m, int):
        return m.to_bytes(6, "big")
    return bytes(m)


def ip_bytes(ip):
    if isinstance(ip, int):
        return ip.to_bytes(4, "big")
    if isinstance(ip, str):
        return bytes(int(x) for x in ip.split("."))
    return bytes(ip)


def csum16(data):
    if len(data) % 2:
        data += b"\x00"
    s = sum(struct.unpack(f">{len(data)//2}H", data))
    while s >> 16:
        s = (s & 0xFFFF) + (s >> 16)
    return (~s) & 0xFFFF


def eth(dst, src, ethertype, payload, pad=True, fcs=False):
    f = mac_bytes(dst) + mac_bytes(src) + struct.pack(">H", ethertype) + payload
    if pad and len(f) < 60:
        f += b"\x00" * (60 - len(f))
    if fcs:
        f += struct.pack("<I", zlib.crc32(f) & 0xFFFFFFFF)
    return f


def ipv4_udp(src_ip, dst_ip, sport, dport, payload, ident=0, ttl=64, udp_csum=False):
    ulen = 8 + len(payload)
    udp = struct.pack(">HHHH", sport, dport, ulen, 0) + payload
    if udp_csum:
        ph = ip_bytes(src_ip) + ip_bytes(dst_ip) + struct.pack(">BBH", 0, 17, ulen)
        c = csum16(ph + udp) or 0xFFFF
        udp = udp[:6] + struct.pack(">H", c) + udp[8:]
    tot = 20 + ulen
    hdr = struct.pack(">BBHHHBBH4s4s", 0x45, 0, tot, ident & 0xFFFF, 0, ttl, 17, 0,
                      ip_bytes(src_ip), ip_bytes(dst_ip))
    hdr = hdr[:10] + struct.pack(">H", csum16(hdr)) + hdr[12:]
    return hdr + udp


def udp_frame(dst_mac, src_mac, src_ip, dst_ip, sport, dport, payload, ident=0, fcs=False):
    return eth(dst_mac, src_mac, 0x0800, ipv4_udp(src_ip, dst_ip, sport, dport, payload, ident), fcs=fcs)


def arp(oper, sha, spa, tha, tpa):
    return struct.pack(">HHBBH6s4s6s4s", 1, 0x0800, 6, 4, oper, mac_bytes(sha), ip_bytes(spa),
                       mac_bytes(tha), ip_bytes(tpa))


def arp_request_frame(src_mac, src_ip, target_ip):
    return eth(BCAST, src_mac, 0x0806, arp(1, src_mac, src_ip, b"\x00" * 6, target_ip))


def parse(frame):
    """解析一帧，返回 dict（type=arp/udp/other）。用于检查 RTL 发出的帧。"""
    d = {"dst": frame[0:6], "src": frame[6:12], "etype": struct.unpack(">H", frame[12:14])[0]}
    if d["etype"] == 0x0806:
        htype, ptype, hl, pl, oper, sha, spa, tha, tpa = struct.unpack(">HHBBH6s4s6s4s", frame[14:42])
        d.update(type="arp", oper=oper, sha=sha, spa=spa, tha=tha, tpa=tpa)
    elif d["etype"] == 0x0800:
        ihl = (frame[14] & 0xF) * 4
        ip = frame[14:14 + ihl]
        tot = struct.unpack(">H", ip[2:4])[0]
        d.update(type="udp" if ip[9] == 17 else "ip", ip_ok=csum16(ip) == 0, src_ip=ip[12:16], dst_ip=ip[16:20])
        u = frame[14 + ihl:14 + tot]
        sp, dp, ul, _ = struct.unpack(">HHHH", u[:8])
        d.update(sport=sp, dport=dp, payload=u[8:ul])
    else:
        d["type"] = "other"
    return d
