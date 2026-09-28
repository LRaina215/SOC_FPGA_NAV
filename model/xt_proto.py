"""XT-M60 (Pro) 网络数据格式（依据厂家手册 V2.0 第 7 章）。

一帧:  START(7E FF AA 55) | size(4B) | data | END(FF 7E 55 AA)
data:  cmdid(1B, 图像帧=251) | type(1B: 0 深度+幅值, 1 深度, 2 灰度) | SN(2B) | W(2B) | H(2B)
       | 像素数据 | FrameInfo（倒数第 3~4 字节=结构体大小，倒数第 2 字节=DevState，最后 1 字节=协议版本）
一帧拆成多个 UDP 包，每包 = 20 字节分包头 + ≤1400 字节数据。

手册未写明、需实机抓包确认的项目在此全部做成参数（与 RTL 寄存器一一对应）:
  hdr_le     头部多字节字段是否小端（手册称网络序大端）
  pix_le     像素字是否小端
  layout     type=0 时像素排布: 'interleaved'（逐像素 深度,幅值）或 'planar'（整幅深度后整幅幅值）
  amp_first  interleaved 时幅值是否在前
  size_adj   size 字段与 data 实际长度的差: data_len = size + size_adj
  udp_skip   每个 UDP 负载开头的分包头长度
"""
import struct

START = bytes([0x7E, 0xFF, 0xAA, 0x55])
END = bytes([0xFF, 0x7E, 0x55, 0xAA])
CMD_FRAME = 251


def _u16(v, le):
    return struct.pack("<H" if le else ">H", v & 0xFFFF)


def _u32(v, le):
    return struct.pack("<I" if le else ">I", v & 0xFFFFFFFF)


def build_frame(depth, amp=None, sn=0, img_type=1, layout="interleaved", amp_first=False,
                hdr_le=False, pix_le=False, info_body=b"\x00" * 36, devstate=3, proto=2,
                size_adj=0, cmdid=CMD_FRAME):
    """depth/amp: H 行 × W 列的整数二维列表（或 numpy 数组）。返回完整帧字节。"""
    h = len(depth)
    w = len(depth[0])
    px = bytearray()
    if img_type == 1:
        for r in range(h):
            for c in range(w):
                px += _u16(int(depth[r][c]), pix_le)
    elif img_type == 0:
        if layout == "interleaved":
            for r in range(h):
                for c in range(w):
                    d, a = _u16(int(depth[r][c]), pix_le), _u16(int(amp[r][c]), pix_le)
                    px += (a + d) if amp_first else (d + a)
        else:
            for r in range(h):
                for c in range(w):
                    px += _u16(int(depth[r][c]), pix_le)
            for r in range(h):
                for c in range(w):
                    px += _u16(int(amp[r][c]), pix_le)
    else:
        for r in range(h):
            for c in range(w):
                px += _u16(int(depth[r][c]), pix_le)
    info = info_body + _u16(len(info_body) + 4, hdr_le) + bytes([devstate & 0xFF, proto & 0xFF])
    data = bytes([cmdid, img_type]) + _u16(sn, hdr_le) + _u16(w, hdr_le) + _u16(h, hdr_le) + bytes(px) + info
    size = len(data) - size_adj
    return START + _u32(size, hdr_le) + data + END


def packetize(frame, chunk=1400, udp_skip=20, sn=0):
    """把一帧拆成 UDP 负载列表。分包头内容未知，这里填可辨认的占位字节（RTL 按 udp_skip 跳过）。"""
    out = []
    n = (len(frame) + chunk - 1) // chunk
    for i in range(n):
        body = frame[i * chunk:(i + 1) * chunk]
        hdr = (b"XTPK" + struct.pack(">HHHHI", sn & 0xFFFF, i, n, len(body), i * chunk))[:udp_skip]
        hdr = hdr + b"\x00" * (udp_skip - len(hdr))
        out.append(hdr + body)
    return out
