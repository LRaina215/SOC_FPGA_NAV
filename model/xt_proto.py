"""XT-M60 网络数据格式与参考收包模型。

格式依据厂家 SDK 源码（github.com/XT-Toffuture/xtsdk_cpp：communicationNet.cpp udppacket、
xtdaemon.cpp ParseFrame、frame.cpp sortData、cmdstructs.h FrameInfo_t），并用上位机 V2.11.6 的调试符号核对过
FrameInfo_t 的大小与字段偏移。所有多字节字段小端（手册 202510 版"统一小端"）；首尾标记按字节顺序发送。

一帧:   START 7E FF AA 55 | len u32 | 负载(len 字节) | END FF 7E 55 AA
负载:   cmdid(251) | type(0 深度+幅值, 1 深度, 3 灰度) | SN u16 | W u16 | H u16
        | 像素（深度 u16；深度+幅值时逐像素交织、深度在前）| 附加平面/IMU 块（可有可无）
        | FrameInfo_t 206 字节（V3；末 4 字节 = infosize u16、devstate、协议版本 3）
UDP:    一帧拆成若干分包，每个分包 = 20 字节分包头 + 不超过 1400 字节帧数据；
        分包头 = 帧序号 u16 | 整帧字节数 u32 | 本包数据字节数 u16 | 本包数据在帧内的偏移 u32 | 8 字节未用。
        一个 UDP 报文可以连放多个分包；IMU 等非图像消息（cmdid 252）也是分包，整帧字节数很小。
无效值: 深度/幅值 >= 64000 为设备给出的无效码（幅值过低、饱和、过曝、坏点、被滤除）。
"""
import struct

START = bytes([0x7E, 0xFF, 0xAA, 0x55])
END = bytes([0xFF, 0x7E, 0x55, 0xAA])
CMD_FRAME = 251
CMD_IMU = 252
TYPE_DIST_AMP, TYPE_DIST, TYPE_GRAY = 0, 1, 3
SUBHDR = 20
CHUNK = 1400
INFO_SIZE = 206
INVALID_MIN = 64000
MIN_IMAGE_TOTAL = 1000          # 整帧字节数小于此值的分包不是图像（SDK 同样把 <1000 的帧丢给其他处理）
TCP_PORT, UDP_PORT = 7787, 7687
DEFAULT_LIDAR_IP = "192.168.0.101"
MCAST_GROUP = "239.255.255.76"

# FrameInfo_t 字段偏移（cmdstructs.h，#pragma pack(1)）
FI = dict(magic=0, endian=4, sn=5, fw=34, lens=53, imageflags=93, hdrmode=94, levelused=95, freq=96,
          integtime=101, integtimegs=111, roix=113, roiy=117, miniamp=121, channel=123, binning=124,
          reduce=125, vcsel=126, fps=127, unit_div=136, width=137, height=139, temperature=141,
          timesync_type=145, timesync_state=146, timestamp=147, ptpoffset=187, imusize=191,
          otherflags=192, version=197, crc32=198, infosize=202, devstate=204, ptclversion=205)
MAGIC_INFO = 0x33CCAA50
LENS_KEYS = ("fx", "fy", "cx", "cy", "k1", "k2", "k3", "p1", "p2")
M60_DEFAULT_LENS = (78.212524, 79.055527, 80.701973, 26.804274, -0.302470, 0.074865, 0.0, -0.002307, -0.001400)


def _u16(v, le=True):
    return struct.pack("<H" if le else ">H", v & 0xFFFF)


def _u32(v, le=True):
    return struct.pack("<I" if le else ">I", v & 0xFFFFFFFF)


def frame_info(w, h, lens=M60_DEFAULT_LENS, sn_str="XT-M60PRO-SIM0001", unit_div=1, imageflags=1,
               devstate=3, temps=(3500, 3600), hdrmode=0, roi=(0, 0), binning=0, ts_ms=0, fps=15):
    """构造 206 字节 FrameInfo_t（V3）。SN 串里含 "M60" 且不在开头（SDK 按 find("M60") > 0 识别型号）。"""
    b = bytearray(INFO_SIZE)
    struct.pack_into("<I", b, FI["magic"], MAGIC_INFO)
    b[FI["endian"]] = 0
    s = sn_str.encode()[:29]
    b[FI["sn"]:FI["sn"] + len(s)] = s
    struct.pack_into("<9f", b, FI["lens"], *lens)
    b[FI["imageflags"]] = imageflags
    b[FI["hdrmode"]] = hdrmode
    struct.pack_into("<HH", b, FI["roix"], roi[0], roi[0] + w - 1)
    struct.pack_into("<HH", b, FI["roiy"], roi[1], roi[1] + h - 1)
    b[FI["binning"]] = binning
    b[FI["fps"]] = fps
    b[FI["unit_div"]] = unit_div
    struct.pack_into("<HH", b, FI["width"], w, h)
    struct.pack_into("<hh", b, FI["temperature"], *temps)
    struct.pack_into("<Q", b, FI["timestamp"], ts_ms)
    struct.pack_into("<H", b, FI["infosize"], INFO_SIZE)
    b[FI["devstate"]] = devstate
    b[FI["ptclversion"]] = 3
    return bytes(b)


def parse_info(b):
    """206 字节 FrameInfo_t -> 字典（只取导航用到的字段）。"""
    lens = struct.unpack_from("<9f", b, FI["lens"])
    return dict(magic=struct.unpack_from("<I", b, 0)[0], lens=dict(zip(LENS_KEYS, lens)),
                lens_raw=struct.unpack_from("<9I", b, FI["lens"]), imageflags=b[FI["imageflags"]],
                hdrmode=b[FI["hdrmode"]], roix0=struct.unpack_from("<H", b, FI["roix"])[0],
                roiy0=struct.unpack_from("<H", b, FI["roiy"])[0], binning=b[FI["binning"]],
                unit_div=b[FI["unit_div"]], width=struct.unpack_from("<H", b, FI["width"])[0],
                height=struct.unpack_from("<H", b, FI["height"])[0],
                temps=struct.unpack_from("<hh", b, FI["temperature"]),
                infosize=struct.unpack_from("<H", b, FI["infosize"])[0], devstate=b[FI["devstate"]],
                ptclversion=b[FI["ptclversion"]])


def build_frame(depth, amp=None, sn=0, img_type=None, info=None, extra=b"", proto=3, cmdid=CMD_FRAME,
                hdr_le=True, pix_le=True):
    """depth/amp: H 行 × W 列整数（列表或数组）。返回完整帧字节（START..END）。
    proto=3：帧尾为 FrameInfo_t（info 缺省按 W/H 生成）；proto=2：旧版 52 字节帧尾（仅末两字节有意义）。"""
    h, w = len(depth), len(depth[0])
    if img_type is None:
        img_type = TYPE_DIST if amp is None else TYPE_DIST_AMP
    px = bytearray()
    for r in range(h):
        for c in range(w):
            px += _u16(int(depth[r][c]), pix_le)
            if img_type == TYPE_DIST_AMP:
                px += _u16(int(amp[r][c]), pix_le)
    if proto == 3:
        trailer = info if info is not None else frame_info(w, h, imageflags=3 if img_type == TYPE_DIST_AMP else 1)
    else:
        trailer = bytes(50) + bytes([3, proto & 0xFF])
    payload = bytes([cmdid, img_type]) + _u16(sn, hdr_le) + _u16(w, hdr_le) + _u16(h, hdr_le) + bytes(px) + extra + trailer
    return START + _u32(len(payload), hdr_le) + payload + END


def subpacket(sn, total, offset, data, hdr_le=True):
    return _u16(sn, hdr_le) + _u32(total, hdr_le) + _u16(len(data), hdr_le) + _u32(offset, hdr_le) + bytes(8) + data


def packetize(frame, sn=0, chunk=CHUNK, hdr_le=True):
    """一帧 -> 分包列表（每个分包 = 20 字节分包头 + 帧数据片段）。"""
    return [subpacket(sn, len(frame), o, frame[o:o + chunk], hdr_le) for o in range(0, len(frame), chunk)]


def imu_subpacket(sn, n_words=19, hdr_le=True):
    """IMU 消息（cmdid 252）：自成一个完整小帧，放在一个分包里。"""
    body = bytes([CMD_IMU]) + b"".join(_u32(0x55AA0000 + i) for i in range(n_words)) + bytes([0, 3, 3])
    msg = START + _u32(len(body), hdr_le) + body + END
    return subpacket(sn, len(msg), 0, msg, hdr_le)


def datagrams(subpkts, per=1, mtu_payload=1472):
    """把分包依次装进 UDP 报文：每个报文最多 per 个分包，且不超过 mtu_payload 字节。"""
    out, cur, n = [], b"", 0
    for p in subpkts:
        if cur and (n >= per or len(cur) + len(p) > mtu_payload):
            out.append(cur)
            cur, n = b"", 0
        cur += p
        n += 1
    if cur:
        out.append(cur)
    return out


def split_subpackets(dgram, hdr_le=True):
    """UDP 负载 -> [(sn, total, offset, data)]（按 SDK udppacket 的方式逐个解析）。"""
    f16 = "<H" if hdr_le else ">H"
    f32 = "<I" if hdr_le else ">I"
    out, pos = [], 0
    while pos + SUBHDR <= len(dgram):
        sn = struct.unpack_from(f16, dgram, pos)[0]
        total = struct.unpack_from(f32, dgram, pos + 2)[0]
        n = struct.unpack_from(f16, dgram, pos + 6)[0]
        off = struct.unpack_from(f32, dgram, pos + 8)[0]
        if n > len(dgram) - pos - SUBHDR:
            break
        out.append((sn, total, off, dgram[pos + SUBHDR:pos + SUBHDR + n]))
        pos += SUBHDR + n
    return out


def sdk_reassemble(dgrams, hdr_le=True):
    """仿 SDK 的宽松重组（按偏移拼、允许乱序，最多 3 帧同时在拼）：返回完整帧的负载列表（去掉首尾 12 字节）。
    只用来检查本文件生成的包能被厂家的逻辑正确拼出，不代表 RTL 行为（RTL 见 RxModel）。"""
    bufs, done = {}, []
    for d in dgrams:
        for sn, total, off, data in split_subpackets(d, hdr_le):
            if total < MIN_IMAGE_TOTAL:
                continue
            b = bufs.setdefault(sn, [bytearray(total), 0])
            b[0][off:off + len(data)] = data
            b[1] += len(data)
            if b[1] >= total:
                del bufs[sn]
                fr = bytes(b[0])
                if b[1] == total and fr[:4] == START and fr[-4:] == END:
                    done.append(fr[8:-4])
            while len(bufs) > 3:
                bufs.pop(min(bufs))
    return done


def parse_frame(payload):
    """帧负载 -> 字典：type/sn/w/h/depth/amp（二维列表，原始值）/info（V3 时）。"""
    proto = payload[-1]
    le = proto == 3
    f16 = "<H" if le else ">H"
    cmdid, itype = payload[0], payload[1]
    sn, w, h = struct.unpack_from(f16, payload, 2)[0], struct.unpack_from(f16, payload, 4)[0], struct.unpack_from(f16, payload, 6)[0]
    step = 4 if itype == TYPE_DIST_AMP else 2
    depth = [[struct.unpack_from("<H", payload, 8 + (r * w + c) * step)[0] for c in range(w)] for r in range(h)]
    amp = None
    if itype == TYPE_DIST_AMP:
        amp = [[struct.unpack_from("<H", payload, 10 + (r * w + c) * 4)[0] for c in range(w)] for r in range(h)]
    info = None
    if le and struct.unpack_from("<H", payload, len(payload) - 4)[0] == INFO_SIZE:
        info = parse_info(payload[-INFO_SIZE:])
    return dict(cmdid=cmdid, type=itype, sn=sn, w=w, h=h, depth=depth, amp=amp, proto=proto, info=info)


class RxModel:
    """RTL xt_frame_rx 的行为模型（判定点与 RTL 逐一对应）。feed(UDP 负载, err) 逐报文输入。

    分包规则：整帧字节数 < 1000 或数据长度 0 的分包跳过（IMU 等）；偏移 0 开始新帧（丢弃未完成的旧帧）；
    同帧（帧序号、整帧字节数都相同）且偏移连续则接上；同帧偏移不连续、越过整帧长度或报文提前结束则整帧丢弃；
    其他零散分包忽略。err（报文末字节的错误标志）只影响从该报文收到过字节的当前帧。
    帧内判定点（帧内字节偏移）：3 起始标记；7 len+12 == 整帧字节数；9 cmdid/类型（非 251 或灰度静默忽略）；
    15 宽高；16 len 够装像素；最后一字节 结束标记、本帧无错误报文。
    frames: 输出帧 [dict(sn, w, h, depth(无效=0), devstate, proto, info)]；bad: 失败帧数；skip: 跳过的分包数。
    """

    def __init__(self, hdr_le=True, pix_le=True, subhdr=SUBHDR, amp_min=0, raw_inv=INVALID_MIN,
                 max_w=160, max_h=60):
        self.hdr_le, self.pix_le, self.subhdr = hdr_le, pix_le, subhdr
        self.amp_min, self.raw_inv, self.max_w, self.max_h = amp_min, raw_inv, max_w, max_h
        self.frames, self.bad, self.skip = [], 0, 0
        self.active = False
        self.buf = bytearray()
        self.cur = None           # (sn, total)
        self.bad_pkt = False

    def _fail(self):
        if self.active:
            self.bad += 1
        self.active = False

    def _dvalid(self, v):
        return v != 0 and (self.raw_inv == 0 or v < self.raw_inv)

    def _check(self, a, b, last_err):
        """帧内偏移 [a, b) 刚到齐：执行落在其中的判定点。返回 False 表示本帧已结束（输出、失败或忽略）。"""
        fr = self.buf
        f16 = "<H" if self.hdr_le else ">H"
        f32 = "<I" if self.hdr_le else ">I"
        total = self.cur[1]
        if a <= 3 < b and bytes(fr[:4]) not in (START, START[::-1]):
            self._fail()
            return False
        if a <= 7 < b and struct.unpack_from(f32, fr, 4)[0] + 12 != total:
            self._fail()
            return False
        if a <= 9 < b:
            if fr[8] != CMD_FRAME or fr[9] == TYPE_GRAY:
                self.active = False                    # 非图像 / 灰度：静默忽略
                return False
            if fr[9] not in (TYPE_DIST, TYPE_DIST_AMP):
                self._fail()
                return False
        if a <= 15 < b:
            w, h = struct.unpack_from(f16, fr, 12)[0], struct.unpack_from(f16, fr, 14)[0]
            if not (1 <= w <= self.max_w) or not (1 <= h <= self.max_h):
                self._fail()
                return False
        if a <= 16 < b:
            w, h = struct.unpack_from(f16, fr, 12)[0], struct.unpack_from(f16, fr, 14)[0]
            pix = w * h * (2 if fr[9] == TYPE_DIST else 4)
            if total - 12 < pix + 12:
                self._fail()
                return False
        if a <= total - 1 < b:
            if bytes(fr[total - 4:total]) not in (END, END[::-1]) or self.bad_pkt or last_err:
                self._fail()
                return False
            self._emit()
            return False
        return True

    def _emit(self):
        fr = bytes(self.buf)
        self.active = False
        f16 = "<H" if self.hdr_le else ">H"
        pf = "<H" if self.pix_le else ">H"
        itype = fr[9]
        w, h = struct.unpack_from(f16, fr, 12)[0], struct.unpack_from(f16, fr, 14)[0]
        step = 2 if itype == TYPE_DIST else 4
        depth = []
        for r in range(h):
            row = []
            for c in range(w):
                o = 16 + (r * w + c) * step
                d = struct.unpack_from(pf, fr, o)[0]
                ok = self._dvalid(d)
                if itype == TYPE_DIST_AMP:
                    a = struct.unpack_from(pf, fr, o + 2)[0]
                    ok = ok and a >= self.amp_min and (self.raw_inv == 0 or a < self.raw_inv)
                row.append(d if ok else 0)
            depth.append(row)
        pay = fr[8:-4]
        info = None
        if pay[-1] == 3 and len(pay) >= w * h * step + 8 + INFO_SIZE and \
                struct.unpack_from("<H", pay, len(pay) - 4)[0] == INFO_SIZE:
            info = parse_info(pay[-INFO_SIZE:])
        self.frames.append(dict(sn=struct.unpack_from(f16, fr, 10)[0], w=w, h=h, depth=depth,
                                devstate=pay[-2], proto=pay[-1], info=info))

    def feed(self, dgram, err=False):
        f16 = "<H" if self.hdr_le else ">H"
        f32 = "<I" if self.hdr_le else ">I"
        pos = 0
        fed = False                                    # 本报文给当前帧送过字节
        while pos + self.subhdr <= len(dgram):
            sn = struct.unpack_from(f16, dgram, pos)[0]
            total = struct.unpack_from(f32, dgram, pos + 2)[0]
            n = struct.unpack_from(f16, dgram, pos + 6)[0]
            off = struct.unpack_from(f32, dgram, pos + 8)[0]
            data = dgram[pos + self.subhdr:pos + self.subhdr + n]
            pos += self.subhdr + n
            if total < MIN_IMAGE_TOTAL or total >= 1 << 20 or n == 0:
                self.skip += 1                         # IMU 等非图像消息
                continue
            if off == 0:                               # 新帧开始：丢弃未完成的旧帧
                self._fail()
                if n > total:                          # 首包就越界：当作零散分包
                    self.skip += 1
                    continue
                self.active, self.buf, self.cur, self.bad_pkt, fed = True, bytearray(), (sn, total), False, False
            elif not (self.active and (sn, total) == self.cur):
                self.skip += 1                         # 不属于当前帧的零散分包：忽略，不影响当前帧
                continue
            elif off != len(self.buf) or off + n > total:
                self._fail()                           # 当前帧内丢包 / 乱序 / 重复 / 越界
                continue
            a = len(self.buf)
            self.buf += data
            fed = fed or len(data) > 0
            last_err = err and pos >= len(dgram) and len(data) == n
            if not self._check(a, len(self.buf), last_err):
                continue
            if 0 < len(data) < n:                      # 报文在分包数据中途结束
                self._fail()
        if err and fed and self.active:
            self.bad_pkt = True
