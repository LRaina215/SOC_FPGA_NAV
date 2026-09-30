"""XT-M60 雷达的最小 TCP 配置工具（日常配置用厂家上位机即可；这里只做脚本化 / 上位机没有的项）。

命令格式依据厂家 SDK（xtdaemon.cpp transceiveCmd、xtsdk.cpp、cmdstructs.h）与手册 7.5/7.6 节：
  请求 = 7E FF AA 55 | len u32 小端 (= 1 + 数据 + 2) | 命令号 | 数据 | 00 01 | FF 7E 55 AA
  应答 = 7E FF AA 55 | len u32 | 命令号 | 数据 | 返回码 | 设备状态 | 03 | FF 7E 55 AA
雷达缺省 IP 192.168.0.101，TCP 端口 7787；电脑网卡需在 192.168.0.x（不能用 101）。
雷达同一时间只理一个 TCP 连接：厂家上位机连着时本工具收不到应答，先关掉上位机（或在上位机里断开）。

用法:
  python tools/xt_config.py [--ip 192.168.0.101] info          设备信息（SN、固件、IP、MAC、当前 UDP 目标）
  python tools/xt_config.py config                             当前配置（输出类型、HDR、帧率、上电自动运行等）
  python tools/xt_config.py lens                               镜头内参 fx fy cx cy k1 k2 k3 p1 p2
  python tools/xt_config.py udp-dest 192.168.0.100 [7687]      设置 UDP 目标（命令 19）。固件 V2.34.2 实测：返回 OK 但不生效，
                                                             雷达总是发给最近连它 TCP 的主机
  python tools/xt_config.py start [dist|amp|gray] [--once]     开始测量（缺省只出深度、连续）；数据发往本机，断开 TCP 后继续发
  python tools/xt_config.py stop
  python tools/xt_config.py maxfps 15 | minamp 80 | hdr 0|1 | reset | test
  python tools/xt_config.py raw 27 01020304                    发任意命令（十六进制数据），打印应答
"""
import argparse
import socket
import struct
import sys
import time

START = bytes([0x7E, 0xFF, 0xAA, 0x55])
END = bytes([0xFF, 0x7E, 0x55, 0xAA])
CMD = dict(test=0, start=1, stop=2, set_ip=3, info=4, config=5, int_time=8, minamp=9, hdr=10, reset=13,
           lens=18, udp_dest=19, roi=51, modfreq=52, maxfps=56)
RESP = {0: "OK", 1: "命令不支持", 2: "设备忙", 3: "设备拒绝", 7: "上报", 8: "格式错误", 9: "数据错误",
        10: "CSI 错误", 11: "I2C 错误", 12: "温度高", 13: "温度低", 14: "未知错误", 15: "超时"}
DEVSTATE = {0: "断开", 1: "初始化中", 2: "空闲", 3: "出图中", 4: "CSI 错误", 5: "I2C 错误", 6: "温度太低",
            7: "温度高警告", 8: "温度太高降帧", 9: "温度太高停测", 0x14: "未标定完成", 0x15: "无相机内参"}


class XtLink:
    def __init__(self, ip, port=7787, timeout=2.0):
        self.s = socket.create_connection((ip, port), timeout=timeout)
        self.s.settimeout(timeout)
        self.buf = b""

    def _read_msg(self, deadline):
        while True:
            i = self.buf.find(START)
            if i >= 0 and len(self.buf) >= i + 8:
                n = struct.unpack_from("<I", self.buf, i + 4)[0]
                if len(self.buf) >= i + 12 + n:
                    body = self.buf[i + 8:i + 8 + n]
                    ok_end = self.buf[i + 8 + n:i + 12 + n] == END
                    self.buf = self.buf[i + 12 + n:]
                    if ok_end:
                        return body
                    continue
            if time.time() > deadline:
                return None
            try:
                chunk = self.s.recv(4096)
            except socket.timeout:
                return None
            if not chunk:
                return None
            self.buf += chunk

    def cmd(self, cid, data=b"", timeout=2.0):
        """发命令，等同命令号的应答：返回 (返回码, 设备状态, 数据)；超时返回 None。"""
        pkt = START + struct.pack("<I", 1 + len(data) + 2) + bytes([cid]) + bytes(data) + b"\x00\x01" + END
        self.s.sendall(pkt)
        deadline = time.time() + timeout
        while True:
            body = self._read_msg(deadline)
            if body is None:
                return None
            if body[0] == cid and len(body) >= 4:
                return body[-3], body[-2], body[1:-3]      # 其他命令号（日志上报等）跳过


def cstr(b):
    return b.split(b"\x00", 1)[0].decode("ascii", "replace")


def show_info(d):
    if len(d) < 120 or struct.unpack_from("<I", d, 0)[0] != 0x33CCAA51:
        print(f"设备信息 {len(d)} 字节（不是 V3 结构）：{d.hex()}")
        return
    sn, fw, boot = cstr(d[7:35]), cstr(d[35:53]), cstr(d[53:71])
    ip = d[71:83]
    mac = d[83:89]
    udp_ip = d[93:97]
    udp_port = struct.unpack_from("<H", d, 97)[0]
    print(f"SN {sn}  固件 {fw}  Boot {boot}")
    print(f"IP {'.'.join(map(str, ip[0:4]))}  掩码 {'.'.join(map(str, ip[4:8]))}  网关 {'.'.join(map(str, ip[8:12]))}")
    print(f"MAC {':'.join(f'{b:02X}' for b in mac)}")
    print(f"UDP 目标 {'.'.join(map(str, udp_ip))}:{udp_port}  标定状态 {d[102]}  上电次数 {struct.unpack_from('<I', d, 103)[0]}")


def show_config(d):
    if len(d) < 60 or struct.unpack_from("<I", d, 0)[0] != 0x33CCAA52:
        print(f"配置 {len(d)} 字节（不是 V3 结构）：{d.hex()}")
        return
    flags, hdr = d[8], d[9]
    freq = list(d[10:15])
    it = struct.unpack_from("<5H", d, 15)
    itgs, = struct.unpack_from("<H", d, 25)
    roix = struct.unpack_from("<2H", d, 27)
    roiy = struct.unpack_from("<2H", d, 31)
    minamp, = struct.unpack_from("<H", d, 35)
    channel, binning, reduce_, setfps, usefps, vcsel, tstype, ptpdom, autorun, dhcp, cutf = d[37:48]
    kinds = {1: "深度", 3: "深度+幅值", 8: "灰度"}
    print(f"输出 {kinds.get(flags, hex(flags))}（imageflags 0x{flags:02X}）  HDR {hdr}  频率档 {freq}  积分时间 {it} 灰度 {itgs} us")
    print(f"ROI x {roix} y {roiy}  最小幅值 {minamp}  合并 0x{binning:02X}  帧率 设定 {setfps} / 实际 {usefps}")
    print(f"上电自动运行 {autorun}  DHCP {dhcp}  时间同步 {tstype}  字节序 {'大端' if d[7] else '小端'}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="192.168.0.101")
    ap.add_argument("--port", type=int, default=7787)
    ap.add_argument("--once", action="store_true")
    ap.add_argument("cmd", nargs="?")
    ap.add_argument("args", nargs="*", default=[])
    a = ap.parse_args()
    if not a.cmd:
        print(__doc__)
        return 1
    try:
        link = XtLink(a.ip, a.port)
    except OSError as e:
        print(f"连不上 {a.ip}:{a.port}（{e}）。检查网线、电脑网卡是否在 192.168.0.x、雷达是否上电。")
        return 1

    def run(cid, data=b"", timeout=2.0):
        r = link.cmd(cid, data, timeout)
        if r is None:
            print(f"命令 {cid} 无应答：厂家上位机或别的程序还连着雷达时，雷达不理第二个 TCP 连接，先关掉上位机再试")
            sys.exit(1)
        code, st, body = r
        print(f"命令 {cid}: {RESP.get(code, code)}，设备状态 {DEVSTATE.get(st, st)}")
        return body

    c = a.cmd.replace("_", "-")
    if c == "test":
        run(CMD["test"])
    elif c == "info":
        show_info(run(CMD["info"], timeout=3.0))
    elif c == "config":
        show_config(run(CMD["config"]))
    elif c == "lens":
        d = run(CMD["lens"])
        if len(d) == 36:
            v = struct.unpack("<9f", d)
            print("  ".join(f"{k}={x:.6f}" for k, x in zip(("fx", "fy", "cx", "cy", "k1", "k2", "k3", "p1", "p2"), v)))
            print("生成方向表: python model/lidar_lut.py --out lut.hex " +
                  " ".join(f"--{k} {x:.6f}" for k, x in zip(("fx", "fy", "cx", "cy", "k1", "k2", "k3", "p1", "p2"), v)))
        else:
            print(f"内参 {len(d)} 字节：{d.hex()}")
    elif c == "udp-dest":
        ip = a.args[0] if a.args else "192.168.0.100"
        port = int(a.args[1]) if len(a.args) > 1 else 7687
        run(CMD["udp_dest"], bytes(int(x) for x in ip.split(".")) + struct.pack("<H", port))
        print(f"已发送 UDP 目标 {ip}:{port}。固件 V2.34.2 实测不生效（雷达发给最近连它 TCP 的主机），用 info 回读确认")
    elif c == "start":
        kind = {"dist": 1, "amp": 2, "gray": 3}[a.args[0] if a.args else "dist"]
        run(CMD["start"], bytes([kind, 0 if a.once else 1]))
    elif c == "stop":
        run(CMD["stop"])
    elif c == "maxfps":
        run(CMD["maxfps"], bytes([int(a.args[0])]))
    elif c == "minamp":
        run(CMD["minamp"], struct.pack("<H", int(a.args[0])))
    elif c == "hdr":
        run(CMD["hdr"], bytes([int(a.args[0])]))
    elif c == "reset":
        link.s.sendall(START + struct.pack("<I", 1 + 6 + 2) + bytes([CMD["reset"]]) + b"XINTAN" + b"\x00\x01" + END)
        print("已发送重启命令（无应答）")
    elif c == "raw":
        body = run(int(a.args[0], 0), bytes.fromhex(a.args[1]) if len(a.args) > 1 else b"")
        print(body.hex())
    else:
        print(__doc__)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
