"""伪 XT-M60：在 PC 上按真实协议（手册第 7 章帧格式、20 字节分包头 + 1400 字节）向 FPGA 发送 UDP 深度帧。
雷达到货前用它驱动板级半实物测试；到货后换成真雷达，FPGA 侧无需改动。

用法:
  python tools/fake_lidar.py 192.168.1.100 [--port 7687] [--fps 10] [--scene default] [--walk]
    --walk  机器人沿直线缓慢前进（位姿由 FPGA 固件/调试工具提供时可关闭）
  python tools/fake_lidar.py --replay capture.bin 192.168.1.100   回放抓包（每行一个 UDP 负载的十六进制）
注意：PC 网卡 IP 需与 FPGA 雷达口同网段（默认 192.168.1.x），源端口按 XT-M60 默认 7687。
"""
import argparse
import math
import socket
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "model"))
import sim_world as sw  # noqa: E402
import xt_proto as xt  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dst")
    ap.add_argument("--port", type=int, default=7687)
    ap.add_argument("--src-port", type=int, default=7687)
    ap.add_argument("--fps", type=float, default=10)
    ap.add_argument("--walk", action="store_true")
    ap.add_argument("--replay")
    ap.add_argument("--frames", type=int, default=0, help="发送帧数，0 = 不停")
    a = ap.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("", a.src_port))
    if a.replay:
        pkts = [bytes.fromhex(l) for l in Path(a.replay).read_text().split()]
        for p in pkts:
            s.sendto(p, (a.dst, a.port))
            time.sleep(0.0002)
        print(f"回放 {len(pkts)} 包")
        return 0

    world = sw.default_scene()
    lid = sw.VirtualLidar(noise_mm=3.0)
    Rbs, tbs = sw.mount_pose()
    k = 0
    t_next = time.time()
    while a.frames == 0 or k < a.frames:
        x = 0.05 * k if a.walk else 0.0
        Rws, tws = sw.compose(sw.rotz(0.0), np.array([x, 0.0, 0.0]), Rbs, tbs)
        depth, _ = lid.capture(world, Rws, tws)
        fr = xt.build_frame(depth.tolist(), sn=k & 0xFFFF, img_type=1)
        for p in xt.packetize(fr, udp_skip=20, sn=k):
            s.sendto(p, (a.dst, a.port))
            time.sleep(0.00015)                   # 百兆链路上约 1500 字节/120 us，留余量
        k += 1
        t_next += 1.0 / a.fps
        time.sleep(max(0.0, t_next - time.time()))
        if k % 10 == 0:
            print(f"已发送 {k} 帧")
    return 0


if __name__ == "__main__":
    sys.exit(main())
