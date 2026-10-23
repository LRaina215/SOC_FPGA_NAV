"""MAC-byte-level integration test. Does not simulate analog SerDes/vendor IP."""
import argparse
import json
import os
from pathlib import Path
import random
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "model"))
import netpkt


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--vivado-bin", type=Path, default=Path(os.environ.get("VIVADO_BIN", "E:/2026.1/Vivado/bin")))
    a = p.parse_args()
    out = a.out.resolve(); out.mkdir(parents=True, exist_ok=False)
    my_mac, peer_mac = 0x024e56000002, 0x0a1b2c3d4e5f
    my_ip, peer_ip = "192.168.2.100", "192.168.2.10"
    frames = [(netpkt.arp_request_frame(peer_mac, peer_ip, my_ip), 0, 15000)]
    expected = []
    rng = random.Random(45)

    def udp(payload, port, keep=True, error=0, gap=15000, truncate=None, dport=9000):
        frame = netpkt.udp_frame(my_mac, peer_mac, peer_ip, my_ip, port, dport, payload)
        frames.append((frame[:truncate] if truncate else frame, error, gap))
        if keep: expected.append((port, payload))

    udp(b"A", 12001)
    udp(bytes(range(18)), 12002)
    udp(bytes(rng.randrange(256) for _ in range(1472)), 12003, gap=12)
    udp(b"drop while payload RAM is occupied", 12004, keep=False)
    udp(b"bad last-byte MAC error", 12005, keep=False, error=1)
    udp(bytes(1473), 12006, keep=False)
    udp(bytes(100), 12007, keep=False, truncate=80)
    udp(b"wrong destination port", 12008, keep=False, dport=9002)
    udp(bytes(rng.randrange(256) for _ in range(255)), 12009)
    udp(b"recovered after drops", 12010)
    rows=[]
    for frame, error, gap in frames:
        for i, byte in enumerate(frame):
            last=int(i==len(frame)-1)
            rows.append(f"{byte | (last<<8) | ((error & last)<<9):03x} {gap if last else 0}\n")
    (out/"stimulus.txt").write_text("".join(rows),encoding="ascii")
    sources=[ROOT / "rtl/common" / n for n in
             ("nav_cdc.v","nav_ram_sdp.v","nav_async_fifo.v","nav_fifo.v","nav_fifo_ob.v")]
    sources += [ROOT / "rtl/net" / n for n in ("net_port.v","udp_rx.v","udp_tx.v")]
    sources += [HERE/"sfp_udp_echo.v"]
    commands=[("xvlog_v",["xvlog"]+[str(n) for n in sources]),
              ("xvlog_sv",["xvlog","-sv",str(HERE/"tb_sfp_echo.sv")]),
              ("xelab",["xelab","-debug","off","-timescale","1ns/1ps","tb_sfp_echo","-s","snap"]),
              ("xsim",["xsim","snap","-R"])]
    for tag, command in commands:
        with (out/(tag+"_stdout.log")).open("wb") as log:
            rc=subprocess.run(["cmd","/c",str(a.vivado_bin/(command[0]+".bat"))]+command[1:],
                              cwd=out,stdout=log,stderr=subprocess.STDOUT,timeout=180).returncode
        if rc: raise RuntimeError(f"{tag} failed: {out}")
    log=(out/"xsim_stdout.log").read_text(errors="replace")
    if "TEST PASSED" not in log or any(s in log for s in ("Fatal", "FATAL", "ERROR:", "TEST FAILED")):
        raise AssertionError(log)
    captured=[]; frame=bytearray()
    for word in (out/"tx.hex").read_text().split():
        v=int(word,16); frame.append(v&255)
        if v&256: captured.append(netpkt.parse(bytes(frame))); frame=bytearray()
    assert not frame and len(captured)==6
    arp=captured[0]
    assert arp["type"]=="arp" and arp["oper"]==2 and arp["tpa"]==netpkt.ip_bytes(peer_ip)
    for parsed,(port,payload) in zip(captured[1:],expected):
        assert parsed["type"]=="udp" and parsed["ip_ok"]
        assert parsed["dst"]==netpkt.mac_bytes(peer_mac) and parsed["src"]==netpkt.mac_bytes(my_mac)
        assert parsed["dst_ip"]==netpkt.ip_bytes(peer_ip) and parsed["src_ip"]==netpkt.ip_bytes(my_ip)
        assert parsed["sport"]==9000 and parsed["dport"]==port and parsed["payload"]==payload
    result=dict(passed=True,scope="New echo logic + existing ARP/UDP/CDC at MAC byte interface",
                echoed_payload_lengths=[len(p) for _,p in expected],mac_frames=6,
                covered="random TX backpressure, busy drop, 1472/1473 limit, CRC error, truncation, wrong port, recovery",
                vendor_pcs_serdes_simulated=False,hardware_tested=False)
    (out/"result.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    print(json.dumps(result,indent=2))


if __name__=="__main__": main()
