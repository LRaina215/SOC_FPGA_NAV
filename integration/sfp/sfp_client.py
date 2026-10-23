"""SFP1 diagnostic / SFP2 joint UART registers and standalone UDP echo."""
import argparse
import json
import socket
import struct
import time

REGISTERS = {0x00:"signature",0x04:"config",0x08:"status",0x0c:"partner",
             0x10:"diagnostic",0x14:"link_losses",0x20:"rx_frames",0x24:"rx_udp",
             0x28:"rx_errors",0x2c:"arp_requests",0x30:"tx_frames",
             0x34:"echoes",0x38:"echo_drops",0x40:"ipv4",0x44:"port"}
JOINT_REGISTERS = {0x00:"signature",0x04:"config",0x08:"status",0x0c:"partner",
                   0x10:"diagnostic",0x14:"link_losses",0x18:"capability",
                   0x20:"telemetry_accepted",0x24:"telemetry_dropped",0x28:"telemetry_sent"}


def inspect_registers(read, write, image="auto", mode=None, use_los=False,
                      settle=3.0, sleep=time.sleep):
    """Verify the image signature before any configuration write."""
    candidates = [("joint",0x1300,0x53465032,JOINT_REGISTERS),
                  ("standalone",0,0x53465031,REGISTERS)]
    selected = next((item for item in candidates
                     if (image=="auto" or image==item[0]) and read(item[1])==item[2]),None)
    if selected is None:
        raise RuntimeError("No matching SFP1/SFP2 signature; configuration was not written")
    name,base,signature,registers = selected
    if mode:
        value={"basex-an":2,"basex-forced":0,"sgmii-an":3}[mode] | (4 if use_los else 0)
        write(base+4,value)
        sleep(settle)
        if read(base)!=signature or read(base+4)!=value:
            raise RuntimeError("SFP image/config readback does not match the requested value")
    data={key:read(base+offset) for offset,key in registers.items()}
    if data["signature"]!=signature:
        raise RuntimeError("SFP image changed during register read")
    diag=data["diagnostic"];status=data["status"]
    data.update(image=name,register_base=base)
    data["decoded"]={"mode":"SGMII" if data["config"]&1 else "1000BASE-X",
                     "autoneg":bool(data["config"]&2),"honour_los":bool(data["config"]&4),
                     "qpll_locked":bool(diag&1),"rx_locked":bool(diag&2),
                     "aligned":bool(diag&4),"los":bool(diag&8),
                     "link_ready":bool(diag&16),"ip_reset":bool(diag&32),
                     "pcs_sync":bool(status&1),"pcs_link":bool(status&2),
                     "an_complete":bool(status&4),
                     "partner_phy_link":bool(status&256) if data["config"]&1 else None}
    return data


def registers(args):
    import serial
    with serial.Serial(args.port,921600,timeout=0.5,write_timeout=0.5) as dev:
        # Silence lets an incomplete old UART command expire; signature precedes writes.
        time.sleep(0.02); dev.reset_input_buffer()
        def read(addr):
            dev.write(b"R"+struct.pack(">H",addr))
            response=dev.read(5)
            if len(response)!=5 or response[0:1]!=b"D":
                raise RuntimeError(f"No SFP register reply at {addr:04x}: {response.hex()}")
            return int.from_bytes(response[1:],"big")
        def write(addr,value):
            dev.write(b"W"+struct.pack(">HI",addr,value))
            if dev.read(1)!=b"K": raise RuntimeError("Configuration write was not acknowledged")
        data=inspect_registers(read,write,args.image,args.mode,args.use_los,args.settle)
        print(json.dumps(data,indent=2))


def echo(args):
    if not 1<=args.size<=1472: raise ValueError("size must be 1..1472 bytes")
    if args.count<1: raise ValueError("count must be positive")
    endpoint=(socket.gethostbyname(args.ip),args.port)
    rtts=[]
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
        sock.bind((args.bind,0)); sock.settimeout(args.timeout)
        sock.connect(endpoint)
        for seq in range(args.count):
            # Vary all payloads, including the one-byte boundary test.
            if args.size<4:
                payload=(seq & 0xffffffff).to_bytes(4,"big")[-args.size:]
            else:
                payload=struct.pack(">I",seq & 0xffffffff)+bytes(((seq+i*17)&255) for i in range(args.size-4))
            started=time.perf_counter(); sock.send(payload)
            answer=sock.recv(2048)
            if answer!=payload: raise RuntimeError(f"Echo mismatch for packet {seq}")
            rtts.append((time.perf_counter()-started)*1000)
            if args.interval: time.sleep(args.interval)
    print(json.dumps(dict(passed=True,endpoint=f"{args.ip}:{args.port}",packets=len(rtts),
                          payload_bytes=args.size,rtt_ms_min=min(rtts),rtt_ms_max=max(rtts)),indent=2))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest="command",required=True)
    s=sub.add_parser("status")
    s.add_argument("--port",required=True,help="CH9102 serial port, e.g. COM11")
    s.add_argument("--image",choices=("auto","standalone","joint"),default="auto",
                   help="verify SFP1 at 0x0000 or joint SFP2 at 0x1300 before writes")
    s.add_argument("--mode",choices=("basex-an","basex-forced","sgmii-an"))
    s.add_argument("--use-los",action="store_true",help="Only applied with --mode")
    s.add_argument("--settle",type=float,default=3.0)
    s.set_defaults(func=registers)
    e=sub.add_parser("echo")
    e.add_argument("--ip",default="192.168.2.100"); e.add_argument("--port",type=int,default=9000)
    e.add_argument("--bind",default="0.0.0.0"); e.add_argument("--count",type=int,default=100)
    e.add_argument("--size",type=int,default=64); e.add_argument("--timeout",type=float,default=3.0)
    e.add_argument("--interval",type=float,default=0.01)
    e.set_defaults(func=echo)
    args=p.parse_args(); args.func(args)


if __name__=="__main__": main()
