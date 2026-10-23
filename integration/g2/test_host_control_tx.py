"""UART equivalence and real UDP packet extraction, including backpressure/reset."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'model'))
import netpkt


def crc16(data):
    crc=65535
    for byte in data:
        crc^=byte<<8
        for _ in range(8):crc=((crc<<1)^0x1021 if crc&0x8000 else crc<<1)&65535
    return crc


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--build',type=Path,required=True);ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args();out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    original=(ROOT/'control/fpga/src/uart_telemetry.v').read_text(encoding='utf-8').replace('module uart_telemetry #','module uart_telemetry_original #')
    (out/'uart_original.v').write_text(original,encoding='utf-8')
    files=[a.build/'control/uart_telemetry.v',out/'uart_original.v',ROOT/'control/fpga/src/board_uart_tx.v']
    files += [ROOT/n for n in ['rtl/common/nav_ram_sdp.v','rtl/common/nav_fifo_ob.v','rtl/common/nav_fifo.v','rtl/net/udp_tx.v','rtl/net/host_control_tx.v']]
    commands=[('xvlog',['-sv']+[str(f.resolve())for f in files]+[str(ROOT/'integration/g2/tb_host_control_tx.sv')]),
              ('xelab',['-debug','off','-timescale','1ns/1ps','tb_host_control_tx','-s','snap']),('xsim',['snap','-R'])]
    for name,args in commands:
        with(out/(name+'.log')).open('wb')as f:
            rc=subprocess.run(['cmd','/c',f'E:/2026.1/Vivado/bin/{name}.bat']+args,cwd=out,stdout=f,stderr=subprocess.STDOUT,timeout=180).returncode
        if rc:raise RuntimeError(f'{name} failed; see {out}')
    log=(out/'xsim.log').read_text(errors='replace')
    assert 'TEST PASSED' in log and 'Fatal' not in log,log[-2500:]
    originals=[]
    for segment in(out/'tap.hex').read_text().split('reset'):
        data=bytes(int(x,16)for x in segment.split())
        while len(data)>=126:
            pos=data.find(b'\xa5\x5a\x02\x78')
            if pos<0:break
            frame=data[pos:pos+126];data=data[pos+1:]
            if len(frame)==126 and crc16(frame[2:-2])==int.from_bytes(frame[-2:],'little'):originals.append(frame)
    packets=[];frame=bytearray()
    for token in(out/'tx.hex').read_text().split():
        word=int(token,16);frame.append(word&255)
        if word&256:
            pkt=netpkt.parse(bytes(frame));frame.clear()
            assert pkt['type']=='udp' and pkt['ip_ok'] and pkt['sport']==9000 and pkt['dport']==9001
            packets.append(pkt['payload'])
    assert not frame
    controls=[p for p in packets if p[:4]==b'NV\x01\x83']
    commands=[p for p in packets if p[:4]!=b'NV\x01\x83']
    assert len(commands)==3 and all(p==bytes(range(0xa0,0xb0))for p in commands)
    assert len(controls)>=4
    # Fixture: seq0/1 are captured before peer learning, seq3 is queued then
    # expired, seq7 is fresh after the timestamp jump, then the core resets.
    assert [int.from_bytes(p[12:14],'little') for p in controls[:3]]==[2,7,0]
    for packet in controls:
        assert len(packet)==130 and packet[4:] in originals
        originals.remove(packet[4:])  # no duplication, exact original snapshot/CRC
    result=dict(passed=True,control_packets=len(controls),mailbox_packets=len(commands),
        uart_bitwise_equivalent=True,complete_v2_bytes_preserved=True,
        exercised=['unknown peer','two slots full','random TX backpressure','mailbox during telemetry','stale queued frame','reset recovery'],
        scope='UART and MAC-byte/UDP digital boundary; not vendor analog SerDes or real-board acceptance')
    (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result))


if __name__=='__main__':main()
