"""Verify repaired uploader against real PNR61, with explicit ACK-loss injection.

Use an isolated tools folder beside this script. Only map/query/clear messages;
no motion/pose/enable/firmware write. Clear the synthetic prior before exit.
"""
from collections import Counter
from datetime import datetime
import json
from pathlib import Path
import struct
import sys
import time
import zlib

sys.path.insert(0,str(Path(__file__).resolve().parent/'tools'))
import nav_host as nh


def make(full=False):
    slots=[(i,(i*17)&0x3ffff) for i in range(16384)] if full else [(0,1),(16383,2)]
    nx=ny=64 if full else 2
    body=b''.join(struct.pack('<H',s)+v.to_bytes(3,'little') for s,v in slots)+bytes(nx*ny)
    h=bytearray(48);h[:4]=b'NVMP'
    struct.pack_into('<HHiiIiiHHIIIhh',h,4,3,0x0107,-64,-64,len(slots),-1,-1,nx,ny,zlib.crc32(body),len(body),0x33580001,350,-300)
    return bytes(h)+body


class Observed:
    def __init__(self,base):self.base=base;self.count=Counter();self.drop=set();self.dropped=[];self.stale=None;self.statuses=0
    def send(self,b):
        assert b[:3]==b'NV\x01' and b[3] in (0x40,0x41,0x42,0x43,0x44)
        self.count[b[3]]+=1;self.base.send(b)
        if b[3]==0x42 and 'stale_end' in self.drop:
            self.drop.remove('stale_end');self.stale=b'NV\x01\x82'+struct.pack('<IB3x',86064,1)
    def recv(self,timeout):
        if self.stale is not None:b=self.stale;self.stale=None;return b
        b=self.base.recv(timeout);a=nh.parse(b) if b else None
        if a and a['type']=='status':
            assert a['vx']==a['wz']==0;self.statuses+=1
        if a and a['type']=='map_ack':
            key='begin' if a['state_code']==1 and a['next']==0 else 'data' if a['state_code']==1 else 'end' if a['state_code']==2 else None
            if key in self.drop:
                self.drop.remove(key);self.dropped.append(dict(injected_loss=key,actual_ACK=a));return None
        return b


def main():
    link=Observed(nh.open_link('uart:/dev/ttyACM0'));logs=[];results=[]
    def clear():
        for _ in range(4):
            link.send(nh.map_clear());end=time.monotonic()+0.5
            while time.monotonic()<end:
                b=link.recv(0.03);a=nh.parse(b) if b else None
                if a and a['type']=='map_ack' and a['state_code']==0:return
        raise AssertionError('Cannot confirm prior cleared')
    clear()
    try:
        link.drop={'begin','data','end','stale_end'}
        t=time.monotonic();ok=nh.upload_map(link,make(True),reliable=True,log=logs.append)
        assert ok and not link.drop and {x['injected_loss'] for x in link.dropped}=={'begin','data','end'}
        results.append(dict(case='maximum_map_ACK_loss_and_stale_RECV',passed=True,bytes=86064,elapsed_s=time.monotonic()-t))
        clear()
        bad=bytearray(make());bad[32]^=1
        assert nh.upload_map(link,bytes(bad),reliable=True,log=logs.append) is False
        results.append(dict(case='CRC_error_returns_false',passed=True));clear()
        bad=bytearray(make());struct.pack_into('<H',bad,6,0x0108)
        assert nh.upload_map(link,bytes(bad),reliable=True,log=logs.append) is False
        results.append(dict(case='geometry_error_returns_false',passed=True));clear()
        print(json.dumps(dict(passed=True,ended_at=datetime.now().astimezone().isoformat(),cases=results,
            injected_ACK_losses=link.dropped,actual_packet_counts=dict(link.count),status_samples=link.statuses,
            all_observed_targets_zero=True,logs=logs,CPU_reset=False,reprogrammed=False,final_prior_cleared=True,
            caveat='Device/CPU transfer is real; three missing ACK observations and one stale receive-state reply were explicitly injected in the host reader.'),ensure_ascii=False))
    finally:clear()


if __name__=='__main__':main()
