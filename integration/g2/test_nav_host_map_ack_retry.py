"""Fault-inject ACK loss/stale receive states at the actual host upload API."""
from collections import Counter
from pathlib import Path
import struct
import sys
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools'))
import nav_host as nh


class Transport:
    def __init__(self,pkg,drop=True,verdict=2):
        self.pkg=pkg;self.drop=drop;self.verdict=verdict;self.queue=[];self.count=Counter();self.offset=0;self.data=bytearray();self.now=0
    def clock(self):self.now+=0.02;return self.now
    def ack(self,state,n):return b'NV\x01\x82'+struct.pack('<IB3x',n,state)
    def send(self,b):
        kind=b[3];self.count[kind]+=1
        if kind==0x40:
            assert struct.unpack_from('<I',b,4)[0]==len(self.pkg)
            self.offset=0;self.data=bytearray()
            if self.drop and self.count[kind]==1:return
            self.queue.append(self.ack(1,0))
        elif kind==0x41:
            off=struct.unpack_from('<I',b,4)[0];key=(kind,off);self.count[key]+=1
            if off==self.offset:self.data.extend(b[8:]);self.offset+=len(b)-8
            else:assert off<self.offset,'Host attempted a gap'
            if self.drop and self.count[key]==1:return
            # A future/stale offset must not make the host skip bytes.
            self.queue.extend([self.ack(1,self.offset+9999),self.ack(1,self.offset)])
        elif kind==0x42:
            assert self.data==self.pkg
            if self.drop and self.count[kind]==1:
                self.queue.append(self.ack(1,self.offset));return
            self.queue.extend([self.ack(1,self.offset),self.ack(self.verdict,self.offset)])
        else:raise AssertionError(kind)
    def recv(self,timeout):
        self.now+=timeout
        return self.queue.pop(0) if self.queue else None


def main():
    cases=0
    for reliable in (False,True):
        for n in (0,67,2051):
            for verdict in (2,3,4):
                pkg=bytes(i&255 for i in range(n));t=Transport(pkg,verdict=verdict)
                with patch.object(nh.time,'monotonic',t.clock):
                    ok=nh.upload_map(t,pkg,reliable=reliable,log=lambda _:None)
                assert ok==(verdict==2)
                assert t.data==pkg and t.count[0x40]==2 and t.count[0x42]==2
                cases+=1
    print(f'PASS actual upload_map ACK loss/stale terminal/progress guard cases={cases}')


if __name__=='__main__':main()
