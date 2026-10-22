"""Independent compact-model vectors for the real coverage RAM/pipeline."""
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'sim/closed_loop'))
from support_coverage import CompactCoverage


def main():
    out=Path(sys.argv[-1]);out.mkdir(parents=True,exist_ok=True);c=CompactCoverage();rows=[];rng=np.random.default_rng(43);now=0
    def epoch(t):
        c.advance(t);rows.append(f'E {t//200000&127:x}')
    def query(gx,gy):
        k,f=c.mask_windows(now);x,y=gx-(c.cx//4-16),gy-(c.cy//4-16)
        q=int(k[y,x])|(int(f[y,x])<<16) if 0<=x<32 and 0<=y<32 else 0
        addr=((gy&127)<<7)|(gx&127);rows.append(f'Q {addr:x} {q:08x}')
    for i in range(40):
        now+=100000;epoch(now)
        cx,cy=([0,0] if i<12 else [1,-1] if i<20 else [20,1] if i<24 else [0,0])
        gx=rng.integers(cx*4-64,cx*4+64,280);gy=rng.integers(cy*4-64,cy*4+64,280)
        # Persistent targets plus adjacent same-address and two-address hazards.
        gx=np.r_[gx,np.tile([0,1,0,2],25)];gy=np.r_[gy,np.zeros(100,dtype=int)]
        P=np.column_stack((np.rint((gx+.5)*65536/5244),np.rint((gy+.5)*65536/5244),np.zeros(len(gx)))).astype(np.int64)
        valid=np.ones(len(P),bool);c.update(P,valid,cx,cy,now,now)
        rows.append(f'F {cx&0xffffffff:08x} {cy&0xffffffff:08x} {now//200000&127:x} 1')
        inside=(gx>=cx*4-64)&(gx<cx*4+64)&(gy>=cy*4-64)&(gy<cy*4+64)
        for x,y in zip(gx[inside],gy[inside]):
            a=(((int(y)//4)&31)<<9)|(((int(x)//4)&31)<<4)|((int(y)&3)<<2)|(int(x)&3);rows.append(f'P {a:x}')
        rows.append('D')
        for x,y in [(cx,cy),(0,0),(cx-16,cy),(cx+15,cy),(cx+16,cy),(-1,-1)]:query(x,y)
    # No received lidar frames: ageing still progresses through counter wrap.
    for i in range(145):
        now+=200000;epoch(now)
        if i in [1,4,90,99,100,110,127,144]:query(0,0)
    # Clear, delayed capture metadata and fresh re-confirmation.
    rows.append('C');c.clear();epoch(now)
    for age in [600000,400000]:
        cap=now-age;c.update(np.array([[6,6,0]],np.int64),np.array([True]),0,0,cap,now)
        rows.append(f'F 00000000 00000000 {cap//200000&127:x} 1');rows+=['P 0','P 0','P 0','D'];query(0,0)
    # Bad capture metadata must neither update nor roll the retained window.
    rows += [f'F 00000050 00000050 {now//200000&127:x} 0','P 0','D']
    query(0,0)
    rows.append('END');(out/'coverage.ops').write_text('\n'.join(rows)+'\n');(out/'input.json').write_text(json.dumps(dict(seed=43,operations=len(rows),oracle='9bit compact capture-time model; no frame-count age'),indent=2));print(len(rows))


if __name__=='__main__':main()
