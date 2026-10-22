"""Read-only contact-center observability on saved C anchor decisions."""
import json
from collections import Counter
from pathlib import Path
import numpy as np
from terrain_study import OUT,dump


def main():
    source=OUT/'n2av/navigation/AY_trace_up8';out=OUT/'n2av/n2az';out.mkdir(exist_ok=True)
    with np.load(source/'map_frames.npz') as z:m={k:z[k] for k in z.files}
    with np.load(source/'support_frames.npz') as z:s={k:z[k] for k in z.files}
    groups={}
    for line in (source/'candidates.jsonl').open():
        r=json.loads(line)
        if r['kind']==7:groups.setdefault(r['us'],[]).append(r['v'])
    rows=[];counts=Counter()
    for us,a in groups.items():
        assert len(a)==4
        t=(us-1000)/1e6;k=int(np.searchsorted(m['t'],t,side='right')-1);j=int(np.searchsorted(s['t'],t,side='right')-1)
        samples=[]
        for ix0,ix1 in [(0,2),(1,3)]:
            x=round((a[ix0][1]+a[ix1][1])/2);y=round((a[ix0][2]+a[ix1][2])/2)
            gx=(x*1311)>>16;gy=(y*1311)>>16;ix=gx-int(m['cx'][k])+64;iy=gy-int(m['cy'][k])+64
            cx=gx-int(s['cx'][j])//4+16;cy=gy-int(s['cy'][j])//4+16
            inside=0<=ix<128 and 0<=iy<128 and 0<=cx<32 and 0<=cy<32
            if not inside:samples.append(dict(ok=False));continue
            bit=1<<((((y*1311)>>14)&3)*4+(((x*1311)>>14)&3))
            known=bool(s['known'][j,cy,cx]&bit);lo=int(m['H'][k,iy,ix]-m['RO'][k,iy,ix]);hi=int(m['H'][k,iy,ix])
            ok=bool(m['cnt'][k,iy,ix] and not m['stale'][k,iy,ix] and m['age'][k,iy,ix]<=60 and hi-lo<=18 and known)
            samples.append(dict(x=x,y=y,lo=lo,hi=hi,span=hi-lo,known=known,count=int(m['cnt'][k,iy,ix]),age=int(m['age'][k,iy,ix]),stale=bool(m['stale'][k,iy,ix]),ok=ok))
        old=sum(v[3]!=0 for v in a)>=3;new=all(v['ok'] for v in samples)
        counts['anchors']+=1;counts['original_valid']+=old;counts['center_pair_valid']+=new;counts['newly_observable']+=new and not old
        rows.append(dict(t=t,old=old,pair=new,z=a[0][5],samples=samples))
    dump(out/'anchor_contact_observability.json',dict(counts=dict(counts),rows=rows,boundary='nearest saved fusion snapshot; prospective sample geometry only, not a closed-loop result'))
    print(dict(counts))


if __name__=='__main__':main()
