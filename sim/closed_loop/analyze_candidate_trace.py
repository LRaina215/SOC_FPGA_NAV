"""Analyze saved decisions; no new physics or truth fed to the controller."""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np


def arrays(path):
    with np.load(path) as z:return {k:z[k] for k in z.files}


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--trial',type=Path,required=True);ap.add_argument('--ordinary',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    m=arrays(a.trial/'map_frames.npz');s=arrays(a.trial/'support_frames.npz');rows=[json.loads(l) for l in (a.trial/'candidates.jsonl').read_text().splitlines()]
    same={}
    for name in ['final_maps.npz','nav_events.npz','map_frames.npz','support_frames.npz']:
        x,y=arrays(a.trial/name),arrays(a.ordinary/name)
        same[name]={k:bool(np.array_equal(x[k],y[k],equal_nan=True)) for k in x if x[k].dtype.kind not in ['O','U','S']}
    assert all(all(v.values()) for v in same.values()),'Instrumentation changed saved navigation/physics inputs or outputs'
    counts=Counter();examples=[]
    for row in rows:
        if row['kind']!=7:continue
        t=(row['us']-1000)/1e6;k=np.searchsorted(m['t'],t,side='right')-1;j=np.searchsorted(s['t'],t,side='right')-1
        i,x,y,ok,h,z=row['v'];gx=(x*1311)>>16;gy=(y*1311)>>16
        mx=gx-int(m['cx'][k])+64;my=gy-int(m['cy'][k])+64;cx=gx-int(s['cx'][j])//4+16;cy=gy-int(s['cy'][j])//4+16
        if not(0<=mx<128 and 0<=my<128 and 0<=cx<32 and 0<=cy<32):counts['outside']+=1;continue
        known,fresh=int(s['known'][j,cy,cx]),int(s['fresh'][j,cy,cx]);bit=1<<((((y*1311)>>14)&3)*4+(((x*1311)>>14)&3))
        rawok=bool(m['cnt'][k,my,mx] and not m['stale'][k,my,mx] and m['age'][k,my,mx]<=60 and m['RO'][k,my,mx]<=40)
        counts['samples']+=1;counts['height_ok']+=ok;counts['raw_ok']+=rawok
        counts['wholecell_stale_but_query_fresh']+=bool(rawok and known&~fresh and fresh&bit)
        counts['raw_ok_query_fresh']+=bool(rawok and fresh&bit)
        if rawok and known&~fresh and fresh&bit and len(examples)<8:
            examples.append(dict(t=t,x=x,y=y,raw_lo=int(m['H'][k,my,mx]-m['RO'][k,my,mx]),raw_hi=int(m['H'][k,my,mx]),known=known,fresh=fresh,query_bit=bit,bridge_ok=ok))
    result=dict(instrumentation_equals_ordinary=same,anchor_counts=dict(counts),examples=examples,
        trace_kinds=dict(Counter(r['kind'] for r in rows)),
        checked_body_support=[dict(body=k[0],support=k[1],count=v) for k,v in Counter(tuple(r['v'][2:]) for r in rows if r['kind']==2).items()],
        anchor_decisions=[dict(accepted_samples=k[0],action=k[1],count=v) for k,v in Counter((r['v'][0],r['v'][5]) for r in rows if r['kind']==8).items()],
        height_rejections=[dict(lo=k[0],hi=k[1],count=v) for k,v in Counter((r['v'][2],r['v'][3]) for r in rows if r['kind']==5).most_common(12)],
        inference_boundary='saved map nearest fusion; query flags/decisions from actual C; finite diagnostic not a success case')
    a.out.write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps({k:v for k,v in result.items() if k!='instrumentation_equals_ordinary'},indent=2))


if __name__=='__main__':main()
