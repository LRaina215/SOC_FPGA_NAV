"""Join saved ordered captures to support rejection intervals, without physics.

All saved maps must replay exactly before any source attribution is accepted.
Ambiguous cell pairs and same-timestamp ordering remain explicit.
"""
import argparse
from collections import Counter,defaultdict
import json
from pathlib import Path
import re

import numpy as np
from fpga_model import FpgaModel,fast


def load(path):
    with np.load(path) as z:return {k:z[k] for k in z.files}


def cdiv(a,b):
    return (abs(a)//abs(b))*(-1 if (a<0)!=(b<0) else 1)


def source_statistics(result):
    index={(s['map_frame'],*s['cell']):s for s in result['sources']}
    groups=Counter();ages=[];gaps=[]
    for row in result['rows']:
        if not row.get('unique_cell_pair'):continue
        pair=row['pairs'][0]
        aa,bb=[index[(row['map_frame'],*pair[k])]for k in ('a','b')]
        groups['same_writer_frame'if aa['writer_frame']==bb['writer_frame']else'different_writer_frames']+=1
        gaps.append(abs(aa['writer_frame']-bb['writer_frame']))
        ages.extend((row['us']-1000)/1e6-s['capture_s']for s in (aa,bb))
    return dict(unique_pair_writer_groups=dict(groups),maximum_writer_gap_frames=max(gaps,default=0),
        capture_age_s=dict(minimum=min(ages,default=0),median=float(np.median(ages))if ages else None,maximum=max(ages,default=0)),
        retained_stale_frame_cells=sum(s['retained_stale']for s in result['sources']),
        maximum_source_map_age_frames=max((s['map_age_frames']for s in result['sources']),default=0))


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--trial',type=Path,required=True)
    ap.add_argument('--firmware',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False);p=a.trial.resolve()
    manifest=json.loads((p/'manifest.json').read_text(encoding='utf-8'))
    metrics=json.loads((p/'metrics.json').read_text(encoding='utf-8'))
    assert metrics['raw_frames_dropped']==0 and metrics['match_applied']==0
    raw=load(p/'raw_frames.npz');maps=load(p/'map_frames.npz');events=load(p/'nav_events.npz')
    count=len(raw['t']);assert count==len(maps['t'])==len(events['commit'])
    assert np.all(np.diff(raw['t'])>0) and np.all(np.diff(maps['t'])>0)
    assert np.all(raw['t']<=maps['t'])
    # Match frozen C arithmetic, including negative division and LUT interpolation.
    header=(a.firmware/'include/nav_sin_lut.h').read_text()
    sine=list(map(int,re.findall(r'-?\d+',header.split('= {',1)[1].split('}',1)[0])))
    assert len(sine)==1024
    def sn(angle):
        angle&=65535;f=angle&63
        return cdiv(sine[angle>>6]*(64-f)+sine[((angle+64)&65535)>>6]*f,64)
    def path(pose,v,w,step,side):
        x,y,_,yaw=pose[:4];x*=16384;y*=16384;th=yaw*65536
        da=cdiv(w*50*65536*65536,6283185);lateral=135 if side else -135
        previous=None
        for i in range(step+1):
            angle=(th>>16)&65535;s=sn(angle);c=sn(angle+16384)
            wx=cdiv(x-lateral*s,16384);wy=cdiv(y+lateral*c,16384)
            if i==step:return wx,wy,s,c,previous
            previous=(wx,wy)
            mid=((th+cdiv(da,2))>>16)&65535
            x+=cdiv(v*sn(mid+16384),20);y+=cdiv(v*sn(mid),20);th+=da
    inv=1311
    def interval(frame,cell):
        gx,gy=cell;x=gx-int(maps['cx'][frame])+64;y=gy-int(maps['cy'][frame])+64
        if 0<=x<128 and 0<=y<128 and maps['V'][frame,y,x]:
            hi=int(maps['H'][frame,y,x]);return hi-int(maps['RO'][frame,y,x]),hi
    rows=[];pose=None;pending=None
    trace=[json.loads(line) for line in (p/'candidates.jsonl').read_text().splitlines()]
    for row in trace:
        if row['kind']==0:pose=row
        if row['kind']==9:
            assert pending is None;pending=row;continue
        if pending is None:continue
        assert row['kind']==3 and row['us']==pending['us'],'unpaired support transition'
        r=dict(us=row['us'],interval_trace=pending['v'],wheel_trace=row['v'],pairs=[])
        pending=None
        if pose is None or pose['us']!=row['us']:
            r['unresolved']='no same-query pose';rows.append(r);continue
        v,w,step,side,wx,wy,result=row['v']
        rx,ry,s,c,previous=path(pose['v'],v,w,step,side)
        r['wheel_geometry_exact']=(rx,ry)==(wx,wy)
        if not r['wheel_geometry_exact']:
            r['unresolved']='wheel geometry differs from frozen trace';rows.append(r);continue
        fusetime=np.rint(maps['t']*1e6).astype(np.int64)+1000
        frame=int(np.searchsorted(fusetime,row['us'],side='right')-1);r['map_frame']=frame
        if frame<0 or fusetime[frame]==row['us']:
            r['unresolved']='same-timestamp map order or no earlier map';rows.append(r);continue
        al,ah,bl,bh,cap,q,lo,hi,code=r['interval_trace']
        assert (bl-ah-2*q,bh-al+2*q,code)==(lo,hi,result)
        center=(wx*inv>>16,wy*inv>>16)
        def add(kind,ca,cb):
            if interval(frame,ca)==(al,ah) and interval(frame,cb)==(bl,bh):
                r['pairs'].append(dict(kind=kind,a=list(ca),b=list(cb)))
        if cap==20:
            margin=(abs(v)+(abs(w)*172+999)//1000+39)//40+2
            aa,bb=34+margin,14+margin
            xr=(aa*abs(c)+bb*abs(s)+16383)//16384;yr=(aa*abs(s)+bb*abs(c)+16383)//16384
            px=wx*inv>>8;py=wy*inv>>8;aq=(aa*inv+255)>>8;bq=(bb*inv+255)>>8
            proj=128*(abs(c)+abs(s));af=aq*16384+proj;at=bq*16384+proj
            for yy in range((wy-yr)*inv>>16,((wy+yr)*inv>>16)+1):
                for xx in range((wx-xr)*inv>>16,((wx+xr)*inv>>16)+1):
                    dx=xx*256+128-px;dy=yy*256+128-py
                    if abs(dx*c+dy*s)<=af and abs(-dx*s+dy*c)<=at:add('wheel_patch',center,(xx,yy))
            if previous is not None:
                pc=(previous[0]*inv>>16,previous[1]*inv>>16)
                if pc!=center:add('previous_step',pc,center)
        else:r['unresolved']='grade/roll transition not reconstructed'
        distinct={(tuple(x['a']),tuple(x['b']))for x in r['pairs']}
        r['unique_cell_pair']=len(distinct)==1
        if not distinct:r.setdefault('unresolved','no matching geometric cell pair in saved map')
        rows.append(r)
    assert pending is None
    needed=defaultdict(set)
    for row in rows:
        for pair in row['pairs']:
            for key in ('a','b'):needed[row['map_frame']].add(tuple(pair[key]))
    fm=FpgaModel(cfg=manifest['terrain_cfg']);writers=np.full(128*128,-1,dtype=np.int32)
    sources=[]
    def points(frame):
        commit=events['commit'][frame];fm.frame(raw['depth'][frame])
        xyz=fm.xyz.copy();valid=fm.pv.copy();world=fast.pc_transform(xyz,commit[:9].reshape(3,3),commit[9:12])
        gx=fast.world_cell(world[:,0],inv);gy=fast.world_cell(world[:,1],inv)
        if fm.cfg.get('fuse_r',0)>0:valid&=np.sum((world[:,:2]-commit[9:11])**2,axis=1)<=fm.cfg['fuse_r']**2
        valid&=(world[:,2]>=max(-32768,int(commit[14])))&(world[:,2]<=min(32767,int(commit[15])))
        valid&=(gx>=commit[12]-64)&(gx<commit[12]+64)&(gy>=commit[13]-64)&(gy<commit[13]+64)
        return xyz,world,gx,gy,valid
    # Prove the FIFO/ordered join by reproducing all archived maps, not timestamps alone.
    for frame in range(count):
        xyz,world,gx,gy,valid=points(frame);commit=events['commit'][frame]
        fm.commit(commit[:12],*commit[12:])
        values=dict(V=fm.view[1],H=fm.view[2],RO=fm.view[3],raw=fm.last_lethal,
                    cnt=fm.em.cnt[fm.view[0]],age=(fm.em.fc-fm.em.stamp[fm.view[0]])&255,stale=fm.em.stale[fm.view[0]])
        for key,value in values.items():assert np.array_equal(value,maps[key][frame]),(frame,key)
        writers[((gy[valid]&127)<<7)|(gx[valid]&127)]=frame
        for cell in sorted(needed[frame]):
            slot=((cell[1]&127)<<7)|(cell[0]&127);writer=int(writers[slot]);assert writer>=0
            sources.append(dict(map_frame=frame,cell=list(cell),interval=list(interval(frame,cell)),writer_frame=writer,
                capture_s=float(raw['t'][writer]),fusion_s=float(maps['t'][writer]),
                map_age_frames=int((fm.em.fc-int(fm.em.stamp[slot]))&255),retained_stale=bool(fm.em.stale[slot])))
    # Re-project only contributing source frames; never advance physics or policy.
    cache={}
    for source in sources:
        writer=source['writer_frame']
        if writer not in cache:cache[writer]=points(writer)
        xyz,world,gx,gy,valid=cache[writer];x,y=source['cell'];ids=np.flatnonzero(valid&(gx==x)&(gy==y))
        assert len(ids) and [int(world[ids,2].min()),int(world[ids,2].max())]==source['interval']
        source.update(point_count=len(ids),point_indices=ids.tolist(),pixel_row_col=np.stack(np.divmod(ids,160),axis=1).tolist(),
            depth_mm=raw['depth'][writer].reshape(-1)[ids].tolist(),sensor_xyz_mm=xyz[ids].tolist(),fused_xyz_mm=world[ids].tolist(),
            minimum_indices=ids[world[ids,2]==source['interval'][0]].tolist(),maximum_indices=ids[world[ids,2]==source['interval'][1]].tolist())
    result=dict(trial=str(p),frozen_firmware=str(a.firmware.resolve()),ordered_captures=count,all_maps_exact=True,
        checked_map_fields=list(values),new_physics_runs=0,support_transition_rejections=len(rows),
        wheel_geometry_exact=sum(r.get('wheel_geometry_exact',False)for r in rows),
        unique_cell_pairs=sum(r.get('unique_cell_pair',False)for r in rows),
        ambiguous_cell_pairs=sum(bool(r['pairs'])and not r['unique_cell_pair']for r in rows),
        unresolved=dict(Counter(r['unresolved']for r in rows if 'unresolved'in r)),
        source_frame_cells=len(sources),source_frames=len(cache),rows=rows,sources=sources,
        scope='archived ordered/no-drop raw to identical maps to possible/unique rejection cell pairs; raw measured points only, not calibrated truth or denoising acceptance')
    result.update(source_statistics(result))
    (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items()if k not in ('rows','sources')},indent=2))


if __name__=='__main__':main()
