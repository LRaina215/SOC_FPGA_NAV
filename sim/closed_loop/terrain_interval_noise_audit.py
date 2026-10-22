"""Same saved physical/capture/fusion poses: sigma4 vs zero range noise.

First reproduce every raw depth and original map. Counterfactual geometry is
diagnostic only and never enters navigation. No policy/physics is integrated.
"""
import argparse
import json
from pathlib import Path
import numpy as np
import mujoco as mj
from terrain_scene import TerrainSpec,TerrainScene,PhysicsRayWorld
from fpga_model import FpgaModel
import sim_world as sw


def arrays(path):
    with np.load(path) as z:return {k:z[k] for k in z.files}


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--trial',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    a.out.mkdir(parents=True,exist_ok=False);p=a.trial
    manifest=json.loads((p/'manifest.json').read_text(encoding='utf-8'))
    scene=TerrainScene(TerrainSpec(**json.loads((p/'terrain_model.json').read_text(encoding='utf-8'))['spec']))
    model=mj.MjModel.from_xml_path(str(p/'physics_backend/assets/robot.xml'));data=mj.MjData(model);mj.mj_forward(model,data)
    rays=PhysicsRayWorld(scene,model,data,mj)
    class Cache:
        def raycast(self,*args,**kwargs):
            if self.reuse:return self.last.copy()
            self.last=rays.raycast(*args,**kwargs);return self.last.copy()
    cache=Cache();cache.reuse=False
    raw=arrays(p/'raw_frames.npz');maps=arrays(p/'map_frames.npz');events=arrays(p/'nav_events.npz')
    assert len(raw['t'])==len(maps['t'])==len(events['commit']),'Only ordered/no-drop captures supported by this audit'
    lid4=sw.VirtualLidar(noise_mm=4.,seed=manifest['arguments']['seed']+7,rng_max=8.)
    lid0=sw.VirtualLidar(noise_mm=0.,rng_max=8.)
    normal=FpgaModel(cfg=manifest['terrain_cfg']);zero=FpgaModel(cfg=manifest['terrain_cfg']);zero_maps=[]
    for i,commit in enumerate(events['commit']):
        cache.reuse=False;d4,_=lid4.capture(cache,raw['R'][i],raw['p'][i]);cache.reuse=True;d0,_=lid0.capture(cache,raw['R'][i],raw['p'][i])
        assert np.array_equal(d4,raw['depth'][i]),f'raw source mismatch frame{i}'
        for fm,depth in [(normal,d4),(zero,d0)]:fm.frame(depth);fm.commit(commit[:12],*commit[12:])
        for key,value in [('V',normal.view[1]),('H',normal.view[2]),('RO',normal.view[3]),('raw',normal.last_lethal)]:
            assert np.array_equal(value,maps[key][i]),f'normal map mismatch frame{i} {key}'
        zero_maps.append((zero.view[1].copy(),zero.view[2].copy(),zero.view[3].copy()))
    zero_v=np.array([v[0] for v in zero_maps]);zero_h=np.array([v[1] for v in zero_maps]);zero_ro=np.array([v[2] for v in zero_maps])
    rows=[]
    for row in (json.loads(l) for l in (p/'candidates.jsonl').read_text().splitlines()):
        if row['kind']!=5:continue
        t=(row['us']-1000)/1e6;k=int(np.searchsorted(maps['t'],t,side='right')-1)
        gx,gy,lo,hi,mask=row['v'];x=gx-int(maps['cx'][k])+64;y=gy-int(maps['cy'][k])+64
        assert 0<=x<128 and 0<=y<128
        original=(int(maps['H'][k,y,x]-maps['RO'][k,y,x]),int(maps['H'][k,y,x]))
        rows.append(dict(t=t,frame=k,cell=[gx,gy],actual_rejection=[lo,hi],matched=original==(lo,hi),
            noiseless_valid=bool(zero_v[k,y,x]),noiseless_interval=[int(zero_h[k,y,x]-zero_ro[k,y,x]),int(zero_h[k,y,x])],
            span4=hi-lo,span0=int(zero_ro[k,y,x]),coverage_held=int(mask)))
    matched=[r for r in rows if r['matched'] and r['noiseless_valid']]
    unique={(r['frame'],*r['cell']):r for r in matched};u=list(unique.values())
    result=dict(raw_reproduced_frames=len(raw['t']),estimated_maps_reproduced_frames=len(raw['t']),actual_wide_rejections=len(rows),
        matched_valid_rejections=len(matched),unique_frame_cells=len(u),
        span4_median_mm=float(np.median([r['span4'] for r in u])) if u else None,
        span0_median_mm=float(np.median([r['span0'] for r in u])) if u else None,
        noiseless_span_still_above18=sum(r['span0']>18 for r in u),rows=rows,
        boundary='same raw capture/fusion transforms and map history; sigma4mm vs0; point assignment may change; diagnostic only; not a proposed denoiser or calibrated real noise bound')
    (a.out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    np.savez_compressed(a.out/'noiseless_intervals.npz',t=maps['t'],V=zero_v,H=zero_h,RO=zero_ro,cx=maps['cx'],cy=maps['cy'])
    print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))


if __name__=='__main__':main()
