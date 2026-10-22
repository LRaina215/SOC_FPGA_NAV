"""Counterfactual range-noise diagnosis on archived poses; no integration."""
import argparse
from collections import Counter,defaultdict
import json
from pathlib import Path

import mujoco as mj
import numpy as np
from fpga_model import FpgaModel
from terrain_scene import TerrainSpec,TerrainScene,PhysicsRayWorld
import sim_world as sw


def load(path):
    with np.load(path) as z:return {k:z[k] for k in z.files}


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--sources',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    proof=json.loads(a.sources.read_text(encoding='utf-8'));p=Path(proof['trial'])
    assert proof['all_maps_exact'] and proof['new_physics_runs']==0
    manifest=json.loads((p/'manifest.json').read_text(encoding='utf-8'))
    raw=load(p/'raw_frames.npz');events=load(p/'nav_events.npz')
    assert len(raw['t'])==proof['ordered_captures']==len(events['commit'])
    scene=TerrainScene(TerrainSpec(**json.loads((p/'terrain_model.json').read_text(encoding='utf-8'))['spec']))
    model=mj.MjModel.from_xml_path(str(p/'physics_backend/assets/robot.xml'));data=mj.MjData(model)
    mj.mj_forward(model,data)
    # Only static terrain is visible to this archived lidar; no robot pose needed.
    assert np.all(model.geom_bodyid[model.geom_group==0]==0)
    world=PhysicsRayWorld(scene,model,data,mj)
    class Cache:
        reuse=False
        def raycast(self,*args,**kwargs):
            if not self.reuse:self.last=world.raycast(*args,**kwargs)
            return self.last.copy()
    cache=Cache();lid4=sw.VirtualLidar(noise_mm=4.,seed=manifest['arguments']['seed']+7,rng_max=8.)
    lid0=sw.VirtualLidar(noise_mm=0.,rng_max=8.);fm=FpgaModel(cfg=manifest['terrain_cfg'])
    needed=defaultdict(set)
    for source in proof['sources']:needed[source['map_frame']].add(tuple(source['cell']))
    intervals={}
    for i,commit in enumerate(events['commit']):
        cache.reuse=False;depth4,_=lid4.capture(cache,raw['R'][i],raw['p'][i])
        assert np.array_equal(depth4,raw['depth'][i]),('raw range/noise source differs',i)
        cache.reuse=True;depth0,_=lid0.capture(cache,raw['R'][i],raw['p'][i])
        fm.frame(depth0);fm.commit(commit[:12],*commit[12:])
        for cell in needed[i]:
            x=cell[0]-fm.cx+64;y=cell[1]-fm.cy+64
            if 0<=x<128 and 0<=y<128 and fm.view[1][y,x]:
                hi=int(fm.view[2][y,x]);intervals[(i,*cell)]=(hi-int(fm.view[3][y,x]),hi)
    rows=[]
    for source in proof['rows']:
        if not source.get('unique_cell_pair'):continue
        pair=source['pairs'][0];frame=source['map_frame'];ia=intervals.get((frame,*pair['a']));ib=intervals.get((frame,*pair['b']))
        r=dict(us=source['us'],map_frame=frame,pair=pair,original=source['interval_trace'],noiseless_a=ia,noiseless_b=ib)
        if ia is not None and ib is not None:
            cap,q=source['interval_trace'][4:6];lo=ib[0]-ia[1]-2*q;hi=ib[1]-ia[0]+2*q
            r.update(noiseless_transition_bounds=[lo,hi],noiseless_result='BLOCK'if lo>cap or hi< -cap else'PASS'if lo>=-cap and hi<=cap else'UNKNOWN')
        else:r['noiseless_result']='INVALID'
        rows.append(r)
    cells=[]
    for source in proof['sources']:
        key=(source['map_frame'],*source['cell']);zero=intervals.get(key)
        if zero is not None:cells.append(dict(map_frame=key[0],cell=source['cell'],span4=source['interval'][1]-source['interval'][0],span0=zero[1]-zero[0]))
    result=dict(trial=str(p),source_proof=str(a.sources.resolve()),raw_depth_frames_exact=len(raw['t']),
        original_maps_already_exact_in_source_proof=True,new_physics_runs=0,policy_inferences=0,
        unique_pairs=len(rows),ambiguous_pairs_excluded=proof['ambiguous_cell_pairs'],noiseless_results=dict(Counter(r['noiseless_result']for r in rows)),
        frame_cells=len(cells),span4_median_mm=float(np.median([r['span4']for r in cells])),
        span0_median_mm=float(np.median([r['span0']for r in cells])),rows=rows,cells=cells,
        scope='sigma4 vs0 on identical archived physical/capture/fusion poses and map order; cell membership/writers may change; only individual interval check, not coverage/admission, a denoiser, closed-loop success, real sensor bounds or board acceptance')
    (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items()if k not in ('rows','cells')},indent=2))


if __name__=='__main__':main()
