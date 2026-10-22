"""Same-input classifier ablation: disable inferred cost filling only."""
import json
from pathlib import Path
import numpy as np
from terrain_study import OUT,TerrainScene,TerrainSpec,MapRecorder,dump


def main():
    pairs=[]
    for label,name in [('primary','flat'),('primary','up8'),('primary','step20'),('primary_recovered','trench40d150'),('primary_recovered','trench100d150')]:
        source=OUT/label/name;out=source/'maps_no_fill';out.mkdir(exist_ok=False)
        r=json.loads((source/'result.json').read_text());sc=TerrainScene(TerrainSpec(**r['scene']['spec']))
        old=np.load(source/'maps_v2/map_frames.npz');raw=np.load(source/'maps_v2/raw_frames.npz');rec=MapRecorder(sc,out,cfg=dict(fill_min=9))
        for i,t in enumerate(raw['t']):rec.add(raw['depth'][i],raw['R'][i],raw['p'][i],float(t),raw['truth'][i])
        rec.close('identical raw frames, poses and geometry; fill_min=9 only; no policy rerun')
        new=np.load(out/'map_frames.npz')
        same=all(np.array_equal(old[k],new[k]) for k in ['t','cx','cy','V','H','RO','raw','age','cnt','stale'])
        assert same
        changed=old['cost']!=new['cost'];assert np.all((new['cost'][changed]==255)&(~new['V'][changed]))
        pairs.append(dict(case=name,input_frames=len(raw['t']),observation_height_raw_hazard_identical=same,
            inferred_free_changed_to_unknown=int(changed.sum()),raw_hazard_quality_changed=False,
            note='does not detect subcell trench when bank points already mark V=true'))
        print(name,pairs[-1]['inferred_free_changed_to_unknown'],flush=True)
    dump(OUT/'classifier_no_fill_comparison.json',pairs)


if __name__=='__main__':main()
