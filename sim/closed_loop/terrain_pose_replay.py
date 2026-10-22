"""Identical raw navigation inputs: estimated transforms vs ideal capture pose."""
import json
from pathlib import Path
import numpy as np
from terrain_scene import TerrainScene,TerrainSpec
from terrain_study import OUT,MapRecorder,dump


def main():
    results=[]
    for name in ['AS_flat','N1c_flat','N1c_up8','AU_up8']:
        p=OUT/'navigation_v2'/name;sc=TerrainScene(TerrainSpec(**json.loads((p/'terrain_model.json').read_text())['spec']))
        raw=np.load(p/'raw_frames.npz');maps=np.load(p/'map_frames.npz');events=np.load(p/'nav_events.npz')
        cfg=json.loads((p/'manifest.json').read_text())['terrain_cfg'];normal=MapRecorder(sc,p/'pose_replay_estimated',cfg=cfg)
        ideal=MapRecorder(sc,p/'pose_replay_ideal',cfg=cfg);all_same=True
        assert len(raw['t'])>=len(events['commit'])==len(maps['t'])
        for i,commit in enumerate(events['commit']):
            fm=normal.fm;fm.frame(raw['depth'][i]);fm.commit(commit[:12],*commit[12:])
            x,y,a=events['alignment'][i];cs,sn=np.cos(a),np.sin(a)
            transform=lambda X,Y:(x+cs*X-sn*Y,y+sn*X+cs*Y)
            normal.observe(fm,float(maps['t'][i]),None,transform)
            ideal.add(raw['depth'][i],raw['R'][i],raw['p'][i],float(maps['t'][i]),raw['truth'][i])
            for j,key in [(3,'V'),(4,'H'),(5,'RO'),(6,'raw'),(7,'cost')]:
                if not np.array_equal(normal.maps[-1][j],maps[key][i]):all_same=False
        normal.close('same-input estimated capture-transform replay; validated against recorded map')
        ideal.close('same raw depth/order/classifier; ideal physical capture transform, diagnostic only')
        row=dict(case=name,frames=len(events['commit']),estimated_replay_equals_recorded_maps=all_same,
                 ordinary_navigation_input_changed=False)
        assert all_same,row
        results.append(row);print(row,flush=True)
    dump(OUT/'pose_replay_comparison.json',results)


if __name__=='__main__':main()
