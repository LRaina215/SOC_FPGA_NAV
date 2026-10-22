"""Same-input prototype gate before changing any firmware/RTL data path."""
import json
from pathlib import Path
import numpy as np
from fpga_model import FpgaModel
import nav_fast as fast
from vertical_overlap import estimate
from terrain_study import OUT,dump


def main():
    p=OUT/'n2av/navigation/AY_trace_up8';out=OUT/'n2av/n2az';out.mkdir(exist_ok=True)
    load=lambda name:dict(np.load(p/name))
    raw=load('raw_frames.npz');maps=load('map_frames.npz');events=load('nav_events.npz')
    cfg=json.loads((p/'manifest.json').read_text(encoding='utf-8'))['terrain_cfg']
    normal,trial=FpgaModel(cfg=cfg),FpgaModel(cfg=cfg);correction_um=0;rows=[]
    for i,commit in enumerate(events['commit']):
        normal.frame(raw['depth'][i]);normal.commit(commit[:12],*commit[12:])
        for key,value in [('H',normal.view[2]),('V',normal.view[1]),('RO',normal.view[3])]:assert np.array_equal(value,maps[key][i]),(i,key)
        trial.frame(raw['depth'][i]);rt=np.array(commit[:12],dtype=np.int64);rt[11]+=int(correction_um/1000)
        points=fast.pc_transform(trial.xyz,rt[:9].reshape(3,3),rt[9:]);zlo,zhi=int(commit[14]),int(commit[15]);valid=trial.pv&(points[:,2]>=zlo+int(correction_um/1000))&(points[:,2]<=zhi+int(correction_um/1000))
        proposal=estimate(trial,points,valid,rt)
        if proposal['ok']:correction_um+=proposal['correction_um']
        rt[11]=int(commit[11])+int(correction_um/1000);trial.commit(rt,*commit[12:14],zlo+int(correction_um/1000),zhi+int(correction_um/1000))
        # Truth is used only after the proposal, to score translation accuracy.
        target=float(raw['p'][i,2])*1000-int(commit[11])
        rows.append(dict(t=float(maps['t'][i]),target_delta_mm=target,extra_correction_mm=correction_um/1000,proposal=proposal))
    target=np.array([r['target_delta_mm'] for r in rows]);corrected=np.array([r['extra_correction_mm'] for r in rows])
    result=dict(frames=len(rows),baseline_maps_reproduced=True,accepted=False,quality_gate='same-input exploratory estimator; not firmware/closed-loop acceptance',
        baseline_translation_z_mae_mm=float(np.mean(abs(target))),prototype_translation_z_mae_mm=float(np.mean(abs(target-corrected))),
        baseline_translation_z_max_mm=float(np.max(abs(target))),prototype_translation_z_max_mm=float(np.max(abs(target-corrected))),
        final_extra_correction_mm=float(corrected[-1]),rows=rows,
        boundary='same capture/XY/orientation and original anchor trajectory; causal fresh-map estimator; no truth in correction; old anchor changes would interact in a real closed loop')
    dump(out/'vertical_overlap_audit.json',result);print({k:v for k,v in result.items() if k not in ['rows','boundary']})


if __name__=='__main__':main()
