"""Correct ray source or classifier on exactly the saved physical trajectory."""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import mujoco as mj
from terrain_scene import TerrainSpec,TerrainScene,PhysicsRayWorld,S
from terrain_study import MapRecorder,dump


def replay(path,label='maps_v2',noise=4.,cfg=None):
    r=json.loads((path/'result.json').read_text(encoding='utf-8'));sc=TerrainScene(TerrainSpec(**r['scene']['spec']))
    out=path/label;out.mkdir(exist_ok=False)
    m=mj.MjModel.from_xml_path(str(path/'assets/robot.xml'));d=mj.MjData(m)
    sc.w=PhysicsRayWorld(sc,m,d,mj);q=np.load(path/'full_state.npz');rec=MapRecorder(sc,out,noise=noise,seed=r['seed'],cfg=cfg)
    base=m.body('base').id
    for t in np.arange(.1,q['t'][-1]+.00001,.1):
        i=min(len(q['t'])-1,int(np.searchsorted(q['t'],t-1e-8)))
        d.qpos[:]=q['qpos'][i];d.qvel[:]=q['qvel'][i];mj.mj_forward(m,d)
        R=S@d.xmat[base].reshape(3,3)@S.T;c=S@(d.xpos[base]+d.xmat[base].reshape(3,3)@[-.08,-.006656542,0])
        p=c-R@[0,0,.1305];yaw=np.arctan2(R[1,0],R[0,0])
        bot=SimpleNamespace(x=p[0],y=p[1],physical_z=p[2],yaw=yaw,body_pose=lambda:(R,p))
        rec.capture(bot,float(t))
    rec.close('ideal-pose diagnostic; corrected geometry rays on exact saved physical states')
    dump(out/'replay.json',dict(physical_source=str(path),physical_integration_changed=False,
        raycast=sc.w.raycast_version,noise_mm=noise,seed=r['seed'],classifier_overrides=cfg or {},
        source_v1_valid_for_map_comparison=False,source_v1_valid_for_physical_trajectory=True))
    print(str(out),flush=True)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('paths',nargs='+',type=Path);ap.add_argument('--label',default='maps_v2');ap.add_argument('--noise',type=float,default=4.)
    ap.add_argument('--cfg',default='');a=ap.parse_args();cfg={k:int(v) for k,v in (s.split('=') for s in filter(None,a.cfg.split(',')))}
    for p in a.paths:
        trials=[p] if (p/'result.json').exists() else [x.parent for x in sorted(p.rglob('result.json'))]
        for trial in trials:
            if not (trial/a.label/'replay.json').exists():replay(trial,a.label,a.noise,cfg)


if __name__=='__main__':main()
