"""Read-only counterfactual on saved frame maps; truth is diagnostic only."""
import argparse
import json
import math
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'model'))
import nav_fixed as nf
import trav_v3
import world as W


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--frames',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    scene=W.scene_field5();rows=[];res=65536/1311
    for f in sorted(a.frames.glob('*.npz')):
        with np.load(f) as d:
            p=d['diag'];cx,cy=int(d['fpga_cx']),int(d['fpga_cy'])
            maps=[]
            for high_only in (0,1):
                em=nf.ElevMap()
                for k in ['zmin','zmax','cnt','tag','stamp']:setattr(em,k,d['em_'+k].copy())
                em.fc=int(d['em_fc'])
                maps.append(trav_v3.trav_v3(em,cx,cy,dict(infl_r=4,step_high_only=high_only)))
            def near(item):
                _,lethal,view=item
                yy,xx=np.nonzero(lethal);lx=(xx+cx-64)*res;ly=(yy+cy-64)*res
                dist=np.hypot(np.maximum(np.maximum(lx-p[3],p[3]-(lx+res)),0),
                              np.maximum(np.maximum(ly-p[4],p[4]-(ly+res)),0))
                i=dist.argmin();ox=(lx[i]+res/2-p[3])/1000;oy=(ly[i]+res/2-p[4])/1000
                da=float(d['truth'][3])-int(p[6])*2*math.pi/65536
                wx=float(d['truth'][0])+math.cos(da)*ox-math.sin(da)*oy
                wy=float(d['truth'][1])+math.sin(da)*ox+math.cos(da)*oy
                return dict(distance_mm=float(dist[i]),cell=[int(xx[i]),int(yy[i])],
                            height_mm=int(view[2][yy[i],xx[i]]),
                            world_center_m=[wx,wy],truth_height_mm=float(scene.height([wx],[wy])[0]*1000),
                            truth_label=int(scene.gt_lookup(np.array([wx]),np.array([wy]))[0]))
            rows.append(dict(file=f.name,now_us=int(p[0]),original=near(maps[0]),higher_side_only=near(maps[1]),
                             removed=int((maps[0][1]&~maps[1][1]).sum())))
    a.out.mkdir(parents=True,exist_ok=True)
    (a.out/'frames.json').write_text(json.dumps(rows,indent=2))
    report=dict(frames=len(rows),original_intrusions=sum(r['original']['distance_mm']<210 for r in rows),
                candidate_intrusions=sum(r['higher_side_only']['distance_mm']<210 for r in rows),
                scope='counterfactual on frozen G maps, not a closed-loop acceptance test')
    (a.out/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))


if __name__=='__main__':main()
