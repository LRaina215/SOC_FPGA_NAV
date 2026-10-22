"""Known-terrain transition checks with an independent disk/rectangle oracle.

Fully observed cell maxima model quantized measurement support, not LiDAR
observability. Claims are limited to these known positive/negative transitions.
"""
import json
import math
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'model'))
import nav_fixed as nf
import trav_v3


def main():
    res=65536/1311
    idx=np.arange(128)-64
    X,Y=np.meshgrid(idx*res,idx*res)
    # Exact cell support: a high surface is represented in every intersected
    # cell, including fractional phases, so no ideal center sampling loophole.
    results=[]
    for height in (21,25,30,31,75,150,300):
        for phase in (0,10,25,49):
            for angle in (0,15,37,90):
                for drop in (False,True):
                    edge=500+phase
                    nx,ny=math.cos(math.radians(angle)),math.sin(math.radians(angle))
                    A=X*nx+Y*ny
                    high=(A<edge) if drop else (A+res*(nx+ny)>edge)
                    H=np.where(high,height,0)
                    em=nf.ElevMap()
                    for wy,iy in enumerate(idx):
                        for wx,ix in enumerate(idx):
                            a=((int(iy)&127)<<7)|(int(ix)&127)
                            em.zmin[a]=em.zmax[a]=H[wy,wx];em.cnt[a]=1
                            em.tag[a]=(((int(iy)>>7)&15)<<4)|((int(ix)>>7)&15)
                    _,lethal,_=trav_v3.trav_v3(em,0,0,dict(infl_r=4,step_high_only=1))
                    yy,xx=np.nonzero(lethal); lx=(xx-64)*res;ly=(yy-64)*res
                    # Approach from the valid side in 5mm increments; independent
                    # first geometric encounter is center+radius >= edge.
                    failures=0; first_reject=None; first_contact=None
                    for along in np.arange(edge-500,edge+10,5):
                        px,py=along*nx,along*ny
                        gapx=np.maximum(np.maximum(lx-px,px-(lx+res)),0)
                        gapy=np.maximum(np.maximum(ly-py,py-(ly+res)),0)
                        reject=bool(np.any(gapx*gapx+gapy*gapy<=210**2))
                        contact=along+210>=edge
                        if reject and first_reject is None:first_reject=float(along)
                        if contact and first_contact is None:first_contact=float(along)
                        if contact and not reject:failures+=1
                    assert failures==0,(height,phase,angle,drop,failures)
                    results.append(dict(height_mm=height,phase_mm=phase,angle_deg=angle,drop=drop,
                                        first_reject_mm=first_reject,first_contact_mm=first_contact,
                                        unsafe_transition_accepts=failures))
    # Known flat surfaces below/at the assumed capability must remain usable.
    for height in (0,15,20):
        em=nf.ElevMap()
        for wy,iy in enumerate(idx):
            for wx,ix in enumerate(idx):
                a=((int(iy)&127)<<7)|(int(ix)&127)
                em.zmin[a]=em.zmax[a]=height if X[wy,wx]+res>500 else 0
                em.cnt[a]=1;em.tag[a]=(((int(iy)>>7)&15)<<4)|((int(ix)>>7)&15)
        _,lethal,_=trav_v3.trav_v3(em,0,0,dict(infl_r=4,step_high_only=1))
        assert not lethal.any(),height
    report=dict(pass_=True,cases=len(results),unsafe_transition_accepts=0,
                capability_assumption_mm=20,low_step_pass_cases=3,
                scope='fully observed positive/negative steps at 0/15/37/90 degrees; 210mm disk',
                excluded=['unobserved thin obstacles','sensor/mount error','vehicle physical capability'],results=results)
    out=Path(__file__).resolve().parents[1]/'build/nav_opt/t1_small_step_boundary_contract.json'
    out.write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='results'}))


if __name__=='__main__':main()
