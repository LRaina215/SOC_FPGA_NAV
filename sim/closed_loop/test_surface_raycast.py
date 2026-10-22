"""Surface first-hit checks against independent analytic plane/AABB geometry."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'model'))
import sim_world as sw


def oracle(origin,dirs,boxes,tmax):
    out=np.full(len(dirs),np.inf)
    down=dirs[:,2]<0
    out[down]=-origin[2]/dirs[down,2]
    for lo,hi in boxes:
        enter=np.full(len(dirs),-np.inf);leave=np.full(len(dirs),np.inf)
        valid=np.ones(len(dirs),bool)
        for k in range(3):
            nz=dirs[:,k]!=0
            a=np.full(len(dirs),-np.inf);b=np.full(len(dirs),np.inf)
            a[nz]=(lo[k]-origin[k])/dirs[nz,k];b[nz]=(hi[k]-origin[k])/dirs[nz,k]
            valid &= nz | ((origin[k]>=lo[k]) & (origin[k]<=hi[k]))
            enter=np.maximum(enter,np.minimum(a,b));leave=np.minimum(leave,np.maximum(a,b))
        t=np.maximum(enter,0);valid &= leave>=t
        out=np.minimum(out,np.where(valid,t,np.inf))
    return np.where((out>=0)&(out<=tmax),out,np.inf)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    a.out.mkdir(parents=True,exist_ok=False);rng=np.random.default_rng(7134)
    world=sw.World(size_m=2,res=.01);boxes=[]
    for x0,x1,y0,y1,z in [(110,111,85,115,.25),(115,125,118,123,.08),(70,80,70,83,.021)]:
        world.h[y0:y1,x0:x1]=z
        boxes.append((np.array([world.o+x0*.01,world.o+y0*.01,0]),
                      np.array([world.o+x1*.01,world.o+y1*.01,float(np.float32(z))])))
    maximum=0.;n=0
    for origin in [np.array([0.,0.,.155]),np.array([-.05,.05,.155]),np.array([0.,0.,.08])]:
        dirs=rng.normal(size=(1024,3));dirs/=np.linalg.norm(dirs,axis=1)[:,None]
        dirs=np.vstack([dirs,[[1,0,0],[0,1,0],[0,0,-1],[0,0,1],[1,1,0]]])
        dirs/=np.linalg.norm(dirs,axis=1)[:,None]
        want=oracle(origin,dirs,boxes,.6);got=world.raycast(origin,dirs,tmax=.6,step=.015)
        assert np.array_equal(np.isfinite(want),np.isfinite(got))
        err=np.abs(want[np.isfinite(want)]-got[np.isfinite(got)])
        maximum=max(maximum,float(err.max(initial=0)));assert maximum<1e-10
        n+=len(dirs)
    flat=sw.World(size_m=2,res=.01)
    dirs=rng.normal(size=(2048,3));dirs[:,2]=-np.abs(dirs[:,2]);dirs/=np.linalg.norm(dirs,axis=1)[:,None]
    got=flat.raycast([0,0,.155],dirs,tmax=.6);want=np.where(.155/-dirs[:,2]<=.6,.155/-dirs[:,2],np.inf)
    assert np.array_equal(np.isfinite(got),np.isfinite(want))
    assert np.allclose(got[np.isfinite(got)],want[np.isfinite(want)],atol=1e-12)
    # An oblique 1cm wall lies entirely between old 15mm march samples.
    origin=np.array([.096,0,.1]);direction=np.array([[1.,0,0]])
    got=world.raycast(origin,direction,tmax=.03)[0]
    assert abs(got-.004)<1e-12
    report=dict(scope=__doc__,analytic_rays=n+len(dirs)+1,max_error_m=maximum,
                flat_surface_bias_m=0,thin_wall_first_hit_m=float(got),sensor_noise_mm=4.0,
                note='4mm noise unchanged in capture; exact intersection of the existing 1cm heightfield, not a real sensor calibration')
    (a.out/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))


if __name__=='__main__':main()
