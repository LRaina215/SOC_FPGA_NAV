"""Capture-age/roll contracts and independent subcell/rectangle geometry."""
import argparse
import ctypes
import math
from pathlib import Path
import subprocess
import numpy as np
from terrain_study import OUT,dump
from support_coverage import SupportCoverage,CompactCoverage


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--fw',type=Path,default=OUT/'n2av/AV_source/fw');ap.add_argument('--out',type=Path,default=OUT/'n2av/coverage_contract');args=ap.parse_args()
    out=args.out;out.mkdir(exist_ok=False);rows=[]
    def check(name,condition):
        rows.append(dict(name=name,passed=bool(condition)));assert condition,name
    P=np.array([[625,0,0]],dtype=np.int64);V=np.array([True])
    c=CompactCoverage()
    def cls(t):return int(c.view(t)[0][64,66]) # fine x50, center48, y0
    c.update(P,V,12,0,100_000,100_000);check('first epoch provisional',cls(100_000)==0)
    c.update(P,V,12,0,150_000,150_000);check('same epoch not independent confirmation',cls(150_000)==0)
    c.update(P,V,12,0,200_000,200_000);check('second epoch confirms',cls(200_000)==1)
    before=c.word.copy();c.update(P,V,12,0,150_000,250_000);check('older capture does not replace newer',np.array_equal(before,c.word))
    check('freshness conservative wall clock',cls(1_200_001)==2)
    check('remembers static evidence explicitly',cls(19_900_000)==2)
    check('no frames does not freeze age',cls(20_200_001)==0)
    check('epoch wrap cannot resurrect',cls(26_000_000)==0)
    c=CompactCoverage();c.update(P,V,12,0,100_000,100_000);c.update(P,V,12,0,200_000,200_000)
    c.update(P,V,60,0,300_000,300_000);c.update(np.empty((0,3),dtype=np.int64),np.empty(0,bool),12,0,400_000,400_000)
    check('rolled-away and revisited tile stays unknown',cls(400_000)==0)
    c=CompactCoverage();c.update(P,V,12,0,0xffff0000,0xffff0000);c.update(P,V,12,0,0xfffff000,0xfffff000)
    check('32bit timer wrap clears',cls(100_000)==0)
    c.update(P,V,12,0,0xfffff000,150_000);check('pre-wrap delayed capture rejected',cls(150_000)==0)
    c=CompactCoverage();c.update(P,V,12,0,100_000,900_000);c.update(P,V,12,0,200_000,1_000_000)
    check('capture time not fusion time for age',cls(1_200_001)==2)
    # Real mask C static patch in the same translation unit, not a reimplementation.
    fw=args.fw;src=out/'patch.c'
    src.write_text('''#include "support_guard.h"
uint32_t hal_time_us(void) {return 0;}
static int tx,ty,missing;
bool hal_map_support(int32_t x,int32_t y,hal_support_t *s) {
 int ix=(int)(((int64_t)x*1311)>>16),iy=(int)(((int64_t)y*1311)>>16);
 *s=(hal_support_t){.lo=0,.hi=0,.seq=7,.count=1,.coverage=(ix==tx&&iy==ty)?(0xffffu^(1u<<missing)):0xffff};return true;
}
#include "support_guard.c"
__declspec(dllexport) int probe(int x,int y,int sn,int cs,int gx,int gy,int sub) {
 tx=gx;ty=gy;missing=sub;support_reset(0);query_t q={.seq=7,.inv=1311};cell_t center;
 return patch(&q,x,y,36,16,sn,cs,&center);
}
__declspec(dllexport) int cache_size(void) {return sizeof(cell_t);}
''')
    dll=out/'patch.dll';subprocess.run(['gcc','-O2','-shared','-std=c99','-o',str(dll)]+['-I'+str(fw/d) for d in ['include','nav','hal','app']]+[str(src),str(fw/'nav/fixmath.c')],check=True)
    d=ctypes.CDLL(str(dll));check('cell cache entry remains16bytes',d.cache_size()==16)
    rng=np.random.default_rng(63);geom=[];res=65536/1311
    for i in range(700):
        x,y=rng.integers(-180,181,2);theta=rng.uniform(-math.pi,math.pi);s,c=math.sin(theta),math.cos(theta)
        # Missing quarter selected near a real footprint, not always an easy far tile.
        center=np.array([x,y])+rng.uniform(-65,65,2)
        gx,gy=np.floor(center/res).astype(int);sx,sy=rng.integers(0,4,2)
        lo=np.array([gx+sx/4,gy+sy/4])*res;hi=lo+res/4
        poly=np.array([[x+u*c-v*s,y+u*s+v*c] for u,v in [(-34,-14),(34,-14),(34,14),(-34,14)]])
        square=np.array([[lo[0],lo[1]],[hi[0],lo[1]],[hi[0],hi[1]],[lo[0],hi[1]]])
        intersect=True
        for axis in [np.array([1.,0]),np.array([0.,1]),np.array([c,s]),np.array([-s,c])]:
            a,b=poly@axis,square@axis
            if a.max()<b.min()-1e-8 or b.max()<a.min()-1e-8:intersect=False;break
        result=d.probe(int(x),int(y),round(s*16384),round(c*16384),int(gx),int(gy),int(sy*4+sx))
        check('no accepted physical tread intersection '+str(i),not intersect or result!=0)
        geom.append(dict(intersection=intersect,result=result))
    dump(out/'result.json',dict(firmware=str(fw.resolve()),contracts=rows,geometry=geom,reference='independent four-axis SAT, physical tread34x14 vs C36x16 margin',sampled_intersections=sum(g['intersection'] for g in geom),false_accepts=sum(g['intersection'] and g['result']==0 for g in geom)))
    print(dict(contracts=len(rows),intersections=sum(g['intersection'] for g in geom),false_accepts=0),flush=True)


if __name__=='__main__':main()
