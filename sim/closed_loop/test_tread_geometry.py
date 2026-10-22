"""C swept tire admission versus independent floating polygon clipping.

The single missing cell is reported as unknown by the test HAL. A true tire
projection intersecting it must never be accepted. No terrain truth is used by
the navigation. Fixed 1ms oracle samples give finite evidence, not a proof.
"""
import argparse
import ctypes
import json
import math
from pathlib import Path
import subprocess
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
RES=65536/1311


def overlaps(poly,x0,y0,x1,y1):
    for axis,bound,sign in ((0,x0,1),(0,x1,-1),(1,y0,1),(1,y1,-1)):
        if not poly:return False
        out=[];a=poly[-1];ina=sign*(a[axis]-bound)>=-1e-9
        for b in poly:
            inb=sign*(b[axis]-bound)>=-1e-9
            if ina!=inb:
                t=(bound-a[axis])/(b[axis]-a[axis])
                out.append((a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])))
            if inb:out.append(b)
            a,ina=b,inb
        poly=out
    return bool(poly)


def oracle(x,y,yaw,v,w,gx,gy):
    t=np.linspace(0,.2 if v==0 else 1.2,201 if v==0 else 1201)
    a0=yaw*math.tau/65536;a=a0+w/1000*t;c,s=np.cos(a),np.sin(a)
    if w:
        xx=x+v/(w/1000)*(s-math.sin(a0));yy=y-v/(w/1000)*(c-math.cos(a0))
    else:xx=x+v*math.cos(a0)*t;yy=y+v*math.sin(a0)*t
    x0,y0,x1,y1=gx*RES,gy*RES,(gx+1)*RES,(gy+1)*RES
    for side in (-135,135):
        wx,wy=xx-side*s,yy+side*c
        dx=np.maximum(0,np.maximum(x0-wx,wx-x1));dy=np.maximum(0,np.maximum(y0-wy,wy-y1))
        for i in np.flatnonzero(dx*dx+dy*dy<=37**2):
            poly=[(wx[i]+u*c[i]-z*s[i],wy[i]+u*s[i]+z*c[i]) for u,z in ((-34,-14),(34,-14),(34,14),(-34,14))]
            if overlaps(poly,x0,y0,x1,y1):return True
    return False


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--source',type=Path,default=ROOT/'fw')
    ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    c=a.out/'probe.c';c.write_text('''#include "support_guard.c"
static int hole_x,hole_y;
bool hal_map_support(int32_t x,int32_t y,hal_support_t *s) {
 int ix=(int32_t)(((int64_t)x*1311)>>16),iy=(int32_t)(((int64_t)y*1311)>>16);
 if(ix==hole_x && iy==hole_y) return false;
 *s=(hal_support_t){.lo=0,.hi=0,.seq=7,.count=1};return true;
}
__declspec(dllexport) int probe(int x,int y,int yaw,int vx,int wz,int hx,int hy) {
 support_reset(0);hole_x=hx;hole_y=hy;
 lp_cfg_t cfg={.safe_steps=12,.dt_ms=100};nav_pose_t p={.x=x,.y=y,.yaw=yaw};
 hal_map_info_t mi={.seq=7,.inv_res=1311,.res_mm=50};nav_cmd_t cmd={vx,0,wz,0};
 return support_check(&p,&cfg,&mi,&cmd);
}
''',encoding='utf-8')
    dll=(a.out/'probe.dll').resolve();fw=a.source.resolve()
    subprocess.run(['gcc','-std=c99','-O2','-shared','-o',str(dll),
        *['-I'+str(fw/d) for d in ('include','hal','nav')],str(c),str(fw/'nav/fixmath.c')],check=True)
    lib=ctypes.CDLL(str(dll));cases=hits=accept=conservative=0;misses=[]
    rng=np.random.default_rng(1701)
    # Deliberate edge phases plus deterministic path-near cells.
    # Append new speeds so the original deterministic cases keep their RNG draws.
    for v,w in ((50,0),(50,150),(50,-150),(125,600),(375,-600),(500,1200),(-150,0),(0,600),(25,0),(25,150),(25,-150)):
        for yaw in (0,1,8192,16384,32767,49152):
            for phase in (0,24,49,-25):
                aa=yaw*math.tau/65536;ts=.1 if v==0 else float(rng.uniform(0,1.2))
                angle=aa+w/1000*ts
                if w:cx=phase+v/(w/1000)*(math.sin(angle)-math.sin(aa));cy=-v/(w/1000)*(math.cos(angle)-math.cos(aa))
                else:cx=phase+v*math.cos(aa)*ts;cy=v*math.sin(aa)*ts
                side=int(rng.choice([-135,135]));cx-=side*math.sin(angle);cy+=side*math.cos(angle)
                hx=int(math.floor(cx/RES))+int(rng.integers(-1,2));hy=int(math.floor(cy/RES))+int(rng.integers(-1,2))
                hit=oracle(phase,0,yaw,v,w,hx,hy);status=lib.probe(phase,0,yaw,v,w,hx,hy)
                cases+=1;hits+=hit;accept+=status==0;conservative+=not hit and status!=0
                if hit and status==0:misses.append(dict(v=v,w=w,yaw=yaw,phase=phase,hole=[hx,hy]))
    r=dict(cases=cases,oracle_intersections=hits,accepted=accept,conservative_reject=conservative,
           false_accept=len(misses),misses=misses,scope='finite 1ms exact-twist polygon oracle, nominal motion only')
    (a.out/'result.json').write_text(json.dumps(r,indent=2));print(json.dumps(r));assert not misses


if __name__=='__main__':main()
