"""Counterfactual C guard probes on a saved final-map snapshot.

V/H/RO reconstruct valid height intervals, but do not contain original age/count
or the exact task cache generation. This is not a full causal replay. No scene
truth, invented support, or bootstrap declaration is passed into the guards.
"""
import argparse
import csv
import ctypes as ct
import json
import math
from pathlib import Path
import subprocess
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
WRAPPER = r'''
#include "support_guard.h"
#include "motion_guard.h"
static const uint8_t *valid;
static const int16_t *height,*rough;
static hal_map_info_t mi;
static int last[7];
bool hal_map_support(int32_t x,int32_t y,hal_support_t *s) {
 int ix=(int32_t)(((int64_t)x*1311)>>16),iy=(int32_t)(((int64_t)y*1311)>>16);
 int wx=ix-(mi.cx-64),wy=iy-(mi.cy-64),k=wy*128+wx;
 int ok=wx>=0 && wx<128 && wy>=0 && wy<128 && valid[k];
 last[0]=x;last[1]=y;last[2]=ix;last[3]=iy;last[4]=ok;
 last[5]=ok?height[k]-rough[k]:0;last[6]=ok?height[k]:0;
 if(!ok) return false;
 *s=(hal_support_t){.lo=height[k]-rough[k],.hi=height[k],.seq=7,.count=1};
 return true;
}
__declspec(dllexport) void setup(const uint8_t *v,const int16_t *h,const int16_t *r,int cx,int cy) {
 valid=v;height=h;rough=r;mi=(hal_map_info_t){7,cx,cy,50,1311};
}
__declspec(dllexport) void probe(const uint8_t *map,int x,int y,int yaw,int vx,int wz,int *out) {
 lp_cfg_t c={.caps={.radius_mm=210,.vx_max=600,.wz_max=1500},.safe_steps=12,.dt_ms=100};
 nav_pose_t p={.x=x,.y=y,.yaw=yaw};nav_cmd_t cmd={vx,0,wz,0};
 support_reset(0);out[0]=motion_check(map,&mi,&p,&c,&cmd);
 out[1]=support_check(&p,&c,&mi,&cmd);
 for(int i=0;i<7;i++) out[i+2]=last[i];
}
'''


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--source', type=Path, required=True)
    ap.add_argument('--batch', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    fw=a.source.resolve();c=a.out/'fixture.c';c.write_text(WRAPPER)
    dll=(a.out/'snapshot.dll').resolve()
    subprocess.run(['gcc','-std=c99','-O2','-shared','-o',str(dll),
        *['-I'+str(fw/d) for d in ['include','hal','nav']],str(c),
        *[str(fw/'nav'/n) for n in ['support_guard.c','motion_guard.c','fixmath.c']]],check=True)
    lib=ct.CDLL(str(dll));u8=ct.POINTER(ct.c_uint8);i16=ct.POINTER(ct.c_int16)
    lib.setup.argtypes=[u8,i16,i16,ct.c_int,ct.c_int]
    lib.probe.argtypes=[u8,*[ct.c_int]*5,ct.POINTER(ct.c_int)]
    rows=[]
    for p in sorted(a.batch.glob('*_s*')):
        if not (p/'final_maps.npz').exists():continue
        d=np.load(p/'final_maps.npz');r=list(csv.DictReader((p/'diagnostics/motion.csv').open()))[-1]
        x,y,yaw=[int(r[k]) for k in ['terrain_x','terrain_y','terrain_yaw']]
        v=np.ascontiguousarray(d['V'],dtype=np.uint8);h=np.ascontiguousarray(d['H'],dtype=np.int16)
        ro=np.ascontiguousarray(d['RO'],dtype=np.int16);cost=np.ascontiguousarray(d['cost'],dtype=np.uint8)
        cx,cy=int(d['cx']),int(d['cy']);lib.setup(v.ctypes.data_as(u8),h.ctypes.data_as(i16),ro.ctypes.data_as(i16),cx,cy)
        probes=[]
        for vx in [0,25,50,125,-50]:
            for wz in [-600,-150,-100,0,100,150,600]:
                if not vx and not wz:continue
                out=(ct.c_int*9)();lib.probe(cost.ctypes.data_as(u8),x,y,yaw,vx,wz,out)
                probes.append(dict(vx=vx,wz=wz,body=out[0],support=out[1],last_query=list(out)[2:]))
        tires=[]
        for side in [-135,135]:
            angle=yaw*math.tau/65536;wx=x-side*math.sin(angle);wy=y+side*math.cos(angle)
            ix,iy=math.floor(wx*1311/65536),math.floor(wy*1311/65536)
            cells=[]
            for gy in range(iy-1,iy+2):
                for gx in range(ix-1,ix+2):
                    tx,ty=gx-cx+64,gy-cy+64
                    if 0<=tx<128 and 0<=ty<128:
                        cells.append(dict(cell=[gx,gy],valid=int(v[ty,tx]),lo=int(h[ty,tx])-int(ro[ty,tx]),hi=int(h[ty,tx]),cost=int(cost[ty,tx])))
            tires.append(dict(side=side,xy=[wx,wy],cells=cells))
        rows.append(dict(case=p.name,pose=[x,y,yaw],tires=tires,probes=probes))
    result=dict(scope=__doc__,source=str(fw),batch=str(a.batch.resolve()),rows=rows)
    (a.out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    for r in rows:
        print(r['case'],'pose',r['pose'])
        print('tires',json.dumps(r['tires']))
        for q in r['probes']:print(q)


if __name__=='__main__':main()
