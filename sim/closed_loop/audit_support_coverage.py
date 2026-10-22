"""Same recorded points, unchanged C guard: compare coarse/fine coverage.

Fine coverage adds only a requirement on observed subregions; heights still
come from the saved coarse interval map. No new scene truth reaches the guard.
"""
import argparse
import ctypes
import json
from pathlib import Path
import subprocess
import numpy as np
from terrain_study import ROOT,OUT,dump
from fpga_model import FpgaModel
import nav_fast as fast
from support_coverage import SupportCoverage


def build(out):
    c=out/'fixture.c';c.write_text('''#include "support_guard.h"
static const short *h,*rough;static const unsigned char *v,*fine;static int cx,cy,fcx,fcy,inv;
bool hal_map_support(int32_t x,int32_t y,hal_support_t *s) {
 int ix=(int)(((int64_t)x*1311)>>16)-cx+64,iy=(int)(((int64_t)y*1311)>>16)-cy+64;
 if(ix<0||iy<0||ix>=128||iy>=128)return false;int a=iy*128+ix;if(!v[a])return false;
 int fx=(int)(((int64_t)x*inv)>>16)-fcx+64,fy=(int)(((int64_t)y*inv)>>16)-fcy+64;
 if(fx<0||fy<0||fx>=128||fy>=128||!fine[fy*128+fx])return false;
 *s=(hal_support_t){.lo=h[a]-rough[a],.hi=h[a],.seq=7,.count=1,.age=1,.retained_stale=fine[fy*128+fx]==2};return true;
}
__declspec(dllexport) int probe(const short *a,const short *b,const unsigned char *c,const unsigned char *d,
 int gx,int gy,int fgx,int fgy,int res,int px,int py,int yaw,int vx,int wz,int boot) {
 h=a;rough=b;v=c;fine=d;cx=gx;cy=gy;fcx=fgx;fcy=fgy;inv=res;support_reset(boot);
 nav_pose_t p={.x=px,.y=py,.yaw=yaw};nav_cmd_t cmd={vx,0,wz,0};
 lp_cfg_t cfg={.dt_ms=100,.safe_steps=12};hal_map_info_t mi={.seq=7,.inv_res=res};return support_check(&p,&cfg,&mi,&cmd);
}
''')
    fw=OUT/'frozen/AS';dll=out/'coverage_guard.dll'
    subprocess.run(['gcc','-O2','-shared','-std=c99','-o',str(dll)]+['-I'+str(fw/d) for d in ['include','nav','hal','app']]+[str(c),str(fw/'nav/support_guard.c'),str(fw/'nav/fixmath.c')],check=True)
    d=ctypes.CDLL(str(dll));d.probe.argtypes=[ctypes.c_void_p]*4+[ctypes.c_int]*11;d.probe.restype=ctypes.c_int;return d


def evaluate(source,out,d,variants):
    m=np.load(source/'maps_v2/map_frames.npz');raw=np.load(source/'maps_v2/raw_frames.npz');fm=FpgaModel();rows=[];snaps={}
    covs={(sub,frames):SupportCoverage(subdiv=sub,min_frames=frames) for sub,frames in variants}
    for i,t in enumerate(raw['t']):
        fm.frame(raw['depth'][i]);R=np.rint(raw['R'][i]*16384).astype(np.int64);tr=np.rint(raw['p'][i]*1000).astype(np.int64)
        points=fast.pc_transform(fm.xyz,R,tr);valid=fm.pv&(points[:,2]>=-1000)&(points[:,2]<=2000)
        x,y,z,yaw=raw['truth'][i];now=round(t*1e6)
        for key,cov in covs.items():
            cov.update(points,valid,int(m['cx'][i]),int(m['cy'][i]),now,now)
            cls,age,cnt,_=cov.view(now)
            if not 2.<=t<=7.:continue
            arrays=[np.ascontiguousarray(m[k][i],dtype=np.int16 if k in ['H','RO'] else np.uint8) for k in ['H','RO','V']]+[cls]
            for vx in [25,50,200,500]:
                status=d.probe(*[a.ctypes.data for a in arrays],int(m['cx'][i]),int(m['cy'][i]),cov.cx,cov.cy,cov.inv,round(x*1000),round(y*1000),round(yaw*65536/(2*np.pi))&65535,vx,0,0)
                rows.append(dict(case=source.name,t=float(t),subdiv=key[0],min_frames=key[1],vx=vx,result=status,x=float(x),y=float(y)))
            if abs(t-4.4)<.001:snaps[f's{key[0]}_c{key[1]}']=dict(cls=cls.copy(),age=age.copy(),cnt=cnt.copy(),cx=cov.cx,cy=cov.cy,inv=cov.inv)
    dump(out/(source.name+'_queries.json'),rows)
    np.savez_compressed(out/(source.name+'_coverage.npz'),**{k+'_'+f:v for k,s in snaps.items() for f,v in s.items()})
    print(source.name,{str(k):{s:sum(r['subdiv']==k[0] and r['min_frames']==k[1] and r['result']==s for r in rows) for s in [0,1,2]} for k in variants},flush=True)
    return rows


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=OUT/'n2av/coverage_probe');a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    d=build(a.out);allrows=[]
    for group,case in [('primary','flat'),('primary','up8'),('primary','step5'),('primary_recovered','trench40d150'),('primary_recovered','trench100d150')]:
        allrows+=evaluate(OUT/group/case,a.out,d,[(1,1),(2,1),(4,1),(4,2),(8,2)])
    dump(a.out/'summary.json',dict(rows=allrows,method='exact same raw frames/poses/coarse intervals; unmodified AS C guard using requested finer resolution',not_vehicle_capability_proof=True))


if __name__=='__main__':main()
