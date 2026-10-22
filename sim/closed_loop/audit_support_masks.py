"""Real candidate C with16bit subregion masks; no scene truth in callbacks."""
import ctypes
from pathlib import Path
import subprocess
import numpy as np
from terrain_study import OUT,dump
from fpga_model import FpgaModel
import nav_fast as fast
from support_coverage import SupportCoverage,CompactCoverage


def build(out):
    c=out/'fixture.c';c.write_text('''#include "support_guard.h"
static const short *h,*rough;static const unsigned char *v;static const unsigned short *mask,*fresh;static int cx,cy;
bool hal_map_support(int32_t x,int32_t y,hal_support_t *s) {
 int ix=(int)(((int64_t)x*1311)>>16)-cx+64,iy=(int)(((int64_t)y*1311)>>16)-cy+64;
 if(ix<0||iy<0||ix>=128||iy>=128)return false;int a=iy*128+ix;if(!v[a])return false;
 int mx=ix-48,my=iy-48;unsigned m=0,f=0;
 if(mx>=0&&my>=0&&mx<32&&my<32){m=mask[my*32+mx];f=fresh[my*32+mx];}
 *s=(hal_support_t){.lo=h[a]-rough[a],.hi=h[a],.seq=7,.count=1,.age=1,.coverage=m,.retained_stale=(m&~f)!=0};return true;
}
__declspec(dllexport) int probe(const short *a,const short *b,const unsigned char *c,const unsigned short *d,const unsigned short *e,
 int gx,int gy,int px,int py,int yaw,int vx,int wz,int boot) {
 h=a;rough=b;v=c;mask=d;fresh=e;cx=gx;cy=gy;support_reset(boot);
 nav_pose_t p={.x=px,.y=py,.yaw=yaw};nav_cmd_t cmd={vx,0,wz,0};lp_cfg_t cfg={.dt_ms=100,.safe_steps=12};hal_map_info_t mi={.seq=7,.inv_res=1311};return support_check(&p,&cfg,&mi,&cmd);
}
''')
    fw=OUT/'n2av/AV_source/fw';dll=out/'mask_guard.dll'
    subprocess.run(['gcc','-O2','-shared','-std=c99','-o',str(dll)]+['-I'+str(fw/d) for d in ['include','nav','hal','app']]+[str(c),str(fw/'nav/support_guard.c'),str(fw/'nav/fixmath.c')],check=True)
    d=ctypes.CDLL(str(dll));d.probe.argtypes=[ctypes.c_void_p]*5+[ctypes.c_int]*8;d.probe.restype=ctypes.c_int;return d


def main():
    out=OUT/'n2av/mask_probe';out.mkdir(exist_ok=False);dll=build(out);allrows=[]
    for group,name in [('primary','flat'),('primary','up8'),('primary','step5'),('primary_recovered','trench40d150'),('primary_recovered','trench100d150')]:
        p=OUT/group/name;m=np.load(p/'maps_v2/map_frames.npz');r=np.load(p/'maps_v2/raw_frames.npz');fm=FpgaModel()
        models={'exact':SupportCoverage(subdiv=4,min_frames=2),'compact9':CompactCoverage()};rows=[]
        for i,t in enumerate(r['t']):
            fm.frame(r['depth'][i]);points=fast.pc_transform(fm.xyz,np.rint(r['R'][i]*16384).astype(np.int64),np.rint(r['p'][i]*1000).astype(np.int64));valid=fm.pv&(points[:,2]>=-1000)&(points[:,2]<=2000);now=round(t*1e6)
            for label,cov in models.items():
                cov.update(points,valid,int(m['cx'][i]),int(m['cy'][i]),now,now)
                if not 2.<=t<=7.:continue
                masks,fresh=cov.mask_windows(now);arrays=[np.ascontiguousarray(m[k][i],dtype=np.int16 if k in ['H','RO'] else np.uint8) for k in ['H','RO','V']]+[masks,fresh]
                x,y,z,yaw=r['truth'][i]
                for vx in [25,50,200,500]:
                    result=dll.probe(*[a.ctypes.data for a in arrays],int(m['cx'][i]),int(m['cy'][i]),round(x*1000),round(y*1000),round(yaw*65536/(2*np.pi))&65535,vx,0,0)
                    rows.append(dict(case=name,model=label,t=float(t),vx=vx,result=result,x=float(x)))
        dump(out/(name+'.json'),rows);allrows+=rows
        print(name,{s:{k:sum(q['model']==s and q['result']==k for q in rows) for k in [0,1,2]} for s in models},flush=True)
    dump(out/'summary.json',allrows)


if __name__=='__main__':main()
