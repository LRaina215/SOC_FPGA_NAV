"""Read-only C interval guard queries on recorded maps, with geometry labels."""
import ctypes
import json
from pathlib import Path
import subprocess
import numpy as np
from terrain_study import OUT,ROOT,dump


def main():
    out=OUT/'support_audit';out.mkdir(exist_ok=False)
    c=out/'fixture.c';c.write_text('''#include "support_guard.h"
static const short *h,*rough;static const unsigned char *v,*ages,*old;static int cx,cy;
bool hal_map_support(int32_t x,int32_t y,hal_support_t *s) {
 int ix=(int)(((int64_t)x*1311)>>16)-cx+64,iy=(int)(((int64_t)y*1311)>>16)-cy+64;
 if(ix<0||iy<0||ix>=128||iy>=128) return false;int i=iy*128+ix;if(!v[i])return false;
 *s=(hal_support_t){.lo=h[i]-rough[i],.hi=h[i],.seq=7,.count=1,.age=ages[i],.retained_stale=old[i]};return true;
}
__declspec(dllexport) int probe(const short *a,const short *b,const unsigned char *cc,
 const unsigned char *d,const unsigned char *e,int gx,int gy,int px,int py,int yaw,int vx,int wz) {
 h=a;rough=b;v=cc;ages=d;old=e;cx=gx;cy=gy;support_reset(0);
 nav_pose_t p={.x=px,.y=py,.yaw=yaw};nav_cmd_t cmd={vx,0,wz,0};
 lp_cfg_t cfg={.dt_ms=100,.safe_steps=12};hal_map_info_t mi={.seq=7,.inv_res=1311};
 return support_check(&p,&cfg,&mi,&cmd);
}
''')
    fw=OUT/'frozen/AS';dll=out/'guard.dll'
    subprocess.run(['gcc','-O2','-shared','-std=c99','-o',str(dll)]+['-I'+str(fw/d) for d in ['include','nav','hal','app']]+[str(c),str(fw/'nav/support_guard.c'),str(fw/'nav/fixmath.c')],check=True)
    d=ctypes.CDLL(str(dll));d.probe.argtypes=[ctypes.c_void_p]*5+[ctypes.c_int]*7;d.probe.restype=ctypes.c_int
    rows=[]
    for group,name in [('primary','flat'),('primary','up8'),('primary','step20'),('primary_recovered','trench40d150'),('primary_recovered','trench100d150')]:
        p=OUT/group/name;m=np.load(p/'maps_v2/map_frames.npz');r=np.load(p/'maps_v2/raw_frames.npz');n=len(m['t'])
        for i in range(n):
            if not 3.<=m['t'][i]<=7.:continue
            arrays=[np.ascontiguousarray(m[k][i],dtype=np.int16 if k in ['H','RO'] else np.uint8) for k in ['H','RO','V','age','stale']]
            x,y,z,yaw=r['truth'][i]
            for vx in [25,50,200,500]:
                result=d.probe(*[a.ctypes.data for a in arrays],int(m['cx'][i]),int(m['cy'][i]),round(x*1000),round(y*1000),round(yaw*65536/(2*np.pi))&65535,vx,0)
                rows.append(dict(case=name,t=float(m['t'][i]),x=float(x),y=float(y),vx=vx,support_result=result,
                    note='C support check only; body guard, candidate selection and actual stopping not inferred'))
    dump(out/'queries.json',rows)
    for name in sorted(set(r['case'] for r in rows)):
        a=[r for r in rows if r['case']==name];print(name,{str(s):sum(r['support_result']==s for r in a) for s in [0,1,2,3]},flush=True)


if __name__=='__main__':main()
