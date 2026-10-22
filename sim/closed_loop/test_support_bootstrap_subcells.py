"""Declared350mm disk: independent1ms physical tread vs candidate bootstrap."""
import ctypes
import math
import subprocess
import numpy as np
from terrain_study import OUT,dump


def build(out,label):
    fw=OUT/'n2av'/f'{label}_source/fw';src=out/(label+'.c')
    src.write_text('''#include "support_guard.h"
bool hal_map_support(int32_t x,int32_t y,hal_support_t *s) {(void)x;(void)y;(void)s;return false;}
__declspec(dllexport) int probe(int x,int y,int yaw,int vx,int wz) {
 support_reset(350);hal_map_info_t mi={.seq=7,.inv_res=1311};lp_cfg_t cfg={.dt_ms=100,.safe_steps=12};
 nav_pose_t origin={0};nav_cmd_t init={25,0,0,0};if(support_check(&origin,&cfg,&mi,&init))return -1;
 nav_pose_t p={.x=x,.y=y,.yaw=yaw};nav_cmd_t cmd={vx,0,wz,0};return support_check(&p,&cfg,&mi,&cmd);
}
''')
    dll=out/(label+'.dll');subprocess.run(['gcc','-O2','-std=c99','-shared','-o',str(dll)]+['-I'+str(fw/d) for d in ['include','nav','hal','app']]+[str(src),str(fw/'nav/support_guard.c'),str(fw/'nav/fixmath.c')],check=True)
    d=ctypes.CDLL(str(dll));return d


def main():
    out=OUT/'n2av/bootstrap_contract';out.mkdir(exist_ok=False);old,new=build(out,'AW'),build(out,'AX');rng=np.random.default_rng(91);rows=[]
    t=np.arange(1201)*.001
    for i in range(400):
        x,y=rng.integers(-260,261,2);yaw=int(rng.integers(0,65536));v=int(rng.choice([25,50]));w=int(rng.choice([-150,0,150]));theta=yaw*2*np.pi/65536;angle=theta+w/1000*t
        if w:cx=x+v/(w/1000)*(np.sin(angle)-math.sin(theta));cy=y-v/(w/1000)*(np.cos(angle)-math.cos(theta))
        else:cx=x+v*t*math.cos(theta);cy=y+v*t*math.sin(theta)
        maximum=0.
        for lateral in [-135,135]:
            for u in [-34,34]:
                for b in [-14,14]:
                    xx=cx+u*np.cos(angle)-(lateral+b)*np.sin(angle);yy=cy+u*np.sin(angle)+(lateral+b)*np.cos(angle);maximum=max(maximum,float(np.hypot(xx,yy).max()))
        a,b=old.probe(int(x),int(y),yaw,v,w),new.probe(int(x),int(y),yaw,v,w)
        row=dict(x=int(x),y=int(y),yaw=yaw,v=v,w=w,reference_max_radius_mm=maximum,old=a,new=b);rows.append(row)
        assert b!=0 or maximum<=350+1e-6,row
    dump(out/'result.json',dict(rows=rows,false_accepts=0,old_accepts=sum(r['old']==0 for r in rows),new_accepts=sum(r['new']==0 for r in rows),newly_accepted=sum(r['old']!=0 and r['new']==0 for r in rows),radius_unchanged_mm=350))
    print(dict(cases=len(rows),old=sum(r['old']==0 for r in rows),new=sum(r['new']==0 for r in rows),false_accepts=0),flush=True)


if __name__=='__main__':main()
