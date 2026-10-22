"""Byte-reference vs bitset: exact camera visibility and frontier decisions."""
import ctypes
import subprocess
import numpy as np
from terrain_study import OUT,dump


def main():
    out=OUT/'n2av/seen_contract';out.mkdir(exist_ok=True);assert not(out/'result.json').exists();libs=[]
    for label,fw in [('byte',OUT/'frozen/AS'),('bits',OUT/'n2av/AX_source/fw')]:
        source=out/(label+'.c');get='explore_seen_get(&e,i)' if label=='bits' else 'e.seen[i]'
        source.write_text('''#include "explore.h"
#include <string.h>
static explore_t e;static gmap_t g;
__declspec(dllexport) void probe(const unsigned char *grid,const int *poses,int n,int fov,unsigned char *seen,int *answers) {
 memset(&e,0,sizeof e);gmap_init(&g);memcpy(g.grid,grid,GM_N*GM_N);g.init=true;g.ox=g.oy=-32;
 explore_init(&e);explore_cfg_t c;explore_default_cfg(&c);c.cam_fov=fov;
 hal_map_info_t mi={.seq=1,.inv_res=1311,.res_mm=50};
 for(int k=0;k<n;k++) {
  nav_pose_t p={.x=poses[k*3],.y=poses[k*3+1],.yaw=poses[k*3+2]};int gx=0,gy=0;
  explore_seen_mark(&e,&c,&g,&mi,&p);int ok=explore_frontier(&e,&c,&g,&mi,&p,true,&gx,&gy);
  answers[k*5]=e.n_seen;answers[k*5+1]=e.n_frontier;answers[k*5+2]=ok;answers[k*5+3]=gx;answers[k*5+4]=gy;
  for(int i=0;i<GM_N*GM_N;i++)seen[k*GM_N*GM_N+i]='''+get+''';
 }
}
__declspec(dllexport) int seen_size(void){return sizeof e;}
''')
        dll=out/(label+'.dll');subprocess.run(['gcc','-O2','-shared','-std=c99','-o',str(dll)]+['-I'+str(fw/d) for d in ['include','nav','hal','app']]+[str(source)]+[str(fw/'nav'/name) for name in ['explore.c','gmap.c','fixmath.c','scratch.c','astar.c']],check=True)
        lib=ctypes.CDLL(str(dll));lib.probe.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_int,ctypes.c_int,ctypes.c_void_p,ctypes.c_void_p];libs.append(lib)
    rng=np.random.default_rng(23);rows=[]
    for i in range(180):
        grid=rng.choice(np.array([0,60,200,253,254,255],np.uint8),(64,64),p=[.65,.05,.03,.04,.12,.11]);grid[28:37,28:37]=0
        poses=rng.integers([-2300,-2300,0],[2301,2301,65536],(24,3),dtype=np.int32);fov=int(rng.choice([1,6000,12743,20000,32768,65535]))
        outputs=[]
        for lib in libs:
            seen=np.empty((24,4096),np.uint8);answers=np.zeros((24,5),np.int32);lib.probe(grid.ctypes.data,poses.ctypes.data,24,fov,seen.ctypes.data,answers.ctypes.data);outputs.append((seen,answers))
        same=all(np.array_equal(a,b) for a,b in zip(*outputs));assert same,i
        rows.append(dict(case=i,fov=fov,visibility_and_frontier_equal=same))
    saved=libs[0].seen_size()-libs[1].seen_size();assert saved==3584,saved
    dump(out/'result.json',dict(cases=rows,pose_steps=180*24,all_equal=True,bss_saved_bytes=saved));print(dict(cases=180,pose_steps=4320,all_equal=True,bss_saved_bytes=saved),flush=True)


if __name__=='__main__':main()
