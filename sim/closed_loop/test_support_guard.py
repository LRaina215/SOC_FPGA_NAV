"""Finite C wheel-support contracts on explicitly constructed interval maps.

No scene truth is exposed to navigation. This isolates admission arithmetic,
unknown support, epoch changes and the declared start-area assumption.
"""
import argparse
import ctypes
import json
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[2]


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    c=a.out/'fixture.c'
    c.write_text('''#include "support_guard.h"
#include <string.h>
static int kind,height;
static hal_map_info_t mi={.seq=7,.inv_res=1311,.res_mm=50};
__declspec(dllexport) int transition(int al,int ah,int bl,int bh,int cap,int q) {
 return support_transition(al,ah,bl,bh,cap,q);
}
bool hal_map_support(int32_t x,int32_t y,hal_support_t *s) {
 if(kind==1 || (kind==8 && y>=150) || (kind==9 && y>=100)) return false;
 int z=kind==3?(x>=50?height:0):kind==7?x*height/1000:0;
 *s=(hal_support_t){.lo=z,.hi=z+(kind==5?30:0),.seq=kind==6?8:7,
                  .count=1,.age=kind==2?70:0,.retained_stale=kind==2};
 return true;
}
__declspec(dllexport) int probe(int k,int h,int boot,int vx,int wz,int yaw) {
 kind=k;height=h;support_reset(boot);
 lp_cfg_t cfg={.safe_steps=12,.dt_ms=100};
 nav_pose_t p={.yaw=yaw};nav_cmd_t cmd={vx,0,wz,0};
 return support_check(&p,&cfg,&mi,&cmd);
}
__declspec(dllexport) int moved_jump(void) {
 kind=1;support_reset(350);lp_cfg_t cfg={.safe_steps=12,.dt_ms=100};
 nav_pose_t p={0};nav_cmd_t cmd={50,0,0,0};
 if(support_check(&p,&cfg,&mi,&cmd)) return -1;
 support_note_cmd(&cmd);support_frame_jump();p.x=1000;
 return support_check(&p,&cfg,&mi,&cmd);
}
__declspec(dllexport) int offset_probe(int k) {
 kind=k;support_reset(0);lp_cfg_t cfg={.safe_steps=12,.dt_ms=100};
 nav_pose_t p={.y=-10};nav_cmd_t cmd={50,0,0,0};
 return support_check(&p,&cfg,&mi,&cmd);
}
''',encoding='utf-8')
    dll=a.out/'support.dll';fw=ROOT/'fw'
    subprocess.run(['gcc','-std=c99','-O2','-Wall','-Wextra','-Werror','-shared','-o',str(dll),
        *['-I'+str(fw/d) for d in ('include','hal','nav','app')],str(c),
        str(fw/'nav/support_guard.c'),str(fw/'nav/fixmath.c')],check=True)
    d=ctypes.CDLL(str(dll.resolve()));rows=[]
    def check(name,got,want):
        rows.append(dict(name=name,got=got,expected=want));assert got==want,rows[-1]
    for sign in (-1,1):
        for mm in (0,15,18,19,20,21,23,30):
            check(f'step_exact_{sign*mm}',d.transition(0,0,sign*mm,sign*mm,20,0),0 if mm<=20 else 2)
        for mm in (18,19,20,21,23):
            check(f'step_quant1_{sign*mm}',d.transition(0,0,sign*mm,sign*mm,20,1),0 if mm<=18 else 2 if mm>22 else 1)
    for yaw in (0,8192,16384,32768,49152):
        check(f'known_flat_{yaw}',d.probe(0,0,0,500,0,yaw),0)
    check('known_turn',d.probe(0,0,0,0,600,0),0)
    check('remembered_static_ground',d.probe(2,0,0,500,0,0),0)
    check('unknown_no_declaration',d.probe(1,0,0,50,0,0),1)
    check('unknown_turn_no_declaration',d.probe(1,0,0,0,600,0),1)
    check('declared_start_crawl',d.probe(1,0,350,50,0,0),0)
    check('declared_start_turn',d.probe(1,0,350,0,600,0),0)
    check('declaration_does_not_extend_to_route',d.probe(1,0,350,500,0,0),1)
    check('frame_jump_after_translation',d.moved_jump(),1)
    check('ambiguous_cell',d.probe(5,0,0,50,0,0),2)
    check('different_epoch',d.probe(6,0,0,50,0,0),3)
    for sign in (-1,1):
        for mm,want in ((18,0),(20,1),(23,2)):
            check(f'wheel_patch_step_{sign*mm}',d.probe(3,sign*mm,0,50,0,0),want)
    check('slope_10deg',d.probe(7,176,0,500,0,0),0)
    check('slope_15deg',d.probe(7,268,0,500,0,0),2)
    check('unobserved_outside_tread',d.offset_probe(8),0)
    check('unobserved_under_tread',d.offset_probe(9),1)
    report=dict(passed=len(rows),scope='finite synthetic interval contracts, not vehicle capability validation',rows=rows)
    (a.out/'result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
