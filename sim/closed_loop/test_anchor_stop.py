"""Actual C contracts for the isolated anchor / stop candidate.

Finite constructed evidence, not calibration or a proof of physical stopping.
"""
import argparse
import ctypes
import json
from pathlib import Path
import subprocess


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--fw',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    a.out.mkdir(parents=True,exist_ok=False);fw=a.fw.resolve();c=a.out/'fixture.c'
    c.write_text(r'''#include "pose.h"
#include "support_guard.h"
#include "stop_guard.h"
#include "motion_guard.h"
#include <string.h>
static stop_guard_t stop;
static nav_cmd_t held;
static int kind,calls,infos;
static uint32_t time_us;
uint32_t hal_time_us(void){return time_us;}
bool hal_map_info(hal_map_info_t *m){*m=(hal_map_info_t){.seq=7,.inv_res=1311,.res_mm=50};if(kind==7 && infos++)m->inv_res=1310;return true;}
bool hal_map_height(int32_t x,int32_t y,int32_t *z){(void)x;(void)y;*z=20;return kind==10;}
bool hal_map_support(int32_t x,int32_t y,hal_support_t *s){
 (void)y;calls++;bool front=x>=30;
 *s=(hal_support_t){.lo=front?26:20,.hi=front?28:22,.seq=7,.count=3,.coverage=65535,.age=front?0:80,.retained_stale=!front};
 if(kind==1 && !front)s->coverage=0;
 if(kind==2 && !front)s->count=1;
 if(kind==3 && !front)s->hi=45;
 if(kind==4 && front)s->lo=s->hi=60;
 if(kind==5 && front)s->retained_stale=1;
 if(kind==6 && calls==4)time_us=200000;
 if(kind==8 && front)s->coverage=0;
 if(kind==9 && !front)s->lo=s->hi=200;
 return true;
}
__declspec(dllexport) int anchor(int k){
 kind=k;calls=infos=0;time_us=0;pose_est_t p;pose_init(&p,0,0,0);
 pose_anchor_z(&p,60,135,64);return p.cur.z;
}
__declspec(dllexport) void reset(void){memset(&stop,0,sizeof(stop));}
__declspec(dllexport) void feed(int v,int y,int w,unsigned t){stop_feedback(&stop,v,y,w,t);}
__declspec(dllexport) int hold(unsigned t,int estop){held=stop_hold(&stop,t,estop);return held.vx;}
__declspec(dllexport) int hold_wz(void){return held.wz;}
__declspec(dllexport) int status(void){return stop.status;}
__declspec(dllexport) int error(void){return stop.error_um;}
__declspec(dllexport) void release(void){stop_release(&stop);}
__declspec(dllexport) void admit(int ok){stop_admission(&stop,ok!=0);}
__declspec(dllexport) int margin(int v,int w,unsigned t){nav_cmd_t c={.vx=v,.wz=w};return stop_margin(&stop,&c,t);}
__declspec(dllexport) int support(int extra,int k){
 kind=k;calls=infos=0;time_us=0;support_reset(0);
 nav_pose_t p={0};nav_cmd_t cmd={0};lp_cfg_t cfg={.safe_steps=12,.dt_ms=100};hal_map_info_t m={.seq=7,.inv_res=1311};
 return support_check_margin(&p,&cfg,&m,&cmd,extra);
}
__declspec(dllexport) int body(int extra,int unknown){
 static uint8_t map[NP_MAP_N*NP_MAP_N];memset(map,0,sizeof map);
 if(unknown)map[64*NP_MAP_N+68]=255;
 hal_map_info_t m={.seq=7,.inv_res=1311,.res_mm=50};nav_pose_t p={0};nav_cmd_t cmd={0};
 lp_cfg_t cfg={.caps={.radius_mm=210},.safe_steps=12,.dt_ms=100};
 return motion_check_margin(map,&m,&p,&cfg,&cmd,extra);
}
''',encoding='utf-8')
    dll=a.out/'contracts.dll'
    subprocess.run(['gcc','-std=c99','-O2','-Wall','-Wextra','-Werror','-shared','-o',str(dll),
        *['-I'+str(fw/d) for d in ('include','nav','hal','app')],str(c),
        *[str(fw/'nav'/n) for n in ('fixmath.c','pose.c','support_guard.c','stop_guard.c','motion_guard.c')]],check=True)
    d=ctypes.CDLL(str(dll.resolve()));rows=[]
    def check(name,got,want):
        rows.append(dict(name=name,got=int(got),expected=int(want)));assert got==want,rows[-1]
    for k,name in enumerate(('remembered_contact','missing_contact_tile','one_hit','mixed_surface','front_step','stale_front','epoch_change','resolution_change','missing_front_tile','outlier')):
        check(name,d.anchor(k),2 if k==0 else 0)
    check('old_regular_anchor_preserved',d.anchor(10),5)
    d.reset();check('no_feedback_request',d.hold(0,0),0);check('no_feedback_unconfirmed',d.status(),4)
    d.reset();d.feed(0,0,0,10000);d.hold(10000,0);d.admit(1)
    for t in range(20000,540000,10000):d.feed(0,0,0,t)
    check('actual_stable_dwell',d.status(),3)
    check('estop_zero',d.hold(530000,1),0);check('estop_does_not_claim_stop',d.status(),0)
    d.reset();d.feed(3,0,0,10000);d.hold(10000,0);d.admit(1)
    for t in range(20000,800000,10000):d.feed(3,0,0,t)
    check('residual_three_not_confirmed',d.status(),2)
    d.admit(0)
    for t in range(800000,1500000,10000):d.feed(0,0,0,t)
    check('rejected_hold_cannot_confirm_between_plan_ticks',d.status(),5)
    d.reset();d.feed(0,0,0,10000);d.hold(10000,0);d.feed(0,0,0,120000)
    check('lost_feedback',d.hold(120000,0),0);check('gap_unconfirmed',d.status(),4)
    for t in range(130000,800000,10000):d.feed(0,0,0,t)
    d.hold(790000,0);check('gap_cannot_reanchor_to_fake_stop',d.status(),4)
    d.release();d.hold(790000,0)
    d.reset();d.feed(0,0,0,10000)
    check('zero_has_hold_reserve',d.margin(0,0,10000),25)
    check('command50_stop_radius',d.margin(50,0,10000),48)
    check('turn600_stop_radius',d.margin(0,600,10000),77)
    check('excessive_unverified_radius',d.margin(500,0,10000),-1)
    d.feed(50,0,0,20000);check('zero_request_uses_actual50',d.margin(0,0,20000),48)
    check('feedback100ms_rejected',d.margin(0,0,120000),-1)
    d.reset();d.feed(0,0,20,10000);d.hold(10000,0);d.admit(1)
    for t in range(20000,4000000,10000):d.feed(0,0,20,t)
    d.hold(3990000,0);check('angular_drift_not_stopped',d.status(),2)
    check('angular_hold_opposes_drift',d.hold_wz()<0,True)
    check('zero_motion_still_checks_missing_tread',d.support(25,1),1)
    check('zero_nominal_contract_preserved',d.body(0,1),0)
    check('zero_stop_tube_known',d.body(25,0),1)
    check('extra_radius_is_not_current_occupied_space',d.body(25,1),5)
    report=dict(passed=len(rows),rows=rows,scope='actual candidate C, finite synthetic evidence; no physical calibration')
    (a.out/'result.json').write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(report))


if __name__=='__main__':main()
