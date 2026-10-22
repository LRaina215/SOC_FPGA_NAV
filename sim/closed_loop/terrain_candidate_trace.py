"""Passive host-C candidate/anchor trace on the fixed4500 terrain runner.

Instrumentation lives in a copied firmware tree and cannot change the checked
candidate budget. Compare the saved trajectories/maps with an ordinary run.
"""
import argparse
import ctypes
import json
from pathlib import Path
import shutil
import subprocess
import sys

import terrain_navigation_coverage as coverage_runner


def instrument(source,out):
    out.mkdir(parents=True,exist_ok=False);fw=out/'fw';shutil.copytree(source,fw)
    declaration='\nextern void n2_trace(int kind,const int32_t *values,int n);\n'
    def change(name,replacements):
        p=fw/'nav'/name;s=p.read_text(encoding='utf-8')
        s='#include <stdint.h>\n'+declaration+s
        for before,after in replacements:
            assert s.count(before)==1,(name,before)
            s=s.replace(before,after,1)
        p.write_text(s,encoding='utf-8')
    def emit(kind,values):
        return 'n2_trace('+str(kind)+',(int32_t[]){'+values+'},'+str(len(values.split(',')))+');'
    change('local_planner.c',[
        ('if(status!=MOTION_OK && status!=MOTION_UNKNOWN) return SUPPORT_BLOCK;',
         'if(status!=MOTION_OK && status!=MOTION_UNKNOWN) {'+emit(2,'cmd->vx,cmd->wz,status,SUPPORT_BLOCK')+'return SUPPORT_BLOCK;}'),
        ('return SUPPORT_UNKNOWN; /* 准入已拒绝',
         '{'+emit(2,'cmd->vx,cmd->wz,status,SUPPORT_UNKNOWN')+'return SUPPORT_UNKNOWN;} /* 准入已拒绝'),
        ('return support_check(p,c,mi,cmd);',
         'int ground=support_check(p,c,mi,cmd);'+emit(2,'cmd->vx,cmd->wz,status,ground')+'return ground;'),
        ('if (t->collide && t->coll_step < c->safe_steps) continue;',
         'if (t->collide && t->coll_step < c->safe_steps) {'+emit(1,'i,r->cv[i],r->cw[i],INT32_MIN,escape,t->coll_step,t->n_unknown,t->max_cost')+'continue;}'),
        ('r->n_valid++;',emit(1,'i,r->cv[i],r->cw[i],score,escape,t->coll_step,t->n_unknown,t->max_cost')+'r->n_valid++;'),
        ('r->ground_checked=false;',emit(0,'pose->x,pose->y,pose->z,pose->yaw,goal_x,goal_y,allow_unknown')+'r->ground_checked=false;')])
    change('support_guard.c',[
        ('if(lo>cap || hi< -cap) return SUPPORT_BLOCK;',
         'if(lo>cap || hi< -cap){'+emit(9,'al,ah,bl,bh,cap,q,lo,hi,SUPPORT_BLOCK')+'return SUPPORT_BLOCK;}'),
        ('return lo>=-cap && hi<=cap?SUPPORT_PASS:SUPPORT_UNKNOWN;',
         'if(lo>=-cap && hi<=cap)return SUPPORT_PASS;'+emit(9,'al,ah,bl,bh,cap,q,lo,hi,SUPPORT_UNKNOWN')+'return SUPPORT_UNKNOWN;'),
        ('q->used|=1u<<k;q->cell[k]=c;*out=c;',
         'q->used|=1u<<k;q->cell[k]=c;*out=c;if(!c.kind){'+emit(6,'ix,iy,c.coverage,q->seq')+'}'),
        ('if(f<=sf && f>=-sf && l<=sl && l>=-sl) return SUPPORT_UNKNOWN;',
         'if(f<=sf && f>=-sf && l<=sl && l>=-sl) {'+emit(4,'xx,yy,sx,sy,c.lo,c.hi,c.coverage')+'return SUPPORT_UNKNOWN;}'),
        ('if(c.hi-c.lo+2>20) return c.hi-c.lo-2>20?SUPPORT_BLOCK:SUPPORT_UNKNOWN;',
         'if(c.hi-c.lo+2>20) {'+emit(5,'xx,yy,c.lo,c.hi,c.coverage')+'return c.hi-c.lo-2>20?SUPPORT_BLOCK:SUPPORT_UNKNOWN;}'),
        ('if(result) {if(result==SUPPORT_BLOCK)',
         'if(result) {'+emit(3,'cmd->vx,cmd->wz,i,side,wx,wy,result')+'if(result==SUPPORT_BLOCK)')])
    pose_source=(fw/'nav/pose.c').read_text(encoding='utf-8')
    anchor_reject=('if (n < 3) return;', 'if(n<3){'+emit(8,'n,sum,p->cur.z,0,gain_q8,0')+'return;}')
    if anchor_reject[0] not in pose_source:
        anchor_reject=('if(!anchor_contacts(p,at,hw,gain_q8))anchor_diag[2]++;',
          emit(8,'n,sum,p->cur.z,0,gain_q8,0')+
          'int32_t z_before=p->cur.z;int ok=anchor_contacts(p,at,hw,gain_q8);'+
          emit(10,'ok,z_before,p->cur.z,anchor_diag[3],anchor_diag[4],anchor_diag[5]')+
          'if(!ok)anchor_diag[2]++;')
    change('pose.c',[
        ('int32_t ox, oy, h;', 'int32_t ox, oy, h=0;'),
        ('if (hal_map_height(ox, oy, &h)) { sum += h; n++; }',
         'int ok=hal_map_height(ox,oy,&h);'+emit(7,'i,ox,oy,ok,h,p->cur.z')+'if(ok){sum+=h;n++;}'),
        anchor_reject,
        ('if (e > 150 || e < -150) return;', 'if(e>150 || e< -150){'+emit(8,'n,sum,p->cur.z,e,gain_q8,2')+'return;}'),
        ('pose_correct_delta(p, 0, 0, e * gain_q8 / 256, 0);',emit(8,'n,sum,p->cur.z,e,gain_q8,1')+'pose_correct_delta(p,0,0,e*gain_q8/256,0);')])
    extra=out/'trace.c';extra.write_text('''#include "hal.h"
typedef void (*trace_cb)(uint32_t,int,const int32_t*,int);
static trace_cb callback;
__declspec(dllexport) void py_set_candidate_trace_cb(trace_cb cb){callback=cb;}
void n2_trace(int kind,const int32_t *values,int n){if(callback)callback(hal_time_us(),kind,values,n);}
''',encoding='utf-8')
    dll=out/'nav_fw.dll'
    host_extra=[fw/'test/support_snapshot_host.c'] if (fw/'test/support_snapshot_host.c').exists() else []
    subprocess.run(['gcc','-std=c99','-O2','-shared','-o',str(dll)]+['-I'+str(fw/d) for d in ['include','nav','hal','app','test']]+list(map(str,list((fw/'nav').glob('*.c'))+[fw/'app/nav_app.c',fw/'test/hal_py.c',extra]+host_extra)),check=True)
    return dll


class TraceObserver(coverage_runner.CoverageObserver):
    def setup(self,bot,fpga,fw,args):
        super().setup(bot,fpga,fw,args)
        self.trace_file=(self.out/'candidates.jsonl').open('w',encoding='utf-8')
        CALLBACK=ctypes.CFUNCTYPE(None,ctypes.c_uint32,ctypes.c_int,ctypes.POINTER(ctypes.c_int32),ctypes.c_int)
        def callback(us,kind,v,n):
            if self.trace_lo<=us/1e6<=self.trace_hi:self.trace_file.write(json.dumps(dict(us=us,kind=kind,v=list(v[:n])))+'\n')
        self.trace_cb=CALLBACK(callback);fw.py_set_candidate_trace_cb(self.trace_cb)

    def finish(self,m):
        self.trace_file.close();super().finish(m)
        m['passive_candidate_trace']=dict(source_copy_only=True,window_s=[self.trace_lo,self.trace_hi],
            kinds={'0':'plan pose and goal','1':'score/index/order; INT32_MIN=hardware early collision',
                   '2':'checked command vx/wz,body result,support result','3':'support failing sample/side/location',
                   '4':'missing subcell','5':'wide height interval','6':'missing raw cell',
                   '7':'anchor sample','8':'anchor decision;0=insufficient,1=applied,2=outlier',
                   '9':'interval transition rejection','10':'contact fallback ok,before_z,after_z,reason,target,delta'})


def main():
    ap=argparse.ArgumentParser(add_help=False);ap.add_argument('--source-fw',type=Path,required=True);ap.add_argument('--trace-build',type=Path,required=True);ap.add_argument('--trace-window',default='0,45');a,rest=ap.parse_known_args()
    TraceObserver.trace_lo,TraceObserver.trace_hi=map(float,a.trace_window.split(','))
    dll=instrument(a.source_fw.resolve(),a.trace_build.resolve())
    coverage_runner.base.Observer=TraceObserver
    sys.argv=[str(Path(coverage_runner.base.__file__)),*rest,'--firmware',str(dll)]
    return coverage_runner.base.main()


if __name__=='__main__':raise SystemExit(main())
