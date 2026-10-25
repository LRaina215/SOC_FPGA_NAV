"""Compile and exercise the actual frozen delivery-trim candidate C sources.

This is a bounded native C test of candidate generation, index reconstruction
and retained motion categories. HAL trajectory/map/support calls are explicit
test stubs; this test does not claim FPGA timing, localization accuracy, ground
acceptance, complete navigation execution or real motor behavior.
"""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import subprocess
import time


SOURCE = r'''
#include <assert.h>
#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include "local_planner.h"
#include "pose_match.h"
#include "loc.h"
#include "motion_guard.h"
#include "support_guard.h"

static const uint64_t CANARY=UINT64_C(0x1277D11A992ECC03);
static hal_map_info_t info={.seq=7,.cx=0,.cy=0,.res_mm=50,.inv_res=1311};
static int fail_hal,eval_calls,max_eval;
static unsigned lp_cases,pm_grids,pm_solves,align_candidates,generated_checks;
bool hal_map_info(hal_map_info_t *out) {*out=info;return fail_hal!=2;}
bool motion_map_same(const hal_map_info_t *a,const hal_map_info_t *b) {return memcmp(a,b,sizeof *a)==0;}
uint8_t motion_check(const uint8_t *m,const hal_map_info_t *mi,const nav_pose_t *p,const lp_cfg_t *c,const nav_cmd_t *cmd)
{(void)m;(void)mi;(void)p;(void)c;(void)cmd;return MOTION_OK;}
int support_check(const nav_pose_t *p,const lp_cfg_t *c,const hal_map_info_t *mi,const nav_cmd_t *cmd)
{(void)p;(void)c;(void)mi;(void)cmd;return SUPPORT_PASS;}
bool hal_traj_eval(const hal_cand_t *cand,int n,int t,int32_t x,int32_t y,angle_t th,uint8_t cmax,hal_traj_res_t *res)
{
 (void)th;(void)cmax;assert(n>0 && n<=LP_MAX_CAND && t==20);eval_calls++;if(n>max_eval)max_eval=n;
 if(fail_hal==1)return false;
 for(int i=0;i<n;i++){res[i]=(hal_traj_res_t){.end_x=x+cand[i].dvx*t,.end_y=y+cand[i].dvy*t};}
 return true;
}
void pose_set(pose_est_t *p,int32_t x,int32_t y,angle_t yaw) {(void)p;(void)x;(void)y;(void)yaw;}

static void planner(void)
{
 lp_cfg_t cfg;lp_default_cfg(&cfg);
 assert(LP_MAX_CAND==EXPECTED_TRAJ && PM_MAX_CAND==EXPECTED_MATCH);
 assert(cfg.steps==20 && cfg.dt_ms==100 && cfg.safe_steps==12 && cfg.goal_tol_mm==150);
 assert(cfg.caps.radius_mm==210 && cfg.caps.vx_max==600 && cfg.caps.wz_max==1500);
 assert(cfg.v_back==150 && cfg.w_max==1200 && cfg.v_max==500);
 for(int holonomic=0;holonomic<2;holonomic++)for(int slow=0;slow<2;slow++)for(int phase=0;phase<4;phase++){
  struct {uint64_t before;lp_result_t r;uint64_t after;} box;
  memset(&box,0,sizeof box);box.before=box.after=CANARY;box.r.turn_phase=(uint8_t)phase;
  cfg.caps.holonomic=holonomic;cfg.caps.vy_max=holonomic?240:0;
  nav_pose_t pose={0};uint8_t map[1]={0};fail_hal=0;
  assert(lp_plan(&cfg,&info,&pose,map,slow,false,1000,0,&box.r));
  assert(box.before==CANARY && box.after==CANARY);
  assert(box.r.n_cand==(int)cfg.nv*cfg.nw+10+(holonomic?4:0));
  assert(box.r.n_cand<=LP_MAX_CAND && box.r.ground_checked && !box.r.map_stale);
  int reverse=0,turn_l=0,turn_r=0,creep=0,side_l=0,side_r=0;
  for(int i=0;i<box.r.n_cand;i++){
   int vx=box.r.cv[i],vy=box.r.cvy[i],w=box.r.cw[i];
   reverse|=vx<0;turn_l|=!vx&&!vy&&w>0;turn_r|=!vx&&!vy&&w<0;
   creep|=vx>0&&vx<=60&&!vy&&!w;side_l|=vy>0;side_r|=vy<0;
  }
  assert(reverse && turn_l && turn_r && creep);
  assert(!holonomic || (side_l && side_r));lp_cases++;
 }
 for(fail_hal=1;fail_hal<=2;fail_hal++){
  lp_result_t r={0};nav_pose_t pose={0};uint8_t map[1]={0};r.cmd=(nav_cmd_t){50,0,100,0};
  assert(!lp_plan(&cfg,&info,&pose,map,false,false,1000,0,&r));
  assert(r.map_stale && r.blocked && !r.cmd.vx && !r.cmd.vy && !r.cmd.wz && !r.ground_checked);lp_cases++;
 }
 fail_hal=0;
}

struct cand_box {uint64_t before;hal_match_cand_t c[PM_MAX_CAND];uint64_t after;};
static void matching(void)
{
 pm_cfg_t cfg;pm_default_cfg(&cfg);assert(cfg.gain_q8==160&&cfg.min_valid==60&&cfg.min_curv==600&&cfg.min_curv_ang==1500);
 struct cand_box b;b.before=b.after=CANARY;
 int k=pose_match_cands(&cfg,b.c);assert(k>0&&k<=PM_MAX_CAND);assert(b.before==CANARY&&b.after==CANARY);
 assert(k==(2*cfg.half+1)*(2*cfg.half+1)*(2*cfg.half_ang+1));
 for(int best=0;best<k;best++){
  struct {uint64_t before;hal_match_res_t r[PM_MAX_CAND];uint64_t after;} results;
  results.before=results.after=CANARY;
  for(int i=0;i<k;i++)results.r[i]=(hal_match_res_t){.n_valid=100,.n_in=90,.cost=(uint32_t)(10000+(i-best)*(i-best)*100),.n_pts=120};
  pm_stat_t state={0};assert(pose_match_solve(&cfg,results.r,k,&state));
  assert(results.before==CANARY&&results.after==CANARY);assert(!state.applied&&state.n_valid==100);pm_solves++;
 }
 for(int h=-1;h<=8;h++)for(int ha=-1;ha<=8;ha++){
  pm_cfg_t trial=cfg;trial.half=h;trial.half_ang=ha;b.before=b.after=CANARY;
  int count=pose_match_cands(&trial,b.c);assert(count>=0&&count<=PM_MAX_CAND);
  if(h<0||ha<0||h>7||ha>7||(2*h+1)*(2*h+1)*(2*ha+1)>PM_MAX_CAND)assert(count==0);
  else assert(count==(2*h+1)*(2*h+1)*(2*ha+1));
  assert(b.before==CANARY&&b.after==CANARY);pm_grids++;
 }
 cfg.half=7;cfg.half_ang=7;cfg.step_mm=INT32_MAX;cfg.step_ang=INT32_MAX;
 assert(pose_match_cands(&cfg,b.c)==0);pm_grids++;
}

static void equal_feed(const loc_t *start,const hal_match_cand_t *cand,int k,int best)
{
 hal_match_res_t res[PM_MAX_CAND];assert(k>0&&k<=PM_MAX_CAND);
 for(int i=0;i<k;i++)res[i]=(hal_match_res_t){.cost=(uint32_t)(i==best?1:1000+i),.n_pts=(uint32_t)(100+i),.n_in=(uint16_t)(80+i)};
 loc_t literal=*start,generated=*start;loc_align_feed(&literal,cand,res,k);loc_align_feed_generated(&generated,res,k);
 assert(memcmp(&literal,&generated,sizeof literal)==0);generated_checks++;
}

static void alignment(void)
{
 loc_t state={0};loc_default_cfg(&state.cfg);
 assert(state.cfg.range_mm==500 && state.cfg.range_ang==1820 && state.cfg.reloc_mm==500 && state.cfg.reloc_ang==1820);
 assert(state.cfg.accept_q==40&&state.cfg.amb_d100==30&&state.cfg.q_ok==45&&state.cfg.q_lost==35);
 assert(state.cfg.lost_us==2000000&&state.cfg.blind_us==5000000&&state.cfg.min_pts==60);
 loc_start_align(&state,0,0);assert(loc_hold(&state));
 unsigned total=state.k_total,seen=0;int maxabsx=0,maxabsy=0,maxabsa=0;
 struct cand_box b;b.before=b.after=CANARY;
 while(state.k_next<state.k_total){
  unsigned begin=state.k_next;int k=loc_align_next(&state,b.c,PM_MAX_CAND);assert(k>0&&k<=PM_MAX_CAND&&state.phase==0);
  assert((unsigned)k==state.k_next-begin);
  for(int j=0;j<k;j++){
   unsigned i=begin+(unsigned)j;int x=(int)(i%(unsigned)state.nxy)-state.nxy/2;
   int y=(int)((i/(unsigned)state.nxy)%(unsigned)state.nxy)-state.nxy/2;
   int a=(int)(i/(unsigned)(state.nxy*state.nxy))-state.na/2;
   assert(b.c[j].dx==x*state.cfg.step_mm&&b.c[j].dy==y*state.cfg.step_mm&&b.c[j].dyaw==a*state.cfg.step_ang);
   if(abs(b.c[j].dx)>maxabsx)maxabsx=abs(b.c[j].dx);if(abs(b.c[j].dy)>maxabsy)maxabsy=abs(b.c[j].dy);
   if(abs(b.c[j].dyaw)>maxabsa)maxabsa=abs(b.c[j].dyaw);
  }
  equal_feed(&state,b.c,k,k/2);seen+=(unsigned)k;
  assert(b.before==CANARY&&b.after==CANARY);
 }
 assert(seen==total && maxabsx==500 && maxabsy==500 && maxabsa==1820);align_candidates=seen;
 for(int phase=1;phase<=2;phase++){
  loc_t before=state;before.phase=phase;before.fine=(loc_cand_t){.dx=113,.dy=-217,.da=331,.cost=100000,.n_in=33};
  int k=loc_align_next(&before,b.c,PM_MAX_CAND);assert(k>0&&k<=PM_MAX_CAND&&before.phase==phase+1);
  assert(b.before==CANARY&&b.after==CANARY);
  // Every possible winning entry tests the generated inverse indexing, not
  // just the central or last candidate where several bad strides coincide.
  for(int best=0;best<k;best++)equal_feed(&before,b.c,k,best);
  loc_t small=state;small.phase=phase;small.fine=before.fine;
  assert(loc_align_next(&small,b.c,k-1)==0 && small.st==LOC_LOST && small.phase==3);
 }
 loc_t invalid=state;loc_start_align(&invalid,32001,0);assert(invalid.st==LOC_LOST&&invalid.k_total==0);
 loc_start_align(&invalid,0,32001);assert(invalid.st==LOC_LOST&&invalid.k_total==0);
}

int main(void)
{
 planner();matching();alignment();
 printf("TEST PASSED firmware match_cap=%d traj_cap=%d lp_cases=%u max_traj=%d pm_grids=%u pm_solves=%u coarse_candidates=%u generated_feed_checks=%u\n",
        PM_MAX_CAND,LP_MAX_CAND,lp_cases,max_eval,pm_grids,pm_solves,align_candidates,generated_checks);
 return 0;
}
'''


def probe(candidate, out, match_max, traj_max, compiler):
    candidate, out = Path(candidate).resolve(), Path(out).resolve()
    assert match_max in (64,128) and traj_max in (16,32)
    assert not out.exists(), 'one-shot proof refuses to overwrite evidence'
    assert (candidate/'fw/nav/loc.c').is_file()
    out.mkdir(parents=True)
    shutil.copytree(candidate/'fw', out/'fw')
    (out/'test_delivery_trim.c').write_text(SOURCE.replace('#include <string.h>', '#include <string.h>\n#include <stdlib.h>'), encoding='utf-8', newline='\n')
    fw = out/'fw'
    command = [str(compiler), '-std=c99', '-O1', '-g', '-Wall', '-Wextra',
               '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections',
               '-DEXPECTED_MATCH='+str(match_max), '-DEXPECTED_TRAJ='+str(traj_max)]
    for folder in ('include','nav','hal'):
        command += ['-I', str(fw/folder)]
    command += [str(out/'test_delivery_trim.c')]+[str(fw/'nav'/name) for name in
        ('local_planner.c','pose_match.c','loc.c','fixmath.c')]+['-o',str(out/'test_delivery_trim.exe')]
    results = []
    environment = os.environ.copy()
    environment['PATH'] = str(Path(compiler).resolve().parent)+os.pathsep+environment.get('PATH','')
    passed = False
    started = datetime.now().astimezone().isoformat()
    try:
        for label, cmd in [('native_compile',command),('native_execution',[str(out/'test_delivery_trim.exe')])]:
            begin = time.monotonic()
            run = subprocess.run(cmd, cwd=out, capture_output=True, text=True, timeout=60, env=environment)
            (out/(label+'.log')).write_text(run.stdout+run.stderr, encoding='utf-8')
            results.append(dict(stage=label, command=cmd, tool_exit=run.returncode,
                cancelled=False, elapsed_s=time.monotonic()-begin,
                ended_at=datetime.now().astimezone().isoformat()))
            assert run.returncode == 0, (label, run.stdout[-2000:], run.stderr[-2000:])
        line = (out/'native_execution.log').read_text(encoding='utf-8').strip()
        assert line.startswith('TEST PASSED firmware ')
        passed = True
    finally:
        bindings = []
        for relative in ('nav/local_planner.h','nav/local_planner.c','nav/pose_match.h',
                         'nav/pose_match.c','nav/loc.c','drv/hal_fpga.c'):
            assert (candidate/'fw'/relative).read_bytes() == (fw/relative).read_bytes()
            bindings.append(dict(candidate='fw/'+relative, proof_source=str(fw/relative)))
        result = dict(passed=passed, source=str(candidate), build=str(out),
            started_at=started, ended_at=datetime.now().astimezone().isoformat(),
            match_candidate_capacity=match_max, trajectory_candidate_capacity=traj_max,
            runtimes=results, source_bindings=bindings,
            global_initialization_radius_and_relocation_radius_retained=passed,
            generated_inverse_indices_match_literal_candidate_feeds=passed,
            all_batches_within_actual_hardware_capacity=passed,
            slow_forward_turn_and_reverse_categories_retained=passed,
            failed_HAL_returns_zero_command=passed,
            HAL_map_trajectory_and_support_are_test_stubs=True,
            full_FPGA_NAV_vehicle_or_timing_acceptance=False,
            native_result=(out/'native_execution.log').read_text(encoding='utf-8') if (out/'native_execution.log').exists() else None)
        (out/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--match-max',type=int,required=True)
    parser.add_argument('--traj-max',type=int,required=True)
    parser.add_argument('--compiler',type=Path,default=Path('C:/msys64/mingw64/bin/gcc.exe'))
    args=parser.parse_args()
    print(json.dumps(probe(args.candidate,args.out,args.match_max,args.traj_max,args.compiler),ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
