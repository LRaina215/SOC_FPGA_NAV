"""Focused numerical, lease-boundary and real-app mailbox proofs for a candidate."""
import argparse,json,subprocess,shutil,os
from pathlib import Path

BASE=Path('F:/nav_integration_g2_20261023/baseline_fusion_preview_safety250_emptyIMU_prepared3')
PUBLIC=['explore_default_cfg','explore_init','explore_seen_reset','explore_seen_mark','explore_mark',
        'explore_pick','explore_blacklist','explore_frontier']
GUARD_PUBLIC=['g_host_link_guard','host_link_guard_init','host_link_guard_poll','host_link_guard_observe','host_link_guard_filter']

ALGORITHM=r'''#include "explore.h"
#include "scratch.h"
#include "nav_search_checkpoint.h"
#include "host_link_guard.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
bool g_nav_search_interrupted;
static unsigned checkpoints,cancel_at;
bool nav_app_search_checkpoint(void) {
 checkpoints++;
 if(cancel_at && checkpoints==cancel_at) {g_nav_search_interrupted=true;return false;}
 return true;
}
extern bool ref_explore_frontier(explore_t*,const explore_cfg_t*,const gmap_t*,const hal_map_info_t*,const nav_pose_t*,bool,int32_t*,int32_t*);
extern bool ref_host_link_guard_observe(host_link_guard_t*,const proto_msg_t*,uint32_t,uint32_t);
static gmap_t gm,saved;static explore_t ref_e,new_e;static nav_scratch_t scratch_ref;
static unsigned rng=0x21a55a31,checks;
static unsigned random32(void) {rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
#define REQUIRE(c) do {checks++;if(!(c)){fprintf(stderr,"FAIL line=%d expr=%s\n",__LINE__,#c);exit(1);}} while(0)
static void reset_hook(unsigned stop) {checkpoints=0;cancel_at=stop;g_nav_search_interrupted=false;}
static void map_case(unsigned id) {
 memset(&gm,0,sizeof gm);memset(&ref_e,0,sizeof ref_e);
 gm.init=gm.ready=true;gm.ox=(id%3==0)?-10:0;gm.oy=(id%5==0)?-7:0;
 for(unsigned i=0;i<4096;i++) {
  unsigned x=i%64,y=i/64,r=random32();
  gm.grid[i]=id%4==0?0:id%4==1?(x>=16&&x<48&&y>=16&&y<48?0:255):id%4==2?(x%8==4&&y!=(x/8%2?4:59)?254:0):(r%10<2?254:r%10==2?255:r%201);
  ref_e.visited[i]=(uint8_t)random32();
  if((id&1)?(x<32):(random32()&1))ref_e.seen_bits[i>>3]|=1u<<(i&7);
 }
 if(id%3==0) {ref_e.bl_n=8;for(unsigned b=0;b<8;b++){ref_e.bl_x[b]=12000+(int)b*2000;ref_e.bl_y[b]=-12000;}}
 saved=gm;
}
int main(void) {
 unsigned cases=0,cancel_cases=0;
 for(unsigned id=0;id<128;id++)for(unsigned seen=0;seen<2;seen++) {
  map_case(id);new_e=ref_e;
  explore_cfg_t cfg;explore_default_cfg(&cfg);hal_map_info_t mi={.seq=1,.inv_res=1311,.res_mm=50};
  nav_pose_t pose={.x=3000,.y=3000};int32_t rx=0x12345678,ry=0x12345678,nx=rx,ny=ry;
  memset(&g_scratch,0x5a,sizeof g_scratch);
  int a=ref_explore_frontier(&ref_e,&cfg,&gm,&mi,&pose,seen,&rx,&ry);scratch_ref=g_scratch;
  memset(&g_scratch,0x5a,sizeof g_scratch);reset_hook(0);
  int b=explore_frontier(&new_e,&cfg,&gm,&mi,&pose,seen,&nx,&ny);
  REQUIRE(a==b&&rx==nx&&ry==ny);REQUIRE(!memcmp(&ref_e,&new_e,sizeof ref_e));
  REQUIRE(!memcmp(&gm,&saved,sizeof gm));REQUIRE(!memcmp(&g_scratch,&scratch_ref,sizeof g_scratch));cases++;
 }
 map_case(0);memset(ref_e.seen_bits,0xff,256);memset(ref_e.seen_bits+256,0,256);
 explore_cfg_t cfg;explore_default_cfg(&cfg);hal_map_info_t mi={.seq=1,.inv_res=1311,.res_mm=50};nav_pose_t pose={.x=3000,.y=3000};
 explore_t initial=ref_e;int32_t rx=99,ry=99;
 int reference=ref_explore_frontier(&ref_e,&cfg,&gm,&mi,&pose,true,&rx,&ry);
 const unsigned stops[]={1,2,5,17,33,70,120,145};
 for(unsigned n=0;n<sizeof stops/sizeof stops[0];n++) {
  new_e=initial;int32_t x=99,y=99;reset_hook(stops[n]);
  int r=explore_frontier(&new_e,&cfg,&gm,&mi,&pose,true,&x,&y);
  REQUIRE(!r&&g_nav_search_interrupted&&x==99&&y==99);REQUIRE(!memcmp(&gm,&saved,sizeof gm));
  new_e=initial;reset_hook(0);r=explore_frontier(&new_e,&cfg,&gm,&mi,&pose,true,&x,&y);
  REQUIRE(r==reference&&x==rx&&y==ry&&new_e.n_frontier==ref_e.n_frontier);cancel_cases++;
 }
 proto_msg_t heartbeat={.type=MSG_TARGET};
 host_link_guard_t old={.have_rx=true,.last_rx_us=0},g=old;
 REQUIRE(ref_host_link_guard_observe(&old,&heartbeat,251000,249900));
 REQUIRE(host_link_guard_observe(&g,&heartbeat,251000,249900));
 REQUIRE(old.locked&&old.timed_out&&!g.locked&&!g.timed_out&&g.last_rx_us==249900);
 for(unsigned wrap=0;wrap<2;wrap++) {
  uint32_t start=wrap?0xffff0000u:0u;
  g=(host_link_guard_t){.have_rx=true,.last_rx_us=start};
  REQUIRE(host_link_guard_observe(&g,&heartbeat,start+251000,start+249900));REQUIRE(!g.locked&&!g.timed_out);
  g=(host_link_guard_t){.have_rx=true,.last_rx_us=start};
  REQUIRE(host_link_guard_observe(&g,&heartbeat,start+250100,start+250000));REQUIRE(g.locked&&g.timed_out);
  g=(host_link_guard_t){.have_rx=true,.last_rx_us=start};
  REQUIRE(!host_link_guard_observe(&g,&heartbeat,start+500000,start+249900));REQUIRE(g.locked&&g.timed_out&&g.last_rx_us==start);
  g=(host_link_guard_t){.have_rx=true,.last_rx_us=start,.locked=true,.timed_out=true,.locked_since_us=start+200000};
  REQUIRE(host_link_guard_observe(&g,&heartbeat,start+251000,start+249900));REQUIRE(g.locked&&g.timed_out);
  proto_msg_t motion={.type=MSG_GOAL};REQUIRE(!host_link_guard_observe(&g,&motion,start+252000,start+251500));REQUIRE(g.locked);
  proto_msg_t idle={.type=MSG_MODE};idle.u.mode.mode=NAV_MODE_IDLE;
  REQUIRE(!host_link_guard_observe(&g,&idle,start+253000,start+199000));REQUIRE(g.locked);
  REQUIRE(host_link_guard_observe(&g,&idle,start+254000,start+253500));REQUIRE(!g.locked&&!g.timed_out);
  g=(host_link_guard_t){.have_rx=true,.last_rx_us=start+100000,.locked=true,.timed_out=true,.locked_since_us=start+200000};
  REQUIRE(host_link_guard_observe(&g,&idle,start+253000,start+199000));REQUIRE(g.locked&&g.timed_out); /* fresh/nonregressing but before lock */
  g=(host_link_guard_t){.have_rx=true,.last_rx_us=start+100000};
  REQUIRE(!host_link_guard_observe(&g,&heartbeat,start+150000,start+150001));REQUIRE(g.last_rx_us==start+100000);
  REQUIRE(!host_link_guard_observe(&g,&heartbeat,start+150000,start+99999));REQUIRE(g.last_rx_us==start+100000);
 }
 printf("FRONTIER_PROOF passed cases=%u cancel_restart=%u checks=%u scratch_bytes=%u\n",cases,cancel_cases,checks,(unsigned)sizeof g_scratch);
 return 0;
}
'''

APP=r'''#include "nav_app.h"
#include "nav_search_checkpoint.h"
#include "host_link_guard.h"
#include "hal_sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned checks;
#define REQUIRE(c) do {checks++;if(!(c)){fprintf(stderr,"FAIL line=%d expr=%s\n",__LINE__,#c);exit(1);}} while(0)
static void admitted(void) {
 sim_reset();nav_app_init();g_app.nt.mode=NAV_MODE_SEARCH;g_app.nt.gm.has_prior=true;
 g_host_link_guard=(host_link_guard_t){.have_rx=true,.last_rx_us=g_sim.now_us};g_nav_search_interrupted=false;
}
int main(void) {
 uint8_t packet[1100];admitted();uint32_t n=g_app.n_msgs;
 g_sim.now_us=249900;int len=proto_build_target(packet,0,0,0,0,0,0,0,0);sim_host_inject(packet,len);
 g_sim.now_us=251000;REQUIRE(nav_app_search_checkpoint());
 REQUIRE(!g_host_link_guard.locked&&!g_host_link_guard.timed_out&&g_host_link_guard.last_rx_us==249900&&g_app.n_msgs==n+1);
 REQUIRE(g_app.nt.mode==NAV_MODE_SEARCH&&!g_nav_search_interrupted);
 /* A queued goal is not dispatched in a local-coordinate call frame. */
 int32_t oldx=g_app.nt.goal_x,oldy=g_app.nt.goal_y;
 g_sim.now_us=252000;len=proto_build_goal(packet,1234,5678,0);sim_host_inject(packet,len);
 len=proto_build_goal(packet,777,888,0);sim_host_inject(packet,len);
 n=g_app.n_msgs;REQUIRE(!nav_app_search_checkpoint());REQUIRE(g_nav_search_interrupted);
 REQUIRE(g_app.nt.goal_x==oldx&&g_app.nt.goal_y==oldy&&!g_sim.cmd.vx&&!g_sim.cmd.vy&&!g_sim.cmd.wz);
 REQUIRE(!nav_app_search_checkpoint());REQUIRE(g_app.n_msgs==n); /* cannot overwrite pending buffer */
 nav_app_poll();REQUIRE(g_app.n_msgs==n+2&&g_app.nt.goal_x==777&&g_app.nt.goal_y==888);
 n=g_app.n_msgs;nav_app_poll();REQUIRE(g_app.n_msgs==n); /* dispatch exactly once */
 /* MAP_DATA's pointer remains in the retained buffer until original on_msg. */
 admitted();g_sim.now_us=2000;
 g_app.prx=(prior_rx_t){.state=PRIOR_RECV,.total=40};
 memset(packet,0,48);packet[0]='N';packet[1]='V';packet[2]=1;packet[3]=MSG_MAP_DATA;
 packet[8]='N';packet[9]='V';packet[10]='M';packet[11]='P';packet[12]=2;
 uint8_t header[40];memcpy(header,packet+8,40);sim_host_inject(packet,48);
 len=proto_build_target(packet,0,0,0,0,0,0,0,0);sim_host_inject(packet,len);
 n=g_app.n_msgs;REQUIRE(!nav_app_search_checkpoint());REQUIRE(!nav_app_search_checkpoint());
 REQUIRE(g_app.prx.next==0);nav_app_poll();
 REQUIRE(g_app.prx.next==40&&g_app.prx.hdr_ok&&!memcmp(header,g_app.prx.hdr,40)&&g_app.n_msgs==n+2);
 /* SET_POSE must be deferred while a local-coordinate view is on the stack. */
 admitted();g_sim.now_us=2000;int32_t px=g_app.nt.pose.cur.x;
 memset(packet,0,24);packet[0]='N';packet[1]='V';packet[2]=1;packet[3]=MSG_SET_POSE;packet[4]=0xd2;packet[5]=4;
 sim_host_inject(packet,24);g_app.nt.stepping_local=true;
 REQUIRE(!nav_app_search_checkpoint());REQUIRE(g_app.nt.pose.cur.x==px);
 g_app.nt.stepping_local=false;n=g_app.n_msgs;nav_app_poll();
 REQUIRE(g_app.nt.pose.cur.x==1234&&g_app.n_msgs==n+1);
 /* A real loss is latched; a heartbeat cannot heal it. */
 admitted();g_sim.now_us=251001;REQUIRE(!nav_app_search_checkpoint());
 REQUIRE(g_host_link_guard.locked&&g_host_link_guard.timed_out&&g_app.nt.mode==NAV_MODE_ESTOP);
 REQUIRE(!g_sim.cmd.vx&&!g_sim.cmd.vy&&!g_sim.cmd.wz);
 g_sim.now_us=252000;len=proto_build_target(packet,0,0,0,0,0,0,0,0);sim_host_inject(packet,len);
 REQUIRE(!nav_app_search_checkpoint());REQUIRE(g_host_link_guard.locked&&g_app.nt.mode==NAV_MODE_ESTOP);
 /* Explicit ESTOP first causes zero,cancel,then original mode dispatch. */
 admitted();g_sim.now_us=2000;len=proto_build_mode(packet,NAV_MODE_ESTOP);sim_host_inject(packet,len);
 REQUIRE(!nav_app_search_checkpoint());REQUIRE(!g_sim.cmd.vx&&!g_sim.cmd.wz);
 n=g_app.n_msgs;nav_app_poll();REQUIRE(g_app.nt.mode==NAV_MODE_ESTOP&&g_app.n_msgs==n+1);
 printf("APP_CHECKPOINT_PROOF passed checks=%u buffer=1100 pending_dispatch_once=1\n",checks);return 0;
}
'''


def run(candidate: Path,name='host_proofs'):
    out=candidate/name;out.mkdir(exist_ok=False);fw=candidate/'fw';cc=shutil.which('gcc');assert cc
    env=os.environ.copy();env['PATH']=str(Path(cc).parent)+';'+env.get('PATH','')
    flags=['-std=c99','-O2','-ffunction-sections','-fdata-sections','-D_USE_MATH_DEFINES','-Wl,--gc-sections']
    flags += ['-I'+str(fw/n) for n in ['include','hal','nav','app','test','bsp']]
    def command(args,label):
        (out/(label+'_command.json')).write_text(json.dumps(args,indent=2))
        r=subprocess.run(args,cwd=out,capture_output=True,text=True,env=env)
        (out/(label+'.log')).write_text(r.stdout+'\n'+r.stderr,encoding='utf-8')
        assert r.returncode==0,(label,r.returncode,r.stdout,r.stderr)
        return r.stdout
    ref=out/'reference_explore.o';guard=out/'reference_guard.o'
    command([cc,*flags,*['-D'+n+'=ref_'+n for n in PUBLIC],'-c',str(BASE/'fw/nav/explore.c'),'-o',str(ref)],'compile_reference_explore')
    command([cc,*flags,*['-D'+n+'=ref_'+n for n in GUARD_PUBLIC],'-c',str(BASE/'fw/nav/host_link_guard.c'),'-o',str(guard)],'compile_reference_guard')
    (out/'algorithm.c').write_text(ALGORITHM);exe=out/'algorithm.exe'
    command([cc,*flags,str(out/'algorithm.c'),str(ref),str(guard),*[str(fw/'nav'/n) for n in
        ['explore.c','scratch.c','astar.c','gmap.c','fixmath.c','host_link_guard.c']],'-o',str(exe),'-lm'],'compile_algorithm')
    algorithm=command([str(exe)],'algorithm_result')
    (out/'app.c').write_text(APP);exe=out/'app.exe'
    command([cc,*flags,str(out/'app.c'),*[str(n) for n in sorted((fw/'nav').glob('*.c'))],
        str(fw/'app/nav_app.c'),str(fw/'test/hal_sim.c'),'-o',str(exe),'-lm'],'compile_app')
    app=command([str(exe)],'app_result')
    result={'passed':True,'algorithm':algorithm.strip(),'actual_app_mailbox_and_lock':app.strip(),
        'source':str(BASE),'candidate':str(candidate),'actual_CPU_timing_proof':False,'physical_stop_proven':False,
        'old249p9arrival_251processing_false_timeout_counterexample_retained':True,
        'fresh_arrival_continuity_not_healing_fault':True,'wrap32_covered':True,
        'arrival_gap_ge250ms_and_expired_processing_do_not_renew':True}
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--candidate',required=True,type=Path);p.add_argument('--proof-name',default='host_proofs');a=p.parse_args();run(a.candidate.resolve(),a.proof_name)
