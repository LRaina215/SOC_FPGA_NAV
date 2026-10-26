/* Finite native-only schedule tests. @FW@ is bound by the Python entry.
 * Real candidate application/message/guard/frontier C; explicit HAL queue and
 * clock substitutions. No UART/Pico/FPGA timing or physical stop assertion.
 */
#include "nav_app.h"
#include "host_link_guard.h"
#include "nav_search_checkpoint.h"
#include "hal_sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define hal_host_recv baseline_hal_host_recv
#define hal_host_send baseline_hal_host_send
#define hal_send_cmd baseline_hal_send_cmd
#define hal_prior_enable baseline_hal_prior_enable
#include "@FW@/test/hal_sim.c"
#undef hal_host_recv
#undef hal_host_send
#undef hal_send_cmd
#undef hal_prior_enable

static void probe_step(nav_task_t *nt,uint32_t now);
static void probe_map_work(nav_task_t *nt,int rows);
#define nav_task_step probe_step
#define nav_task_map_work probe_map_work
#include "@FW@/app/nav_app.c"
#undef nav_task_step
#undef nav_task_map_work

static unsigned checks,cases;
#define REQUIRE(c) do{checks++;if(!(c)){fprintf(stderr,"FAIL line=%d expr=%s\n",__LINE__,#c);exit(1);}}while(0)
typedef struct {uint8_t bytes[PROTO_MAX_LEN];int len;uint32_t rx;} frame_t;
static frame_t queue[16];static unsigned head,count,received,refill,rx_step;
static unsigned step_calls,map_calls,pending_hook,send_calls,zero_calls,send_type_n;
static unsigned prior_delay;static uint8_t send_types[64];static uint32_t zero_at;
static uint8_t expected_header[40];

static void enqueue(const uint8_t *b,int len,uint32_t rx)
{
    REQUIRE(count<16 && len>0 && len<=PROTO_MAX_LEN);
    frame_t *f=&queue[(head+count)%16];memcpy(f->bytes,b,(size_t)len);f->len=len;f->rx=rx;count++;
}

static void heartbeat(void)
{
    uint8_t b[32];int n=proto_build_target(b,0,0,0,0,0,0,0,0);enqueue(b,n,g_sim.now_us);
}

int hal_host_recv(uint8_t *buf,int maxlen,uint32_t *rx)
{
    if(!count)return 0;
    frame_t *f=&queue[head];REQUIRE(f->len<=maxlen);int len=f->len;
    memcpy(buf,f->bytes,(size_t)len);*rx=f->rx;head=(head+1)%16;count--;received++;
    g_sim.now_us+=rx_step;if(refill)heartbeat();return len;
}

bool hal_host_send(const uint8_t *buf,int len)
{
    if(len>=4 && send_type_n<64)send_types[send_type_n++]=buf[3];
    return baseline_hal_host_send(buf,len);
}

void hal_send_cmd(const nav_cmd_t *in)
{
    nav_cmd_t cmd=*in;host_link_guard_filter(&g_host_link_guard,g_sim.now_us,&cmd);
    send_calls++;
    if(!cmd.vx&&!cmd.vy&&!cmd.wz){zero_calls++;zero_at=g_sim.now_us;}
    baseline_hal_send_cmd(&cmd);
}

void hal_prior_enable(bool enabled,int32_t ox,int32_t oy)
{
    baseline_hal_prior_enable(enabled,ox,oy);g_sim.now_us+=prior_delay;prior_delay=0;
}

static void admitted(uint32_t now)
{
    sim_reset();memset(&g_app,0,sizeof g_app);g_sim.now_us=now;nav_app_init();
    head=count=received=refill=rx_step=0;step_calls=map_calls=pending_hook=0;
    send_calls=zero_calls=send_type_n=zero_at=prior_delay=0;
    host_pending_len=0;host_pending_rx=0;g_nav_search_interrupted=false;
    g_host_link_guard=(host_link_guard_t){.have_rx=true,.last_rx_us=now};
    g_app.nt.mode=NAV_MODE_SEARCH;g_app.nt.gm.has_prior=true;
    pose_odom(&g_app.nt.pose,0,0,0,now); /* valid capture-time pose for TARGET */
    apply_host_link_lock(now); /* reset original lock-edge publication state */
}

static void map_data(void)
{
    uint8_t b[48]={0};b[0]='N';b[1]='V';b[2]=PROTO_VER;b[3]=MSG_MAP_DATA;
    b[8]='N';b[9]='V';b[10]='M';b[11]='P';b[12]=2;
    memcpy(expected_header,b+8,40);enqueue(b,48,g_sim.now_us);
}

static void probe_step(nav_task_t *nt,uint32_t now)
{
    (void)now;step_calls++;
    if(pending_hook){
        nt->stepping_local=true;REQUIRE(!nav_app_search_checkpoint());
        REQUIRE(host_pending_len==48 && g_app.prx.next==0);
        REQUIRE(!memcmp(expected_header,host_rx_buf+8,40));
        unsigned r=received;REQUIRE(!nav_app_search_checkpoint());REQUIRE(received==r);
        nt->stepping_local=false;
    }
}

static void probe_map_work(nav_task_t *nt,int rows)
{map_calls++;nav_task_map_work(nt,rows);}

static void frontier_fixture(hal_map_info_t *mi)
{
    gmap_t *g=&g_app.nt.gm;g->init=true;g->ox=g->oy=-GM_N/2;
    memset(g->grid,255,sizeof g->grid);
    for(int y=4;y<GM_N-4;y++)for(int x=4;x<GM_N-4;x++)g->grid[y*GM_N+x]=0;
    *mi=(hal_map_info_t){.inv_res=1311,.seq=1};
}

int main(void)
{
    uint8_t b[PROTO_MAX_LEN];unsigned n;
    /* A perpetually refilled finite queue cannot extend ordinary drain past4.
       Budget completion itself neither cancels a healthy search nor sends0. */
    admitted(200000);for(int i=0;i<8;i++)heartbeat();refill=1;rx_step=100;
    g_app.nt.path_valid=true;n=g_app.n_msgs;drain_host_messages(4u);
    REQUIRE(received==4 && count==8 && g_app.n_msgs==n+4);
    REQUIRE(!g_host_link_guard.locked&&!g_host_link_guard.timed_out&&!g_nav_search_interrupted);
    REQUIRE(g_app.nt.mode==NAV_MODE_SEARCH&&g_app.nt.path_valid&&send_calls==0);cases++;
    /* Full real poll reaches planner entry and status with a nonempty queue. */
    g_app.t_plan=g_app.t_status=0;n=received;nav_app_poll();
    REQUIRE(received==n+4 && count==8 && step_calls==1 && send_type_n==1 && send_types[0]==MSG_STATUS);
    REQUIRE(!g_host_link_guard.locked&&!g_nav_search_interrupted&&send_calls==0);cases++;
    /* Invalid AND guard-rejected frames consume quota. */
    admitted(1000);for(int i=0;i<8;i++){b[0]=0;enqueue(b,1,1000);}
    n=g_app.n_msgs;drain_host_messages(4u);REQUIRE(received==4&&count==4&&g_app.n_msgs==n);cases++;
    admitted(300000);for(int i=0;i<6;i++){int l=proto_build_mode(b,255);enqueue(b,l,300000);}
    drain_host_messages(4u);REQUIRE(received==4&&count==2&&g_host_link_guard.rejected==4&&!g_host_link_guard.locked);cases++;
    /* Source queue order and actual handlers: MODE,GOAL,BEGIN,DATA,END. */
    admitted(2000);int len=proto_build_mode(b,NAV_MODE_IDLE);enqueue(b,len,2000);
    len=proto_build_goal(b,1234,5678,0);enqueue(b,len,2000);
    b[0]='N';b[1]='V';b[2]=PROTO_VER;b[3]=MSG_MAP_BEGIN;b[4]=40;b[5]=b[6]=b[7]=0;enqueue(b,8,2000);
    map_data();b[3]=MSG_MAP_END;enqueue(b,4,2000);
    drain_host_messages(1u);REQUIRE(g_app.nt.mode==NAV_MODE_IDLE&&count==4);
    drain_host_messages(1u);REQUIRE(g_app.nt.goal_x==1234&&g_app.nt.goal_y==5678&&g_app.nt.mode==NAV_MODE_GOTO);
    drain_host_messages(1u);REQUIRE(g_app.prx.state==PRIOR_RECV&&g_app.prx.next==0);
    drain_host_messages(1u);REQUIRE(g_app.prx.next==40&&!memcmp(expected_header,g_app.prx.hdr,40));
    drain_host_messages(1u);REQUIRE(count==0&&g_app.prx.state==PRIOR_READY);
    REQUIRE(send_type_n==3&&send_types[0]==MSG_MAP_ACK&&send_types[1]==MSG_MAP_ACK&&send_types[2]==MSG_MAP_ACK);cases++;
    /* The fifth packet is popped only by a real checkpoint. It must survive
       pending while local coordinates are active and be dispatched before
       status reuses host_rx_buf. The sixth packet stays queued this poll. */
    admitted(200000);g_app.t_plan=g_app.t_status=0;g_app.prx=(prior_rx_t){.state=PRIOR_RECV,.total=40};
    for(int i=0;i<4;i++)heartbeat();
    map_data();len=proto_build_goal(b,777,888,0);enqueue(b,len,200000);pending_hook=1;
    nav_app_poll();REQUIRE(step_calls==1&&received==5&&count==1&&host_pending_len==0);
    REQUIRE(g_app.prx.next==40&&!memcmp(expected_header,g_app.prx.hdr,40));
    REQUIRE(g_app.n_msgs==5&&g_app.nt.goal_x!=777&&send_type_n==2);
    REQUIRE(send_types[0]==MSG_MAP_ACK&&send_types[1]==MSG_STATUS);cases++;
    /* Real frozen frontier C remains live under harmless heartbeat refill,
       and computes the same target as the finite no-packet reference. */
    hal_map_info_t mi;int32_t gx=0,gy=0,rx=0,ry=0;bool result;
    admitted(200000);frontier_fixture(&mi);
    result=explore_frontier(&g_app.nt.exp,&g_app.nt.ex_cfg,&g_app.nt.gm,&mi,&g_app.nt.pose.cur,false,&gx,&gy);
    REQUIRE(result&&!g_nav_search_interrupted);int32_t expected_x=gx,expected_y=gy;
    admitted(200000);frontier_fixture(&mi);for(int i=0;i<8;i++)heartbeat();refill=1;rx_step=100;
    result=explore_frontier(&g_app.nt.exp,&g_app.nt.ex_cfg,&g_app.nt.gm,&mi,&g_app.nt.pose.cur,false,&rx,&ry);
    REQUIRE(result&&rx==expected_x&&ry==expected_y&&received>4&&count==8);
    REQUIRE(!g_nav_search_interrupted&&!g_host_link_guard.locked&&zero_calls==0);cases++;
    /* Meaningful command/map/target changes cancel the actual frontier at its
       checkpoint, while mutation waits for the original pending dispatch. */
    for(int event=0;event<5;event++){
        admitted(2000);frontier_fixture(&mi);g_app.nt.goal_x=17;g_app.nt.goal_y=19;
        if(event==0)len=proto_build_goal(b,1234,5678,0);
        if(event==1){b[0]='N';b[1]='V';b[2]=PROTO_VER;b[3]=MSG_MAP_CLEAR;len=4;}
        if(event==2)len=proto_build_target(b,1,0,255,600,700,0,0,3);
        if(event==3)len=proto_build_mode(b,NAV_MODE_ESTOP);
        if(event==4){memset(b,0,24);b[0]='N';b[1]='V';b[2]=PROTO_VER;b[3]=MSG_SET_POSE;b[4]=0xd2;b[5]=4;len=24;}
        enqueue(b,len,2000);gx=101;gy=103;
        REQUIRE(!explore_frontier(&g_app.nt.exp,&g_app.nt.ex_cfg,&g_app.nt.gm,&mi,&g_app.nt.pose.cur,false,&gx,&gy));
        REQUIRE(g_nav_search_interrupted&&host_pending_len==len&&g_app.nt.goal_x==17&&g_app.nt.goal_y==19&&gx==101&&gy==103);
        REQUIRE(g_app.n_msgs==0&&zero_calls>=1);drain_host_messages(1u);
        REQUIRE(host_pending_len==0&&g_app.n_msgs==1);
        if(event==0)REQUIRE(g_app.nt.goal_x==1234&&g_app.nt.goal_y==5678&&g_app.nt.mode==NAV_MODE_GOTO);
        if(event==1)REQUIRE(!g_app.nt.gm.has_prior);
        if(event==2)REQUIRE(g_app.nt.tgt.valid&&g_app.nt.mode==NAV_MODE_TRACK);
        if(event==3)REQUIRE(g_app.nt.mode==NAV_MODE_ESTOP);
        if(event==4)REQUIRE(g_app.nt.pose.cur.x==1234);
        cases++;
    }
    /* Real guard boundary, latched timeout, old queued IDLE and fresh IDLE. */
    admitted(1000);g_sim.now_us=250999;apply_host_link_lock(g_sim.now_us);REQUIRE(!g_host_link_guard.locked&&zero_calls==0);
    g_sim.now_us=251000;apply_host_link_lock(g_sim.now_us);REQUIRE(g_host_link_guard.timed_out&&g_app.nt.mode==NAV_MODE_ESTOP&&zero_calls==1&&zero_at==251000);
    g_sim.now_us=252000;heartbeat();drain_host_messages(4u);REQUIRE(g_host_link_guard.locked&&g_host_link_guard.timed_out);
    len=proto_build_mode(b,NAV_MODE_IDLE);enqueue(b,len,250999);drain_host_messages(1u);REQUIRE(g_host_link_guard.locked);
    g_sim.now_us=252001;enqueue(b,len,252001);drain_host_messages(1u);REQUIRE(!g_host_link_guard.locked&&!g_host_link_guard.timed_out&&g_app.nt.mode==NAV_MODE_IDLE);cases++;
    /* Malformed continuous traffic cannot skip actual expiry publication. */
    admitted(1000);g_sim.now_us=251000;for(int i=0;i<8;i++){b[0]=0;enqueue(b,1,251000);}
    drain_host_messages(4u);REQUIRE(received==4&&count==4&&g_host_link_guard.timed_out&&zero_calls==1);cases++;
    /* Explicit synthetic callback delay tests the after-handler poll, without
       calling that chosen delay a measured C callback runtime. */
    admitted(1000);g_sim.now_us=250999;b[0]='N';b[1]='V';b[2]=PROTO_VER;b[3]=MSG_MAP_CLEAR;enqueue(b,4,250999);prior_delay=250000;
    drain_host_messages(4u);REQUIRE(g_app.n_msgs==1&&g_host_link_guard.timed_out&&zero_calls==1&&zero_at==500999);cases++;
    printf("DRAIN_SCHEDULE passed cases=%u checks=%u queue_capacity=16 quota=4 pending_quota=1 frontier_reference_target=%ld,%ld\n",cases,checks,(long)expected_x,(long)expected_y);
    return 0;
}
