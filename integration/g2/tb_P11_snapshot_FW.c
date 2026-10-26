#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include "hal.h"
#include "nav_regs_map.h"
#include "p11_snapshot_service.h"
static unsigned checks,range_cases,cut_cases,zero_writes,command_writes;
#define CHECK(x)do{checks++;if(!(x)){fprintf(stderr,"FAIL line%d %s\n",__LINE__,#x);exit(1);}}while(0)
static uint32_t now_us,hw_seq,hw_word,hw_ts,event_mask,read_index;
static uint32_t raw_data[258],raw_ts[34],raw_n,raw_tn;
static uint32_t baud,control_writes,last_safe_v,last_safe_w,first_zero_at;
static bool parser_enabled;
static uint8_t reply_bytes[32];static int reply_length;
static void publish(int16_t v,int16_t w,uint32_t stamp)
{if(hw_seq!=UINT32_MAX){hw_word=(uint16_t)v|((uint32_t)(uint16_t)w<<16);hw_ts=stamp;hw_seq++;}}
static void before_read(unsigned role)
{
    CHECK(read_index<4);CHECK(role==(read_index==0||read_index==3?0:read_index));
    if(event_mask&(1u<<read_index))publish((int16_t)((hw_seq+1)*17u),(int16_t)(-(int32_t)((hw_seq+1)*31u)),1000000u+hw_seq+1u);
    read_index++;
}
uint32_t p11_snapshot_hal_seq(void){before_read(0);return hw_seq;}
uint32_t p11_snapshot_hal_command(void){before_read(1);return hw_word;}
uint32_t p11_snapshot_hal_timestamp(void){before_read(2);return hw_ts;}
uint32_t p11_snapshot_hal_time_us(void){return now_us;}
uint32_t hal_time_us(void){return now_us;}
uint32_t p11_raw_hal_count(void){return raw_n;}
uint32_t p11_raw_hal_timestamp(void)
{if(!raw_tn)return 0;uint32_t r=raw_ts[0];memmove(raw_ts,raw_ts+1,--raw_tn*sizeof *raw_ts);return r;}
static uint32_t nav_rd(uint32_t address)
{
    if(address==NAV_IMU_RX_DATA){if(!raw_n)return 0;uint32_t r=raw_data[0];memmove(raw_data,raw_data+1,--raw_n*sizeof *raw_data);return r;}
    if(address==NAV_IMU_RX_TS)return p11_raw_hal_timestamp();
    if(address==NAV_IMU_RX_COUNT)return raw_n;
    CHECK(0);return 0;
}
static void nav_wr(uint32_t address,uint32_t value)
{
    if(address==NAV_SAFE_CMD_V)last_safe_v=value;
    else if(address==NAV_SAFE_CMD_W){last_safe_w=value;command_writes++;
        if(!(last_safe_v&0xffffffffu)&&!(value&65535u)){zero_writes++;if(!first_zero_at)first_zero_at=now_us;}}
    else CHECK(0);
}
void hal_raw_uart_config(uint32_t divisor,uint32_t idle)
{CHECK(divisor==217||divisor==868);CHECK(idle==500);baud=divisor;}
void hal_p11_parser_control(uint32_t enable)
{CHECK(enable<=1);control_writes++;hw_seq=hw_word=hw_ts=0;parser_enabled=enable!=0;}
bool hal_host_send(const uint8_t *data,int n)
{CHECK(n<=32);memcpy(reply_bytes,data,n);reply_length=n;return true;}
/* Generated from exact experimental app prefix and actual final HAL functions.
 * The substitutes above implement proposed/actual CSR behavior only. */
#include "actual_app_prefix.inc"
#include "actual_hal_send_imu.inc"

static void feedback(p11_command_guard_t *g,uint32_t t,bool zero)
{p11_guard_feedback(g,0,0,t,t,zero);}
static uint32_t setup(p11_command_guard_t *g,uint32_t auth,bool arm)
{
    p11_guard_init(g);
    for(unsigned i=0;i<=50;i++)feedback(g,auth-500000u+i*10000u,true);
    CHECK(p11_guard_manage(g,P11_AUTHORIZE,1,auth,true,true,true)==P11_OK);
    if(arm)CHECK(p11_guard_capture(g,0,0,auth+1000u,auth+1500u,true));
    return auth+2000u;
}
static void service(p11_command_guard_t *g,uint32_t *seq,bool permit,bool *edge)
{read_index=0;event_mask=0;*edge=p11_snapshot_service(g,seq,permit);CHECK(read_index==0||read_index==4);}
static uint32_t le32(const uint8_t *p)
{return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static void app_setup(void)
{
    memset(&g_app,0,sizeof g_app);raw_n=raw_tn=0;hw_seq=hw_word=hw_ts=0;
    event_mask=read_index=0;now_us=1000000;zero_writes=command_writes=first_zero_at=0;
    p11_guard_init(&p11);p11_set_protocol(0);p11_zero_published=false;
    host_link_guard_init(&g_host_link_guard,now_us);
    proto_msg_t idle={.type=MSG_MODE};idle.u.mode.mode=NAV_MODE_IDLE;
    CHECK(host_link_guard_observe(&g_host_link_guard,&idle,now_us,now_us));CHECK(!g_host_link_guard.locked);
    for(unsigned i=0;i<=50;i++)feedback(&p11,now_us-500000u+i*10000u,true);
}
static void app_manage(unsigned action,unsigned session)
{proto_msg_t m={.type=MSG_UART_OWNER};m.u.uart.action=action;m.u.uart.session=session;p11_manage(&m);CHECK(reply_length==16);CHECK(reply_bytes[3]==0x85);}

int main(void)
{
    p11_command_guard_t g;uint32_t t,seq;bool edge;
    unsigned long_frames=0;
#ifndef APP_ONLY
    /* All commit cuts between four sequential reads. No concurrent CTRL write. */
    for(unsigned mask=0;mask<16;mask++){
        hw_seq=0;publish(17,-31,1000001);event_mask=mask;read_index=0;
        p11_snapshot_t s={0};p11_snapshot_result_t r=p11_snapshot_read(0,&s);CHECK(read_index==4);
        if(r==P11_SNAPSHOT_NEW){CHECK(s.sequence==1);CHECK(s.forward_mmps==17);CHECK(s.yaw_mradps==-31);CHECK(s.start_us==1000001);}
        else CHECK(r==P11_SNAPSHOT_LOST||r==P11_SNAPSHOT_UNSTABLE);
        cut_cases++;
    }
    event_mask=0;read_index=0;hw_seq=0;CHECK(p11_snapshot_read(1,&(p11_snapshot_t){0})==P11_SNAPSHOT_LOST);
    read_index=0;CHECK(p11_snapshot_read(0,&(p11_snapshot_t){0})==P11_SNAPSHOT_NONE);
    read_index=0;hw_seq=0xffffffff;CHECK(p11_snapshot_read(0xfffffffe,&(p11_snapshot_t){0})==P11_SNAPSHOT_LOST);
    /* Preserve the explicit out-of-contract ABA witness: reset plus re-publish
       can reuseSEQ1. Serialized sole-FW CTRL writes are a necessary precondition. */
    hw_seq=1;hw_word=100;hw_ts=1000;uint32_t a=hw_seq,word=hw_word;
    hw_seq=hw_word=hw_ts=0;publish(200,10,2000);uint32_t stamp=hw_ts,b=hw_seq;
    CHECK(a==b&&word!=hw_word&&stamp==2000);
    printf("KNOWN_EXTERNAL_CTRL_ABA oldSEQ=%u newSEQ=%u mixedCMD=%u newTS=%u outside_serialized_owner_contract=1\n",a,b,word,stamp);
    for(int value=INT16_MIN;value<=INT16_MAX;value++){
        t=setup(&g,1000000,true);bool ok=p11_guard_capture(&g,(int16_t)value,0,t,t+500,true);
        CHECK(ok==(value>=-600&&value<=600));range_cases++;
        t=setup(&g,1000000,true);ok=p11_guard_capture(&g,0,(int16_t)value,t,t+500,true);
        CHECK(ok==(value>=-2900&&value<=2900));range_cases++;
    }
    t=setup(&g,1000000,false);CHECK(!p11_guard_capture(&g,0,0,999999,1001000,true));CHECK(g.state==P11_WAIT_ZERO);
    CHECK(!p11_guard_capture(&g,1,0,1001000,1001500,true));CHECK(g.state==P11_WAIT_ZERO);
    CHECK(p11_guard_capture(&g,0,0,1002000,1002500,true));
    CHECK(!p11_guard_capture(&g,100,0,1002000,1002600,true));
    CHECK(!p11_guard_capture(&g,0,0,0,1002600,true));
    feedback(&g,1202000,false);CHECK(p11_guard_poll(&g,1202000,true));CHECK(g.state==P11_LOCKED);
    CHECK(!p11_guard_capture(&g,0,0,1203000,1203500,true));
    /* >2^31us authorization age, then a real unsigned32 time wrap. No state force. */
    uint32_t auth=1000000;t=setup(&g,auth,true);long_frames=233000;
    for(unsigned i=0;i<long_frames;i++){
        t=auth+2000u+20000u*i;feedback(&g,t,false);
        CHECK(p11_guard_capture(&g,100,-100,t,t+440,true));CHECK(g.state==P11_ARMED);
    }
    printf("LONG_SESSION frames=%u elapsed64_us=%llu crossed_half=1 crossed_full_time_wrap=1\n",long_frames,(unsigned long long)2000+20000ull*(long_frames-1));
    /* A sequence gap is immediately latched, and subsequent valid snapshots
       cannot recover it. Rejecting a nonzero WAIT snapshot still consumesSEQ. */
    t=setup(&g,1000000,false);now_us=t;seq=0;hw_seq=0;publish(100,0,t);service(&g,&seq,true,&edge);
    CHECK(!edge&&seq==1&&g.state==P11_WAIT_ZERO);
    now_us=t+2000;publish(0,0,t+1000);service(&g,&seq,true,&edge);CHECK(!edge&&seq==2&&g.state==P11_ARMED);
    publish(100,0,t+3000);publish(100,0,t+4000);now_us=t+4500;service(&g,&seq,true,&edge);CHECK(edge&&g.state==P11_LOCKED);
    publish(0,0,t+5000);service(&g,&seq,true,&edge);CHECK(!edge&&g.state==P11_LOCKED);
 #endif
    /* Warm CPU HOLD preserves NAV state, so the actual init pair must flush
       an old hardware snapshot explicitly and leave parser/owner disarmed. */
    app_setup();hw_seq=7;hw_word=600;hw_ts=now_us;parser_enabled=true;
    p11_guard_init(&p11);host_link_guard_init(&g_host_link_guard,now_us);p11_set_protocol(0);
    CHECK(!hw_seq&&!hw_word&&!hw_ts&&!parser_enabled&&p11.state==P11_OWNER_OFF&&g_host_link_guard.locked);
    nav_cmd_t warm_cmd={100,0,20,0};hal_send_cmd(&warm_cmd);CHECK(last_safe_v==0&&(last_safe_w&65535u)==0);
    /* Actual experimental app helpers + original final HAL gate. */
    app_setup();g_app.nt.tgt.valid=true;g_app.nt.wp_n=2;g_app.nt.path_valid=true;g_app.nt.search_goal_valid=true;
    app_manage(1,1);CHECK(le32(reply_bytes+4)==P11_OK);CHECK(p11.state==P11_WAIT_ZERO);CHECK(!g_app.nt.tgt.valid&&!g_app.nt.wp_n&&!g_app.nt.path_valid&&!g_app.nt.search_goal_valid);
    uint32_t authorized=p11.authorized_us;CHECK(control_writes>=3);CHECK(parser_enabled);
    /* Old in-flight FIRST can arrive after enabling; sequence alone must not arm. */
    now_us=authorized+100;publish(0,0,authorized-10);read_index=0;p11_raw_work(now_us);CHECK(p11.state==P11_WAIT_ZERO&&p11_sequence==1);
    now_us=authorized+2000;publish(0,0,authorized+1000);read_index=0;p11_raw_work(now_us);CHECK(p11.state==P11_ARMED&&p11_sequence==2);
    now_us=authorized+21000;publish(100,20,authorized+20000);read_index=0;p11_raw_work(now_us);p11_note_proposal(now_us);
    CHECK(g_app.nt.manual_owner&&g_app.nt.manual_v==100&&g_app.nt.manual_w==20);
    nav_cmd_t cmd={100,0,20,0};hal_send_cmd(&cmd);CHECK((int16_t)last_safe_v==100&&(int16_t)last_safe_w==20);
    now_us=authorized+220000;feedback(&p11,now_us,false);unsigned oldzero=zero_writes;read_index=0;p11_raw_work(now_us);
    CHECK(p11.state==P11_LOCKED&&zero_writes==oldzero+1);CHECK(last_safe_v==0&&(last_safe_w&65535u)==0);
    /* R0 expires despite continuing hardware P11 command publications. */
    app_setup();app_manage(1,2);authorized=p11.authorized_us;
    now_us=authorized+1000;publish(0,0,now_us);read_index=0;p11_raw_work(now_us);
    unsigned beforelosszero=zero_writes;first_zero_at=0;
    for(unsigned i=1;i<=30;i++){
        now_us=authorized+i*10000;feedback(&p11,now_us,false);
        host_link_guard_poll(&g_host_link_guard,now_us);
        publish(100,20,now_us);read_index=0;p11_raw_work(now_us);p11_note_proposal(now_us);
        hal_send_cmd(&cmd);
        if(i>=25)CHECK(last_safe_v==0&&(last_safe_w&65535u)==0);
    }
    CHECK(g_host_link_guard.locked&&p11.state==P11_LOCKED&&zero_writes>beforelosszero);
    CHECK(first_zero_at==authorized+250000);
    /* Explicit release to WIT preserves a bounded post-switch drain phase,
       finalzero and halfpacket reset; parser byte consumer remains disabled. */
    app_setup();app_manage(3,3);CHECK(le32(reply_bytes+4)==P11_OK);
    CHECK(!p11_protocol&&p11_wit_pending&&baud==868);
    now_us+=100;raw_data[0]=0x80000155;raw_ts[0]=now_us-200;raw_n=raw_tn=1;
    p11_raw_work(now_us);CHECK(!raw_n&&!raw_tn&&p11_wit_pending);
    g_app.imu.n=7; /* explicit parser partial-state fixture, not a hardware force */
    now_us+=1999;p11_raw_work(now_us);CHECK(p11_wit_pending);
    hal_send_cmd(&cmd);CHECK(last_safe_v==0&&(last_safe_w&65535u)==0);
    now_us+=1;p11_raw_work(now_us);CHECK(!p11_wit_pending);CHECK(g_app.imu.n==0);
    /* Full old258-byte /34timestamp queues cannot escape to WIT. Every
       invocation drains <=32 bytes; a newly arriving byte restarts quiet. */
    app_setup();app_manage(3,4);raw_n=258;raw_tn=34;
    for(unsigned i=0;i<258;i++)raw_data[i]=0x80000155u;
    for(unsigned i=0;i<34;i++)raw_ts[i]=now_us-1000u+i;
    unsigned drain_calls=0;
    while(raw_n){unsigned before=raw_n;now_us+=1000;p11_raw_work(now_us);
        CHECK(before-raw_n<=32);CHECK(p11_wit_pending);drain_calls++;}
    CHECK(drain_calls==9&&!raw_tn);
    now_us+=1999;raw_data[0]=0x80000155u;raw_ts[0]=now_us;raw_n=raw_tn=1;p11_raw_work(now_us);
    CHECK(p11_wit_pending&&!raw_n&&!raw_tn);
    now_us+=1999;p11_raw_work(now_us);CHECK(p11_wit_pending);
    now_us+=1;p11_raw_work(now_us);CHECK(!p11_wit_pending);
    printf("WIT_HANDOVER full_old_bytes=258 full_old_TS=34 drain_calls=%u max_bytes_per_call=32 quiet_restart=1\n",drain_calls);
    printf("PASS P11_SNAPSHOT_COMPONENT checks=%u range_cases=%u four_read_cut_cases=%u long_frames=%u guard_bytes=%zu actual_app_helpers=1 original_HAL_gate=1\n",checks,range_cases,cut_cases,long_frames,sizeof g);
    return 0;
}
