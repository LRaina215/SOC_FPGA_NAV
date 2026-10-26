#include "p11_command_guard.h"
#include "p11_raw_service.h"
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned checks,frames,range_cases;
#define CHECK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static uint16_t crc16(const uint8_t *p,unsigned n)
{
    uint16_t crc=0xffffu;
    while(n--){
        crc^=(uint16_t)*p++<<8;
        for(unsigned j=0;j<8;j++){
            uint32_t shifted=(uint32_t)crc<<1;
            crc=(uint16_t)((crc&0x8000u)?shifted^0x1021u:shifted);
        }
    }
    return crc;
}
static unsigned packet(uint8_t *p,int16_t v,int16_t w,bool legacy)
{
    p[0]=0xa5;p[1]=0x5a;p[2]=0x10;p[3]=legacy?2:4;
    p[4]=(uint8_t)v;p[5]=(uint8_t)((uint16_t)v>>8);
    p[6]=(uint8_t)w;p[7]=(uint8_t)((uint16_t)w>>8);
    unsigned n=legacy?8:10;uint16_t c=crc16(p+2,n-4);p[n-2]=(uint8_t)c;p[n-1]=(uint8_t)(c>>8);return n;
}
static bool feed(p11_command_guard_t *g,const uint8_t *b,unsigned n,uint32_t stamp,bool permit)
{
    bool valid=false;frames++;
    for(unsigned i=0;i<n;i++)valid=p11_guard_byte(g,b[i],i==0,stamp,stamp+44u*(i+1u),permit)||valid;
    return valid;
}
static uint32_t setup(p11_command_guard_t *g,uint32_t now,bool arm)
{
    p11_guard_init(g);
    for(unsigned i=0;i<=50;i++)p11_guard_feedback(g,0,0,now-500000u+i*10000u,now-500000u+i*10000u,true);
    CHECK(p11_guard_stationary(g,now));
    CHECK(p11_guard_manage(g,P11_AUTHORIZE,1,now,true,true,true)==P11_OK);
    CHECK(g->state==P11_WAIT_ZERO);
    if(arm){uint8_t b[10];unsigned n=packet(b,0,0,false);CHECK(feed(g,b,n,now+1000u,true));CHECK(g->state==P11_ARMED);}
    return now+2000u;
}
static void fresh(p11_command_guard_t *g,uint32_t now)
{p11_guard_feedback(g,0,0,now,now,false);}
static void zero_proposal(p11_command_guard_t *g,uint32_t now,bool permit)
{int16_t v=123,w=456;CHECK(p11_guard_proposal(g,now,permit,&v,&w));CHECK(v==0&&w==0);}

/* Test double exposes exactly current MMIO, not a TS-overflow or valid CSR. */
typedef struct {uint32_t data[258],ts[34];unsigned dn,tn;uint32_t overflow,now;unsigned inject_after_data,ts_drops;} raw_t;
static raw_t raw;
static void raw_push(uint8_t b,bool first,uint32_t stamp)
{
    if(raw.dn<258)raw.data[raw.dn++]=0x80000000u|(first?0x100u:0)|b;else raw.overflow++;
    if(first){if(raw.tn<34)raw.ts[raw.tn++]=stamp;else raw.ts_drops++;}
}
uint32_t p11_raw_hal_count(void){return raw.dn;}
uint32_t p11_raw_hal_overflow(void){return raw.overflow;}
uint32_t p11_raw_hal_time_us(void){return raw.now;}
uint32_t p11_raw_hal_data(void)
{
    if(!raw.dn)return 0;
    uint32_t r=raw.data[0];memmove(raw.data,raw.data+1,(--raw.dn)*sizeof(uint32_t));
    unsigned n=raw.inject_after_data;raw.inject_after_data=0;
    for(unsigned i=0;i<n;i++)raw_push(0x77,true,raw.now-500u+i);
    return r;
}
uint32_t p11_raw_hal_timestamp(void)
{
    if(!raw.tn)return 0;
    uint32_t r=raw.ts[0];memmove(raw.ts,raw.ts+1,(--raw.tn)*sizeof(uint32_t));return r;
}
static void raw_packet(const uint8_t *b,unsigned n,uint32_t stamp)
{for(unsigned i=0;i<n;i++)raw_push(b[i],i==0,stamp);}

int main(void)
{
    p11_command_guard_t g;uint8_t b[10];int16_t v,w;bool edge;
    p11_guard_init(&g);packet(b,100,100,false);CHECK(!feed(&g,b,10,10000,true));CHECK(!p11_guard_proposal(&g,11000,true,&v,&w));
    uint32_t now=setup(&g,1000000,false);
    packet(b,100,100,false);CHECK(!feed(&g,b,10,now,true));CHECK(g.state==P11_WAIT_ZERO);zero_proposal(&g,now+500,true);
    packet(b,0,0,false);CHECK(feed(&g,b,10,now+1000,true));CHECK(g.state==P11_ARMED);
    packet(b,300,-1000,false);CHECK(feed(&g,b,10,now+2000,true));CHECK(g.forward_mmps==300&&g.yaw_mradps==-1000);
    packet(b,-600,0,true);CHECK(feed(&g,b,8,now+3000,true));CHECK(g.forward_mmps==-600&&g.yaw_mradps==0);
    for(int value=INT16_MIN;value<=INT16_MAX;value++){
        now=setup(&g,1000000,true);packet(b,(int16_t)value,0,false);
        bool accepted=feed(&g,b,10,now,true);CHECK(accepted==(value>=-600&&value<=600));
        CHECK(g.forward_mmps==(accepted?value:0));range_cases++;
        now=setup(&g,1000000,true);packet(b,0,(int16_t)value,false);
        accepted=feed(&g,b,10,now,true);CHECK(accepted==(value>=-2900&&value<=2900));
        CHECK(g.yaw_mradps==(accepted?value:0));range_cases++;
    }
    for(unsigned bit=0;bit<80;bit++){
        now=setup(&g,1000000,true);packet(b,300,-1000,false);b[bit/8]^=(uint8_t)(1u<<(bit%8));
        CHECK(!feed(&g,b,10,now,true));CHECK(g.forward_mmps==0&&g.yaw_mradps==0);
    }
    now=setup(&g,1000000,true);uint32_t arrival=g.arrival_us;
    fresh(&g,arrival+199999u);CHECK(!p11_guard_poll(&g,arrival+199999u,true));CHECK(g.state==P11_ARMED);
    fresh(&g,arrival+200000u);CHECK(p11_guard_poll(&g,arrival+200000u,true));CHECK(g.state==P11_LOCKED);zero_proposal(&g,arrival+200000u,true);
    packet(b,0,0,false);CHECK(!feed(&g,b,10,arrival+201000u,true));CHECK(g.state==P11_LOCKED);
    now=setup(&g,1000000,true);CHECK(p11_guard_poll(&g,g.feedback_us+50000u,true));CHECK(g.state==P11_LOCKED);
    now=setup(&g,1000000,true);CHECK(p11_guard_poll(&g,now,false));CHECK(g.state==P11_LOCKED);
    for(unsigned i=0;i<100;i++){fresh(&g,now+i*1000);packet(b,600,2900,false);CHECK(!feed(&g,b,10,now+i*1000,false));zero_proposal(&g,now+i*1000,false);}
    CHECK(!feed(&g,b,10,now+101000,true));CHECK(g.state==P11_LOCKED);
    now+=700000u;for(unsigned i=0;i<=50;i++)p11_guard_feedback(&g,0,0,now-500000u+i*10000u,now-500000u+i*10000u,true);
    CHECK(p11_guard_manage(&g,P11_AUTHORIZE,2,now,true,true,true)==P11_OK);packet(b,0,0,false);CHECK(feed(&g,b,10,now+1000,true));
    CHECK(p11_guard_manage(&g,P11_RELEASE,3,now+2000,false,true,true)==P11_NOT_TRUSTED);
    CHECK(p11_guard_manage(&g,P11_RELEASE,3,now+2000,true,false,true)==P11_NOT_IDLE);
    CHECK(p11_guard_manage(&g,P11_RELEASE,3,now+2000,true,true,false)==P11_NOT_DRAINED);
    CHECK(p11_guard_manage(&g,P11_RELEASE,2,now+2000,true,true,true)==P11_BAD_SESSION);
    CHECK(p11_guard_manage(&g,P11_RELEASE,3,now+2000,true,true,true)==P11_OK);CHECK(g.state==P11_OWNER_OFF);
    now=setup(&g,UINT32_MAX-500u,true);packet(b,600,-2900,false);CHECK(feed(&g,b,10,now,true));CHECK(g.forward_mmps==600&&g.yaw_mradps==-2900);
    now=setup(&g,1000000,true);packet(b,300,-1000,false);CHECK(!feed(&g,b,10,999999u,true));CHECK(g.forward_mmps==0);
    now=setup(&g,1000000,true);memset(&raw,0,sizeof raw);raw.now=now+500;raw_packet(b,10,now);p11_raw_epoch_t ep={0,true};
    CHECK(p11_raw_service(&g,&ep,true,&edge)==10);CHECK(!edge);CHECK(g.forward_mmps==300&&g.yaw_mradps==-1000);
    now=setup(&g,1000000,true);memset(&raw,0,sizeof raw);raw.now=now+500;raw_packet(b,10,0);ep=(p11_raw_epoch_t){0,true};
    CHECK(p11_raw_service(&g,&ep,true,&edge)==0);CHECK(edge&&g.state==P11_LOCKED&&!ep.clean_epoch);
    now=setup(&g,1000000,true);memset(&raw,0,sizeof raw);raw.now=now+500;raw_packet(b,10,now);raw.overflow=1;ep=(p11_raw_epoch_t){0,true};
    CHECK(p11_raw_service(&g,&ep,true,&edge)==0);CHECK(edge&&g.state==P11_LOCKED);
    for(unsigned backlog=31;backlog<=258;backlog++){
        now=setup(&g,1000000,true);memset(&raw,0,sizeof raw);raw.now=now+500;
        for(unsigned i=0;i<backlog;i++)raw_push(0x77,true,now+i);
        ep=(p11_raw_epoch_t){0,true};
        CHECK(p11_raw_service(&g,&ep,true,&edge)==0);CHECK(edge&&g.state==P11_LOCKED);zero_proposal(&g,now,true);
    }
    now=setup(&g,1000000,true);memset(&raw,0,sizeof raw);raw.now=now+500;raw_packet(b,10,now);raw.inject_after_data=35;ep=(p11_raw_epoch_t){0,true};
    CHECK(p11_raw_service(&g,&ep,true,&edge)==0);CHECK(edge&&g.state==P11_LOCKED);
    /* A permit loss discovered INSIDE the byte call must still publish zero
       immediately; returning a missed edge would wait for a later app poll. */
    now=setup(&g,1000000,true);memset(&raw,0,sizeof raw);raw.now=now+500;raw_packet(b,10,now);ep=(p11_raw_epoch_t){0,true};
    p11_raw_service(&g,&ep,false,&edge);CHECK(g.state==P11_LOCKED);CHECK(edge);zero_proposal(&g,now,false);
    printf("PASS P11_COMPONENT checks=%u frames=%u full_signed_range_cases=%u sizeof_guard=%zu sizeof_epoch=%zu\n",checks,frames,range_cases,sizeof g,sizeof ep);return 0;
}
