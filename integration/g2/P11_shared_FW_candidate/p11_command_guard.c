#include "p11_command_guard.h"
#include <string.h>

#if defined(__GNUC__)
__attribute__((optimize("no-optimize-crc")))
#endif
static uint16_t crc_byte(uint16_t old, uint8_t value)
{
    uint16_t c=(uint16_t)(old^((uint16_t)value<<8));
    for(unsigned i=0;i<8;i++){
        uint32_t shifted=(uint32_t)c<<1;
        c=(uint16_t)((c&0x8000u)?shifted^0x1021u:shifted);
    }
    return c;
}
static bool lock_zero(p11_command_guard_t *g)
{
    bool edge=g->state!=P11_LOCKED;
    g->state=P11_LOCKED;g->forward_mmps=g->yaw_mradps=0;
    g->have_arrival=false;g->count=0;g->have_burst=false;
    if(edge)g->locks++;
    return edge;
}
void p11_guard_init(p11_command_guard_t *g)
{
    memset(g,0,sizeof *g);g->state=P11_OWNER_OFF;
}
bool p11_guard_stationary(const p11_command_guard_t *g,uint32_t now)
{
    return g->have_feedback&&g->still&&
        (uint32_t)(now-g->feedback_us)<P11_FEEDBACK_AGE_US&&
        (uint32_t)(now-g->still_since_us)>=P11_STILL_DWELL_US;
}
void p11_guard_feedback(p11_command_guard_t *g,int16_t vx,int16_t wz,
                       uint32_t stamp,uint32_t now,bool published_zero)
{
    bool fresh=(uint32_t)(now-stamp)<P11_FEEDBACK_AGE_US;
    bool consecutive=g->have_feedback&&
        (uint32_t)(stamp-g->feedback_us)<P11_FEEDBACK_AGE_US;
    bool slow=fresh&&published_zero&&vx>=-5&&vx<=5&&wz>=-20&&wz<=20;
    if(!slow||!consecutive||!g->still)g->still_since_us=stamp;
    g->feedback_us=stamp;g->have_feedback=fresh;g->still=slow;
}
p11_result_t p11_guard_manage(p11_command_guard_t *g,p11_action_t action,
    uint32_t session,uint32_t now,bool trusted_R0,bool nav_idle,bool raw_drained_quiet)
{
    if(action==P11_QUERY)return P11_OK;
    if(action!=P11_AUTHORIZE&&action!=P11_RELEASE)return P11_BAD_ACTION;
    if(!trusted_R0)return P11_NOT_TRUSTED;
    if(!nav_idle)return P11_NOT_IDLE;
    if(!p11_guard_stationary(g,now))return P11_NOT_STILL;
    if(!raw_drained_quiet)return P11_NOT_DRAINED;
    if(session==0||session==g->session)return P11_BAD_SESSION;
    g->session=session;g->authorized_us=now;g->arrival_us=0;
    g->forward_mmps=g->yaw_mradps=0;g->have_arrival=false;
    g->count=0;g->total=0;g->have_burst=false;g->burst_used=false;
    g->state=action==P11_AUTHORIZE?P11_WAIT_ZERO:P11_OWNER_OFF;
    return P11_OK;
}
bool p11_guard_poll(p11_command_guard_t *g,uint32_t now,bool R0_permit)
{
    if(g->state==P11_OWNER_OFF||g->state==P11_LOCKED)return false;
    if(!R0_permit||!g->have_feedback||
       (uint32_t)(now-g->feedback_us)>=P11_FEEDBACK_AGE_US)return lock_zero(g);
    uint32_t since=g->have_arrival?g->arrival_us:g->authorized_us;
    return (uint32_t)(now-since)>=P11_COMMAND_AGE_US?lock_zero(g):false;
}
bool p11_guard_quarantine(p11_command_guard_t *g)
{
    if(g->state==P11_OWNER_OFF)return false;
    return lock_zero(g);
}
bool p11_guard_byte(p11_command_guard_t *g,uint8_t byte,bool first,
                    uint32_t stamp,uint32_t now,bool R0_permit)
{
    p11_guard_poll(g,now,R0_permit);
    if(g->state!=P11_WAIT_ZERO&&g->state!=P11_ARMED)return false;
    if(first){g->have_burst=true;g->burst_us=stamp;g->burst_used=false;g->count=0;}
    if(!g->have_burst||g->burst_used)return false;
    if(g->burst_us==0||(uint32_t)(now-g->burst_us)>=P11_COMMAND_AGE_US||
       (int32_t)(g->burst_us-g->authorized_us)<0){g->time_bad++;g->have_burst=false;return false;}
    unsigned n=g->count;
    if(n==0){if(byte==0xa5u){g->bytes[0]=byte;g->count=1;}return false;}
    if(n==1&&byte!=0x5au){g->count=byte==0xa5u?1:0;return false;}
    if(n==2&&byte!=0x10u){g->count=byte==0xa5u?1:0;return false;}
    if(n==3){
        if(byte!=2&&byte!=4){g->count=byte==0xa5u?1:0;return false;}
        g->total=(uint8_t)(byte+6u);
    }
    g->bytes[n]=byte;g->count=(uint8_t)(n+1u);
    if(g->count<4||g->count!=g->total)return false;
    g->count=0;uint16_t crc=0xffffu;
    for(unsigned i=2;i<(unsigned)g->total-2u;i++)crc=crc_byte(crc,g->bytes[i]);
    if(crc!=(uint16_t)(g->bytes[g->total-2u]|((uint16_t)g->bytes[g->total-1u]<<8))){g->crc_bad++;g->rejected++;return false;}
    int16_t v=(int16_t)(g->bytes[4]|((uint16_t)g->bytes[5]<<8));
    int16_t w=g->bytes[3]==4?(int16_t)(g->bytes[6]|((uint16_t)g->bytes[7]<<8)):0;
    if(v< -600||v>600||w< -2900||w>2900){g->range_bad++;g->rejected++;return false;}
    g->burst_used=true;
    if(g->have_arrival&&(int32_t)(g->burst_us-g->arrival_us)<=0){g->time_bad++;g->rejected++;return false;}
    if(g->state==P11_WAIT_ZERO&&(v!=0||w!=0||!p11_guard_stationary(g,now))){g->rejected++;return false;}
    g->forward_mmps=v;g->yaw_mradps=w;g->arrival_us=g->burst_us;
    g->have_arrival=true;g->state=P11_ARMED;g->accepted++;return true;
}
bool p11_guard_proposal(p11_command_guard_t *g,uint32_t now,bool R0_permit,
                        int16_t *forward,int16_t *yaw)
{
    p11_guard_poll(g,now,R0_permit);
    if(g->state==P11_OWNER_OFF)return false;
    *forward=g->state==P11_ARMED?g->forward_mmps:0;
    *yaw=g->state==P11_ARMED?g->yaw_mradps:0;
    return true;
}
