#include "stop_guard.h"
#include "fixmath.h"
bool stop_fresh(const stop_guard_t *s,uint32_t now) {
    return s->have && (uint32_t)(now-s->stamp)<100000u;
}
void stop_release(stop_guard_t *s) {
    s->active=s->stable=s->fault=s->admitted=0;s->error_um=s->integral_um_s=0;s->status=STOP_MOVING;
    s->yaw_urad=s->yaw_integral_urad_s=s->lateral_um=0;
}
void stop_admission(stop_guard_t *s,bool admitted) {
    s->admitted=admitted;
    if(!admitted){s->stable=0;s->status=s->fault?STOP_FEEDBACK:STOP_ENVELOPE;}
}
void stop_feedback(stop_guard_t *s,int vx,int vy,int wz,uint32_t t) {
    if(s->have && (int32_t)(t-s->stamp)<=0)return;
    uint32_t dt=s->have?t-s->stamp:0;
    s->vx=vx;s->vy=vy;s->wz=wz;s->stamp=t;s->have=1;
    if(!dt || dt>100000u) {
        s->filtered_q8=vx*256;s->filtered_w_q8=wz*256;s->stable=0;
        if(s->active){s->fault=1;s->status=STOP_FEEDBACK;}
        return;
    }
    s->filtered_q8+=(int32_t)((int64_t)(vx*256-s->filtered_q8)*dt/(200000u+dt));
    s->filtered_w_q8+=(int32_t)((int64_t)(wz*256-s->filtered_w_q8)*dt/(200000u+dt));
    if(!s->active || s->fault)return;
    s->error_um+=(int64_t)vx*dt/1000;
    s->lateral_um+=(int64_t)vy*dt/1000;
    s->yaw_urad+=(int64_t)wz*dt/1000;
    s->yaw_integral_urad_s+=s->yaw_urad*dt/1000000;
    if(s->yaw_integral_urad_s>600000)s->yaw_integral_urad_s=600000;
    if(s->yaw_integral_urad_s< -600000)s->yaw_integral_urad_s=-600000;
    s->integral_um_s+=s->error_um*dt/1000000;
    if(s->integral_um_s>200000)s->integral_um_s=200000;
    if(s->integral_um_s< -200000)s->integral_um_s=-200000;
    bool still=s->admitted && nav_abs32(vx)<=2 && nav_abs32(vy)<=2 && nav_abs32(wz)<=50 &&
               s->error_um<=15000 && s->error_um>=-15000 &&
               s->lateral_um<=15000 && s->lateral_um>=-15000 &&
               s->yaw_urad<=50000 && s->yaw_urad>=-50000;
    if(!still)s->stable=0;
    else if(!s->stable){s->stable=1;s->stable_since=t;}
    s->status=!s->admitted?STOP_ENVELOPE:s->stable && (uint32_t)(t-s->stable_since)>=500000u?STOP_CONFIRMED:STOP_HOLDING;
}
nav_cmd_t stop_hold(stop_guard_t *s,uint32_t now,bool estop) {
    nav_cmd_t cmd={0};
    if(estop){s->active=s->stable=0;s->status=STOP_UNCONFIRMED;return cmd;}
    if(!stop_fresh(s,now)){s->fault=1;s->stable=0;s->status=STOP_FEEDBACK;return cmd;}
    if(s->fault){s->status=STOP_FEEDBACK;return cmd;}
    if(!s->active){s->active=1;s->stable=s->admitted=0;s->error_um=s->integral_um_s=0;s->yaw_urad=s->yaw_integral_urad_s=s->lateral_um=0;}
    int64_t numerator=-10*s->error_um-(int64_t)s->filtered_q8*2000/256-s->integral_um_s;
    int u=(int)((numerator+(numerator>=0?10000:-10000))/20000);
    cmd.vx=(int16_t)nav_clamp32(u,-25,25);
    int64_t wn=-10*s->yaw_urad-(int64_t)s->filtered_w_q8*2000/256-s->yaw_integral_urad_s;
    cmd.wz=(int16_t)nav_clamp32((int)((wn+(wn>=0?10000:-10000))/20000),-75,75);
    if(s->status!=STOP_CONFIRMED)s->status=s->admitted?STOP_HOLDING:STOP_ENVELOPE;
    return cmd;
}
int stop_margin(const stop_guard_t *s,const nav_cmd_t *c,uint32_t now) {
    if(!stop_fresh(s,now))return -1;
    int v=nav_max32(nav_abs32(s->vx)+nav_abs32(s->vy),nav_abs32(c->vx)+nav_abs32(c->vy));
    int w=nav_max32(nav_abs32(s->wz),nav_abs32(c->wz));
    if(v>500 || w>2000)return -1;
    /* L1 translation bound plus wheel-point angular displacement <=172*angle.
     * Isotropic tube also reserves the reverse correction, not just forward.
     * Reject large unverified envelopes rather than truncating them. */
    int translation=(v*200+999)/1000+(v*v+199)/200;
    int angle_mrad=(w*200+999)/1000+(w*w+1999)/2000;
    int margin=25+translation+(172*angle_mrad+999)/1000;
    return margin<=96?margin:-1;
}
