#include "support_guard.h"
#include "fixmath.h"

static bool enabled,boot_set,moved,scan_done;
static int boot_mm;
static int32_t boot_x,boot_y,boot_z;
static uint32_t unknowns,blocked,assumed,remembered;
void support_reset(int r) {enabled=true;boot_set=moved=false;boot_mm=r;scan_done=r<=0;unknowns=blocked=assumed=remembered=0;}
void support_enable(bool on) {enabled=on;}
void support_frame_jump(void) {if(boot_set || moved) boot_mm=0;boot_set=false;scan_done=boot_mm<=0;}
bool support_scan_needed(void) {return enabled && boot_mm>0 && !scan_done;}
void support_scan_finish(void) {scan_done=true;}
void support_note_cmd(const nav_cmd_t *c) {if(c->vx || c->vy) moved=true;}
void support_diagnostics(uint32_t v[4]) {v[0]=unknowns;v[1]=blocked;v[2]=assumed;v[3]=remembered;}
int support_transition(int al,int ah,int bl,int bh,int cap,int q) {
    int lo=bl-ah-2*q,hi=bh-al+2*q;
    if(lo>cap || hi< -cap) return SUPPORT_BLOCK;
    return lo>=-cap && hi<=cap?SUPPORT_PASS:SUPPORT_UNKNOWN;
}
typedef struct {int32_t x,y;int16_t lo,hi;uint8_t kind;} cell_t;
typedef struct {cell_t cell[32];uint32_t used;uint16_t seq,inv;} query_t;

static int cell(query_t *q,int32_t ix,int32_t iy,cell_t *out) {
    unsigned k=((uint32_t)ix*17u+(uint32_t)iy*7u)&31u;
    if((q->used&(1u<<k)) && q->cell[k].x==ix && q->cell[k].y==iy) {*out=q->cell[k];return out->kind?SUPPORT_PASS:SUPPORT_UNKNOWN;}
    int32_t x=(int32_t)(((int64_t)ix*65536+32768)/q->inv);
    int32_t y=(int32_t)(((int64_t)iy*65536+32768)/q->inv);
    cell_t c={.x=ix,.y=iy};hal_support_t s;
    if(hal_map_support(x,y,&s) && s.count && s.lo<=s.hi) {
        if(s.seq!=q->seq) return SUPPORT_FRAME;
        c.lo=s.lo;c.hi=s.hi;
        c.kind=s.retained_stale || s.age>60?2:1;
        if(c.kind==2) remembered++;
    } else if(boot_mm>0) {
        /* 全格落在初始已声明平地内才借用启动假设。 */
        int32_t half=(65536/q->inv+1)/2;
        int64_t dx=nav_abs32(x-boot_x)+half+1,dy=nav_abs32(y-boot_y)+half+1;
        if(dx*dx+dy*dy<=(int64_t)boot_mm*boot_mm) {c.lo=c.hi=(int16_t)boot_z;c.kind=3;assumed++;}
    }
    q->used|=1u<<k;q->cell[k]=c;*out=c;
    return c.kind?SUPPORT_PASS:SUPPORT_UNKNOWN;
}
static int patch(query_t *q,int32_t x,int32_t y,int a,int b,int32_t sn_,int32_t cs_,cell_t *center) {
    int32_t ix=(int32_t)(((int64_t)x*q->inv)>>16),iy=(int32_t)(((int64_t)y*q->inv)>>16);
    int status=cell(q,ix,iy,center);if(status) return status;
    int rx=(a*nav_abs32(cs_)+b*nav_abs32(sn_)+16383)/16384;
    int ry=(a*nav_abs32(sn_)+b*nav_abs32(cs_)+16383)/16384;
    int x0=(int32_t)(((int64_t)(x-rx)*q->inv)>>16),x1=(int32_t)(((int64_t)(x+rx)*q->inv)>>16);
    int y0=(int32_t)(((int64_t)(y-ry)*q->inv)>>16),y1=(int32_t)(((int64_t)(y+ry)*q->inv)>>16);
    int32_t px=(int32_t)(((int64_t)x*q->inv)>>8),py=(int32_t)(((int64_t)y*q->inv)>>8);
    int32_t aq=(a*q->inv+255)>>8,bq=(b*q->inv+255)>>8;
    int32_t cell_projection=128*(nav_abs32(cs_)+nav_abs32(sn_));
    for(int yy=y0;yy<=y1;yy++) for(int xx=x0;xx<=x1;xx++) {
        int32_t dx=xx*256+128-px,dy=yy*256+128-py;
        int64_t forward=(int64_t)dx*cs_+(int64_t)dy*sn_;
        int64_t lateral=-(int64_t)dx*sn_+(int64_t)dy*cs_;
        int64_t af=(int64_t)aq*16384+cell_projection,al=(int64_t)bq*16384+cell_projection;
        if(forward>af || forward< -af || lateral>al || lateral< -al) continue;
        cell_t c;status=cell(q,xx,yy,&c);if(status) return status;
        /* Within-cell ambiguity is not converted into extra step capability. */
        if(c.hi-c.lo+2>20) return c.hi-c.lo-2>20?SUPPORT_BLOCK:SUPPORT_UNKNOWN;
        status=support_transition(center->lo,center->hi,c.lo,c.hi,20,1);
        if(status) return status;
    }
    return SUPPORT_PASS;
}
static int32_t sn(angle_t a) {int f=a&63;return (fx_sin(a)*(64-f)+fx_sin((angle_t)(a+64))*f)/64;}
int support_check(const nav_pose_t *p,const lp_cfg_t *cfg,const hal_map_info_t *mi,const nav_cmd_t *cmd) {
    if(!enabled || (!cmd->vx && !cmd->vy && !cmd->wz)) return SUPPORT_PASS;
    if(!mi->inv_res) return SUPPORT_UNKNOWN;
    if(!boot_set) {boot_x=p->x;boot_y=p->y;boot_z=p->z;boot_set=true;}
    query_t q={.used=0,.seq=mi->seq,.inv=mi->inv_res};
    cell_t previous[2]={{0}};bool have[2]={false,false};
    cell_t grade[2]={{0}};int32_t grade_x[2]={0},grade_y[2]={0};
    int steps=(cfg->safe_steps*cfg->dt_ms+49)/50;
    if(steps>64) return SUPPORT_UNKNOWN;
    if(!cmd->vx && !cmd->vy) steps=4; /* 一规划周期加100ms响应；持续转动每周期复核 */
    /* 旧CAD半径33.65/胎宽26.6mm，分别上取34/14；当前实物待复核。
     * 50ms样本间任意轮上点到最近端点的位移上界speed/40，另加2mm数值裕量。
     * 矩形是整个轮胎水平投影，不把轮半径误当胎面横向宽度。 */
    int speed=nav_abs32(cmd->vx)+nav_abs32(cmd->vy)+(nav_abs32(cmd->wz)*172+999)/1000;
    int margin=(speed+39)/40+2;
    int64_t x=(int64_t)p->x*16384,y=(int64_t)p->y*16384,th=(int64_t)p->yaw*65536;
    int64_t da=(int64_t)cmd->wz*50*65536*65536/6283185;
    for(int i=0;i<=steps;i++) {
        angle_t yaw=(angle_t)(th>>16);
        int32_t s=sn(yaw),c=sn((angle_t)(yaw+16384));
        cell_t at[2];
        for(int side=0;side<2;side++) {
            int lateral=side?135:-135;
            int32_t wx=(int32_t)((x-(int64_t)lateral*s)/16384);
            int32_t wy=(int32_t)((y+(int64_t)lateral*c)/16384);
            cell_t now;
            int result=patch(&q,wx,wy,34+margin,14+margin,s,c,&now);
            if(result==SUPPORT_PASS && have[side] &&
               (now.x!=previous[side].x || now.y!=previous[side].y))
                result=support_transition(previous[side].lo,previous[side].hi,now.lo,now.hi,20,1);
            int32_t gd=have[side]?fx_dist(wx-grade_x[side],wy-grade_y[side]):0;
            if(result==SUPPORT_PASS && gd>=200)
                result=support_transition(grade[side].lo,grade[side].hi,now.lo,now.hi,gd*212/1000,1);
            if(result) {if(result==SUPPORT_BLOCK) blocked++;else unknowns++;return result;}
            if(!have[side] || gd>=200) {grade[side]=now;grade_x[side]=wx;grade_y[side]=wy;}
            previous[side]=now;have[side]=true;
            at[side]=now;
        }
        int roll=support_transition(at[0].lo,at[0].hi,at[1].lo,at[1].hi,57,1);
        if(roll) {if(roll==SUPPORT_BLOCK) blocked++;else unknowns++;return roll;}
        angle_t mid=(angle_t)((th+da/2)>>16);
        int32_t ss=sn(mid),cc=sn((angle_t)(mid+16384));
        x+=((int64_t)cmd->vx*cc-(int64_t)cmd->vy*ss)/20;
        y+=((int64_t)cmd->vx*ss+(int64_t)cmd->vy*cc)/20;
        th+=da;
    }
    return SUPPORT_PASS;
}
