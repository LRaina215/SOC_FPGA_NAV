#include "motion_guard.h"
#include "fixmath.h"

bool motion_map_same(const hal_map_info_t *a, const hal_map_info_t *b)
{
    return a->seq == b->seq && a->cx == b->cx && a->cy == b->cy && a->inv_res == b->inv_res;
}

/* 圆心到闭格方框的最短距离（Q8），格边界相切也保守地计为占用。 */
static int32_t axis_gap(int32_t p, int cell)
{
    int32_t lo = cell * 256, hi = lo + 256;
    return p < lo ? lo - p : (p > hi ? p - hi : 0);
}

/* Q16栅格坐标：只有整个未知格都在起始圆内，才能借用当前占用空间。
 * “格与起始圆相交”不足以证明格外侧的新扫掠部分已知。 */
static bool contained(int cx,int cy,int32_t x0,int32_t y0,int64_t rr)
{
    int32_t dx=nav_max32(nav_abs32(cx*65536-x0),nav_abs32((cx+1)*65536-x0));
    int32_t dy=nav_max32(nav_abs32(cy*65536-y0),nav_abs32((cy+1)*65536-y0));
    return (int64_t)dx*dx+(int64_t)dy*dy<=rr;
}

static int64_t max_projection(int cx,int cy,int32_t x0,int32_t y0,int32_t dx,int32_t dy)
{
    int32_t x=(dx>=0?cx+1:cx)*65536-x0,y=(dy>=0?cy+1:cy)*65536-y0;
    return (int64_t)x*dx+(int64_t)y*dy;
}

/* 仅用于CPU几何检查；硬件轨迹评分仍用原表格采样，不能改变其数值合同。 */
static int32_t guard_sin(angle_t a)
{
    int f = a & 63;
    return (fx_sin(a) * (64-f) + fx_sin((angle_t)(a+64)) * f) / 64;
}

static uint8_t check(const uint8_t *map, const hal_map_info_t *mi,
                     const nav_pose_t *pose, const lp_cfg_t *cfg, const nav_cmd_t *cmd, const int32_t *end)
{
    if (!end && !cmd->vx && !cmd->vy && !cmd->wz) return MOTION_ZERO;
    if (!mi->inv_res) return MOTION_NO_MAP;
    if (!end && ((!cfg->caps.holonomic && cmd->vy) || nav_abs32(cmd->vx) > cfg->caps.vx_max ||
        nav_abs32(cmd->vy) > cfg->caps.vy_max || nav_abs32(cmd->wz) > cfg->caps.wz_max)
        ) return MOTION_LIMIT;
    /* 每50ms采样；圆形包络覆盖采样间隙，不分配轨迹缓存。 */
    int32_t speed = end ? 0 : nav_abs32(cmd->vx) + nav_abs32(cmd->vy);
    int horizon = end ? 32 : speed ? (cfg->safe_steps * cfg->dt_ms + 49) / 50 : 0;
    if (!end && horizon > 64) return MOTION_LIMIT; /* 最多64步，即3.2s预测；不是CPU时限 */
    int32_t rad = (int32_t)(((int64_t)cfg->caps.radius_mm * mi->inv_res + 255) >> 8);
    if (rad <= 0) return MOTION_LIMIT;
    if (!end) {
        int64_t length = ((int64_t)speed * horizon + 19) / 20;
        int64_t w = nav_abs32(cmd->wz);
        int64_t ds = ((int64_t)speed * mi->inv_res + 5119) / 5120;
        int64_t chord_den = (int64_t)8 * rad;
        int64_t arc_ppm = (w*w + 9599) / 9600;
        /* 中点积分弦长误差 <= L*(w*0.05/1000)^2/24；
         * 圆间覆盖r+ds²/(8r)，再加中心弧线弓高v*w*dt²/8、
         * 查表插值和4个Q8坐标量化余量。所有误差用Q8上取整。 */
        rad += (int32_t)((ds*ds + chord_den-1) / chord_den) +
               (int32_t)(((int64_t)speed*w*mi->inv_res + 819199999) / 819200000) +
               (int32_t)((length*mi->inv_res*arc_ppm + 255999999) / 256000000) +
               (int32_t)((length*mi->inv_res + 1023999) / 1024000) + 4;
    }
    if (rad > NP_MAP_N * 256) return MOTION_OUTSIDE;
    int32_t x0 = (int32_t)(((int64_t)pose->x * mi->inv_res) >> 8) - (mi->cx - NP_MAP_N / 2) * 256;
    int32_t y0 = (int32_t)(((int64_t)pose->y * mi->inv_res) >> 8) - (mi->cy - NP_MAP_N / 2) * 256;
    int32_t ex = 0, ey = 0;
    if (end) {
        ex = (int32_t)(((int64_t)end[0] * mi->inv_res) >> 8) - (mi->cx - NP_MAP_N / 2) * 256;
        ey = (int32_t)(((int64_t)end[1] * mi->inv_res) >> 8) - (mi->cy - NP_MAP_N / 2) * 256;
        if (x0 < 0 || y0 < 0 || ex < 0 || ey < 0 || x0 >= NP_MAP_N * 256 ||
            y0 >= NP_MAP_N * 256 || ex >= NP_MAP_N * 256 || ey >= NP_MAP_N * 256) return MOTION_OUTSIDE;
        /* sqrt(r²+(ds/2)²) <= r+ds²/(8r)，L1上界覆盖线段欧氏步距。
         * 再加4个Q8单位覆盖端点量化与两个轴插值误差；不减小物理半径。 */
        int64_t d = nav_abs32(ex - x0) + nav_abs32(ey - y0);
        int64_t den = (int64_t)8 * 32 * 32 * rad;
        if (den <= 0) return MOTION_LIMIT;
        rad += (int32_t)((d * d + den - 1) / den) + 4;
    }
    int64_t rr = (int64_t)rad * rad;
    int32_t qx=(int32_t)((int64_t)pose->x*mi->inv_res-(int64_t)(mi->cx-NP_MAP_N/2)*65536);
    int32_t qy=(int32_t)((int64_t)pose->y*mi->inv_res-(int64_t)(mi->cy-NP_MAP_N/2)*65536);
    int32_t r0=(int32_t)((int64_t)cfg->caps.radius_mm*mi->inv_res),rs=rad*256;
    int64_t rr0=(int64_t)r0*r0;
    int32_t x = x0, y = y0;
    int64_t xp = (int64_t)x0 * 16384, yp = (int64_t)y0 * 16384;
    int64_t th = (int64_t)pose->yaw * 65536;
    int64_t dvx = end ? 0 : (int64_t)cmd->vx * 50 * mi->inv_res;
    int64_t dvy = end ? 0 : (int64_t)cmd->vy * 50 * mi->inv_res;
    int64_t dth = end ? 0 : (int64_t)cmd->wz * 50 * 65536 * 65536 / 6283185;
    bool unknown = false;
    for (int s = 0; s <= horizon; s++) {
        for (int cy = (y - rad) >> 8; cy <= (y + rad) >> 8; cy++) {
            for (int cx = (x - rad) >> 8; cx <= (x + rad) >> 8; cx++) {
                bool in = cx >= 0 && cy >= 0 && cx < NP_MAP_N && cy < NP_MAP_N;
                uint8_t c = in ? map[cy * NP_MAP_N + cx] : 0;
                if (in && c != NP_COST_LETHAL &&
                    (c != NP_COST_UNKNOWN || (s == 0 && !speed) || (!end && s > 4))) continue;
                int32_t dx = axis_gap(x, cx), dy = axis_gap(y, cy);
                if ((int64_t)dx * dx + (int64_t)dy * dy > rr) continue;
                if (!in) return MOTION_OUTSIDE;
                if (c == NP_COST_LETHAL) return MOTION_HAZARD;
                if (c == NP_COST_UNKNOWN && (end || s <= 4)) {
                    if(!contained(cx,cy,qx,qy,rr0)) {
                        int32_t ux,uy;
                        if(!s) {
                            int32_t sn=guard_sin(pose->yaw),cs=guard_sin((angle_t)(pose->yaw+16384));
                            ux=(int32_t)((int64_t)cmd->vx*cs-(int64_t)cmd->vy*sn);
                            uy=(int32_t)((int64_t)cmd->vx*sn+(int64_t)cmd->vy*cs);
                            int32_t farx=nav_max32(nav_abs32(cx*65536-qx),nav_abs32((cx+1)*65536-qx));
                            int32_t fary=nav_max32(nav_abs32(cy*65536-qy),nav_abs32((cy+1)*65536-qy));
                            /* 起点外接采样裕量也可能覆盖最初的短暂新区域。
                             * 2个Q14单位覆盖方向插值/取整误差。 */
                            if(max_projection(cx,cy,qx,qy,ux,uy)+(int64_t)2*speed*(farx+fary)>0) unknown=true;
                            continue;
                        }
                        ux=(int32_t)((xp>>6)-qx);uy=(int32_t)((yp>>6)-qy);
                        /* 两圆差集的必要条件：2 z·d > |d|²+R0²-Rs²。
                         * 不满足则仅是离开未知的后缘，不能误当新进入。 */
                        if(2*max_projection(cx,cy,qx,qy,ux,uy)>(int64_t)ux*ux+(int64_t)uy*uy+rr0-(int64_t)rs*rs) unknown=true;
                    }
                }
            }
        }
        if (end) {
            x = x0 + (int32_t)((int64_t)(ex - x0) * (s + 1) / 32);
            y = y0 + (int32_t)((int64_t)(ey - y0) * (s + 1) / 32);
        } else {
            angle_t mid = (angle_t)((th + dth/2) >> 16);
            int32_t sn = guard_sin(mid), cs = guard_sin((angle_t)(mid + 16384));
            xp += (dvx * cs - dvy * sn) / 256000;
            yp += (dvx * sn + dvy * cs) / 256000;
            th += dth;
            x = (int32_t)(xp >> 14); y = (int32_t)(yp >> 14);
        }
    }
    return unknown ? MOTION_UNKNOWN : MOTION_OK;
}

uint8_t motion_check(const uint8_t *map, const hal_map_info_t *mi,
                     const nav_pose_t *pose, const lp_cfg_t *cfg, const nav_cmd_t *cmd)
{ return check(map, mi, pose, cfg, cmd, 0); }

uint8_t motion_segment(const uint8_t *map, const hal_map_info_t *mi,
                       const nav_pose_t *pose, const lp_cfg_t *cfg, int32_t gx, int32_t gy)
{
    if (!mi->inv_res) return MOTION_NO_MAP;
    int32_t r = (int32_t)(((int64_t)cfg->caps.radius_mm * mi->inv_res + 255) >> 8);
    if (r <= 0) return MOTION_LIMIT;
    r += 4; /* 两端Q8坐标量化的上界；连续几何不需要采样间填充 */
    int32_t x0 = (int32_t)(((int64_t)pose->x*mi->inv_res)>>8) - (mi->cx-NP_MAP_N/2)*256;
    int32_t y0 = (int32_t)(((int64_t)pose->y*mi->inv_res)>>8) - (mi->cy-NP_MAP_N/2)*256;
    int32_t x1 = (int32_t)(((int64_t)gx*mi->inv_res)>>8) - (mi->cx-NP_MAP_N/2)*256;
    int32_t y1 = (int32_t)(((int64_t)gy*mi->inv_res)>>8) - (mi->cy-NP_MAP_N/2)*256;
    int32_t xmin=nav_min32(x0,x1)-r, xmax=nav_max32(x0,x1)+r;
    int32_t ymin=nav_min32(y0,y1)-r, ymax=nav_max32(y0,y1)+r;
    if (xmin<0 || ymin<0 || xmax>=NP_MAP_N*256 || ymax>=NP_MAP_N*256) return MOTION_OUTSIDE;
    int32_t dx=x1-x0,dy=y1-y0;
    int32_t qx=(int32_t)((int64_t)pose->x*mi->inv_res-(int64_t)(mi->cx-NP_MAP_N/2)*65536);
    int32_t qy=(int32_t)((int64_t)pose->y*mi->inv_res-(int64_t)(mi->cy-NP_MAP_N/2)*65536);
    int32_t r0=(int32_t)((int64_t)cfg->caps.radius_mm*mi->inv_res);
    int64_t rr0=(int64_t)r0*r0;
    int64_t rr=(int64_t)r*r, ll=(int64_t)dx*dx+(int64_t)dy*dy;
    bool unknown=false;
    for(int y=ymin>>8;y<=ymax>>8;y++) for(int x=xmin>>8;x<=xmax>>8;x++) {
        uint8_t c=map[y*NP_MAP_N+x];
        if(c!=NP_COST_LETHAL && c!=NP_COST_UNKNOWN) continue;
        int32_t ax=axis_gap(x0,x),ay=axis_gap(y0,y);
        bool start=(int64_t)ax*ax+(int64_t)ay*ay<=rr;
        if(c==NP_COST_UNKNOWN && start &&
           ((!dx && !dy) || contained(x,y,qx,qy,rr0) ||
            max_projection(x,y,qx,qy,gx-pose->x,gy-pose->y)<=0)) continue;
        ax=axis_gap(x1,x);ay=axis_gap(y1,y);
        bool hit=start || (int64_t)ax*ax+(int64_t)ay*ay<=rr;
        /* 距离最小值在端点/矩形角点，或者中心线穿矩形。
         * 角点投影使用cross²<=r²*length²，不做除法。 */
        for(int k=0;k<4 && !hit;k++) {
            int32_t bx=(x+(k&1))*256,by=(y+(k>>1))*256;
            int64_t dot=(int64_t)(bx-x0)*dx+(int64_t)(by-y0)*dy;
            int64_t cross=(int64_t)(bx-x0)*dy-(int64_t)(by-y0)*dx;
            if(ll && dot>=0 && dot<=ll && cross*cross<=rr*ll) hit=true;
            if(dx && bx>=nav_min32(x0,x1) && bx<=nav_max32(x0,x1)) {
                int64_t yn=(int64_t)y0*dx+(int64_t)(bx-x0)*dy;
                int64_t a=(int64_t)y*256*dx,b=(int64_t)(y+1)*256*dx;
                if(yn>= (a<b?a:b) && yn<=(a>b?a:b)) hit=true;
            }
            if(dy && by>=nav_min32(y0,y1) && by<=nav_max32(y0,y1)) {
                int64_t xn=(int64_t)x0*dy+(int64_t)(by-y0)*dx;
                int64_t a=(int64_t)x*256*dy,b=(int64_t)(x+1)*256*dy;
                if(xn>=(a<b?a:b) && xn<=(a>b?a:b)) hit=true;
            }
        }
        if(hit) {if(c==NP_COST_LETHAL) return MOTION_HAZARD; unknown=true;}
    }
    return unknown?MOTION_UNKNOWN:MOTION_OK;
}
