#include "nav_task.h"
#include "fixmath.h"
#include "support_guard.h"

#define N NP_MAP_N

void nav_mm_to_win(const hal_map_info_t *mi, int32_t x, int32_t y, int *wx, int *wy)
{
    int32_t ix = (int32_t)(((int64_t)x * mi->inv_res) >> 16);
    int32_t iy = (int32_t)(((int64_t)y * mi->inv_res) >> 16);
    *wx = ix - (mi->cx - N / 2);
    *wy = iy - (mi->cy - N / 2);
}

void nav_win_to_mm(const hal_map_info_t *mi, int wx, int wy, int32_t *x, int32_t *y)
{
    int32_t ix = wx + (mi->cx - N / 2), iy = wy + (mi->cy - N / 2);
    *x = (int32_t)(((int64_t)ix * 65536 + 32768) / mi->inv_res);
    *y = (int32_t)(((int64_t)iy * 65536 + 32768) / mi->inv_res);
}

void nav_task_init(nav_task_t *nt)
{
    astar_cancel();
    *nt = (nav_task_t){0};
    nt->cfg.lookahead_mm = 800;
    nt->cfg.replan_us = 500000;
    nt->cfg.blocked_us = 3000000;
    nt->cfg.search_repick_us = 12000000;
    nt->cfg.track_hold_mm = 200;
    nt->cfg.wp_tol_mm = 350;
    nt->cfg.stall_us = 6000000;
    nt->cfg.stall_mm = 150;
    nt->cfg.near_goal_mm = 600;
    nt->cfg.goal_pose_margin_mm = 50; /* 150mm要求保留50mm定位预算，估计位置需进入100mm */
    nt->cfg.settle_us = 500000;
    nt->cfg.recover_us = 1500000;
    lp_default_cfg(&nt->lp_cfg);
    astar_default_cfg(&nt->as_cfg);
    astar_default_cfg(&nt->as_cfg_g);
    /* 记忆粗格：整格受限为253，部分受限为252高代价。
     * 细图LOS与最终210mm足迹仍执行硬约束。 */
    nt->as_cfg_g.clear_cells = 4;
    nt->as_cfg_g.clear_cost = 15;
    nt->as_cfg_g.near_cells = 3;
    nt->as_cfg_g.near_cost = 150;
    nt->use_gmap = true;
    gmap_default_cfg(&nt->gm.cfg);
    gmap_init(&nt->gm);
    explore_default_cfg(&nt->ex_cfg);
    /* 雷达安装位姿（10-31 静止地面拟合，雷达相对 IMU）：下俯 2.1°、横滚 3°（11-01 左右镜像修正后反号），在轮轴前 123 mm、离地 150 mm */
#ifndef APP_MOUNT_PITCH_CDEG
#define APP_MOUNT_PITCH_CDEG 210
#endif
#ifndef APP_MOUNT_ROLL_CDEG
#define APP_MOUNT_ROLL_CDEG 300
#endif
#ifndef APP_MOUNT_Z_MM
#define APP_MOUNT_Z_MM 150
#endif
    pose_init(&nt->pose, ANGLE_FROM_CDEG(APP_MOUNT_PITCH_CDEG), 123, APP_MOUNT_Z_MM);
    /* Mount roll relative to the IMU body frame (Rx, + = left side down in the map). */
    fx_rot_zyx(&nt->pose.r_bs, 0, ANGLE_FROM_CDEG(APP_MOUNT_PITCH_CDEG), ANGLE_FROM_CDEG(APP_MOUNT_ROLL_CDEG));
    target_init(&nt->tgt, 1500000, 700);
    explore_init(&nt->exp);
    nt->mode = NAV_MODE_IDLE;
    nt->state = ST_IDLE;
}

void nav_task_set_mode(nav_task_t *nt, nav_mode_t mode)
{
    astar_cancel();
    nt->mode = mode;
    if(mode==NAV_MODE_IDLE || mode==NAV_MODE_ESTOP) {
        nt->scan_active=nt->scan_settling=nt->scan_failed=false;nt->scan_exit_valid=false;
    }
    nt->path_valid = false;
    nt->path.n=0;
    nt->refine_path = nt->path_fine = false;
    nt->search_goal_valid = false;
    nt->last_ok_us = hal_time_us();
    nt->prog_t_us = hal_time_us();
    nt->prog_x = nt->pose.cur.x; nt->prog_y = nt->pose.cur.y;
    nt->plan_goal_x=nt->goal_x;nt->plan_goal_y=nt->goal_y;
    if(!nt->gm.has_prior && !nt->stepping_local)
    {
        local_frame_point(&nt->terrain_frame,nt->prog_x,nt->prog_y,&nt->prog_x,&nt->prog_y,false);
        local_frame_point(&nt->terrain_frame,nt->plan_goal_x,nt->plan_goal_y,&nt->plan_goal_x,&nt->plan_goal_y,false);
    }
    nt->recovering = false;
    nt->settling = false;
    switch (mode) {
    case NAV_MODE_IDLE:   nt->state = ST_IDLE; break;
    case NAV_MODE_GOTO:   nt->state = ST_MOVING; break;
    case NAV_MODE_TRACK:  nt->state = ST_TRACKING; break;
    case NAV_MODE_SEARCH: nt->state = ST_SEARCHING; break;
    case NAV_MODE_ESTOP:  nt->state = ST_ESTOP; break;
    }
}

void nav_task_set_goal(nav_task_t *nt, int32_t x, int32_t y)
{
    nt->scan_active=nt->scan_settling=nt->scan_failed=false;nt->scan_exit_valid=false;
    nt->goal_x = x;
    nt->goal_y = y;
    nt->wp_n = 0;
    nav_task_set_mode(nt, NAV_MODE_GOTO);
}

void nav_task_on_msg(nav_task_t *nt, const proto_msg_t *m, uint32_t rx_us)
{
    switch (m->type) {
    case MSG_GOAL:
        nav_task_set_goal(nt, m->u.goal.x, m->u.goal.y);
        break;
    case MSG_WAYPOINTS:
        if (m->u.wp.n == 0) break;
        nt->scan_active=nt->scan_settling=nt->scan_failed=false;nt->scan_exit_valid=false;
        for (int i = 0; i < m->u.wp.n; i++) { nt->wp_x[i] = m->u.wp.x[i]; nt->wp_y[i] = m->u.wp.y[i]; }
        nt->wp_n = m->u.wp.n;
        nt->wp_i = 0;
        nt->wp_loop = m->u.wp.loop != 0;
        nt->goal_x = nt->wp_x[0];
        nt->goal_y = nt->wp_y[0];
        nav_task_set_mode(nt, NAV_MODE_GOTO);
        break;
    case MSG_TARGET:
        target_update(&nt->tgt, &nt->pose, m->u.target.found, m->u.target.x, m->u.target.y,
                      m->u.target.latency_ms, m->u.target.track_id, m->u.target.cls, rx_us);
        if (m->u.target.found && nt->mode == NAV_MODE_SEARCH) nav_task_set_mode(nt, NAV_MODE_TRACK);
        break;
    case MSG_MODE:
        if (m->u.mode.mode <= NAV_MODE_ESTOP) nav_task_set_mode(nt, (nav_mode_t)m->u.mode.mode);
        break;
    default:
        break;
    }
}

/* 目标在窗口外时，把 A* 目标夹到窗口边界（沿机器人->目标方向） */
static void clamp_goal_to_window(int rx, int ry, int *gx, int *gy)
{
    const int lo = 1, hi = N - 2;
    if (*gx >= lo && *gx <= hi && *gy >= lo && *gy <= hi) return;
    int dx = *gx - rx, dy = *gy - ry;
    /* 求最大 t∈[0,1] 使点仍在窗口内 */
    int32_t t = 1024;
    if (dx > 0 && rx + dx > hi) t = nav_min32(t, (hi - rx) * 1024 / dx);
    if (dx < 0 && rx + dx < lo) t = nav_min32(t, (lo - rx) * 1024 / dx);
    if (dy > 0 && ry + dy > hi) t = nav_min32(t, (hi - ry) * 1024 / dy);
    if (dy < 0 && ry + dy < lo) t = nav_min32(t, (lo - ry) * 1024 / dy);
    if (t < 0) t = 0;
    *gx = rx + dx * t / 1024;
    *gy = ry + dy * t / 1024;
}

/* 代价图上两点（窗口坐标）之间的直线是否全部可走（未知格视为可走） */
static bool line_clear(const uint8_t *map, int x0, int y0, int x1, int y1, uint8_t block)
{
    int dx = nav_abs32(x1 - x0), dy = -nav_abs32(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1, err = dx + dy;
    for (;;) {
        if (x0 >= 0 && y0 >= 0 && x0 < N && y0 < N) {
            uint8_t c = map[y0 * N + x0];
            if (c != 255 && c >= block) return false;
        } else return false;
        if (x0 == x1 && y0 == y1) return true;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

/* 路径点 i -> mm 与局部窗口坐标 */
static void path_pt(const nav_task_t *nt, int i, int32_t *x, int32_t *y, int *wx, int *wy)
{
    if (nt->path_fine) {
        hal_map_info_t origin = nt->mi;
        origin.cx = nt->path_cx; origin.cy = nt->path_cy;
        nav_win_to_mm(&origin, nt->path.wx[i], nt->path.wy[i], x, y);
        nav_mm_to_win(&nt->mi, *x, *y, wx, wy);
    } else if (nt->use_gmap) {
        gmap_cell_to_mm(&nt->gm, &nt->mi, nt->path.wx[i], nt->path.wy[i], x, y);
        nav_mm_to_win(&nt->mi, *x, *y, wx, wy);
    } else {
        *wx = nt->path.wx[i]; *wy = nt->path.wy[i];
        nav_win_to_mm(&nt->mi, *wx, *wy, x, y);
    }
}

static int plan_coarse(nav_task_t *nt,const nav_pose_t *p,int32_t gx,int32_t gy)
{
    nt->search_work_done=true;
    nt->search_fine=false;
    nt->search_keep_route=nt->path_valid && (!nt->use_gmap || nt->path_fine || nt->path_shift==nt->gm.n_shift);
    nt->search_shift=nt->gm.n_shift;
    if (nt->use_gmap) {
        int rx, ry, tx, ty;
        gmap_mm_to_cell(&nt->gm, &nt->mi, p->x, p->y, &rx, &ry);
        gmap_mm_to_cell(&nt->gm, &nt->mi, gx, gy, &tx, &ty);
        /* 目标在记忆栅格外：夹到边界（沿机器人->目标方向），clamp 按局部图尺寸写的，这里换算比例 */
        bool clipped = tx < 1 || tx > GM_N - 2 || ty < 1 || ty > GM_N - 2;
        if (clipped) {
            int32_t dx = tx - rx, dy = ty - ry, t = 1024;
            if (dx > 0 && tx > GM_N - 2) t = nav_min32(t, (GM_N - 2 - rx) * 1024 / dx);
            if (dx < 0 && tx < 1) t = nav_min32(t, (1 - rx) * 1024 / dx);
            if (dy > 0 && ty > GM_N - 2) t = nav_min32(t, (GM_N - 2 - ry) * 1024 / dy);
            if (dy < 0 && ty < 1) t = nav_min32(t, (1 - ry) * 1024 / dy);
            if (t < 0) t = 0;
            tx = rx + dx * t / 1024;
            ty = ry + dy * t / 1024;
        }
        astar_complete_only(true);
        int rc = astar_grid_bounded(&nt->as_cfg_g, nt->gm.grid, GM_N, rx, ry, tx, ty, &nt->path,128);
        nt->path_target_is_goal=!clipped;
        if(rc!=2) {
            nt->path_fine=false;nt->path_shift=nt->search_shift;
            nt->path_complete=rc==0 && !clipped;
        }
        return rc;
    }
    int rx, ry, tx, ty;
    nav_mm_to_win(&nt->mi, p->x, p->y, &rx, &ry);
    nav_mm_to_win(&nt->mi, gx, gy, &tx, &ty);
    bool clipped = tx < 1 || tx > N - 2 || ty < 1 || ty > N - 2;
    clamp_goal_to_window(rx, ry, &tx, &ty);
    astar_complete_only(true);
    int rc = astar_plan_bounded(&nt->as_cfg, nt->map, nav_clamp32(rx, 0, N - 1), nav_clamp32(ry, 0, N - 1), tx, ty, &nt->path,128);
    nt->path_target_is_goal=!clipped;
    if(rc!=2) {nt->path_fine=false;nt->path_complete=rc==0 && !clipped;}
    return rc;
}

/* 只在粗路径无法执行时细化；远目标先取粗路径在近场内的可观测子目标。
 * 子目标永远不冒充最终目标，也不将窗边裁剪当作到达。 */
static int plan_path(nav_task_t *nt,const nav_pose_t *p,int32_t gx,int32_t gy,bool fine)
{
    if(nt->search_work_done) return nt->path_valid?1:-1;
    if(astar_busy()) {
        if(fx_dist(gx-nt->plan_goal_x,gy-nt->plan_goal_y)>100 ||
           (nt->use_gmap && !nt->search_fine && nt->search_shift!=nt->gm.n_shift)) {
            astar_cancel();nt->path.n=0;nt->path_fine=false;
        }
        else {
            nt->search_work_done=true;
            int rc=astar_continue(&nt->path,128);
            if(rc!=2) {
                nt->path_fine=nt->search_fine;nt->path_shift=nt->search_shift;
                nt->path_complete=rc==0 && nt->path_target_is_goal;
            }
            return rc;
        }
    }
    nt->plan_goal_x=gx;nt->plan_goal_y=gy;
    bool cached=fine && !nt->path_fine && nt->path.n>1 && nt->path_shift==nt->gm.n_shift;
    int rc=cached?(nt->path_complete?0:1):plan_coarse(nt,p,gx,gy);
    if(astar_busy() || !fine || !nt->use_gmap || nt->gm.has_prior || rc<0) return rc;
    if(nt->search_work_done) return rc; /* 粗搜索已消耗本周期预算，细化延后。 */
    int rx,ry,tx,ty;
    nav_mm_to_win(&nt->mi,p->x,p->y,&rx,&ry);
    nav_mm_to_win(&nt->mi,gx,gy,&tx,&ty);
    int ox=nav_clamp32(rx-ASTAR_N/2,0,N-ASTAR_N);
    int oy=nav_clamp32(ry-ASTAR_N/2,0,N-ASTAR_N);
    bool final=tx>=ox && ty>=oy && tx<ox+ASTAR_N && ty<oy+ASTAR_N;
    bool found=final;
    if(!final) {
        for(int i=nt->path.n-1;i>0 && !found;i--) {
            int32_t x,y;int wx,wy;
            path_pt(nt,i,&x,&y,&wx,&wy);
            if(wx<ox+2 || wy<oy+2 || wx>=ox+ASTAR_N-2 || wy>=oy+ASTAR_N-2 ||
               fx_dist(x-p->x,y-p->y)<150) continue;
            int best=INT32_MAX;
            /* 粗格中心可能落在膨胀格；在其邻近细格找已观测接入点。
             * 这里只选子目标，真正可达性仍由细A*和完整足迹检查决定。 */
            for(int dy=-2;dy<=2;dy++) for(int dx=-2;dx<=2;dx++) {
                int xx=wx+dx,yy=wy+dy;
                if(xx<=ox || yy<=oy || xx>=ox+ASTAR_N-1 || yy>=oy+ASTAR_N-1 ||
                   nt->map[yy*N+xx]>=nt->as_cfg.block) continue;
                int d=dx*dx+dy*dy;
                if(d<best) {best=d;tx=xx;ty=yy;found=true;}
            }
        }
    }
    if(!found) return rc;
    astar_cfg_t cfg=nt->as_cfg;cfg.allow_unknown=false;
    nt->search_keep_route=nt->path_valid;
    nt->search_fine=true;nt->search_shift=nt->gm.n_shift;
    astar_complete_only(true);
    nt->search_work_done=true;
    rc=astar_window_bounded(&cfg,nt->map,rx,ry,tx,ty,&nt->path,128);
    nt->path_target_is_goal=final;
    if(rc!=2) {nt->path_fine=true;nt->path_complete=final && rc==0;}
    nt->path_cx=nt->mi.cx;nt->path_cy=nt->mi.cy;
    return rc;
}

static bool waypoint_visible(const nav_task_t *nt, const nav_pose_t *p, int32_t x, int32_t y)
{
    uint8_t result = motion_segment(nt->map, &nt->mi, p, &nt->lp_cfg, x, y);
    /* 路线只提供方向，细化不会把圆外缘的未知变成已知。允许未知时让
     * 局部规划评估受限微动；最终车体/支撑出口仍决定能否发送命令。 */
    return result == MOTION_OK || (result == MOTION_UNKNOWN && nt->as_cfg.allow_unknown);
}

/* 沿全局路径取前视点；返回 false 表示没有路径 */
static bool pick_lookahead(nav_task_t *nt, const nav_pose_t *p, int32_t gx, int32_t gy, int32_t *lx, int32_t *ly)
{
    uint32_t now = hal_time_us();
    int tx, ty;
    nav_mm_to_win(&nt->mi, gx, gy, &tx, &ty);
    bool goal_in = tx >= 1 && tx <= N - 2 && ty >= 1 && ty <= N - 2;
    bool stale = nt->use_gmap && !nt->path_fine && nt->path_shift != nt->gm.n_shift;
    if(nt->use_gmap && !nt->path_fine && nt->path_valid && !stale && !astar_busy()) {
        int rx,ry,best=0,dist=INT32_MAX;
        gmap_mm_to_cell(&nt->gm,&nt->mi,p->x,p->y,&rx,&ry);
        for(int i=0;i<nt->path.n;i++) {
            int dx=nt->path.wx[i]-rx,dy=nt->path.wy[i]-ry,d=dx*dx+dy*dy;
            if(d<dist) {dist=d;best=i;}
        }
        /* 新观测堵住尚未走过的路线即重搜；已走过的格不反复使路线失效。 */
        for(int i=best+1;i<nt->path.n;i++) {
            int x=nt->path.wx[i],y=nt->path.wy[i];
            if(x<0 || y<0 || x>=GM_N || y>=GM_N) {stale=true;break;}
            uint8_t c=nt->gm.grid[y*GM_N+x];
            if(c>=GM_COST_BLOCKED && c!=NP_COST_UNKNOWN) {stale=true;break;}
        }
    }
    bool changed=fx_dist(gx-nt->plan_goal_x,gy-nt->plan_goal_y)>100;
    if(changed) {astar_cancel();nt->path_valid=false;nt->path.n=0;nt->refine_path=false;}
    /* 新图仍逐周期复核路径接入与最终运动；可执行旧路线不因定时器
     * 反复启动全局搜索。目标/滚图改变、接入或局部执行失败才重新规划。 */
    if (!nt->path_valid || stale || astar_busy()) {
        int rc = plan_path(nt, p, gx, gy, nt->refine_path);
        nt->path_valid = rc==2?nt->search_keep_route:(rc>=0 && nt->path.n>0);
        nt->path_seq = nt->mi.seq;
        nt->path_t_us = now;
        if (!nt->path_valid) return false;
    }
    /* 近场路径不能提供已观测的接入时，最多回退一次记忆图，不无限重试。 */
    for (int attempt = 0; attempt < 2; attempt++) {
        if (nt->path_complete && goal_in && fx_dist(gx - p->x, gy - p->y) <= nt->cfg.lookahead_mm &&
            waypoint_visible(nt, p, gx, gy)) { *lx = gx; *ly = gy; return true; }
        int best = 0;
        int32_t bd = INT32_MAX;
        for (int i = 0; i < nt->path.n; i++) {
            int32_t x, y; int wx, wy;
            path_pt(nt, i, &x, &y, &wx, &wy);
            int32_t d = fx_dist(x - p->x, y - p->y);
            if (d < bd) { bd = d; best = i; }
        }
        int hi = best;
        for (; hi < nt->path.n - 1; hi++) {
            int32_t x, y; int wx, wy;
            path_pt(nt, hi, &x, &y, &wx, &wy);
            if (fx_dist(x - p->x, y - p->y) >= nt->cfg.lookahead_mm) break;
        }
        /* 从远到近最多8个接入点，各自做完整线段足迹检查。 */
        int last = -1;
        for (int j = 0; j < 8; j++) {
            int lo = nav_min32(hi, best + 2);
            int i = j < 6 ? hi - (hi - lo) * j / 5 : j == 6 ? nav_min32(hi, best + 1) : best;
            if (i == last) continue;
            last = i;
            int32_t x, y; int wx, wy;
            path_pt(nt, i, &x, &y, &wx, &wy);
            /* 最近的自身格不是向前接入；仅余它时请求细化/受阻，不能循环追自身。 */
            if (i == best && fx_dist(x-p->x, y-p->y) < nt->lp_cfg.goal_tol_mm &&
                fx_dist(gx-p->x, gy-p->y) > nt->lp_cfg.goal_tol_mm) continue;
            if (waypoint_visible(nt, p, x, y)) { *lx = x; *ly = y; return true; }
        }
        if(astar_busy()) {nt->path_valid=false;return false;}
        bool refine = !nt->path_fine && !nt->refine_path && nt->use_gmap && !nt->gm.has_prior;
        if (!nt->path_fine && !refine) break;
        nt->refine_path = refine;
        int rc = plan_path(nt, p, gx, gy, refine);
        nt->path_valid = rc==2?nt->search_keep_route:(rc>=0 && nt->path.n>0);
        if (!nt->path_valid) return false;
    }
    nt->path_valid = false;
    return false;
}

/* 朝 (gx, gy) 行进一个周期；返回是否已到达（距离 < tol） */
/* 身后 len_mm 的直线在代价图上是否可走（倒车脱困前检查） */
static bool back_clear(const nav_task_t *nt, const nav_pose_t *p, int32_t len_mm)
{
    int32_t bx, by;
    int rx, ry, tx, ty;
    pose_body_to_odom(p, -len_mm, 0, &bx, &by);
    nav_mm_to_win(&nt->mi, p->x, p->y, &rx, &ry);
    nav_mm_to_win(&nt->mi, bx, by, &tx, &ty);
    nav_cmd_t cmd = {(int16_t)-nt->lp_cfg.v_back, 0, 0, 0};
    return line_clear(nt->map, rx, ry, tx, ty, NP_COST_LETHAL) &&
           motion_check(nt->map, &nt->mi, p, &nt->lp_cfg, &cmd) == MOTION_OK;
}

static bool drive_to(nav_task_t *nt, uint32_t now, int32_t gx, int32_t gy, int32_t tol)
{
    const nav_pose_t *p = &nt->pose.cur;
    nt->cmd = (nav_cmd_t){0, 0, 0, 0};
    int32_t dgoal = fx_dist(gx - p->x, gy - p->y);
    bool final = nt->mode == NAV_MODE_GOTO &&
                 !(nt->wp_n > 0 && (nt->wp_i + 1 < nt->wp_n || nt->wp_loop));
    if (dgoal < tol) {
        if (!final) return true;  /* 中间航点只是途经，不强制逐点停稳 */
        bool stopped = nt->pose.have_odo && (uint32_t)(now - nt->pose.last_odo_us) < 100000 &&
                       nav_abs32(nt->pose.odo_vx) <= 20 && nav_abs32(nt->pose.odo_vy) <= 20 &&
                       nav_abs32(nt->pose.odo_wz) <= 50;
        if (!stopped) { nt->settling = false; return false; }
        if (!nt->settling) { nt->settling = true; nt->settle_t_us = now; }
        nt->last_ok_us = now;
        return (uint32_t)(now - nt->settle_t_us) >= nt->cfg.settle_us;
    }
    nt->settling = false;
    /* 卡住检测：stall_us 内位移不到 stall_mm */
    int32_t progress=nt->cfg.stall_mm;
    int speed=nt->progress_speed;
    if(nt->pose.have_odo && (uint32_t)(now-nt->pose.last_odo_us)<100000)
        speed=nav_max32(speed,nav_abs32(nt->pose.odo_vx)+nav_abs32(nt->pose.odo_vy));
    if(speed>0) progress=nav_min32(progress,nav_max32(5,(int32_t)((int64_t)speed*nt->cfg.stall_us/2000000)));
    /* 有意低速接入不能要求6s内仍前进150mm；用半个停滞窗口的
     * 已下发速度计算门槛。零速/纯转向仍用原门槛，不能靠定位抖动保活。 */
    if (fx_dist(p->x - nt->prog_x, p->y - nt->prog_y) > progress) {
        nt->prog_x = p->x; nt->prog_y = p->y; nt->prog_t_us = now;
    } else if (!nt->recovering && (uint32_t)(now - nt->prog_t_us) > nt->cfg.stall_us) {
        /* 近目标卡住先零速/报告受阻；下一周期仍可重新规划受检查的动作。
         * 永不重置计时会让此分支持续提前返回，连新图上的可行接入也无法尝试。 */
        if (dgoal < nt->cfg.near_goal_mm) {
            if (nt->mode == NAV_MODE_TRACK) {
                /* 跟踪没有ARRIVED状态：保留原有站位/朝向处理，不用定点受阻规则替代它。 */
                nt->prog_t_us = now;
                return true;
            }
            nt->last_ok_us = now - nt->cfg.blocked_us - 1u;
            nt->prog_t_us=now;nt->path_valid=false;
            nt->refine_path=nt->use_gmap && !nt->gm.has_prior;
            return false;
        }
        nt->prog_t_us = now;
        /* 同一处第二次卡住：全局规划认为能过、局部规划过不去（窄缝、斜向缝隙的量化差异）——把前视点那里临时堵上 */
        if (nt->use_gmap &&
            fx_dist(nt->look_x - p->x, nt->look_y - p->y) > nt->lp_cfg.caps.radius_mm + 2 * GM_DS * nt->mi.res_mm &&
            fx_dist(p->x - nt->stuck_x, p->y - nt->stuck_y) < 300 && nt->n_recover > 0)
            gmap_block_add(&nt->gm, &nt->mi, nt->look_x, nt->look_y, 2, 300);
        nt->stuck_x = p->x; nt->stuck_y = p->y;
        nt->recovering = true;
        nt->recover_until_us = now + nt->cfg.recover_us;
        nt->recover_dir = (int8_t)(back_clear(nt, p, 400) ? 1 : 0);
        nt->path_valid = false;
        nt->turning = false;
        nt->n_recover++;
    }
    if (nt->recovering) {
        if ((int32_t)(now - nt->recover_until_us) >= 0) {
            nt->recovering = false;
        } else {
            /* 倒车（身后可走）或原地转半圈，打破局部规划的来回摆动 */
            if (nt->recover_dir) nt->cmd.vx = (int16_t)-nt->lp_cfg.v_back;
            else nt->cmd.wz = (int16_t)(nt->lp_cfg.w_max / 2);
            nt->cmd_source = CMD_RECOVERY;
            nt->last_ok_us = now;
            return false;
        }
    }
    int32_t lx = gx, ly = gy;
    if (!pick_lookahead(nt, p, gx, gy, &lx, &ly)) {
        nt->look_x = p->x; nt->look_y = p->y;
        return false;
    }
    nt->look_x = lx; nt->look_y = ly;
    /* 前视点方向偏差大：先原地转向（带滞回：>70° 进入，<20° 退出） */
    int32_t dh0 = fx_angle_diff(p->yaw, fx_atan2(ly - p->y, lx - p->x));
    if (nav_abs32(dh0) > 12743) nt->turning = true;           /* 70° */
    else if (nav_abs32(dh0) < 3641) nt->turning = false;      /* 20° */
    if (nt->turning) {
        int32_t wz = (int32_t)((int64_t)dh0 * 3000 / 32768);  /* 约 3 rad/s·rad 的比例转向 */
        int32_t wmax = nt->lp_cfg.w_max;
        wz = nav_clamp32(wz, -wmax, wmax);
        if (wz > 0 && wz < wmax / 4) wz = wmax / 4;
        if (wz < 0 && wz > -wmax / 4) wz = -wmax / 4;
        nt->cmd.wz = (int16_t)wz;
        nt->cmd_source = CMD_TURN;
        nt->lp.prev_wz = (int16_t)wz;
        nt->last_ok_us = now;
        return false;
    }
    /* 只在最终目标接近区缩短速度格：2s前推的最小125mm/s会在距目标125mm
     * 附近偏好原地转向，不能只缩小到达半径。按剩余距离限制前进上限，使候选
     * 在原有前推时域内到达目标；中间航点和TRACK保留原速度格。 */
    lp_cfg_t approach = nt->lp_cfg;
    int32_t horizon = (int32_t)approach.steps * approach.dt_ms;
    int32_t dlook = fx_dist(lx - p->x, ly - p->y);
    if (horizon > 0 && approach.nv > 0 &&
        dlook * 1000 < (int32_t)approach.v_max * horizon / approach.nv) {
        /* 粗/细路径的短接入都需要低速格，避免最低速度越过拐点而只剩转向。 */
        approach.v_max = (int16_t)nav_min32(approach.v_max, nav_max32(20, dlook * 1000 / horizon));
    }
    if (final && dgoal < nt->cfg.near_goal_mm) {
        int32_t limit = horizon > 0 ? nav_max32(20, dgoal * 1000 / horizon) : 20;
        approach.v_max = (int16_t)nav_min32(approach.v_max, limit);
    }
    if (lp_plan(&approach, &nt->mi, p, nt->map, nt->slow, nt->as_cfg.allow_unknown, lx, ly, &nt->lp)) {
        nt->cmd = nt->lp.cmd;
        nt->cmd_source = CMD_LOCAL;
        nt->last_ok_us = now;
    } else {
        if (nt->lp.map_stale) {
            nt->motion_result=MOTION_MAP_CHANGED;
            if(!nt->use_gmap && !nt->path_fine) nt->path_valid=false;
            return false;
        }
        /* 无可行候选：原地转向前视点方向，重新规划 */
        int32_t dh = fx_angle_diff(p->yaw, fx_atan2(ly - p->y, lx - p->x));
        nt->cmd.wz = (int16_t)(dh > 0 ? nt->lp_cfg.w_max / 2 : -nt->lp_cfg.w_max / 2);
        nt->cmd_source = CMD_FALLBACK;
        if (nt->use_gmap && !nt->gm.has_prior) nt->refine_path = true;
        nt->path_valid = false;
    }
    return false;
}

static void task_step_local(nav_task_t *nt, uint32_t now)
{
    nt->search_work_done=false;
    nt->cmd_source = CMD_STOP;
    nt->motion_result = MOTION_ZERO;
    nt->cmd = (nav_cmd_t){0};
    /* 刷新代价图 */
    hal_map_info_t mi;
    bool fresh = hal_map_info(&mi) && !(nt->stale && mi.seq == nt->stale_seq);
    if (fresh) nt->stale = false;
    if (fresh && (!nt->have_map || mi.seq != nt->mi.seq)) {
        nt->have_map = hal_map_copy(nt->map, &nt->mi) != 0;
        if (nt->have_map && !nt->async_gmap) gmap_update(&nt->gm, nt->map, &nt->mi, nt->pose.cur.x, nt->pose.cur.y);
        else if(!nt->have_map) nt->path_valid = false;
    }
    explore_mark(&nt->exp, &nt->pose.cur);
    if (nt->gm.has_prior && nt->have_map && !nt->hold) explore_seen_mark(&nt->exp, &nt->ex_cfg, &nt->gm, &nt->mi, &nt->pose.cur);
    const nav_pose_t *p = &nt->pose.cur;

    if ((!nt->have_map || nt->hold) && nt->mode != NAV_MODE_ESTOP) {
        nt->settling = false;
        nt->prog_t_us = now;             /* 保持期间不算卡住 */
        nt->last_ok_us = now;
        nt->cmd = (nav_cmd_t){0, 0, nt->hold ? nt->hold_wz : 0, 0};
        nt->cmd_source = CMD_HOLD;
        goto send_checked;
    }
    if(nt->async_gmap && nt->use_gmap && !nt->gm.ready && nt->mode!=NAV_MODE_ESTOP) {
        nt->prog_t_us=now;nt->last_ok_us=now;nt->cmd_source=CMD_HOLD;
        goto send_checked; /* 首张记忆图完成后再规划，避免发布全未知初图路线。 */
    }
    if(nt->mode==NAV_MODE_TRACK && !target_alive(&nt->tgt,now)) {
        nav_task_set_mode(nt,NAV_MODE_SEARCH);goto send_checked;
    }
    int32_t scan_gx=nt->mode==NAV_MODE_TRACK?nt->tgt.x:nt->goal_x;
    int32_t scan_gy=nt->mode==NAV_MODE_TRACK?nt->tgt.y:nt->goal_y;
    bool need_motion=(nt->mode==NAV_MODE_GOTO &&
        fx_dist(nt->goal_x-p->x,nt->goal_y-p->y)>nav_max32(1,nt->lp_cfg.goal_tol_mm-nt->cfg.goal_pose_margin_mm)) ||
        nt->mode==NAV_MODE_SEARCH || (nt->mode==NAV_MODE_TRACK && nt->tgt.valid);
    if(need_motion && support_scan_needed()) {
        /* 启动平地声明只支撑当前原地观察，不能随车扩大。
         * 按实际反馈位姿完成一圈并停稳后，才进入平移规划。 */
        nt->cmd_source=CMD_GROUND_SCAN;nt->prog_t_us=nt->last_ok_us=now;
        if(!nt->scan_active) {
            nt->scan_active=true;nt->scan_start_us=now;nt->scan_turned=0;nt->scan_last_yaw=p->yaw;
            nt->scan_dir=fx_angle_diff(p->yaw,fx_atan2(scan_gy-p->y,scan_gx-p->x))< -182?-1:1;
        }
        nt->scan_turned+=nt->scan_dir*fx_angle_diff(nt->scan_last_yaw,p->yaw);nt->scan_last_yaw=p->yaw;
        if((uint32_t)(now-nt->scan_start_us)>24000000) nt->scan_failed=true;
        bool feedback=nt->pose.have_odo && (uint32_t)(now-nt->pose.last_odo_us)<100000;
        if(!nt->scan_failed && feedback && nt->scan_turned>=65536 && !nt->scan_exit_valid &&
           nav_abs32(fx_angle_diff(p->yaw,fx_atan2(scan_gy-p->y,scan_gx-p->x)))<=8192) {
            /* 一圈不是观测充分的证明：必须实际查到一条450mm支撑走廊。
             * 375仅用于只读前推（1.2s=450mm），实际退出速度另限50。 */
            nav_cmd_t probe={375,0,0,0};
            lp_cfg_t observe=nt->lp_cfg;observe.dt_ms=50;observe.safe_steps=24;
            int32_t x=p->x+(int32_t)((int64_t)450*fx_cos(p->yaw)/Q14);
            int32_t y=p->y+(int32_t)((int64_t)450*fx_sin(p->yaw)/Q14);
            uint8_t body=motion_segment(nt->map,&nt->mi,p,&observe,x,y);
            if((body==MOTION_OK || body==MOTION_UNKNOWN) && support_check(p,&observe,&nt->mi,&probe)==SUPPORT_PASS) {
                nt->scan_exit_x=x;nt->scan_exit_y=y;
                nt->scan_exit_valid=true;
            }
        }
        if(!nt->scan_failed && feedback && !nt->scan_exit_valid) nt->cmd.wz=(int16_t)(nt->scan_dir*600);
        if(nt->scan_exit_valid && feedback && nav_abs32(nt->pose.odo_wz)<=50 &&
           nav_abs32(nt->pose.odo_vx)<=20 && nav_abs32(nt->pose.odo_vy)<=20) {
            if(!nt->scan_settling) {nt->scan_settling=true;nt->scan_settle_us=now;}
            else if((uint32_t)(now-nt->scan_settle_us)>=500000) {
                support_scan_finish();nt->scan_active=false;nt->path_valid=false;
            }
        } else nt->scan_settling=false;
        if(nt->scan_failed) nt->state=ST_BLOCKED;
        goto send_checked;
    }
    if(need_motion && nt->scan_exit_valid && !support_scan_needed()) {
        nt->cmd_source=CMD_GROUND_EXIT;nt->look_x=nt->scan_exit_x;nt->look_y=nt->scan_exit_y;
        nt->last_ok_us=now;
        nt->state=nt->mode==NAV_MODE_GOTO?ST_MOVING:nt->mode==NAV_MODE_TRACK?ST_TRACKING:ST_SEARCHING;
        int d=fx_dist(nt->scan_exit_x-p->x,nt->scan_exit_y-p->y);
        if(d<=100) {nt->scan_exit_valid=false;nt->path_valid=false;nt->prog_t_us=now;}
        else {
            if(fx_dist(p->x-nt->prog_x,p->y-nt->prog_y)>25) {
                nt->prog_x=p->x;nt->prog_y=p->y;nt->prog_t_us=now;
            }
            if((uint32_t)(now-nt->prog_t_us)>nt->cfg.stall_us) nt->scan_failed=true;
            int32_t error=fx_angle_diff(p->yaw,fx_atan2(nt->scan_exit_y-p->y,nt->scan_exit_x-p->x));
            bool feedback=nt->pose.have_odo && (uint32_t)(now-nt->pose.last_odo_us)<100000;
            if(!nt->scan_failed && feedback) {
                if(nav_abs32(error)>182) {
                    int w=nav_clamp32((int32_t)((int64_t)error*12566/65536),-400,400);
                    nt->cmd.wz=(int16_t)(w<0?nav_min32(-100,w):nav_max32(100,w));
                } else if(nav_abs32(nt->pose.odo_wz)<=50) nt->cmd.vx=(int16_t)nav_min32(50,nt->lp_cfg.v_max);
            }
            if(nt->scan_failed) nt->state=ST_BLOCKED;
        }
        goto send_checked; /* 每一步仍用当前图/轮支撑复核；不会假报全局任务到达。 */
    }

    switch (nt->mode) {
    case NAV_MODE_IDLE:
    case NAV_MODE_ESTOP:
        nt->cmd = (nav_cmd_t){0, 0, 0, (uint8_t)nt->mode};
        nt->cmd_source = nt->mode == NAV_MODE_ESTOP ? CMD_ESTOP : CMD_STOP;
        break;

    case NAV_MODE_GOTO:
        if (drive_to(nt, now, nt->goal_x, nt->goal_y,
                     (nt->wp_n > 0 && (nt->wp_i + 1 < nt->wp_n || nt->wp_loop)) ? nt->cfg.wp_tol_mm :
                     nav_max32(1, nt->lp_cfg.goal_tol_mm - nt->cfg.goal_pose_margin_mm))) {
            if (nt->wp_n > 0 && (nt->wp_i + 1 < nt->wp_n || nt->wp_loop)) {
                nt->wp_i = (nt->wp_i + 1) % nt->wp_n;
                astar_cancel();
                nt->goal_x = nt->wp_x[nt->wp_i];
                nt->goal_y = nt->wp_y[nt->wp_i];
                if(!nt->gm.has_prior)
                    local_frame_point(&nt->terrain_frame,nt->goal_x,nt->goal_y,&nt->goal_x,&nt->goal_y,false);
                nt->path_valid = false;
                nt->refine_path = nt->path_fine = false;
                nt->prog_t_us = now;
            } else {
                nt->mode = NAV_MODE_IDLE;
                nt->state = ST_ARRIVED;
            }
        } else {
            nt->state = ((uint32_t)(now - nt->last_ok_us) > nt->cfg.blocked_us) ? ST_BLOCKED : ST_MOVING;
        }
        break;

    case NAV_MODE_TRACK:
        if (!target_alive(&nt->tgt, now)) {
            nav_task_set_mode(nt, NAV_MODE_SEARCH);
            nt->cmd = (nav_cmd_t){0, 0, 0, 0};
            break;
        }
        target_approach_goal(&nt->tgt, p, &nt->goal_x, &nt->goal_y);
        if (drive_to(nt, now, nt->goal_x, nt->goal_y, nt->cfg.track_hold_mm)) {
            /* 已在站位点：转向面对目标 */
            int32_t dh = fx_angle_diff(p->yaw, fx_atan2(nt->tgt.y - p->y, nt->tgt.x - p->x));
            int32_t wz = dh * 2 / 3;                                /* 比例转向（angle -> mrad/s 近似） */
            nt->cmd.wz = (int16_t)nav_clamp32(wz, -nt->lp_cfg.w_max / 2, nt->lp_cfg.w_max / 2);
            if (nav_abs32(dh) < 900) nt->cmd.wz = 0;                /* < 5° */
            nt->cmd_source = CMD_TRACK_FACE;
        }
        nt->state = ST_TRACKING;
        break;

    case NAV_MODE_SEARCH:
        if (target_alive(&nt->tgt, now)) { nav_task_set_mode(nt, NAV_MODE_TRACK); break; }
        if (nt->search_goal_valid && (uint32_t)(now - nt->search_t_us) > nt->cfg.search_repick_us) {
            /* 超时没到：这个目标够不着，拉黑后重选 */
            explore_blacklist(&nt->exp, nt->goal_x, nt->goal_y);
            nt->search_goal_valid = false;
        }
        if (!nt->search_goal_valid) {
            const bool seen_mode = nt->gm.has_prior;
            nt->search_goal_valid = nt->use_gmap &&
                explore_frontier(&nt->exp, &nt->ex_cfg, &nt->gm, &nt->mi, p, seen_mode, &nt->goal_x, &nt->goal_y);
            if (!nt->search_goal_valid && seen_mode && nt->exp.n_seen > 0) {
                /* 地图里能看的地方都看过了：清空重来（目标可能刚走进来） */
                explore_seen_reset(&nt->exp);
                explore_seen_mark(&nt->exp, &nt->ex_cfg, &nt->gm, &nt->mi, p);
                nt->search_goal_valid =
                    explore_frontier(&nt->exp, &nt->ex_cfg, &nt->gm, &nt->mi, p, true, &nt->goal_x, &nt->goal_y);
            }
            if (!nt->search_goal_valid)
                nt->search_goal_valid = explore_pick(&nt->exp, &nt->ex_cfg, nt->map, &nt->mi, p, &nt->goal_x, &nt->goal_y);
            nt->search_t_us = now;
            nt->path_valid=false;
            nt->refine_path = nt->path_fine = false;
        }
        if (!nt->search_goal_valid) {
            nt->cmd = (nav_cmd_t){0, 0, (int16_t)(nt->lp_cfg.w_max / 2), 0};     /* 原地转一圈找路 */
            nt->cmd_source = CMD_SEARCH;
        } else if (drive_to(nt, now, nt->goal_x, nt->goal_y, 700)) {     /* 边界点：走近到能看清即可 */
            nt->search_goal_valid = false;
        }
        nt->state = ST_SEARCHING;
        break;
    }
send_checked:
    if (nt->slow && nt->cmd_source != CMD_LOCAL) {
        nt->cmd.vx = (int16_t)(nt->cmd.vx / 2);
        nt->cmd.vy = (int16_t)(nt->cmd.vy / 2);
        nt->cmd.wz = (int16_t)(nt->cmd.wz / 2);
    }
    /* 所有非零运动只有这一个出口。先约束最终指令，再检查同代地图和完整足迹。 */
    if (nt->cmd.vx || nt->cmd.vy || nt->cmd.wz) {
        hal_map_info_t check;
        if (!nt->have_map || nt->stale || !fresh) nt->motion_result = MOTION_NO_MAP;
        else if (!hal_map_info(&check) || !motion_map_same(&check, &nt->mi)) nt->motion_result = MOTION_MAP_CHANGED;
        else {
            nt->motion_result = motion_check(nt->map, &nt->mi, p, &nt->lp_cfg, &nt->cmd);
            if(nt->motion_result==MOTION_OK || nt->motion_result==MOTION_UNKNOWN) {
                /* 同一次无IRQ调用内，局部规划已核对同一最终命令/同代地图；
                 * 复用轮支撑结论。其它来源或改动过的命令仍在此完整查询。 */
                bool checked=nt->cmd_source==CMD_LOCAL && nt->lp.ground_checked &&
                    nt->cmd.vx==nt->lp.cmd.vx && nt->cmd.vy==nt->lp.cmd.vy && nt->cmd.wz==nt->lp.cmd.wz;
                int ground=checked?SUPPORT_PASS:support_check(p,&nt->lp_cfg,&nt->mi,&nt->cmd);
                if(ground!=SUPPORT_PASS) nt->motion_result=ground==SUPPORT_FRAME?MOTION_MAP_CHANGED:
                    ground==SUPPORT_BLOCK?MOTION_SUPPORT_BLOCK:MOTION_SUPPORT_UNKNOWN;
            }
            if (!hal_map_info(&check) || !motion_map_same(&check, &nt->mi)) nt->motion_result = MOTION_MAP_CHANGED;
        }
        /* 现有雷达近场有盲区：保留A*允许未知时的前向探索，并明确记录UNKNOWN，
         * 不将其记为已证实自由。未观测的新足迹不允许倒车；近场支撑/停止区待T1/N3。 */
        /* 圆形包络下任意曲线200ms中心路程仍<=12mm；轮胎旋转扫掠单独检查。
         * UNKNOWN标志保留。实际延迟/刹停并未由这条名义上界覆盖。 */
        bool uncertain_creep = nt->motion_result == MOTION_UNKNOWN && nt->as_cfg.allow_unknown &&
                               nt->cmd.vx>=0 && nt->cmd.vx<=60 && nt->cmd.vy==0;
        if (nt->motion_result != MOTION_OK && !uncertain_creep) {
            nt->cmd = (nav_cmd_t){0, 0, 0, 0};
            /* 切图丢弃本次速度，世界坐标路线保留到下周期重新复核。 */
            if(nt->motion_result!=MOTION_MAP_CHANGED || (!nt->use_gmap && !nt->path_fine)) nt->path_valid=false;
            nt->last_ok_us = now - nt->cfg.blocked_us - 1u;
            if (nt->mode == NAV_MODE_GOTO) nt->state = ST_BLOCKED;
        }
    }
    nt->pose.cmd_still = (nt->cmd.vx == 0 && nt->cmd.vy == 0 && nt->cmd.wz == 0);
    nt->progress_speed=(int16_t)(nav_abs32(nt->cmd.vx)+nav_abs32(nt->cmd.vy));
    nt->lp.prev_wz = nt->cmd.wz;
    support_note_cmd(&nt->cmd);
    hal_send_cmd(&nt->cmd);
}

void nav_task_map_work(nav_task_t *nt,int rows)
{
    if(!nt->async_gmap || !nt->have_map || nt->stale) return;
    nav_pose_t p=nt->pose.cur;
    if(!nt->gm.has_prior) local_frame_pose(&nt->terrain_frame,&nt->pose.cur,&p);
    gmap_update_rows(&nt->gm,nt->map,&nt->mi,p.x,p.y,rows);
}

void nav_task_step(nav_task_t *nt,uint32_t now)
{
    if(nt->gm.has_prior) { task_step_local(nt,now);return; }
    /* 对外位姿/目标保持全局定位系；规划、局部图、进展与恢复用连续系。
     * 只替换本次规划视图，不改变全局历史积分器。 */
    nav_pose_t global=nt->pose.cur;
    int32_t gx=nt->goal_x,gy=nt->goal_y,tx=nt->tgt.x,ty=nt->tgt.y;
    int wi=nt->wp_i;
    local_frame_pose(&nt->terrain_frame,&global,&nt->pose.cur);
    local_frame_point(&nt->terrain_frame,gx,gy,&nt->goal_x,&nt->goal_y,false);
    local_frame_point(&nt->terrain_frame,tx,ty,&nt->tgt.x,&nt->tgt.y,false);
    int32_t lgx=nt->goal_x,lgy=nt->goal_y;
    nt->stepping_local=true;
    task_step_local(nt,now);
    nt->stepping_local=false;
    nt->pose.cur=global;nt->tgt.x=tx;nt->tgt.y=ty;
    if(nt->wp_i!=wi) {nt->goal_x=nt->wp_x[nt->wp_i];nt->goal_y=nt->wp_y[nt->wp_i];}
    else if(nt->goal_x==lgx && nt->goal_y==lgy) {nt->goal_x=gx;nt->goal_y=gy;}
    else local_frame_point(&nt->terrain_frame,nt->goal_x,nt->goal_y,&nt->goal_x,&nt->goal_y,true);
}

void nav_task_status(const nav_task_t *nt, proto_status_t *s, uint32_t now_us)
{
    const nav_pose_t *p = &nt->pose.cur;
    s->mode = (uint8_t)nt->mode;
    s->state = nt->state;
    s->map_seq = nt->mi.seq;
    s->x = p->x; s->y = p->y;
    s->yaw_cdeg = ANGLE_TO_CDEG(p->yaw);
    s->goal_x = nt->goal_x; s->goal_y = nt->goal_y;
    s->vx = nt->cmd.vx; s->wz = nt->cmd.wz;
    s->flags = (target_alive(&nt->tgt, now_us) ? STF_TARGET_VALID : 0) | (nt->have_map ? STF_LIDAR_OK : 0) |
               (nt->pose.have_odo ? STF_ODOM_OK : 0);
    s->t_us = now_us;
}
