#include "nav_task.h"
#include "fixmath.h"

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
    nt->cfg.recover_us = 1500000;
    lp_default_cfg(&nt->lp_cfg);
    astar_default_cfg(&nt->as_cfg);
    astar_default_cfg(&nt->as_cfg_g);
    /* 记忆栅格 10 cm 格（5 m 场地）：车身外接圆 140 mm，离致命格 1 格（障碍面约 5 cm）必撞 -> 贴障重罚；
     * 2–3 格按距离加罚，路径尽量走通道中间 */
    nt->as_cfg_g.clear_cells = 4;
    nt->as_cfg_g.clear_cost = 15;
    nt->as_cfg_g.near_cells = 2;
    nt->as_cfg_g.near_cost = 150;
    nt->use_gmap = true;
    gmap_default_cfg(&nt->gm.cfg);
    gmap_init(&nt->gm);
    explore_default_cfg(&nt->ex_cfg);
    pose_init(&nt->pose, ANGLE_FROM_CDEG(1000), 80, 200);
    target_init(&nt->tgt, 1500000, 700);
    explore_init(&nt->exp);
    nt->mode = NAV_MODE_IDLE;
    nt->state = ST_IDLE;
}

void nav_task_set_mode(nav_task_t *nt, nav_mode_t mode)
{
    nt->mode = mode;
    nt->path_valid = false;
    nt->search_goal_valid = false;
    nt->last_ok_us = hal_time_us();
    nt->prog_t_us = hal_time_us();
    nt->prog_x = nt->pose.cur.x; nt->prog_y = nt->pose.cur.y;
    nt->recovering = false;
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
        }
        if (x0 == x1 && y0 == y1) return true;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

/* 路径点 i -> mm 与局部窗口坐标 */
static void path_pt(const nav_task_t *nt, int i, int32_t *x, int32_t *y, int *wx, int *wy)
{
    if (nt->use_gmap) {
        gmap_cell_to_mm(&nt->gm, &nt->mi, nt->path.wx[i], nt->path.wy[i], x, y);
        nav_mm_to_win(&nt->mi, *x, *y, wx, wy);
    } else {
        *wx = nt->path.wx[i]; *wy = nt->path.wy[i];
        nav_win_to_mm(&nt->mi, *wx, *wy, x, y);
    }
}

static int plan_path(nav_task_t *nt, const nav_pose_t *p, int32_t gx, int32_t gy)
{
    if (nt->use_gmap) {
        int rx, ry, tx, ty;
        gmap_mm_to_cell(&nt->gm, &nt->mi, p->x, p->y, &rx, &ry);
        gmap_mm_to_cell(&nt->gm, &nt->mi, gx, gy, &tx, &ty);
        /* 目标在记忆栅格外：夹到边界（沿机器人->目标方向），clamp 按局部图尺寸写的，这里换算比例 */
        if (tx < 1 || tx > GM_N - 2 || ty < 1 || ty > GM_N - 2) {
            int32_t dx = tx - rx, dy = ty - ry, t = 1024;
            if (dx > 0 && tx > GM_N - 2) t = nav_min32(t, (GM_N - 2 - rx) * 1024 / dx);
            if (dx < 0 && tx < 1) t = nav_min32(t, (1 - rx) * 1024 / dx);
            if (dy > 0 && ty > GM_N - 2) t = nav_min32(t, (GM_N - 2 - ry) * 1024 / dy);
            if (dy < 0 && ty < 1) t = nav_min32(t, (1 - ry) * 1024 / dy);
            if (t < 0) t = 0;
            tx = rx + dx * t / 1024;
            ty = ry + dy * t / 1024;
        }
        nt->path_shift = nt->gm.n_shift;
        return astar_grid(&nt->as_cfg_g, nt->gm.grid, GM_N, rx, ry, tx, ty, &nt->path);
    }
    int rx, ry, tx, ty;
    nav_mm_to_win(&nt->mi, p->x, p->y, &rx, &ry);
    nav_mm_to_win(&nt->mi, gx, gy, &tx, &ty);
    clamp_goal_to_window(rx, ry, &tx, &ty);
    return astar_plan(&nt->as_cfg, nt->map, nav_clamp32(rx, 0, N - 1), nav_clamp32(ry, 0, N - 1), tx, ty, &nt->path);
}

/* 沿全局路径取前视点；返回 false 表示没有路径 */
static bool pick_lookahead(nav_task_t *nt, const nav_pose_t *p, int32_t gx, int32_t gy, int32_t *lx, int32_t *ly)
{
    uint32_t now = hal_time_us();
    int rx, ry, tx, ty;
    nav_mm_to_win(&nt->mi, p->x, p->y, &rx, &ry);
    nav_mm_to_win(&nt->mi, gx, gy, &tx, &ty);
    bool goal_in = tx >= 1 && tx <= N - 2 && ty >= 1 && ty <= N - 2;
    bool stale = nt->use_gmap && nt->path_shift != nt->gm.n_shift;
    if (!nt->path_valid || stale ||
        (nt->path_seq != nt->mi.seq && (uint32_t)(now - nt->path_t_us) > nt->cfg.replan_us)) {
        int rc = plan_path(nt, p, gx, gy);
        nt->path_valid = (rc >= 0 && nt->path.n > 0);
        nt->path_seq = nt->mi.seq;
        nt->path_t_us = now;
        if (!nt->path_valid) return false;
    }
    /* 目标比前视距离近：直接用目标 */
    if (fx_dist(gx - p->x, gy - p->y) <= nt->cfg.lookahead_mm) {
        *lx = gx; *ly = gy;
        return true;
    }
    /* 找离机器人最近的路径点，再向前找第一个超过前视距离的点 */
    int best = 0;
    int32_t bd = INT32_MAX;
    for (int i = 0; i < nt->path.n; i++) {
        int32_t x, y;
        int wx, wy;
        path_pt(nt, i, &x, &y, &wx, &wy);
        int32_t d = fx_dist(x - p->x, y - p->y);
        if (d < bd) { bd = d; best = i; }
    }
    /* 向前取：直线可见（不切过障碍角）且不超过前视距离的最远路径点 */
    int pick = best;
    for (int i = best; i < nt->path.n; i++) {
        int32_t x, y;
        int wx, wy;
        path_pt(nt, i, &x, &y, &wx, &wy);
        if (i > best && !line_clear(nt->map, rx, ry, wx, wy, nt->as_cfg.block)) break;
        pick = i;
        if (fx_dist(x - p->x, y - p->y) >= nt->cfg.lookahead_mm) break;
    }
    if (pick == best && pick + 1 < nt->path.n) pick++;       /* 至少向前一格 */
    int wx, wy;
    path_pt(nt, pick, lx, ly, &wx, &wy);
    if (pick == nt->path.n - 1 && goal_in) { *lx = gx; *ly = gy; }
    return true;
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
    return line_clear(nt->map, rx, ry, tx, ty, NP_COST_LETHAL);
}

static bool drive_to(nav_task_t *nt, uint32_t now, int32_t gx, int32_t gy, int32_t tol)
{
    const nav_pose_t *p = &nt->pose.cur;
    nt->cmd = (nav_cmd_t){0, 0, 0, 0};
    int32_t dgoal = fx_dist(gx - p->x, gy - p->y);
    if (dgoal < tol) return true;
    /* 卡住检测：stall_us 内位移不到 stall_mm */
    if (fx_dist(p->x - nt->prog_x, p->y - nt->prog_y) > nt->cfg.stall_mm) {
        nt->prog_x = p->x; nt->prog_y = p->y; nt->prog_t_us = now;
    } else if (!nt->recovering && (uint32_t)(now - nt->prog_t_us) > nt->cfg.stall_us) {
        nt->prog_t_us = now;
        if (dgoal < nt->cfg.near_goal_mm) return true;         /* 已到目标附近但进不去：接受 */
        /* 同一处第二次卡住：全局规划认为能过、局部规划过不去（窄缝、斜向缝隙的量化差异）——把前视点那里临时堵上 */
        if (nt->use_gmap && fx_dist(p->x - nt->stuck_x, p->y - nt->stuck_y) < 300 && nt->n_recover > 0)
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
            nt->last_ok_us = now;
            return false;
        }
    }
    int32_t lx = gx, ly = gy;
    if (!pick_lookahead(nt, p, gx, gy, &lx, &ly)) { lx = gx; ly = gy; }
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
        nt->lp.prev_wz = (int16_t)wz;
        nt->last_ok_us = now;
        return false;
    }
    if (lp_plan(&nt->lp_cfg, &nt->mi, p, lx, ly, &nt->lp)) {
        nt->cmd = nt->lp.cmd;
        nt->last_ok_us = now;
    } else {
        /* 无可行候选：原地转向前视点方向，重新规划 */
        int32_t dh = fx_angle_diff(p->yaw, fx_atan2(ly - p->y, lx - p->x));
        nt->cmd.wz = (int16_t)(dh > 0 ? nt->lp_cfg.w_max / 2 : -nt->lp_cfg.w_max / 2);
        nt->path_valid = false;
    }
    return false;
}

void nav_task_step(nav_task_t *nt, uint32_t now)
{
    /* 刷新代价图 */
    hal_map_info_t mi;
    bool fresh = hal_map_info(&mi) && !(nt->stale && mi.seq == nt->stale_seq);
    if (fresh) nt->stale = false;
    if (fresh && (!nt->have_map || mi.seq != nt->mi.seq)) {
        hal_map_copy(nt->map, &nt->mi);
        nt->have_map = true;
        gmap_update(&nt->gm, nt->map, &nt->mi, nt->pose.cur.x, nt->pose.cur.y);
    }
    explore_mark(&nt->exp, &nt->pose.cur);
    if (nt->gm.has_prior && nt->have_map && !nt->hold) explore_seen_mark(&nt->exp, &nt->ex_cfg, &nt->gm, &nt->mi, &nt->pose.cur);
    const nav_pose_t *p = &nt->pose.cur;

    if ((!nt->have_map || nt->hold) && nt->mode != NAV_MODE_ESTOP) {
        nt->prog_t_us = now;             /* 保持期间不算卡住 */
        nt->last_ok_us = now;
        nt->cmd = (nav_cmd_t){0, 0, nt->hold ? nt->hold_wz : 0, 0};
        nt->pose.cmd_still = nt->cmd.wz == 0;
        hal_send_cmd(&nt->cmd);
        return;
    }

    switch (nt->mode) {
    case NAV_MODE_IDLE:
    case NAV_MODE_ESTOP:
        nt->cmd = (nav_cmd_t){0, 0, 0, (uint8_t)nt->mode};
        break;

    case NAV_MODE_GOTO:
        if (drive_to(nt, now, nt->goal_x, nt->goal_y,
                     (nt->wp_n > 0 && (nt->wp_i + 1 < nt->wp_n || nt->wp_loop)) ? nt->cfg.wp_tol_mm : nt->lp_cfg.goal_tol_mm)) {
            if (nt->wp_n > 0 && (nt->wp_i + 1 < nt->wp_n || nt->wp_loop)) {
                nt->wp_i = (nt->wp_i + 1) % nt->wp_n;
                nt->goal_x = nt->wp_x[nt->wp_i];
                nt->goal_y = nt->wp_y[nt->wp_i];
                nt->path_valid = false;
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
            nt->path_valid = false;
        }
        if (!nt->search_goal_valid) {
            nt->cmd = (nav_cmd_t){0, 0, (int16_t)(nt->lp_cfg.w_max / 2), 0};     /* 原地转一圈找路 */
        } else if (drive_to(nt, now, nt->goal_x, nt->goal_y, 700)) {     /* 边界点：走近到能看清即可 */
            nt->search_goal_valid = false;
        }
        nt->state = ST_SEARCHING;
        break;
    }
    if (nt->slow) {
        nt->cmd.vx = (int16_t)(nt->cmd.vx / 2);
        nt->cmd.vy = (int16_t)(nt->cmd.vy / 2);
    }
    nt->pose.cmd_still = (nt->cmd.vx == 0 && nt->cmd.vy == 0 && nt->cmd.wz == 0);
    hal_send_cmd(&nt->cmd);
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
