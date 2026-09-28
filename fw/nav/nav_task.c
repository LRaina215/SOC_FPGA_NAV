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
    nt->cfg.search_repick_us = 8000000;
    nt->cfg.track_hold_mm = 200;
    lp_default_cfg(&nt->lp_cfg);
    astar_default_cfg(&nt->as_cfg);
    explore_default_cfg(&nt->ex_cfg);
    pose_init(&nt->pose, ANGLE_FROM_CDEG(1000), 100, 200);
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

/* 沿全局路径取前视点；返回 false 表示没有路径 */
static bool pick_lookahead(nav_task_t *nt, const nav_pose_t *p, int32_t gx, int32_t gy, int32_t *lx, int32_t *ly)
{
    uint32_t now = hal_time_us();
    int rx, ry, tx, ty;
    nav_mm_to_win(&nt->mi, p->x, p->y, &rx, &ry);
    nav_mm_to_win(&nt->mi, gx, gy, &tx, &ty);
    bool goal_in = tx >= 1 && tx <= N - 2 && ty >= 1 && ty <= N - 2;
    if (!nt->path_valid || (nt->path_seq != nt->mi.seq && (uint32_t)(now - nt->path_t_us) > nt->cfg.replan_us)) {
        clamp_goal_to_window(rx, ry, &tx, &ty);
        int rc = astar_plan(&nt->as_cfg, nt->map, nav_clamp32(rx, 0, N - 1), nav_clamp32(ry, 0, N - 1), tx, ty, &nt->path);
        nt->path_valid = (rc >= 0 && nt->path.n > 0);
        nt->path_seq = nt->mi.seq;
        nt->path_t_us = now;
        if (!nt->path_valid) return false;
    }
    /* 目标在窗口内且比前视距离近：直接用目标 */
    if (goal_in && fx_dist(gx - p->x, gy - p->y) <= nt->cfg.lookahead_mm) {
        *lx = gx; *ly = gy;
        return true;
    }
    /* 找离机器人最近的路径点，再向前找第一个超过前视距离的点 */
    int best = 0;
    int32_t bd = INT32_MAX;
    for (int i = 0; i < nt->path.n; i++) {
        int32_t x, y;
        nav_win_to_mm(&nt->mi, nt->path.wx[i], nt->path.wy[i], &x, &y);
        int32_t d = fx_dist(x - p->x, y - p->y);
        if (d < bd) { bd = d; best = i; }
    }
    /* 向前取：直线可见（不切过障碍角）且不超过前视距离的最远路径点 */
    int pick = best;
    for (int i = best; i < nt->path.n; i++) {
        int32_t x, y;
        nav_win_to_mm(&nt->mi, nt->path.wx[i], nt->path.wy[i], &x, &y);
        if (i > best && !line_clear(nt->map, rx, ry, nt->path.wx[i], nt->path.wy[i], nt->as_cfg.block)) break;
        pick = i;
        if (fx_dist(x - p->x, y - p->y) >= nt->cfg.lookahead_mm) break;
    }
    if (pick == best && pick + 1 < nt->path.n) pick++;       /* 至少向前一格 */
    nav_win_to_mm(&nt->mi, nt->path.wx[pick], nt->path.wy[pick], lx, ly);
    if (pick == nt->path.n - 1 && goal_in) { *lx = gx; *ly = gy; }
    return true;
}

/* 朝 (gx, gy) 行进一个周期；返回是否已到达（距离 < tol） */
static bool drive_to(nav_task_t *nt, uint32_t now, int32_t gx, int32_t gy, int32_t tol)
{
    const nav_pose_t *p = &nt->pose.cur;
    nt->cmd = (nav_cmd_t){0, 0, 0, 0};
    if (fx_dist(gx - p->x, gy - p->y) < tol) return true;
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
    if (hal_map_info(&mi) && (!nt->have_map || mi.seq != nt->mi.seq)) {
        hal_map_copy(nt->map, &nt->mi);
        nt->have_map = true;
    }
    explore_mark(&nt->exp, &nt->pose.cur);
    const nav_pose_t *p = &nt->pose.cur;

    if (!nt->have_map && nt->mode != NAV_MODE_ESTOP) {
        nt->cmd = (nav_cmd_t){0, 0, 0, 0};
        nt->pose.cmd_still = true;
        hal_send_cmd(&nt->cmd);
        return;
    }

    switch (nt->mode) {
    case NAV_MODE_IDLE:
    case NAV_MODE_ESTOP:
        nt->cmd = (nav_cmd_t){0, 0, 0, (uint8_t)nt->mode};
        break;

    case NAV_MODE_GOTO:
        if (drive_to(nt, now, nt->goal_x, nt->goal_y, nt->lp_cfg.goal_tol_mm)) {
            if (nt->wp_n > 0 && (nt->wp_i + 1 < nt->wp_n || nt->wp_loop)) {
                nt->wp_i = (nt->wp_i + 1) % nt->wp_n;
                nt->goal_x = nt->wp_x[nt->wp_i];
                nt->goal_y = nt->wp_y[nt->wp_i];
                nt->path_valid = false;
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
        if (!nt->search_goal_valid || (uint32_t)(now - nt->search_t_us) > nt->cfg.search_repick_us) {
            nt->search_goal_valid = explore_pick(&nt->exp, &nt->ex_cfg, nt->map, &nt->mi, p, &nt->goal_x, &nt->goal_y);
            nt->search_t_us = now;
            nt->path_valid = false;
        }
        if (!nt->search_goal_valid) {
            nt->cmd = (nav_cmd_t){0, 0, (int16_t)(nt->lp_cfg.w_max / 2), 0};     /* 原地转一圈找路 */
        } else if (drive_to(nt, now, nt->goal_x, nt->goal_y, 300)) {
            nt->search_goal_valid = false;
        }
        nt->state = ST_SEARCHING;
        break;
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
