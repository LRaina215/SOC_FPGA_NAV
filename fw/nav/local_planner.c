#include "local_planner.h"
#include "fixmath.h"

void lp_default_cfg(lp_cfg_t *c)
{
    c->caps.holonomic = false;
    c->caps.vx_max = 600;
    c->caps.vy_max = 0;
    c->caps.wz_max = 1500;
    c->caps.radius_mm = 140;
    c->v_min = 0;
    c->v_max = 500;
    c->w_max = 1200;
    c->nv = 4;
    c->nw = 9;
    c->steps = 20;
    c->dt_ms = 100;
    c->safe_steps = 12;
    c->w_progress = 100;
    c->w_heading = 40;
    c->w_cost = 30;
    c->w_unknown = 20;
    c->goal_tol_mm = 150;
    c->v_back = 150;
    c->w_back = 300;
    c->w_hyst = 60;
}

/* mm -> 世界栅格 Q8 */
static int32_t mm_to_q8(int32_t mm, uint16_t inv_res)
{
    return (int32_t)(((int64_t)mm * inv_res) >> 8);
}

static void select_best(const lp_cfg_t *c, const nav_pose_t *pose, const hal_cand_t *cand, int n,
                        int32_t x0, int32_t y0, int32_t gx, int32_t gy, int escape, lp_result_t *r)
{
    int32_t d0 = fx_dist(gx - x0, gy - y0);
    r->best = -1;
    r->best_score = INT32_MIN;
    r->n_valid = 0;
    for (int i = 0; i < n; i++) {
        const hal_traj_res_t *t = &r->res[i];
        if (t->collide && t->coll_step < c->safe_steps) continue;
        int steps_done = t->collide ? t->coll_step : c->steps;
        int32_t d1 = fx_dist(gx - t->end_x, gy - t->end_y);
        /* 终点航向 = 起始航向 + dth*步数。原地转向只看 1 秒后的朝向（避免整段前推转过头） */
        int hs = (r->cv[i] == 0 && r->cvy[i] == 0) ? nav_min32(steps_done, 1000 / c->dt_ms) : steps_done;
        angle_t th_end = (angle_t)(pose->yaw + cand[i].dth * hs);
        angle_t th_goal = fx_atan2(gy - t->end_y, gx - t->end_x);
        int32_t dh = nav_abs32(fx_angle_diff(th_end, th_goal));      /* 0..32768 */
        if (d1 < (int32_t)(256 * c->goal_tol_mm / 50)) dh = 0;         /* 已在目标附近：不计航向 */
        int32_t known = steps_done - t->n_unknown;
        int32_t avg_cost = known > 0 ? (int32_t)(t->sum_cost / known) : 0;
        int32_t score = c->w_progress * (d0 - d1) / 256        /* 靠近的格数 */
                      - c->w_heading * dh / 4096               /* 每 22.5° 一个单位 */
                      - c->w_cost * avg_cost / 50
                      - c->w_unknown * t->n_unknown / 4;
        if (escape) score -= 4 * c->w_cost * avg_cost / 50;   /* 脱困：膨胀区里走得越少越好 */
        if (t->collide) score -= 50 * (c->steps - t->coll_step);
        if (r->cv[i] < 0) score -= c->w_back;
        if ((int32_t)r->cw[i] * r->prev_wz > 0) score += c->w_hyst;
        r->n_valid++;
        if (score > r->best_score) { r->best_score = score; r->best = i; }
    }
}

bool lp_plan(const lp_cfg_t *c, const hal_map_info_t *mi, const nav_pose_t *pose,
             int32_t goal_x, int32_t goal_y, lp_result_t *r)
{
    hal_cand_t cand[LP_MAX_CAND];
    int n = 0;
    /* 每步位移：v(mm/s) * dt(ms)/1000 -> mm/步 -> 栅格 Q8 */
    for (int iv = 0; iv < c->nv && n < LP_MAX_CAND; iv++) {
        int32_t v = (c->nv > 1) ? c->v_min + (c->v_max - c->v_min) * (iv + 1) / c->nv : c->v_max;
        for (int iw = 0; iw < c->nw && n < LP_MAX_CAND; iw++) {
            int32_t w = (c->nw > 1) ? -c->w_max + 2 * c->w_max * iw / (c->nw - 1) : 0;
            r->cv[n] = (int16_t)v; r->cvy[n] = 0; r->cw[n] = (int16_t)w;
            n++;
        }
    }
    /* 原地转向 */
    for (int s = -1; s <= 1 && n < LP_MAX_CAND; s += 2) {
        r->cv[n] = 0; r->cvy[n] = 0; r->cw[n] = (int16_t)(s * c->w_max / 2); n++;
    }
    /* 倒车脱困（带大惩罚，只在前方都不可行时被选中） */
    if (c->v_back > 0) {
        for (int s = -1; s <= 1 && n < LP_MAX_CAND; s++) {
            r->cv[n] = (int16_t)-c->v_back; r->cvy[n] = 0; r->cw[n] = (int16_t)(s * c->w_max / 3); n++;
        }
    }
    /* 全向底盘：横移候选 */
    if (c->caps.holonomic && c->caps.vy_max > 0) {
        for (int s = -1; s <= 1 && n < LP_MAX_CAND; s += 2) {
            r->cv[n] = 0; r->cvy[n] = (int16_t)(s * c->caps.vy_max); r->cw[n] = 0; n++;
            r->cv[n] = (int16_t)(c->v_max / 2); r->cvy[n] = (int16_t)(s * c->caps.vy_max / 2); r->cw[n] = 0; n++;
        }
    }
    for (int i = 0; i < n; i++) {
        int32_t dmm = r->cv[i] * c->dt_ms / 1000;
        int32_t dmmy = r->cvy[i] * c->dt_ms / 1000;
        cand[i].dvx = (int16_t)mm_to_q8(dmm, mi->inv_res);
        cand[i].dvy = (int16_t)mm_to_q8(dmmy, mi->inv_res);
        /* mrad/s × dt -> angle/步：65536/(2π·1000) */
        cand[i].dth = (int16_t)((int64_t)r->cw[i] * c->dt_ms * 65536 / 6283185);
    }
    r->n_cand = n;

    int32_t x0 = mm_to_q8(pose->x, mi->inv_res), y0 = mm_to_q8(pose->y, mi->inv_res);
    int32_t gx = mm_to_q8(goal_x, mi->inv_res), gy = mm_to_q8(goal_y, mi->inv_res);
    /* 第一遍：膨胀区（253）也算碰撞。全部不可行时（多半是机器人已在膨胀区里——贴近障碍、或误报/漂移
     * 让障碍"长"到了脚下），第二遍只把真正的致命格（254）算碰撞，膨胀区按高代价计，选一条离开的轨迹 */
    r->escape = false;
    for (int pass = 0; pass < 2; pass++) {
        hal_traj_eval(cand, n, c->steps, x0, y0, pose->yaw, pass ? 254 : 253, r->res);
        select_best(c, pose, cand, n, x0, y0, gx, gy, pass, r);
        if (r->best >= 0) { r->escape = pass != 0; break; }
    }
    r->blocked = (r->best < 0);
    if (r->blocked) {
        r->cmd = (nav_cmd_t){0, 0, 0, 0};
        return false;
    }
    r->cmd.vx = r->cv[r->best];
    r->cmd.vy = r->cvy[r->best];
    r->cmd.wz = r->cw[r->best];
    r->cmd.mode = 0;
    r->prev_wz = r->cmd.wz;
    return true;
}
