#include "explore.h"
#include "fixmath.h"

void explore_default_cfg(explore_cfg_t *c)
{
    c->w_clear = 10;
    c->w_unknown = 6;
    c->w_trav = 1;
    c->w_slope = 1;
    c->w_visited = 8;
    c->max_range_cells = 40;
    c->block = 253;
    c->min_goal_mm = 600;
}

void explore_init(explore_t *e)
{
    for (int i = 0; i < EXP_VIS_N * EXP_VIS_N; i++) e->visited[i] = 0;
    e->best = -1;
}

static int vis_idx(int32_t x, int32_t y)
{
    int32_t cx = x >= 0 ? x / EXP_VIS_CELL_MM : (x - EXP_VIS_CELL_MM + 1) / EXP_VIS_CELL_MM;
    int32_t cy = y >= 0 ? y / EXP_VIS_CELL_MM : (y - EXP_VIS_CELL_MM + 1) / EXP_VIS_CELL_MM;
    return (int)(((uint32_t)cy & (EXP_VIS_N - 1)) * EXP_VIS_N + ((uint32_t)cx & (EXP_VIS_N - 1)));
}

void explore_mark(explore_t *e, const nav_pose_t *pose)
{
    int i = vis_idx(pose->x, pose->y);
    if (e->visited[i] < 255) e->visited[i]++;
}

bool explore_pick(explore_t *e, const explore_cfg_t *c, const uint8_t *map, const hal_map_info_t *mi,
                  const nav_pose_t *pose, int32_t *gx, int32_t *gy)
{
    const int N = HAL_MAP_N;
    /* 机器人在窗口中的位置（栅格） */
    int32_t rx = (int32_t)(((int64_t)pose->x * mi->inv_res) >> 16) - (mi->cx - N / 2);
    int32_t ry = (int32_t)(((int64_t)pose->y * mi->inv_res) >> 16) - (mi->cy - N / 2);
    e->best = -1;
    int32_t best = INT32_MIN;
    for (int d = 0; d < EXP_DIRS; d++) {
        angle_t a = (angle_t)(d * (65536 / EXP_DIRS));
        int32_t ca = fx_cos(a), sa = fx_sin(a);
        int clear = 0, unknown = 0, known = 0, maxc = 0;
        int32_t sumc = 0;
        bool hit = false;
        for (int s = 1; s <= c->max_range_cells; s++) {
            int32_t x = rx + (ca * s + Q14 / 2) / Q14;
            int32_t y = ry + (sa * s + Q14 / 2) / Q14;
            if (x < 0 || y < 0 || x >= N || y >= N) { unknown += c->max_range_cells - s + 1; break; }
            uint8_t v = map[y * N + x];
            if (v == 255) { unknown++; continue; }
            if (v >= c->block) { hit = true; break; }
            if (!hit) clear = s;
            known++;
            sumc += v;
            if (v > maxc) maxc = v;
        }
        e->clear[d] = clear;
        int32_t avg = known ? sumc / known : 0;
        /* 方向末端的访问热度 */
        int32_t clear_mm = (int32_t)((int64_t)clear * 65536 / mi->inv_res);
        int32_t ex_mm = pose->x + (int32_t)((int64_t)ca * clear_mm / Q14);
        int32_t ey_mm = pose->y + (int32_t)((int64_t)sa * clear_mm / Q14);
        int32_t vis = e->visited[vis_idx(ex_mm, ey_mm)];
        int32_t sc = c->w_clear * clear + c->w_unknown * unknown + c->w_trav * (200 - avg) / 10
                   - c->w_slope * maxc / 10 - c->w_visited * vis;
        e->score[d] = sc;
        /* 目标太近（前方就是障碍）的方向不选 */
        int32_t dist_mm = (int32_t)((int64_t)clear * 65536 / mi->inv_res);
        if (dist_mm < c->min_goal_mm && unknown == 0) continue;
        if (sc > best) { best = sc; e->best = d; }
    }
    if (e->best < 0) return false;
    angle_t a = (angle_t)(e->best * (65536 / EXP_DIRS));
    /* 目标取可通行距离的 80%，至少 min_goal */
    int32_t dist_mm = (int32_t)((int64_t)e->clear[e->best] * 65536 / mi->inv_res * 8 / 10);
    if (dist_mm < c->min_goal_mm) dist_mm = c->min_goal_mm;
    *gx = pose->x + (int32_t)((int64_t)fx_cos(a) * dist_mm / Q14);
    *gy = pose->y + (int32_t)((int64_t)fx_sin(a) * dist_mm / Q14);
    return true;
}
