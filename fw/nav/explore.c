#include "explore.h"
#include "fixmath.h"
#include "nav_params.h"

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
    c->frontier_min_mm = 1000;
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

void explore_blacklist(explore_t *e, int32_t x, int32_t y)
{
    e->bl_x[e->bl_i] = x;
    e->bl_y[e->bl_i] = y;
    e->bl_i = (e->bl_i + 1) % EXP_BL;
    if (e->bl_n < EXP_BL) e->bl_n++;
}

static uint16_t s_bd[GM_N * GM_N];         /* 广度优先距离（格），0xFFFF = 未到达 */
static uint16_t s_bq[GM_N * GM_N];

bool explore_frontier(explore_t *e, const explore_cfg_t *c, const gmap_t *gm, const hal_map_info_t *mi,
                      const nav_pose_t *pose, int32_t *gx, int32_t *gy)
{
    static const int8_t DX4[4] = {1, -1, 0, 0}, DY4[4] = {0, 0, 1, -1};
    int rx, ry;
    gmap_mm_to_cell(gm, mi, pose->x, pose->y, &rx, &ry);
    e->n_frontier = 0;
    if (rx < 0 || ry < 0 || rx >= GM_N || ry >= GM_N) return false;
    for (int i = 0; i < GM_N * GM_N; i++) s_bd[i] = 0xFFFF;
    int qh = 0, qt = 0;
    s_bd[ry * GM_N + rx] = 0;
    s_bq[qt++] = (uint16_t)(ry * GM_N + rx);
    int best = -1;
    int32_t best_cost = INT32_MAX;
    /* 目标至少离开 min_goal（格） */
    int32_t cell_mm = (int32_t)((int64_t)GM_DS * 65536 / (mi->inv_res ? mi->inv_res : 1311));
    int32_t min_cells = c->min_goal_mm / cell_mm + 1;
    while (qh < qt) {
        int i = s_bq[qh++];
        int x = i % GM_N, y = i / GM_N;
        uint16_t d = s_bd[i];
        /* 是否边界格 */
        bool frontier = false;
        for (int k = 0; k < 4; k++) {
            int nx = x + DX4[k], ny = y + DY4[k];
            if (nx < 0 || ny < 0 || nx >= GM_N || ny >= GM_N) continue;
            uint8_t v = gm->grid[ny * GM_N + nx];
            if (v == 255) { frontier = true; continue; }
            if (v >= NP_COST_LETHAL || s_bd[ny * GM_N + nx] != 0xFFFF) continue;
            s_bd[ny * GM_N + nx] = (uint16_t)(d + 1);
            s_bq[qt++] = (uint16_t)(ny * GM_N + nx);
        }
        if (!frontier || d < min_cells) continue;
        /* 紧挨致命格的边界（平台/坡道侧沿外被遮挡的一侧）多半下不去，不选 */
        bool near_leth = false;
        for (int yy = y - 1; yy <= y + 1 && !near_leth; yy++)
            for (int xx = x - 1; xx <= x + 1; xx++)
                if (xx >= 0 && yy >= 0 && xx < GM_N && yy < GM_N && gm->grid[yy * GM_N + xx] == NP_COST_LETHAL) {
                    near_leth = true; break;
                }
        if (near_leth) continue;
        e->n_frontier++;
        int32_t fx, fy;
        gmap_cell_to_mm(gm, mi, x, y, &fx, &fy);
        if (fx_dist(fx - pose->x, fy - pose->y) < c->frontier_min_mm) continue;   /* 直线距离也要够远（否则一选就"到达"） */
        bool bl = false;
        for (int b = 0; b < e->bl_n; b++)
            if (fx_dist(fx - e->bl_x[b], fy - e->bl_y[b]) < 1000) { bl = true; break; }
        if (bl) continue;
        int32_t cost = (int32_t)d * 10 + c->w_visited * e->visited[vis_idx(fx, fy)];
        if (cost < best_cost) { best_cost = cost; best = i; }
    }
    if (best < 0) return false;
    gmap_cell_to_mm(gm, mi, best % GM_N, best / GM_N, gx, gy);
    return true;
}
