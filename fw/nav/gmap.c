#include "gmap.h"
#include "nav_params.h"

#define LN NP_MAP_N
#define UNSEEN (-128)

static int32_t fdiv(int32_t a, int32_t b) { return (a >= 0) ? a / b : -((-a + b - 1) / b); }

void gmap_default_cfg(gmap_cfg_t *c)
{
    c->occ_hit = 2;
    c->occ_miss = -1;
    c->occ_max = 6;
    c->occ_min = -4;
    c->occ_thr = 2;
    c->min_known = 6;
}

void gmap_init(gmap_t *g)
{
    gmap_cfg_t c = g->cfg;
    *g = (gmap_t){0};
    g->cfg = c;
    for (int i = 0; i < GM_N * GM_N; i++) { g->occ[i] = UNSEEN; g->cost[i] = 0; g->grid[i] = 255; }
}

static int32_t fine_idx(int32_t v_mm, uint16_t inv) { return (int32_t)(((int64_t)v_mm * inv) >> 16); }

void gmap_mm_to_cell(const gmap_t *g, const hal_map_info_t *mi, int32_t x, int32_t y, int *gx, int *gy)
{
    uint16_t inv = mi->inv_res ? mi->inv_res : 1311;
    *gx = fdiv(fine_idx(x, inv), GM_DS) - g->ox;
    *gy = fdiv(fine_idx(y, inv), GM_DS) - g->oy;
}

void gmap_cell_to_mm(const gmap_t *g, const hal_map_info_t *mi, int gx, int gy, int32_t *x, int32_t *y)
{
    uint16_t inv = mi->inv_res ? mi->inv_res : 1311;
    int64_t fx = (int64_t)(gx + g->ox) * GM_DS, fy = (int64_t)(gy + g->oy) * GM_DS;
    *x = (int32_t)((fx * 65536 + GM_DS * 32768) / inv);
    *y = (int32_t)((fy * 65536 + GM_DS * 32768) / inv);
}

/* 原地平移：new[y][x] = old[y+sy][x+sx]，越界填未观测 */
static void shift(gmap_t *g, int sx, int sy)
{
    int y0 = sy >= 0 ? 0 : GM_N - 1, y1 = sy >= 0 ? GM_N : -1, ys = sy >= 0 ? 1 : -1;
    int x0 = sx >= 0 ? 0 : GM_N - 1, x1 = sx >= 0 ? GM_N : -1, xs = sx >= 0 ? 1 : -1;
    for (int y = y0; y != y1; y += ys)
        for (int x = x0; x != x1; x += xs) {
            int i = y * GM_N + x, oxx = x + sx, oyy = y + sy;
            if (oxx >= 0 && oxx < GM_N && oyy >= 0 && oyy < GM_N) {
                int j = oyy * GM_N + oxx;
                g->occ[i] = g->occ[j];
                g->cost[i] = g->cost[j];
            } else {
                g->occ[i] = UNSEEN;
                g->cost[i] = 0;
            }
        }
    g->ox += sx;
    g->oy += sy;
    g->n_shift++;
}

void gmap_update(gmap_t *g, const uint8_t *map, const hal_map_info_t *mi, int32_t rx, int32_t ry)
{
    const gmap_cfg_t *c = &g->cfg;
    uint16_t inv = mi->inv_res ? mi->inv_res : 1311;
    int32_t rcx = fdiv(fine_idx(rx, inv), GM_DS), rcy = fdiv(fine_idx(ry, inv), GM_DS);
    if (!g->init) {
        g->ox = rcx - GM_N / 2;
        g->oy = rcy - GM_N / 2;
        g->init = true;
    } else {
        int lx = rcx - g->ox, ly = rcy - g->oy;
        int sx = 0, sy = 0;
        if (lx < GM_N / 4 || lx >= GM_N - GM_N / 4) sx = lx - GM_N / 2;
        if (ly < GM_N / 4 || ly >= GM_N - GM_N / 4) sy = ly - GM_N / 2;
        if (sx || sy) shift(g, sx, sy);
    }
    /* 完整落在局部窗口内的粗格 */
    int32_t f0x = mi->cx - LN / 2, f0y = mi->cy - LN / 2;
    int32_t cgx0 = -fdiv(-f0x, GM_DS), cgy0 = -fdiv(-f0y, GM_DS);       /* 向上取整 */
    int32_t cgx1 = fdiv(f0x + LN, GM_DS), cgy1 = fdiv(f0y + LN, GM_DS);  /* 不含 */
    for (int32_t cgy = cgy0; cgy < cgy1; cgy++) {
        int gy = cgy - g->oy;
        if (gy < 0 || gy >= GM_N) continue;
        for (int32_t cgx = cgx0; cgx < cgx1; cgx++) {
            int gx = cgx - g->ox;
            if (gx < 0 || gx >= GM_N) continue;
            int wx0 = cgx * GM_DS - f0x, wy0 = cgy * GM_DS - f0y;
            int nk = 0, nl = 0, mc = 0;
            for (int j = 0; j < GM_DS; j++)
                for (int i = 0; i < GM_DS; i++) {
                    uint8_t v = map[(wy0 + j) * LN + wx0 + i];
                    if (v == NP_COST_UNKNOWN) continue;
                    nk++;
                    if (v == NP_COST_LETHAL) nl++;
                    else if (v <= NP_COST_FREE_MAX && v > mc) mc = v;
                }
            if (nk < c->min_known) continue;
            int k = gy * GM_N + gx;
            int o = g->occ[k];
            int d = (nl >= 2) ? c->occ_hit : c->occ_miss;
            o = (o == UNSEEN) ? d : o + d;
            if (o > c->occ_max) o = c->occ_max;
            if (o < c->occ_min) o = c->occ_min;
            g->occ[k] = (int8_t)o;
            g->cost[k] = (uint8_t)mc;
        }
    }
    for (int i = 0; i < GM_N * GM_N; i++) {
        int o = g->occ[i];
        g->grid[i] = (o == UNSEEN) ? 255 : (o >= c->occ_thr) ? NP_COST_LETHAL : g->cost[i];
    }
    g->n_upd++;
}
