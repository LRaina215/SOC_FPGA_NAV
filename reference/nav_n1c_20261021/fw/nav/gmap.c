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
    c->min_known = (GM_DS * GM_DS * 3 + 7) / 8;      /* 一格内至少 3/8 的局部格已知（4×4 时 6、2×2 时 2） */
}

void gmap_init(gmap_t *g)
{
    gmap_cfg_t c = g->cfg;
    *g = (gmap_t){0};
    g->cfg = c;
    for (int i = 0; i < GM_N * GM_N; i++) { g->occ[i] = UNSEEN; g->cost[i] = 0; g->grid[i] = 255; g->prior[i] = 255; }
}

static void compose(gmap_t *g)
{
    const gmap_cfg_t *c = &g->cfg;
    for (int i = 0; i < GM_N * GM_N; i++) {
        int o = g->occ[i];
        uint8_t obs = (o == UNSEEN) ? 255 : (o >= c->occ_thr) ? NP_COST_LETHAL : g->cost[i];
        uint8_t pr = g->has_prior ? g->prior[i] : 255;
        if (obs == 255) g->grid[i] = pr;
        else if (pr == NP_COST_LETHAL && obs != NP_COST_LETHAL && o > c->occ_min + 1) g->grid[i] = pr;
        else g->grid[i] = obs;
    }
    for (int k = 0; k < GM_VBLK; k++) {
        if (!g->vblk[k].ttl) continue;
        int r = g->vblk[k].r, bx = g->vblk[k].gx - g->ox, by = g->vblk[k].gy - g->oy;
        for (int y = by - r; y <= by + r; y++)
            for (int x = bx - r; x <= bx + r; x++)
                if (x >= 0 && y >= 0 && x < GM_N && y < GM_N && (x - bx) * (x - bx) + (y - by) * (y - by) <= r * r + 1)
                    g->grid[y * GM_N + x] = NP_COST_LETHAL;
    }
}

void gmap_block_add(gmap_t *g, const hal_map_info_t *mi, int32_t x, int32_t y, int r, int ttl)
{
    int gx, gy;
    gmap_mm_to_cell(g, mi, x, y, &gx, &gy);
    int k = 0;
    for (int i = 1; i < GM_VBLK; i++) if (g->vblk[i].ttl < g->vblk[k].ttl) k = i;     /* 替换剩余时间最短的 */
    g->vblk[k].gx = (int16_t)(gx + g->ox); g->vblk[k].gy = (int16_t)(gy + g->oy);      /* 存绝对粗格坐标（栅格平移后仍有效） */
    g->vblk[k].r = (uint8_t)r;
    g->vblk[k].ttl = (uint16_t)ttl;
    compose(g);
    g->n_shift++;                   /* 路径失效，重规划 */
}

void gmap_clear_obs(gmap_t *g)
{
    for (int i = 0; i < GM_N * GM_N; i++) { g->occ[i] = UNSEEN; g->cost[i] = 0; }
    if (!g->has_prior) g->init = false;
    compose(g);
    g->n_shift++;                   /* 已有路径失效 */
}

void gmap_prior_begin(gmap_t *g, int32_t gox, int32_t goy, int gnx, int gny)
{
    (void)gnx; (void)gny;
    for (int i = 0; i < GM_N * GM_N; i++) g->prior[i] = 255;
    g->has_prior = false;
    g->pgnx = gnx;
    g->pox = gox; g->poy = goy;
}

void gmap_prior_put(gmap_t *g, int i, uint8_t v)
{
    int x = i % g->pgnx, y = i / g->pgnx;
    if (x < GM_N && y < GM_N) g->prior[y * GM_N + x] = v;
}

void gmap_prior_done(gmap_t *g)
{
    for (int i = 0; i < GM_N * GM_N; i++) { g->occ[i] = UNSEEN; g->cost[i] = 0; }
    g->ox = g->pox; g->oy = g->poy;
    g->init = true;
    g->has_prior = true;
    g->n_shift++;
    compose(g);
}

void gmap_prior_drop(gmap_t *g)
{
    g->has_prior = false;
    gmap_clear_obs(g);
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
    } else if (!g->has_prior) {
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
            int d = (nl >= (GM_DS * GM_DS >= 16 ? 2 : 1)) ? c->occ_hit : c->occ_miss;     /* 4×4 时 2 格致命、2×2 时 1 格 */
            o = (o == UNSEEN) ? d : o + d;
            if (o > c->occ_max) o = c->occ_max;
            if (o < c->occ_min) o = c->occ_min;
            g->occ[k] = (int8_t)o;
            g->cost[k] = (uint8_t)mc;
        }
    }
    for (int k = 0; k < GM_VBLK; k++) if (g->vblk[k].ttl) g->vblk[k].ttl--;
    compose(g);
    g->n_upd++;
}
