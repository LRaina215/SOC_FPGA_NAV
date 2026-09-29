#include "astar.h"
#include "scratch.h"

#define AN   ASTAR_N
#define NC   ASTAR_NC
#define INF  0xFFFFu

/* 大数组在 g_scratch（与探索 BFS 共用，见 scratch.h） */
#define s_cost   (g_scratch.as.cost)
#define s_g      (g_scratch.as.g)
#define s_par    (g_scratch.as.par)
#define s_closed (g_scratch.as.closed)
#define s_dist   (g_scratch.as.dist)
#define s_heap   (g_scratch.as.heap)
static const uint8_t *s_grid;    /* 当前搜索的栅格 */
static int      s_n;             /* 当前栅格边长 */
typedef astar_hnode_t hnode_t;
static int      s_hn, s_expanded, s_hmax;

static const int8_t DX[8] = {1, 1, 0, -1, -1, -1, 0, 1};
static const int8_t DY[8] = {0, 1, 1, 1, 0, -1, -1, -1};
static const uint8_t DC[8] = {10, 14, 10, 14, 10, 14, 10, 14};

void astar_default_cfg(astar_cfg_t *c)
{
    c->block = 253;
    c->unknown_cost = 20;
    c->cost_scale = 16;
    c->allow_unknown = true;
    c->clear_cells = 3;
    c->clear_cost = 12;
    c->near_cells = 0;
    c->near_cost = 0;
}

/* 两遍扫描距离变换（8 邻接，单位粗格） */
static void dist_transform(uint8_t block)
{
    const int AN_ = s_n;
    for (int i = 0; i < AN_ * AN_; i++) {
        uint8_t c = s_grid[i];
        s_dist[i] = (c != 255 && c >= block) ? 0 : 250;
    }
    for (int y = 0; y < AN_; y++)
        for (int x = 0; x < AN_; x++) {
            uint8_t *d = &s_dist[y * AN_ + x];
            if (x > 0 && s_dist[y * AN_ + x - 1] + 1 < *d) *d = (uint8_t)(s_dist[y * AN_ + x - 1] + 1);
            if (y > 0) {
                for (int k = -1; k <= 1; k++) {
                    int xx = x + k;
                    if (xx >= 0 && xx < AN_ && s_dist[(y - 1) * AN_ + xx] + 1 < *d) *d = (uint8_t)(s_dist[(y - 1) * AN_ + xx] + 1);
                }
            }
        }
    for (int y = AN_ - 1; y >= 0; y--)
        for (int x = AN_ - 1; x >= 0; x--) {
            uint8_t *d = &s_dist[y * AN_ + x];
            if (x < AN_ - 1 && s_dist[y * AN_ + x + 1] + 1 < *d) *d = (uint8_t)(s_dist[y * AN_ + x + 1] + 1);
            if (y < AN_ - 1) {
                for (int k = -1; k <= 1; k++) {
                    int xx = x + k;
                    if (xx >= 0 && xx < AN_ && s_dist[(y + 1) * AN_ + xx] + 1 < *d) *d = (uint8_t)(s_dist[(y + 1) * AN_ + xx] + 1);
                }
            }
        }
}

int astar_last_expanded(void) { return s_expanded; }
int astar_heap_max(void) { return s_hmax; }

static void heap_push(uint16_t idx, uint16_t f)
{
    if (s_hn >= (int)(sizeof(s_heap) / sizeof(s_heap[0]))) return;   /* 堆满：丢弃（极端情况下路径次优） */
    int i = s_hn++;
    if (s_hn > s_hmax) s_hmax = s_hn;
    while (i > 0) {
        int p = (i - 1) >> 1;
        if (s_heap[p].f <= f) break;
        s_heap[i] = s_heap[p];
        i = p;
    }
    s_heap[i].idx = idx;
    s_heap[i].f = f;
}

static hnode_t heap_pop(void)
{
    hnode_t top = s_heap[0], last = s_heap[--s_hn];
    int i = 0;
    for (;;) {
        int l = 2 * i + 1, r = l + 1, m = i;
        uint16_t mf = last.f;
        if (l < s_hn && s_heap[l].f < mf) { m = l; mf = s_heap[l].f; }
        if (r < s_hn && s_heap[r].f < mf) { m = r; }
        if (m == i) break;
        s_heap[i] = s_heap[m];
        i = m;
    }
    if (s_hn > 0) s_heap[i] = last;
    return top;
}

static uint16_t heur(int x, int y, int gx, int gy)
{
    int dx = nav_abs32(x - gx), dy = nav_abs32(y - gy);
    int mn = dx < dy ? dx : dy, mx = dx < dy ? dy : dx;
    return (uint16_t)(14 * mn + 10 * (mx - mn));
}

static void downsample(const uint8_t *map)
{
    for (int y = 0; y < AN; y++)
        for (int x = 0; x < AN; x++) {
            uint8_t m = 0;
            int unk = 0, tot = 0;
            for (int j = 0; j < ASTAR_DS; j++)
                for (int i = 0; i < ASTAR_DS; i++) {
                    uint8_t c = map[(y * ASTAR_DS + j) * NP_MAP_N + x * ASTAR_DS + i];
                    tot++;
                    if (c == 255) { unk++; continue; }
                    if (c > m) m = c;
                }
            /* 全部未知才算未知；否则取已知格最大代价 */
            s_cost[y * AN + x] = (unk == tot) ? 255 : m;
        }
}

int astar_grid(const astar_cfg_t *cfg, const uint8_t *grid, int n, int sx, int sy, int gx, int gy, astar_path_t *out)
{
    const int AN_ = n;
    out->n = 0;
    if (n > ASTAR_MAXN) return -1;
    s_grid = grid;
    s_n = n;
    dist_transform(cfg->block);
    sx = nav_clamp32(sx, 0, AN_ - 1); sy = nav_clamp32(sy, 0, AN_ - 1);
    gx = nav_clamp32(gx, 0, AN_ - 1); gy = nav_clamp32(gy, 0, AN_ - 1);

    for (int i = 0; i < AN_ * AN_; i++) { s_g[i] = INF; s_par[i] = 0xFF; }
    for (int i = 0; i < (AN_ * AN_ + 7) / 8; i++) s_closed[i] = 0;
    s_hn = 0;
    s_expanded = 0;

    int start = sy * AN_ + sx, goal = gy * AN_ + gx;
    s_g[start] = 0;
    s_par[start] = 0xFE;
    heap_push((uint16_t)start, heur(sx, sy, gx, gy));
    int best = start;
    uint16_t best_h = heur(sx, sy, gx, gy);

    while (s_hn > 0) {
        hnode_t nd = heap_pop();
        int idx = nd.idx;
        if (s_closed[idx >> 3] & (1 << (idx & 7))) continue;
        s_closed[idx >> 3] |= (uint8_t)(1 << (idx & 7));
        s_expanded++;
        int x = idx % AN_, y = idx / AN_;
        uint16_t h = heur(x, y, gx, gy);
        if (h < best_h) { best_h = h; best = idx; }
        if (idx == goal) break;
        for (int d = 0; d < 8; d++) {
            int nx = x + DX[d], ny = y + DY[d];
            if (nx < 0 || ny < 0 || nx >= AN_ || ny >= AN_) continue;
            int ni = ny * AN_ + nx;
            uint8_t c = grid[ni];
            uint32_t step = DC[d];
            if (c == 255) {
                if (!cfg->allow_unknown) continue;
                step += cfg->unknown_cost;
            } else if (c >= cfg->block) {
                continue;
            } else {
                step += (uint32_t)c * cfg->cost_scale / 64;
            }
            if (s_dist[ni] < cfg->clear_cells) step += (uint32_t)(cfg->clear_cells - s_dist[ni]) * cfg->clear_cost;
            if (s_dist[ni] < cfg->near_cells) step += cfg->near_cost;
            /* 斜穿：两侧直邻都不可走时禁止切角 */
            if (d & 1) {
                uint8_t c1 = grid[y * AN_ + nx], c2 = grid[ny * AN_ + x];
                if (c1 != 255 && c1 >= cfg->block && c2 != 255 && c2 >= cfg->block) continue;
            }
            uint32_t ng = s_g[idx] + step;
            if (ng >= INF - 1) continue;
            if (ng < s_g[ni]) {
                s_g[ni] = (uint16_t)ng;
                s_par[ni] = (uint8_t)d;
                uint32_t f = ng + heur(nx, ny, gx, gy);
                heap_push((uint16_t)ni, (uint16_t)(f > 0xFFFE ? 0xFFFE : f));
            }
        }
    }

    int end = (s_par[goal] != 0xFF) ? goal : best;
    /* 回溯（反向），再翻转 */
    int *tmp = g_scratch.as.path_tmp;
    int np_ = 0, cur = end;
    while (np_ < ASTAR_MAX_PATH) {
        tmp[np_++] = cur;
        uint8_t p = s_par[cur];
        if (p == 0xFE || p == 0xFF) break;
        int x = cur % AN_ - DX[p], y = cur / AN_ - DY[p];
        cur = y * AN_ + x;
    }
    /* 路径过长被截断时保留靠近起点的一段 */
    for (int i = 0; i < np_; i++) {
        int c = tmp[np_ - 1 - i];
        out->wx[i] = (int16_t)(c % AN_);
        out->wy[i] = (int16_t)(c / AN_);
    }
    out->n = np_;
    if (end == goal) return 0;
    return (end == start) ? -1 : 1;
}

int astar_plan(const astar_cfg_t *cfg, const uint8_t *map, int sx, int sy, int gx, int gy, astar_path_t *out)
{
    downsample(map);
    int rc = astar_grid(cfg, s_cost, AN, sx / ASTAR_DS, sy / ASTAR_DS, gx / ASTAR_DS, gy / ASTAR_DS, out);
    for (int i = 0; i < out->n; i++) {
        out->wx[i] = (int16_t)(out->wx[i] * ASTAR_DS + ASTAR_DS / 2);
        out->wy[i] = (int16_t)(out->wy[i] * ASTAR_DS + ASTAR_DS / 2);
    }
    return rc;
}
