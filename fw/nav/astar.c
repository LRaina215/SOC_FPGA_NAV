#include "astar.h"
#include "scratch.h"

#define AN   ASTAR_N
#define NC   ASTAR_NC
#define INF  0xFFFFu

/* 大数组在 g_scratch（与探索 BFS 共用，见 scratch.h） */
#define s_cost   (g_scratch.as.cost)
#define s_g      (g_scratch.as.work.g)
#define s_par    (g_scratch.as.par)
#define s_dist   (g_scratch.as.dist)
#define s_heap   (g_scratch.as.heap)
static const uint8_t *s_grid;    /* 当前搜索的栅格 */
static int      s_n;             /* 当前栅格边长 */
typedef astar_hnode_t hnode_t;
static int      s_hn, s_expanded, s_hmax;
static int s_dropped;
static astar_cfg_t s_cfg;
static int s_start,s_goal,s_best,s_ox,s_oy,s_scale;
static uint16_t s_best_h;
static bool s_active;
static bool s_complete_only,s_next_complete_only;
void astar_complete_only(bool enable) {s_next_complete_only=enable;}
bool astar_busy(void) {return s_active;}
void astar_cancel(void) {s_active=false;}


static uint8_t parent_get(int i) { return (uint8_t)((s_par[i >> 1] >> ((i & 1) * 4)) & 15); }
static void parent_set(int i, uint8_t v)
{
    int sh = (i & 1) * 4;
    s_par[i >> 1] = (uint8_t)((s_par[i >> 1] & ~(15 << sh)) | (v << sh));
}

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

static uint8_t dist_get(int i) { return (uint8_t)((s_dist[i>>1]>>((i&1)*4))&15); }
/* 临时字节距离及行位图均复用尚未初始化的g存储。
 * L∞距离<净空阈值等价于方形逐层膨胀；每格只在第一次到达时写距离。 */
static void dist_transform(uint8_t block,int limit)
{
    const int n=s_n;
    uint8_t *d=g_scratch.as.work.dt.d;
    uint64_t *src=g_scratch.as.work.dt.mask,*dst=src+n;
    bool any=false;
    for(int y=0;y<n;y++) {
        src[y]=0;
        for(int x=0;x<n;x++) {
            int i=y*n+x;uint8_t c=s_grid[i];bool wall=c!=255 && c>=block;
            d[i]=wall?0:15;
            if(wall) {any=true;if(x<64) src[y]|=(uint64_t)1<<x;}
        }
    }
#if ASTAR_MAXN > 64
    if(n>64) {
        for(int pass=0;pass<2;pass++) {
            int dir=pass?-1:1;
            for(int y=pass?n-1:0;y>=0 && y<n;y+=dir)
                for(int x=pass?n-1:0;x>=0 && x<n;x+=dir) {
                    int k=y*n+x,px=x-dir,py=y-dir;
                    if(px>=0 && px<n && d[y*n+px]+1<d[k]) d[k]=(uint8_t)(d[y*n+px]+1);
                    if(py>=0 && py<n) for(int j=-1;j<=1;j++) {
                        int xx=x+j;
                        if(xx>=0 && xx<n && d[py*n+xx]+1<d[k]) d[k]=(uint8_t)(d[py*n+xx]+1);
                    }
                }
        }
    } else
#endif
    if(any) {
        uint64_t mask=n==64?~(uint64_t)0:((uint64_t)1<<n)-1;
        for(int radius=1;radius<limit;radius++) {
            for(int y=0;y<n;y++) {
                uint64_t v=src[y]|(y?src[y-1]:0)|(y+1<n?src[y+1]:0);
                dst[y]=(v|(v<<1)|(v>>1))&mask;
                uint64_t added=dst[y]&~src[y];
                while(added) {int x=__builtin_ctzll(added);d[y*n+x]=(uint8_t)radius;added&=added-1;}
            }
            uint64_t *tmp=src;src=dst;dst=tmp;
        }
    }
    for(int i=0;i<n*n;i+=2) s_dist[i>>1]=(uint8_t)(d[i]|((i+1<n*n?d[i+1]:15)<<4));
}

int astar_last_expanded(void) { return s_expanded; }
int astar_heap_max(void) { return s_hmax; }
int astar_heap_dropped(void) {return s_dropped;}

static void heap_push(uint16_t idx, uint16_t f)
{
    if (s_hn >= (int)(sizeof(s_heap) / sizeof(s_heap[0]))) {s_dropped++;return;}
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

int astar_grid_bounded(const astar_cfg_t *cfg, const uint8_t *grid, int n, int sx, int sy, int gx, int gy, astar_path_t *out,int limit)
{
    const int AN_ = n;
    s_complete_only=s_next_complete_only;s_next_complete_only=false;
    if(!s_complete_only) out->n=0;
    s_active=false;
    if (n<=0 || n > ASTAR_MAXN) return -1;
    if(cfg->clear_cells>15 || cfg->near_cells>15) {out->n=0;return -1;}
    s_active=false;s_ox=s_oy=0;s_scale=1;
    if(grid!=s_cost) for(int i=0;i<n*n;i++) s_cost[i]=grid[i];
    s_grid=s_cost;
    s_n = n;
    dist_transform(cfg->block,nav_max32(cfg->clear_cells,cfg->near_cells));
    sx = nav_clamp32(sx, 0, AN_ - 1); sy = nav_clamp32(sy, 0, AN_ - 1);
    gx = nav_clamp32(gx, 0, AN_ - 1); gy = nav_clamp32(gy, 0, AN_ - 1);

    for (int i = 0; i < AN_ * AN_; i++) s_g[i] = INF;
    for (int i = 0; i < (AN_ * AN_ + 1) / 2; i++) s_par[i] = 0;
    s_hn = 0;
    s_expanded = 0;

    int start = sy * AN_ + sx, goal = gy * AN_ + gx;
    s_g[start] = 0;
    heap_push((uint16_t)start, heur(sx, sy, gx, gy));
    s_cfg=*cfg;s_start=start;s_goal=goal;s_best=start;
    s_best_h=heur(sx,sy,gx,gy);s_active=true;
    return astar_continue(out,limit);
}

int astar_continue(astar_path_t *out,int limit)
{
    if(!s_active) {out->n=0;return -1;}
    const astar_cfg_t *cfg=&s_cfg;
    const uint8_t *grid=s_grid;
    int AN_=s_n,start=s_start,goal=s_goal,best=s_best;
    int gx=goal%AN_,gy=goal/AN_,work=0;
    uint16_t best_h=s_best_h;
    bool reached=false;
    while (s_hn > 0 && (limit<=0 || work<limit)) {
        work++;
        hnode_t nd = heap_pop();
        int idx = nd.idx;
        if (parent_get(idx) & 8) continue;
        parent_set(idx, (uint8_t)(parent_get(idx) | 8));
        s_expanded++;
        int x = idx % AN_, y = idx / AN_;
        uint16_t h = heur(x, y, gx, gy);
        if (h < best_h) { best_h = h; best = idx; }
        if (idx == goal) {reached=true;break;}
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
            uint8_t distance=dist_get(ni);
            if (distance < cfg->clear_cells) step += (uint32_t)(cfg->clear_cells - distance) * cfg->clear_cost;
            if (distance < cfg->near_cells) step += cfg->near_cost;
            /* 斜穿：两侧直邻都不可走时禁止切角 */
            if (d & 1) {
                uint8_t c1 = grid[y * AN_ + nx], c2 = grid[ny * AN_ + x];
                if (c1 != 255 && c1 >= cfg->block && c2 != 255 && c2 >= cfg->block) continue;
            }
            uint32_t ng = s_g[idx] + step;
            if (ng >= INF - 1) continue;
            if (ng < s_g[ni]) {
                s_g[ni] = (uint16_t)ng;
                parent_set(ni, (uint8_t)((parent_get(ni) & 8) | d));
                uint32_t f = ng + heur(nx, ny, gx, gy);
                heap_push((uint16_t)ni, (uint16_t)(f > 0xFFFE ? 0xFFFE : f));
            }
        }
    }

    s_best=best;s_best_h=best_h;
    int end=(reached || !s_hn) && s_g[goal]!=INF?goal:best;
    s_active=s_hn>0 && !reached;
    if(s_complete_only && s_active) return 2;
    /* 发布时才借用输出x数组作反向格索引环；不占额外512B。
     * 原子发布等待期间已在上方返回，不会破坏仍在执行的旧路径。 */
    uint16_t *tmp = (uint16_t *)out->wx;
    int np_ = 0, cur = end;
    /* 环形存最近的256个反向点：长路径仍保留靠起点的前缀，不能丢失起点却报告完整。 */
    do {
        tmp[np_++ % ASTAR_MAX_PATH] = cur;
        if (cur == start || s_g[cur] == INF) break;
        uint8_t p = parent_get(cur) & 7;
        int x = cur % AN_ - DX[p], y = cur / AN_ - DY[p];
        cur = y * AN_ + x;
    } while (np_ < AN_ * AN_);
    /* 路径过长被截断时保留靠近起点的一段 */
    int kept = nav_min32(np_, ASTAR_MAX_PATH);
    /* 所需次序是tmp[(np-1-i)%256]：分别反转环的两段即可原地完成。 */
    int cut=np_<ASTAR_MAX_PATH?0:np_%ASTAR_MAX_PATH;
    for(int part=0;part<2;part++) {
        int l=part?cut:0,r=part?kept-1:cut-1;
        while(l<r) {uint16_t t=tmp[l];tmp[l++]=tmp[r];tmp[r--]=t;}
    }
    for (int i = 0; i < kept; i++) {
        int c = tmp[i];
        out->wx[i] = (int16_t)((c % AN_)*s_scale+s_ox);
        out->wy[i] = (int16_t)((c / AN_)*s_scale+s_oy);
    }
    out->n = kept;
    if (end == goal && np_ <= ASTAR_MAX_PATH) return 0;
    return (end == start) ? -1 : 1;
}

int astar_grid(const astar_cfg_t *cfg,const uint8_t *grid,int n,int sx,int sy,int gx,int gy,astar_path_t *out)
{return astar_grid_bounded(cfg,grid,n,sx,sy,gx,gy,out,0);}

int astar_plan_bounded(const astar_cfg_t *cfg, const uint8_t *map, int sx, int sy, int gx, int gy, astar_path_t *out,int limit)
{
    downsample(map);
    int rc = astar_grid_bounded(cfg, s_cost, AN, sx / ASTAR_DS, sy / ASTAR_DS, gx / ASTAR_DS, gy / ASTAR_DS, out,limit);
    s_scale=ASTAR_DS;s_ox=s_oy=ASTAR_DS/2;
    for (int i = 0; rc!=2 && i < out->n; i++) {
        out->wx[i] = (int16_t)(out->wx[i] * ASTAR_DS + ASTAR_DS / 2);
        out->wy[i] = (int16_t)(out->wy[i] * ASTAR_DS + ASTAR_DS / 2);
    }
    return rc;
}

int astar_plan(const astar_cfg_t *cfg,const uint8_t *map,int sx,int sy,int gx,int gy,astar_path_t *out)
{return astar_plan_bounded(cfg,map,sx,sy,gx,gy,out,0);}

int astar_window_bounded(const astar_cfg_t *cfg, const uint8_t *map, int sx, int sy, int gx, int gy, astar_path_t *out,int limit)
{
    astar_cancel();
    if(!s_next_complete_only) out->n=0;
    if (sx < 0 || sy < 0 || sx >= NP_MAP_N || sy >= NP_MAP_N) {out->n=0;s_next_complete_only=false;return -2;}
    int ox = nav_clamp32(sx - AN / 2, 0, NP_MAP_N - AN);
    int oy = nav_clamp32(sy - AN / 2, 0, NP_MAP_N - AN);
    if (gx < ox || gy < oy || gx >= ox + AN || gy >= oy + AN) {out->n=0;s_next_complete_only=false;return -2;}
    for (int y = 0; y < AN; y++) for (int x = 0; x < AN; x++)
        s_cost[y * AN + x] = map[(y + oy) * NP_MAP_N + x + ox];
    int rc = astar_grid_bounded(cfg, s_cost, AN, sx - ox, sy - oy, gx - ox, gy - oy, out,limit);
    s_ox=ox;s_oy=oy;
    for (int i = 0; rc!=2 && i < out->n; i++) { out->wx[i] += (int16_t)ox; out->wy[i] += (int16_t)oy; }
    return rc;
}

int astar_window(const astar_cfg_t *cfg,const uint8_t *map,int sx,int sy,int gx,int gy,astar_path_t *out)
{return astar_window_bounded(cfg,map,sx,sy,gx,gy,out,0);}
