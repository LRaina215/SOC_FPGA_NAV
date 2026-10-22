/* A* 全局路径（软核运行）。
 * astar_grid：在任意 n×n（n <= ASTAR_MAXN）代价栅格上搜索——导航用全局记忆栅格 gmap（20 cm 格）；
 * astar_plan：在局部代价图的粗栅格上搜索（每 ASTAR_DS×ASTAR_DS 格取最大代价）。
 * 默认64×64的临时缓存约22.5KiB（4bit父方向/CLOSED和距离），与探索BFS复用。
 * 8 邻接；代价 >= ASTAR_BLOCK 不可走；未知格可走但加罚（允许向未探索区规划）。
 */
#ifndef NAV_ASTAR_H
#define NAV_ASTAR_H

#include "nav_types.h"
#include "nav_params.h"

#ifndef ASTAR_DS
#define ASTAR_DS 2
#endif
#define ASTAR_N (NP_MAP_N / ASTAR_DS)
#ifndef ASTAR_MAXN
#define ASTAR_MAXN 64
#endif
#define ASTAR_MAX_PATH 256

typedef struct {
    uint8_t  block;          /* 代价 >= block 不可走（默认 253：膨胀区也不走） */
    uint16_t unknown_cost;   /* 未知格额外代价（每格，单位同 10=直行一格） */
    uint8_t  cost_scale;     /* 可通行代价 0..200 的权重：额外代价 = cost * scale / 64 */
    bool     allow_unknown;
    uint8_t  clear_cells;    /* 离障距离（粗格）小于此值时加罚，使路径远离墙边 */
    uint16_t clear_cost;     /* 每少 1 格距离的额外代价 */
    uint8_t  near_cells;     /* 离障距离小于此值的格按"贴障"处理：额外代价 near_cost（很大，只在别无通路时走，如窄通道中心两侧） */
    uint16_t near_cost;
} astar_cfg_t;

typedef struct {
    int n;                                   /* 路径点数（含起终点） */
    int16_t wx[ASTAR_MAX_PATH], wy[ASTAR_MAX_PATH];   /* astar_plan：代价图窗口坐标；astar_grid：栅格坐标 */
} astar_path_t;

void astar_default_cfg(astar_cfg_t *c);
/* map: 窗口坐标代价图 N×N（hal_map_copy 的输出）。起终点为代价图窗口坐标。
 * 目标不可达时规划到离目标最近的可达点，返回值：0 成功到达目标，1 部分路径，-1 起点被困 */
int astar_plan(const astar_cfg_t *cfg, const uint8_t *map, int sx, int sy, int gx, int gy, astar_path_t *out);
/* 近场原分辨率裁窗，复用现有A*缓存；目标出裁窗返回-2，不把边界当作目标。 */
int astar_window(const astar_cfg_t *cfg, const uint8_t *map, int sx, int sy, int gx, int gy, astar_path_t *out);
/* grid: n×n 栅格代价（255 未知，>= block 不可走），起终点为栅格坐标 */
int astar_grid(const astar_cfg_t *cfg, const uint8_t *grid, int n, int sx, int sy, int gx, int gy, astar_path_t *out);
/* 有界节点展开；limit=0为不限制。预算耗尽只返回实际找到的路径前缀。 */
int astar_grid_bounded(const astar_cfg_t *cfg,const uint8_t *grid,int n,int sx,int sy,int gx,int gy,astar_path_t *out,int limit);
int astar_plan_bounded(const astar_cfg_t *cfg,const uint8_t *map,int sx,int sy,int gx,int gy,astar_path_t *out,int limit);
int astar_window_bounded(const astar_cfg_t *cfg,const uint8_t *map,int sx,int sy,int gx,int gy,astar_path_t *out,int limit);
bool astar_busy(void);
void astar_cancel(void);
int astar_continue(astar_path_t *out,int limit);
/* 下一搜索只在结束时替换out；进行中返回2，保留调用方旧路径。 */
void astar_complete_only(bool enable);
/* 统计：上一次展开的节点数 */
int astar_last_expanded(void);
int astar_heap_max(void);          /* 上电以来堆的最大占用（项） */
int astar_heap_dropped(void);

#endif
