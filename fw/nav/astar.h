/* A* 全局路径（软核运行）。
 * astar_grid：在任意 n×n（n <= ASTAR_MAXN）代价栅格上搜索——导航用全局记忆栅格 gmap（20 cm 格）；
 * astar_plan：在局部代价图的粗栅格上搜索（每 ASTAR_DS×ASTAR_DS 格取最大代价）。
 * 内存（静态，按 ASTAR_MAXN=96）：g 值 18 KB + 父方向 9 KB + 距离 9 KB + 二叉堆 27 KB ≈ 65 KB。
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
#define ASTAR_MAXN 96
#endif
#define ASTAR_MAX_PATH 256

typedef struct {
    uint8_t  block;          /* 代价 >= block 不可走（默认 253：膨胀区也不走） */
    uint16_t unknown_cost;   /* 未知格额外代价（每格，单位同 10=直行一格） */
    uint8_t  cost_scale;     /* 可通行代价 0..200 的权重：额外代价 = cost * scale / 64 */
    bool     allow_unknown;
    uint8_t  clear_cells;    /* 离障距离（粗格）小于此值时加罚，使路径远离墙边 */
    uint16_t clear_cost;     /* 每少 1 格距离的额外代价 */
} astar_cfg_t;

typedef struct {
    int n;                                   /* 路径点数（含起终点） */
    int16_t wx[ASTAR_MAX_PATH], wy[ASTAR_MAX_PATH];   /* astar_plan：代价图窗口坐标；astar_grid：栅格坐标 */
} astar_path_t;

void astar_default_cfg(astar_cfg_t *c);
/* map: 窗口坐标代价图 N×N（hal_map_copy 的输出）。起终点为代价图窗口坐标。
 * 目标不可达时规划到离目标最近的可达点，返回值：0 成功到达目标，1 部分路径，-1 起点被困 */
int astar_plan(const astar_cfg_t *cfg, const uint8_t *map, int sx, int sy, int gx, int gy, astar_path_t *out);
/* grid: n×n 栅格代价（255 未知，>= block 不可走），起终点为栅格坐标 */
int astar_grid(const astar_cfg_t *cfg, const uint8_t *grid, int n, int sx, int sy, int gx, int gy, astar_path_t *out);
/* 统计：上一次展开的节点数 */
int astar_last_expanded(void);

#endif
