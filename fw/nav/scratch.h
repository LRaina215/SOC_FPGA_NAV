/* 规划用的大块临时内存：A*（astar.c）和边界探索广度优先搜索（explore.c explore_frontier）
 * 只在各自函数调用期间使用、从不嵌套调用，共用同一块（软核 RAM 按 128 KB 装得下的关键之一）。
 * 新增使用者前先确认它不会在 A* 或 BFS 执行中途被调用。 */
#ifndef NAV_SCRATCH_H
#define NAV_SCRATCH_H

#include "astar.h"
#include "gmap.h"

#define ASTAR_NC     (ASTAR_MAXN * ASTAR_MAXN)
/* 堆容量：5 m 场地 50 次闭环实测最多 468 项，留 3 倍余量；堆满时丢弃新项（路径可能次优，不会失败） */
#ifndef ASTAR_HEAP_N
#define ASTAR_HEAP_N 1536
#endif

typedef struct { uint16_t idx; uint16_t f; } astar_hnode_t;

typedef union {
    struct {
        uint8_t  cost[ASTAR_N * ASTAR_N];      /* 窗口粗栅格代价（astar_plan） */
        uint16_t g[ASTAR_NC];
        uint8_t  par[ASTAR_NC];                /* 0..7 = 来自的方向，0xFF = 未访问，0xFE = 起点 */
        uint8_t  closed[ASTAR_NC / 8];
        uint8_t  dist[ASTAR_NC];               /* 到最近不可走粗格的距离（切比雪夫，饱和） */
        astar_hnode_t heap[ASTAR_HEAP_N];
        int      path_tmp[ASTAR_MAX_PATH];     /* 回溯路径时的倒序缓冲 */
    } as;
    struct {
        uint16_t bd[GM_N * GM_N];              /* 广度优先距离（格），0xFFFF = 未到达 */
        uint16_t bq[GM_N * GM_N];
    } bfs;
} nav_scratch_t;

extern nav_scratch_t g_scratch;

#endif
