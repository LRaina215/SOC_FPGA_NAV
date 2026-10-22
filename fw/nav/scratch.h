/* 规划用的大块临时内存：A*（astar.c）和边界探索广度优先搜索（explore.c explore_frontier）
 * 只在各自函数调用期间使用、从不嵌套调用，共用同一块（软核 RAM 按 128 KB 装得下的关键之一）。
 * 新增使用者前先确认它不会在 A* 或 BFS 执行中途被调用。 */
#ifndef NAV_SCRATCH_H
#define NAV_SCRATCH_H

#include "astar.h"
#include "gmap.h"

#define ASTAR_NC     (ASTAR_MAXN * ASTAR_MAXN)
/* 历史受测峰值514项；1152项留约2.24倍。满时记录丢弃，不保证搜索完备。 */
#ifndef ASTAR_HEAP_N
#define ASTAR_HEAP_N 1152
#endif

typedef struct { uint16_t idx; uint16_t f; } astar_hnode_t;

typedef union {
    struct {
        uint8_t  cost[ASTAR_NC];              /* 冻结搜索快照；分周期搜索不读变化中的gmap */
        union {
            uint16_t g[ASTAR_NC];
            struct {uint8_t d[ASTAR_NC];uint64_t mask[2*ASTAR_MAXN];} dt;
        } work; /* 距离变换结束后才初始化g，两个阶段复用同一空间 */
        uint8_t  par[(ASTAR_NC + 1) / 2];      /* 方向3bit + CLOSED；未访问由g=INF表示 */
        uint8_t  dist[(ASTAR_NC+1)/2];          /* 切比雪夫距离4bit饱和15，配置<=15 */
        astar_hnode_t heap[ASTAR_HEAP_N];
    } as;
    struct {
        uint16_t bd[GM_N * GM_N];              /* 广度优先距离（格），0xFFFF = 未到达 */
        uint16_t bq[GM_N * GM_N];
    } bfs;
} nav_scratch_t;

extern nav_scratch_t g_scratch;

#endif
