/* 全局记忆栅格（软核）：把 FPGA 局部代价图（5 cm、6.4 m 窗口、数秒过期）累积成大范围粗栅格，供 A* 使用。
 *
 * 为什么需要：局部图只覆盖 6.4 m 且未观测区会过期，A* 只在局部图上规划时，
 * 长障碍（路沿、墙）的另一端一旦出窗或过期就被"忘掉"，路径在两侧来回翻转，机器人原地打转（闭环仿真实测）。
 *
 * 格子：GM_DS×GM_DS 个局部格合为一格（默认 4 → 20 cm），GM_N×GM_N（默认 96 → 19.2 m），
 * 机器人离边界不足 1/4 边长时整体平移半个边长（滚动）。
 * 每格：occ 证据计数（有致命格 +2，无致命格 -1，饱和）+ 最近一次的可通行代价上限。
 * 局部图里未知的格不改动记忆——看不到的障碍不会被遗忘，只有再次看到"可通行"才会被清除。
 */
#ifndef NAV_GMAP_H
#define NAV_GMAP_H

#include "nav_types.h"
#include "hal.h"

#ifndef GM_N
#define GM_N 96
#endif
#ifndef GM_DS
#define GM_DS 4
#endif

typedef struct {
    int8_t  occ_hit, occ_miss, occ_max, occ_min;   /* 证据增量与饱和 */
    int8_t  occ_thr;                                /* occ >= thr 视为致命 */
    uint8_t min_known;                              /* 一格内至少这么多已知局部格才更新 */
} gmap_cfg_t;

typedef struct {
    gmap_cfg_t cfg;
    int32_t ox, oy;                 /* 栅格 (0,0) 对应的局部格索引 / GM_DS（绝对粗格坐标） */
    bool    init;
    int8_t  occ[GM_N * GM_N];       /* -128 = 未观测 */
    uint8_t cost[GM_N * GM_N];      /* 可通行代价（0..200），A* 用 */
    uint8_t grid[GM_N * GM_N];      /* 合成后的 A* 栅格：255 未知、254 致命、其余代价 */
    uint32_t n_upd, n_shift;
} gmap_t;

void gmap_default_cfg(gmap_cfg_t *c);
void gmap_init(gmap_t *g);
/* map: 局部代价图窗口（hal_map_copy 输出），mi: 其窗口信息；rx/ry: 机器人位置 mm */
void gmap_update(gmap_t *g, const uint8_t *map, const hal_map_info_t *mi, int32_t rx, int32_t ry);
/* 坐标换算：mm <-> 栅格坐标（mi 提供分辨率） */
void gmap_mm_to_cell(const gmap_t *g, const hal_map_info_t *mi, int32_t x, int32_t y, int *gx, int *gy);
void gmap_cell_to_mm(const gmap_t *g, const hal_map_info_t *mi, int gx, int gy, int32_t *x, int32_t *y);

#endif
