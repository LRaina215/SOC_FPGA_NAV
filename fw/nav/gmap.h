/* 全局记忆栅格（软核）：把 FPGA 局部代价图（5 cm、6.4 m 窗口、数秒过期）累积成大范围粗栅格，供 A* 使用。
 *
 * 为什么需要：局部图只覆盖 6.4 m 且未观测区会过期，A* 只在局部图上规划时，
 * 长障碍（路沿、墙）的另一端一旦出窗或过期就被"忘掉"，路径在两侧来回翻转，机器人原地打转（闭环仿真实测）。
 *
 * 格子：GM_DS×GM_DS 个局部格合为一格（默认 4 → 20 cm），GM_N×GM_N（默认 96 → 19.2 m），
 * 机器人离边界不足 1/4 边长时整体平移半个边长（滚动）。
 * 每格：occ 证据计数（有致命格 +2，无致命格 -1，饱和）+ 最近一次的可通行代价上限。
 * 局部图里未知的格不改动记忆——看不到的障碍不会被遗忘，只有再次看到"可通行"才会被清除。
 *
 * 有图模式：先验代价图（离线由整张地图算出，同为 20 cm）作为静态层，栅格原点固定为先验图原点、不再滚动。
 * 合成：没观测过的格用先验；观测过的格用观测（动态障碍）；先验致命而观测为可通行时，
 * 须证据降到 occ_min + 1 以下才采信（地图之后被搬走的物体），避免局部图的偶发漏检把墙"擦掉"。
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
    uint8_t prior[GM_N * GM_N];     /* 先验代价（有图模式；255 = 无） */
    bool    has_prior;
    int32_t pox, poy;               /* 下载中的先验图原点 / 宽 */
    int     pgnx;
    uint32_t n_upd, n_shift;
} gmap_t;

void gmap_default_cfg(gmap_cfg_t *c);
void gmap_init(gmap_t *g);
/* map: 局部代价图窗口（hal_map_copy 输出），mi: 其窗口信息；rx/ry: 机器人位置 mm */
void gmap_update(gmap_t *g, const uint8_t *map, const hal_map_info_t *mi, int32_t rx, int32_t ry);
/* 有图模式：先验层。begin 固定原点并清空；put 按先验图行优先下标写一格；done 生效 */
void gmap_prior_begin(gmap_t *g, int32_t gox, int32_t goy, int gnx, int gny);
void gmap_prior_put(gmap_t *g, int i, uint8_t v);
void gmap_prior_done(gmap_t *g);
void gmap_prior_drop(gmap_t *g);
/* 清掉观测层（位姿跳变到地图系后，旧坐标系下的观测作废），先验层保留 */
void gmap_clear_obs(gmap_t *g);
/* 坐标换算：mm <-> 栅格坐标（mi 提供分辨率） */
void gmap_mm_to_cell(const gmap_t *g, const hal_map_info_t *mi, int32_t x, int32_t y, int *gx, int *gy);
void gmap_cell_to_mm(const gmap_t *g, const hal_map_info_t *mi, int gx, int gy, int32_t *x, int32_t *y);

#endif
