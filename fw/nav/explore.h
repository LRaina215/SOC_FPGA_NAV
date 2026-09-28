/* 自主探索（无图模式）：在机器人周围 16 个方向打分，选出下一个探索目标点。
 *   Score = W_CLEAR·可通行距离 + W_UNKNOWN·方向末端未知格数 + W_TRAV·(200-平均代价)
 *           - W_SLOPE·最大代价 - W_VISITED·访问热度
 * 访问热度：里程计系 0.5 m 粗格的计数（环形哈希表，64×64 覆盖 32 m）。
 * 得到的目标点交给局部规划器。
 */
#ifndef NAV_EXPLORE_H
#define NAV_EXPLORE_H

#include "nav_types.h"
#include "hal.h"

#define EXP_DIRS 16
#define EXP_VIS_N 64
#define EXP_VIS_CELL_MM 500

typedef struct {
    int32_t w_clear, w_unknown, w_trav, w_slope, w_visited;
    int     max_range_cells;       /* 射线长度 */
    uint8_t block;                 /* >= 视为障碍 */
    int32_t min_goal_mm;           /* 目标至少离开这么远 */
} explore_cfg_t;

typedef struct {
    uint8_t  visited[EXP_VIS_N * EXP_VIS_N];
    int32_t  score[EXP_DIRS];
    int      clear[EXP_DIRS];
    int      best;
} explore_t;

void explore_default_cfg(explore_cfg_t *c);
void explore_init(explore_t *e);
/* 每次位姿更新调用，记录访问热度 */
void explore_mark(explore_t *e, const nav_pose_t *pose);
/* map：窗口坐标代价图；返回是否找到目标（目标写入 gx, gy，里程计系 mm） */
bool explore_pick(explore_t *e, const explore_cfg_t *c, const uint8_t *map, const hal_map_info_t *mi,
                  const nav_pose_t *pose, int32_t *gx, int32_t *gy);

#endif
