/* 自主探索。
 * 边界探索（默认，explore_frontier）：在全局记忆栅格上，从机器人所在格对"已知可走格"做广度优先搜索，
 *   边界格 = 已知可走且 4 邻域有未知格；代价 = 路程（格）+ W_VISITED·访问热度，取最小者（离机器人 >= min_goal）。
 *   去过但没到达的目标进黑名单（最近 EXP_BL 个），避免反复选同一个够不着的边界。
 * 方向打分（局部图，explore_pick，边界探索找不到目标时退回）：在机器人周围 16 个方向打分，选出下一个探索目标点。
 *   Score = W_CLEAR·可通行距离 + W_UNKNOWN·方向末端未知格数 + W_TRAV·(200-平均代价)
 *           - W_SLOPE·最大代价 - W_VISITED·访问热度
 * 访问热度：里程计系 0.5 m 粗格的计数（环形哈希表，64×64 覆盖 32 m）。
 * 有图搜索（explore_seen_mark + explore_frontier(seen)）：地图已知，"未知"改为"摄像头还没看过"——
 *   按相机视场（缺省 70°、4 m）沿射线标记已看过的格（被先验/观测致命格挡住为止），
 *   在"看过的可走格"与"没看过的可走格"之间找边界，同样按路程 + 访问热度选最近的，走过去让相机看。
 *   全部看过后清空重来（目标可能在移动）。
 * 得到的目标点交给局部规划器。
 */
#ifndef NAV_EXPLORE_H
#define NAV_EXPLORE_H

#include "nav_types.h"
#include "hal.h"
#include "gmap.h"

#define EXP_DIRS 16
#define EXP_VIS_N 64
#define EXP_VIS_CELL_MM 500
#define EXP_BL 8

typedef struct {
    int32_t w_clear, w_unknown, w_trav, w_slope, w_visited;
    int     max_range_cells;       /* 射线长度 */
    uint8_t block;                 /* >= 视为障碍 */
    int32_t min_goal_mm;           /* 目标至少离开这么远 */
    int32_t frontier_min_mm;       /* 边界目标离机器人的最小直线距离（须大于到达容差） */
    int32_t cam_fov;               /* 有图搜索：相机水平视场（angle） */
    int32_t cam_range_mm;          /* 有图搜索：认为能看清目标的距离 */
} explore_cfg_t;

typedef struct {
    uint8_t  visited[EXP_VIS_N * EXP_VIS_N];
    int32_t  score[EXP_DIRS];
    int      clear[EXP_DIRS];
    int      best;
    int32_t  bl_x[EXP_BL], bl_y[EXP_BL];      /* 黑名单（里程计系 mm） */
    int      bl_n, bl_i;
    uint32_t n_frontier;                    /* 上次找到的边界格数（调试） */
    uint8_t  seen[GM_N * GM_N];             /* 有图搜索：记忆栅格坐标下已被相机看过的格 */
    uint32_t n_seen, n_seen_reset;
} explore_t;

void explore_default_cfg(explore_cfg_t *c);
void explore_init(explore_t *e);
/* 每次位姿更新调用，记录访问热度 */
void explore_mark(explore_t *e, const nav_pose_t *pose);
/* map：窗口坐标代价图；返回是否找到目标（目标写入 gx, gy，里程计系 mm） */
bool explore_pick(explore_t *e, const explore_cfg_t *c, const uint8_t *map, const hal_map_info_t *mi,
                  const nav_pose_t *pose, int32_t *gx, int32_t *gy);

/* 边界探索：gm 为全局记忆栅格；seen 非空时为有图搜索（边界 = 看过/没看过的可走格之间）；返回是否找到目标 */
bool explore_frontier(explore_t *e, const explore_cfg_t *c, const gmap_t *gm, const hal_map_info_t *mi,
                      const nav_pose_t *pose, bool seen_mode, int32_t *gx, int32_t *gy);
/* 有图搜索：按当前位姿与相机视场标记看过的格 */
void explore_seen_mark(explore_t *e, const explore_cfg_t *c, const gmap_t *gm, const hal_map_info_t *mi,
                       const nav_pose_t *pose);
void explore_seen_reset(explore_t *e);
/* 把没到达的目标加入黑名单 */
void explore_blacklist(explore_t *e, int32_t x, int32_t y);

#endif
