/* 局部规划器：生成候选速度 -> 硬件 traj_eval 逐条复用数据通路前推查代价 -> 软件按目标打分选最优。
 * 打分（越大越好）：
 *   + W_PROGRESS × 终点向目标靠近的距离
 *   - W_HEADING  × 终点航向与"指向目标方向"的偏差
 *   - W_COST     × 平均代价
 *   - W_UNKNOWN  × 未知步数
 *   碰撞：碰撞步数早于 SAFE_STEPS 的候选淘汰；否则按碰撞前距离折减
 * 底盘不能横移时只生成 (vx, wz)；能横移时额外生成带 vy 的候选。
 */
#ifndef NAV_LOCAL_PLANNER_H
#define NAV_LOCAL_PLANNER_H

#include "nav_types.h"
#include "hal.h"

#define LP_MAX_CAND 64

typedef struct {
    nav_chassis_caps_t caps;
    int16_t  v_min, v_max;          /* 前进速度候选范围 mm/s（v_min 可为负表示允许倒车） */
    int16_t  w_max;                 /* 角速度候选 mrad/s */
    uint8_t  nv, nw;                /* 速度 × 角速度 网格数 */
    uint8_t  steps;                 /* 前推步数 T */
    uint16_t dt_ms;                 /* 每步时长 */
    uint8_t  safe_steps;            /* 碰撞早于此步数则淘汰 */
    int32_t  w_progress, w_heading, w_cost, w_unknown;
    int32_t  goal_tol_mm;           /* 物理位置要求/近目标航向评分区；最终估计容差另扣定位预算 */
    int16_t  v_back;                /* 倒车脱困候选速度 mm/s（0 = 不倒车） */
    int32_t  w_back;                /* 倒车候选额外惩罚 */
    int32_t  w_hyst;                /* 与上一周期转向同号的奖励（抑制左右摆） */
} lp_cfg_t;

typedef struct {
    int      n_cand, n_valid;
    int      best;
    int32_t  best_score;
    nav_cmd_t cmd;
    bool     blocked;               /* 所有候选都被淘汰 */
    bool     escape;                /* 本次结果来自脱困遍（机器人在膨胀区内） */
    bool     map_stale;             /* 硬件评估失败或地图与调用方缓存不一致 */
    bool     ground_checked;        /* 同次同步调用内的最终cmd已通过轮支撑查询 */
    uint8_t  turn_phase;            /* bit0全失败后交替转向；bit1未平移时交替25/50微动 */
    hal_traj_res_t res[LP_MAX_CAND];
    int16_t  cv[LP_MAX_CAND], cvy[LP_MAX_CAND], cw[LP_MAX_CAND];
    int16_t  prev_wz;               /* 上一周期输出（调用方保留） */
} lp_result_t;

void lp_default_cfg(lp_cfg_t *c);
/* 目标点（里程计系 mm）。机器人位姿 pose。返回是否找到可行候选 */
bool lp_plan(const lp_cfg_t *c, const hal_map_info_t *mi, const nav_pose_t *pose,
             const uint8_t *map, bool slow, bool allow_unknown,
             int32_t goal_x, int32_t goal_y, lp_result_t *r);

#endif
