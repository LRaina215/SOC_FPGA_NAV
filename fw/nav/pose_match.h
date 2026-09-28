/* 匹配定位（SLF：结构似然场，硬件打分见 model/slf_match.py 与 docs/algorithms.md 第 6 节）。
 *
 * 每帧流程（先融合、后匹配——帧只需流过一遍）：
 *   1. 按预测位姿提交帧：高程图融合，同时硬件提取竖直结构点存入点缓冲；
 *   2. hal_match_ready() 后，固件生成候选增量 (dx, dy, dyaw)，硬件对缓冲的结构点逐候选打分（hal_match_eval）；
 *   3. 固件取最优：平移用 3×3 邻域二次曲面的 Hessian 特征分解，只沿约束足够的方向做牛顿步（沿墙方向不修正），
 *      角度轴抛物线拟合；按增益修正位姿（含历史）；
 *   4. hal_match_update(修正量)：硬件把缓冲点按修正量变换后并入结构图，重建法向/距离场。
 * 为什么适合 FPGA：每个 (点, 候选) 只需查表旋转 + 一次 2×2 格读 + 4 次乘加，没有迭代、没有 KD 树；候选相互独立。
 */
#ifndef NAV_POSE_MATCH_H
#define NAV_POSE_MATCH_H

#include "nav_types.h"
#include "hal.h"
#include "pose.h"

#define PM_MAX_CAND 256

typedef struct {
    bool     enable;
    int32_t  step_mm;          /* 粗搜索平移步长 */
    int32_t  step_ang;         /* 粗搜索角度步长（angle） */
    int      half;             /* 平移搜索半宽（-half..half） */
    int      half_ang;         /* 角度搜索半宽 */
    int32_t  gain_q8;          /* 修正增益 /256 */
    uint32_t min_valid;        /* 参与匹配的最少有效点数 */
    int32_t  min_curv;         /* 平移 Hessian 特征值下限（每千点，低于视为该方向退化，不修正） */
    int32_t  min_curv_ang;     /* 角度轴曲率下限（每千点） */
} pm_cfg_t;

typedef struct {
    int32_t  dx, dy, dyaw;           /* 本次采用的修正 */
    uint32_t n_valid, cost0, cost_best;
    bool     applied, degen_x, degen_y, degen_yaw;
    uint32_t n_runs, n_applied;
} pm_stat_t;

void pm_default_cfg(pm_cfg_t *c);
/* at：帧时刻的预测位姿（即提交帧时用的位姿）。成功时修正 pe（当前位姿与历史），返回是否修正 */
bool pose_match_run(const pm_cfg_t *c, pose_est_t *pe, const nav_pose_t *at, pm_stat_t *st);

#endif
