/* 匹配定位（SLF：结构似然场，硬件打分见 model/slf_match.py 与 docs/4_design/algorithms.md 第 6 节）。
 *
 * 每帧流程（复用雷达双缓冲重读，不新增点云缓存）：
 *   1. 按预测位姿提取结构点，保留原始深度帧，暂不融合高程；
 *   2. hal_match_ready() 后，固件生成候选增量 (dx, dy, dyaw)，硬件对缓冲的结构点逐候选打分
 *      （hal_match_start 发起，约 40 ms，期间主循环照常运行；hal_match_result 取回）；
 *   3. 固件取最优：平移用 3×3 邻域二次曲面的 Hessian 特征分解，只沿约束足够的方向做牛顿步（沿墙方向不修正），
 *      角度轴抛物线拟合；仅生成提案，由定位状态门控后提交位姿（含历史）；
 *   4. hal_match_update(修正量)：更新结构图；按已接受的采集位姿重读同一帧，仅融合高程。
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
    uint32_t n_in_best;              /* 最优候选的近邻命中点数（定位质量用） */
    bool     at_edge;                /* 最优候选在约束充分方向的搜索边界上（误差可能超出搜索范围） */
    uint32_t n_runs, n_applied;
} pm_stat_t;

void pm_default_cfg(pm_cfg_t *c);
/* 生成候选网格（(2·half+1)² 平移 × (2·half_ang+1) 角度），返回个数；未启用返回 0 */
int pose_match_cands(const pm_cfg_t *c, hal_match_cand_t *cand);
/* 由打分结果求提案，不改变位姿或 n_applied；返回提案是否有效。 */
bool pose_match_solve(const pm_cfg_t *c, const hal_match_res_t *res, int k, pm_stat_t *st);
/* solve只提案；门控接受后才以采集时刻机器人位置为旋转中心提交。 */
void pose_match_commit(pose_est_t *pe, int32_t cx, int32_t cy, pm_stat_t *st);

#endif
