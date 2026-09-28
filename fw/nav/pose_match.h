/* 匹配定位：高程区间一致性匹配（Elevation-Interval Consistency Matching, EICM）。
 *
 * 思路：局部高程图每格存了 [zmin, zmax]。位姿正确时，新一帧的点应落在各自栅格的高度区间内；
 * 位姿有偏差时，墙面/箱体侧面/台阶立面上的点会落到"地面格"（区间很窄）里，产生大的高度残差。
 * 于是对一组候选位姿增量逐一统计截断残差和，取最小者——这是在 2.5D 高程图上做的相关匹配。
 *
 * 为什么适合 FPGA：
 *   · 每个 (点, 候选) 只需 一次旋转（查表正余弦 + 4 次乘法）、一次查图、两次比较，没有迭代、没有 KD 树；
 *   · 候选之间完全独立，可按 K 路并行；地图就是已有的高程图存储，不增加存储；
 *   · 帧缓冲本来就是双缓冲，"先匹配、后融合"只需把同一帧读两遍。
 * 与只用里程计相比：航向漂移和打滑造成的位置误差被持续拉回；
 * 同时用地面点的有符号残差估计高度漂移（坡道上 z 积分误差）。
 *
 * 退化处理：沿某一轴代价曲面平坦（如长走廊、空旷平地）时，该轴不修正，只信里程计。
 */
#ifndef NAV_POSE_MATCH_H
#define NAV_POSE_MATCH_H

#include "nav_types.h"
#include "hal.h"
#include "pose.h"

#define PM_MAX_CAND 128

typedef struct {
    bool     enable;
    int32_t  step_mm;          /* 粗搜索平移步长 */
    int32_t  step_ang;         /* 粗搜索角度步长（angle） */
    int      half;             /* 粗搜索半宽（-half..half） */
    int32_t  gain_q8;          /* 修正增益 /256 */
    int32_t  gain_z_q8;
    uint32_t min_valid;        /* 参与匹配的最少有效点数 */
    int32_t  min_curv;         /* 每轴代价曲率下限（低于视为退化，不修正该轴） */
    int32_t  kf_dist_mm;       /* 离上一关键帧超过此距离 -> 新关键帧 */
    int32_t  kf_ang;           /* 或转角超过此值（angle） */
    uint32_t kf_min_valid;     /* 或有效点数低于此值 */
} pm_cfg_t;

typedef struct {
    int32_t  dx, dy, dz, dyaw;       /* 本次采用的修正 */
    uint32_t n_valid, cost0, cost_best;
    bool     applied, degen_x, degen_y, degen_yaw;
    uint32_t n_runs, n_applied, n_kf;
    bool     have_kf;
    int32_t  kf_x, kf_y;
    angle_t  kf_yaw;
} pm_stat_t;

void pm_default_cfg(pm_cfg_t *c);
/* at：帧时刻的预测位姿；r/t：由 at 合成的雷达变换。成功时修正 pe（当前位姿与历史） */
/* 帧融合之后调用：满足条件时让硬件抓取新关键帧 */
void pose_match_after_commit(const pm_cfg_t *c, const nav_pose_t *at, pm_stat_t *st);
bool pose_match_run(const pm_cfg_t *c, pose_est_t *pe, const nav_pose_t *at,
                    const int32_t r_q14[3][3], const int32_t t_mm[3], pm_stat_t *st);

#endif
