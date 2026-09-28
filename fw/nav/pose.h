/* 位姿估计与雷达外参合成。
 * 预测：底盘里程计（vx, vy, wz）三维积分——前进速度按 IMU 俯仰分解为水平位移和高度变化；
 *       IMU 给俯仰/横滚（航向可选）。
 * 修正：pose_match（点云与局部高程图区间一致性匹配）给出 dx/dy/dyaw/dz，经 pose_correct_delta() 施加。
 * 保存最近 1 s 的位姿历史，用于把 RK3588 目标（带采集时刻）换算到里程计系。
 */
#ifndef NAV_POSE_H
#define NAV_POSE_H

#include "nav_types.h"
#include "fixmath.h"

#define POSE_HIST 64
#define POSE_ATT_JUMP (16 * 273)     /* 姿态新息超过 1.5°（angle×16）视为突变 */

typedef struct {
    nav_pose_t cur;
    /* 亚毫米累计，避免积分截断误差 */
    int64_t    x_um, y_um, z_um;
    /* 陀螺零偏：静止（轮速为零）持续 ZUPT_US 后，用角速度读数估计零偏并从此扣除 */
    int32_t    wz_bias_q8;      /* mrad/s × 256 */
    uint32_t   still_us;
    bool       bias_valid;
    uint32_t   bias_n;         /* 已用于零偏估计的样本数：前 256 个取累计平均，之后 1/256 指数平均 */
    int32_t    pitch_f, roll_f;  /* 俯仰/横滚 α-β 滤波状态（angle × 16） */
    int32_t    pitch_r, roll_r;  /* 角速度估计（angle × 16 / 20 ms） */
    uint32_t   att_jump_us;      /* 最近一次姿态突变时刻 */
    bool       att_jumped;
    uint32_t   att_n;          /* 已收姿态帧数 */
    bool       cmd_still;       /* 当前下发的速度指令为零（由导航任务设置）：只有指令为零时才做零偏估计 */
    uint32_t   last_odo_us;
    bool       have_odo;
    bool       imu_att_valid;   /* IMU 航向有效（用 IMU 航向代替陀螺积分） */
    bool       have_att;        /* 收到过姿态 */
    uint32_t   imu_t_us;
    nav_pose_t hist[POSE_HIST];
    int        hist_head, hist_n;
    /* 雷达安装外参：机身系 */
    fx_mat3_t  r_bs;
    int32_t    t_bs[3];
} pose_est_t;

void pose_init(pose_est_t *p, angle_t mount_pitch, int32_t mount_x_mm, int32_t mount_z_mm);
/* 底盘速度回传，积分到当前位姿 */
void pose_odom(pose_est_t *p, int16_t vx, int16_t vy, int16_t wz, uint32_t t_us);
/* IMU 姿态（绝对航向可选）。yaw_valid=false 时只用俯仰横滚 */
void pose_imu_att(pose_est_t *p, angle_t yaw, angle_t pitch, angle_t roll, bool yaw_valid, uint32_t t_us);
/* t_us 之前 win_us 内发生过姿态突变 */
bool pose_att_unsettled(const pose_est_t *p, uint32_t t_us, uint32_t win_us);
/* 外部定位修正（预留） */
void pose_correct(pose_est_t *p, int32_t x, int32_t y, angle_t yaw);
/* 增量修正（匹配定位用），同时修正位姿历史，使随后的历史查询一致 */
void pose_correct_delta(pose_est_t *p, int32_t dx, int32_t dy, int32_t dz, int32_t dyaw);
/* 查询 t_us 时刻的位姿（历史线性插值；超出范围取最近） */
void pose_at(const pose_est_t *p, uint32_t t_us, nav_pose_t *out);
/* 合成 雷达->里程计 变换（寄存器格式） */
void pose_lidar_tf(const pose_est_t *p, const nav_pose_t *at, int32_t r_q14[3][3], int32_t t_mm[3]);
/* 足迹高度锚定：机器人轮下栅格（此前已观测、现处盲区）的地面高度就是机器人应有的高度。
 * 用它把位姿 z 拉向地图，消除里程计高度积分漂移造成的"新旧观测高差"（假台阶）。 */
void pose_anchor_z(pose_est_t *p, int32_t half_len_mm, int32_t half_wid_mm, int32_t gain_q8);
/* 机身系点 -> 里程计系点 */
void pose_body_to_odom(const nav_pose_t *at, int32_t bx, int32_t by, int32_t *ox, int32_t *oy);

#endif
