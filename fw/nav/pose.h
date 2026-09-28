/* 位姿估计与雷达外参合成。
 * 当前实现：底盘里程计（vx, vy, wz）积分 + IMU 姿态（若有）覆盖航向/俯仰/横滚。
 * 预留：pose_correct() 供以后接入先验地图匹配定位（PDF 架构中的"定位加速器"）。
 * 保存最近 1 s 的位姿历史，用于把 RK3588 目标（带采集时刻）换算到里程计系。
 */
#ifndef NAV_POSE_H
#define NAV_POSE_H

#include "nav_types.h"
#include "fixmath.h"

#define POSE_HIST 64

typedef struct {
    nav_pose_t cur;
    /* 亚毫米累计，避免积分截断误差 */
    int64_t    x_um, y_um;
    uint32_t   last_odo_us;
    bool       have_odo;
    bool       imu_att_valid;
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
/* 外部定位修正（预留） */
void pose_correct(pose_est_t *p, int32_t x, int32_t y, angle_t yaw);
/* 查询 t_us 时刻的位姿（历史线性插值；超出范围取最近） */
void pose_at(const pose_est_t *p, uint32_t t_us, nav_pose_t *out);
/* 合成 雷达->里程计 变换（寄存器格式） */
void pose_lidar_tf(const pose_est_t *p, const nav_pose_t *at, int32_t r_q14[3][3], int32_t t_mm[3]);
/* 机身系点 -> 里程计系点 */
void pose_body_to_odom(const nav_pose_t *at, int32_t bx, int32_t by, int32_t *ox, int32_t *oy);

#endif
