#include "pose.h"

static void hist_push(pose_est_t *p)
{
    p->hist[p->hist_head] = p->cur;
    p->hist_head = (p->hist_head + 1) % POSE_HIST;
    if (p->hist_n < POSE_HIST) p->hist_n++;
}

void pose_init(pose_est_t *p, angle_t mount_pitch, int32_t mount_x_mm, int32_t mount_z_mm)
{
    *p = (pose_est_t){0};
    fx_rot_zyx(&p->r_bs, 0, mount_pitch, 0);
    p->t_bs[0] = mount_x_mm;
    p->t_bs[1] = 0;
    p->t_bs[2] = mount_z_mm;
    hist_push(p);
}

void pose_odom(pose_est_t *p, int16_t vx, int16_t vy, int16_t wz, uint32_t t_us)
{
    if (!p->have_odo) {
        p->have_odo = true;
        p->last_odo_us = t_us;
        p->cur.t_us = t_us;
        return;
    }
    int32_t dt = (int32_t)(t_us - p->last_odo_us);
    p->last_odo_us = t_us;
    if (dt <= 0 || dt > 200000) {           /* 回传中断过久：不积分这一段 */
        p->cur.t_us = t_us;
        hist_push(p);
        return;
    }
    /* 航向：没有 IMU 航向时用 wz 积分（mrad/s -> angle：65536 / (2pi*1000)） */
    if (!p->imu_att_valid || (uint32_t)(t_us - p->imu_t_us) > 100000) {
        int64_t dth = (int64_t)wz * dt * 65536 / 6283185 / 1000;
        p->cur.yaw = (angle_t)(p->cur.yaw + (int32_t)dth);
    }
    /* 机身速度转到里程计系，um 精度累加 */
    int64_t c = fx_cos(p->cur.yaw), s = fx_sin(p->cur.yaw);
    int64_t dx_um = ((int64_t)vx * c - (int64_t)vy * s) * dt / Q14 / 1000;
    int64_t dy_um = ((int64_t)vx * s + (int64_t)vy * c) * dt / Q14 / 1000;
    p->x_um += dx_um;
    p->y_um += dy_um;
    p->cur.x = (int32_t)(p->x_um / 1000);
    p->cur.y = (int32_t)(p->y_um / 1000);
    p->cur.t_us = t_us;
    hist_push(p);
}

void pose_imu_att(pose_est_t *p, angle_t yaw, angle_t pitch, angle_t roll, bool yaw_valid, uint32_t t_us)
{
    if (yaw_valid) p->cur.yaw = yaw;
    p->cur.pitch = pitch;
    p->cur.roll = roll;
    p->imu_att_valid = yaw_valid;
    p->imu_t_us = t_us;
}

void pose_correct(pose_est_t *p, int32_t x, int32_t y, angle_t yaw)
{
    p->cur.x = x;
    p->cur.y = y;
    p->cur.yaw = yaw;
    p->x_um = (int64_t)x * 1000;
    p->y_um = (int64_t)y * 1000;
}

void pose_at(const pose_est_t *p, uint32_t t_us, nav_pose_t *out)
{
    if (p->hist_n == 0) { *out = p->cur; return; }
    /* 从新到旧找第一个时间 <= t_us 的样本 */
    nav_pose_t newer = p->cur;
    for (int i = 0; i < p->hist_n; i++) {
        int idx = (p->hist_head - 1 - i + 2 * POSE_HIST) % POSE_HIST;
        const nav_pose_t *h = &p->hist[idx];
        if ((int32_t)(t_us - h->t_us) >= 0) {
            int32_t span = (int32_t)(newer.t_us - h->t_us);
            int32_t k = (span > 0) ? (int32_t)((int64_t)(t_us - h->t_us) * 1024 / span) : 0;
            if (k > 1024) k = 1024;
            *out = *h;
            out->x = h->x + (int32_t)((int64_t)(newer.x - h->x) * k / 1024);
            out->y = h->y + (int32_t)((int64_t)(newer.y - h->y) * k / 1024);
            out->yaw = (angle_t)(h->yaw + fx_angle_diff(h->yaw, newer.yaw) * k / 1024);
            out->t_us = t_us;
            return;
        }
        newer = *h;
    }
    *out = newer;       /* 比历史还早：取最旧 */
}

void pose_lidar_tf(const pose_est_t *p, const nav_pose_t *at, int32_t r_q14[3][3], int32_t t_mm[3])
{
    fx_mat3_t r_ob, r_os;
    fx_rot_zyx(&r_ob, at->yaw, at->pitch, at->roll);
    fx_mat_mul(&r_os, &r_ob, &p->r_bs);
    int32_t tb[3];
    fx_mat_vec(tb, &r_ob, p->t_bs);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) r_q14[i][j] = r_os.m[i][j];
    t_mm[0] = at->x + tb[0];
    t_mm[1] = at->y + tb[1];
    t_mm[2] = at->z + tb[2];
}

void pose_body_to_odom(const nav_pose_t *at, int32_t bx, int32_t by, int32_t *ox, int32_t *oy)
{
    int64_t c = fx_cos(at->yaw), s = fx_sin(at->yaw);
    *ox = at->x + (int32_t)(((int64_t)bx * c - (int64_t)by * s) / Q14);
    *oy = at->y + (int32_t)(((int64_t)bx * s + (int64_t)by * c) / Q14);
}
