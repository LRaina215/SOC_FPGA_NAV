#include "pose.h"
#include "hal.h"

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
    /* 零速更新：轮速为零时角速度读数就是零偏（EMA，时间常数约 1 s） */
    /* 轮子不转且角速度读数很小（< 3°/s，远大于零偏、远小于任何主动转向）才视为静止 */
    if (p->cmd_still && vx == 0 && vy == 0 && wz > -50 && wz < 50) {
        p->still_us += (uint32_t)dt;
        if (p->still_us > 300000) {
            int32_t w8 = (int32_t)wz * 256;
            if (p->bias_n < 256) p->bias_n++;
            /* 累计平均：第 n 个样本权重 1/n；满 256 后变为 1/256 的慢速跟踪（温漂） */
            p->wz_bias_q8 += (w8 - p->wz_bias_q8) / (int32_t)p->bias_n;
            p->bias_valid = true;
        }
    } else {
        p->still_us = 0;
    }
    int32_t wzc = (int32_t)wz - (p->wz_bias_q8 + (p->wz_bias_q8 >= 0 ? 128 : -128)) / 256;
    if (p->still_us > 300000) wzc = 0;            /* 静止时不积分航向 */
    /* 航向：没有 IMU 航向时用 wz 积分（mrad/s -> angle：65536 / (2pi*1000)） */
    if (!p->imu_att_valid || (uint32_t)(t_us - p->imu_t_us) > 100000) {
        int64_t dth = (int64_t)wzc * dt * 65536 / 6283185 / 1000;
        p->cur.yaw = (angle_t)(p->cur.yaw + (int32_t)dth);
    }
    /* 机身速度按俯仰分解：水平分量 vx·cos(pitch)，高度 -vx·sin(pitch)（pitch>0 为低头），um 精度累加 */
    int64_t cp = fx_cos(p->cur.pitch), sp = fx_sin(p->cur.pitch);
    int64_t vh = (int64_t)vx * cp / Q14;
    int64_t c = fx_cos(p->cur.yaw), s = fx_sin(p->cur.yaw);
    int64_t dx_um = (vh * c - (int64_t)vy * s) * dt / Q14 / 1000;
    int64_t dy_um = (vh * s + (int64_t)vy * c) * dt / Q14 / 1000;
    int64_t dz_um = -(int64_t)vx * sp * dt / Q14 / 1000;
    p->x_um += dx_um;
    p->y_um += dy_um;
    p->z_um += dz_um;
    p->cur.x = (int32_t)(p->x_um / 1000);
    p->cur.y = (int32_t)(p->y_um / 1000);
    p->cur.z = (int32_t)(p->z_um / 1000);
    p->cur.t_us = t_us;
    hist_push(p);
}

void pose_imu_att(pose_est_t *p, angle_t yaw, angle_t pitch, angle_t roll, bool yaw_valid, uint32_t t_us)
{
    if (yaw_valid) p->cur.yaw = yaw;
    /* 俯仰/横滚 α-β 滤波（α=1/4，β=1/32，按 50 Hz 采样）：姿态噪声会让整帧点云整体倾斜，远处地面高度逐帧跳动，
     * 在高程图里形成假台阶；但一阶低通有滞后——原地转向时车体姿态以 10–20°/s 变化，0.16 s 的滞后就是 2–3°，
     * 4 m 外的地面高差 15–20 cm（闭环仿真中理想传感器下误报率也到 12%）。α-β 同时估计角速度，匀速变化时无稳态滞后。 */
    int32_t pi = (int32_t)(int16_t)pitch * 16, ro = (int32_t)(int16_t)roll * 16;
    int32_t dt = (int32_t)(t_us - p->imu_t_us);
    if (p->att_n == 0 || dt <= 0 || dt > 200000) {
        p->pitch_f = pi; p->roll_f = ro; p->pitch_r = 0; p->roll_r = 0;
    } else {
        int32_t pp = p->pitch_f + (int32_t)((int64_t)p->pitch_r * dt / 20000);
        int32_t pr = p->roll_f + (int32_t)((int64_t)p->roll_r * dt / 20000);
        int32_t ep = pi - pp, er = ro - pr;
        if (nav_abs32(ep) > POSE_ATT_JUMP || nav_abs32(er) > POSE_ATT_JUMP) {
            /* 姿态突变（单侧轮/足登上台阶）：远超噪声，直接采用测量值，并记下时刻——
             * 突变前后的雷达帧姿态不可靠，由上层决定是否融合 */
            p->pitch_f = pi; p->roll_f = ro; p->pitch_r = 0; p->roll_r = 0;
            p->att_jump_us = t_us;
            p->att_jumped = true;
        } else {
            p->pitch_f = pp + ep / 4; p->roll_f = pr + er / 4;
            p->pitch_r += ep / 32; p->roll_r += er / 32;
        }
    }
    p->att_n++;
    p->cur.pitch = (angle_t)(int16_t)(p->pitch_f / 16);
    p->cur.roll = (angle_t)(int16_t)(p->roll_f / 16);
    p->imu_att_valid = yaw_valid;
    p->imu_t_us = t_us;
    p->have_att = true;
}

bool pose_att_unsettled(const pose_est_t *p, uint32_t t_us, uint32_t win_us)
{
    return p->att_jumped && (uint32_t)(t_us - p->att_jump_us) < win_us;
}

void pose_correct(pose_est_t *p, int32_t x, int32_t y, angle_t yaw)
{
    p->cur.x = x;
    p->cur.y = y;
    p->cur.yaw = yaw;
    p->x_um = (int64_t)x * 1000;
    p->y_um = (int64_t)y * 1000;
}


void pose_set(pose_est_t *p, int32_t x, int32_t y, angle_t yaw)
{
    int32_t da = fx_angle_diff(p->cur.yaw, yaw);
    int64_t c = fx_cos((angle_t)da), s = fx_sin((angle_t)da);
    int32_t cx = p->cur.x, cy = p->cur.y;
    for (int i = 0; i < p->hist_n; i++) {
        int idx = (p->hist_head - 1 - i + 2 * POSE_HIST) % POSE_HIST;
        nav_pose_t *h = &p->hist[idx];
        int64_t rx = h->x - cx, ry = h->y - cy;
        h->x = x + (int32_t)((rx * c - ry * s) / Q14);
        h->y = y + (int32_t)((rx * s + ry * c) / Q14);
        h->yaw = (angle_t)(h->yaw + da);
    }
    p->cur.x = x; p->cur.y = y; p->cur.yaw = yaw;
    p->x_um = (int64_t)x * 1000;
    p->y_um = (int64_t)y * 1000;
}

void pose_correct_delta(pose_est_t *p, int32_t dx, int32_t dy, int32_t dz, int32_t dyaw)
{
    p->x_um += (int64_t)dx * 1000;
    p->y_um += (int64_t)dy * 1000;
    p->z_um += (int64_t)dz * 1000;
    p->cur.x = (int32_t)(p->x_um / 1000);
    p->cur.y = (int32_t)(p->y_um / 1000);
    p->cur.z = (int32_t)(p->z_um / 1000);
    p->cur.yaw = (angle_t)(p->cur.yaw + dyaw);
    for (int i = 0; i < p->hist_n; i++) {
        int idx = (p->hist_head - 1 - i + 2 * POSE_HIST) % POSE_HIST;
        p->hist[idx].x += dx; p->hist[idx].y += dy; p->hist[idx].z += dz;
        p->hist[idx].yaw = (angle_t)(p->hist[idx].yaw + dyaw);
    }
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
            out->z = h->z + (int32_t)((int64_t)(newer.z - h->z) * k / 1024);
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

void pose_anchor_z(pose_est_t *p, int32_t hl, int32_t hw, int32_t gain_q8)
{
    static const int8_t SX[4] = {1, 1, -1, -1}, SY[4] = {1, -1, 1, -1};
    int32_t sum = 0, n = 0;
    for (int i = 0; i < 4; i++) {
        int32_t ox, oy, h;
        pose_body_to_odom(&p->cur, SX[i] * hl, SY[i] * hw, &ox, &oy);
        if (hal_map_height(ox, oy, &h)) { sum += h; n++; }
    }
    if (n < 3) return;
    int32_t e = sum / n - p->cur.z;                 /* 地图说机器人应在的高度 - 估计高度 */
    if (e > 150 || e < -150) return;                /* 离谱的差值：多半是轮下格被障碍占据，不用 */
    pose_correct_delta(p, 0, 0, e * gain_q8 / 256, 0);
}

void pose_body_to_odom(const nav_pose_t *at, int32_t bx, int32_t by, int32_t *ox, int32_t *oy)
{
    int64_t c = fx_cos(at->yaw), s = fx_sin(at->yaw);
    *ox = at->x + (int32_t)(((int64_t)bx * c - (int64_t)by * s) / Q14);
    *oy = at->y + (int32_t)(((int64_t)bx * s + (int64_t)by * c) / Q14);
}
