#include "nav_app.h"
#include "hal.h"

nav_app_t g_app;

void nav_app_init(void)
{
    nav_task_init(&g_app.nt);
    imu_parser_init(&g_app.imu, IMU_PROTO_WIT55);
    uint32_t now = hal_time_us();
    g_app.t_plan = now;
    g_app.t_status = now;
    g_app.use_imu_yaw = false;
    pm_default_cfg(&g_app.pm_cfg);
    g_app.pm_cfg.enable = false;            /* 匹配定位：实验功能，评测见 docs/algorithms.md，默认关闭 */
    g_app.zband_lo = -600;
    g_app.zband_hi = 1200;
    g_app.anchor_gain = 64;
}

void nav_app_poll(void)
{
    nav_task_t *nt = &g_app.nt;
    uint32_t now = hal_time_us();

    /* 1. 底盘里程计 */
    int16_t vx, vy, wz;
    uint32_t t;
    while (hal_odom(&vx, &vy, &wz, &t)) pose_odom(&nt->pose, vx, vy, wz, t);

    /* 2. IMU 姿态 */
    uint8_t b;
    bool first;
    uint32_t tb = 0;
    for (int i = 0; i < 64 && hal_imu_byte(&b, &first, &tb); i++) {
        if (imu_parser_feed(&g_app.imu, b, first, tb))
            pose_imu_att(&nt->pose, g_app.imu.yaw, g_app.imu.pitch, g_app.imu.roll, g_app.use_imu_yaw,
                         g_app.imu.att_t_us);
    }

    /* 3. 雷达帧：预测位姿 -> 匹配修正（硬件打分）-> 用修正后的位姿融合，窗口中心跟随机器人 */
    uint32_t ft;
    /* 等到帧时刻之后的姿态已到达（IMU 50 Hz，最多等 100 ms）再处理：否则帧时刻恰逢姿态突变时，
     * 突变还没被检测到，这一帧就会带着旧姿态融合 */
    bool att_ready = false;
    if (hal_lidar_waiting(&ft))
        att_ready = !nt->pose.have_att || (int32_t)(nt->pose.imu_t_us - ft) >= 0 || (uint32_t)(now - ft) > 100000;
    if (att_ready) {
        nav_pose_t at;
        int32_t r[3][3], tt[3];
        pose_at(&nt->pose, ft, &at);
        pose_lidar_tf(&nt->pose, &at, r, tt);
        if (pose_match_run(&g_app.pm_cfg, &nt->pose, &at, r, tt, &g_app.pm)) {
            pose_at(&nt->pose, ft, &at);                /* 历史已同步修正 */
            pose_lidar_tf(&nt->pose, &at, r, tt);
        }
        hal_map_info_t mi;
        hal_map_info(&mi);
        uint16_t inv = mi.inv_res ? mi.inv_res : 1311;
        int32_t cx = (int32_t)(((int64_t)at.x * inv) >> 16);
        int32_t cy = (int32_t)(((int64_t)at.y * inv) >> 16);
        /* 姿态突变前后 200 ms 的帧：点云整体倾斜未知，融合会在远处造成大片假台阶。
         * 提交空高度带（zmin > zmax）——帧被消费、不写图，地形扫描照常进行 */
        if (pose_att_unsettled(&nt->pose, now, 200000)) {
            hal_lidar_commit(r, tt, cx, cy, 1, 0);
            g_app.n_skip++;
        } else {
            hal_lidar_commit(r, tt, cx, cy, at.z + g_app.zband_lo, at.z + g_app.zband_hi);
        }
        pose_match_after_commit(&g_app.pm_cfg, &at, &g_app.pm);
        pose_anchor_z(&nt->pose, 200, 180, g_app.anchor_gain);
        g_app.n_frames++;
    }

    /* 4. 主机报文 */
    uint8_t buf[300];
    uint32_t rx_us;
    int len;
    while ((len = hal_host_recv(buf, sizeof buf, &rx_us)) > 0) {
        proto_msg_t m;
        if (proto_parse(buf, len, &m)) {
            nav_task_on_msg(nt, &m, rx_us);
            g_app.n_msgs++;
        }
    }

    /* 5. 规划 */
    if ((uint32_t)(now - g_app.t_plan) >= APP_PLAN_PERIOD_US) {
        g_app.t_plan += APP_PLAN_PERIOD_US;
        if ((uint32_t)(now - g_app.t_plan) >= APP_PLAN_PERIOD_US) g_app.t_plan = now;   /* 落后太多：重新对齐 */
        nav_task_step(nt, now);
    }

    /* 6. 状态回传 */
    if ((uint32_t)(now - g_app.t_status) >= APP_STATUS_PERIOD_US) {
        g_app.t_status = now;
        proto_status_t s;
        nav_task_status(nt, &s, now);
        int n = proto_build_status(buf, &s);
        hal_host_send(buf, n);
    }
}
