#include "nav_app.h"
#include "hal.h"
#include "support_guard.h"

nav_app_t g_app;

bool nav_app_have_prior(void) { return g_app.prx.state == PRIOR_READY; }

void nav_app_init(void)
{
    nav_task_init(&g_app.nt);
    g_app.nt.async_gmap=true;
    support_reset(0); /* 默认不声明启动盲区已知；实验须显式给出平地前提。 */
    imu_parser_init(&g_app.imu, IMU_PROTO_WIT55);
    uint32_t now = hal_time_us();
    g_app.t_plan = now;
    g_app.t_status = now;
    g_app.use_imu_yaw = false;
    pm_default_cfg(&g_app.pm_cfg);
    g_app.pm_cfg.enable = true;             /* SLF 匹配定位（硬件打分不可用时 hal_match_eval 返回 false，自动退化为纯航位推算） */
    g_app.pm_cfg_prior = g_app.pm_cfg;
    g_app.pm_cfg_prior.gain_q8 = 208;       /* 先验图是固定的真值，不会"自己拉自己"，可以修得更快 */
    /* 退化判定门限放低：先验图里沿墙方向的代价是真平的（没有自建图质心跳动的噪声），少量垂直结构
     * （远处的路沿、箱体正面）就足以约束前进方向；600 时平台上沿行驶方向漂 0.28 m，200 时最大 6 cm */
    g_app.pm_cfg_prior.min_curv = 200;
    g_app.zband_lo = -600;
    g_app.zband_hi = 1200;
    g_app.anchor_gain = 64;
    g_app.anchor_hl = 60;                   /* 足迹采样点：轮子着地处前后 ±60 mm */
    g_app.anchor_hw = 135;                  /* 轮距 270 mm（胎面中心，tools/robot_calib.py） */
    prior_rx_init(&g_app.prx);
    loc_default_cfg(&g_app.loc.cfg);
    loc_init(&g_app.loc);
    hal_lidar_init();
    lidar_cal_init(&g_app.lcal);            /* 先按 M60 缺省内参生成，收到雷达自带内参后按需重算 */
    g_app.lcal_budget = 8;                  /* 软核每像素约 0.1 ms：每轮 8 个，整张表约 1–2 s */
    xt_link_init(&g_app.xl, hal_time_us());
}

/* 位姿进入新坐标系（SET_POSE / 对准完成）：旧坐标系下的局部观测全部作废 */
static void frame_jumped(nav_task_t *nt)
{
    astar_cancel();
    support_frame_jump();
    nt->scan_active=nt->scan_settling=nt->scan_failed=false;
    nt->scan_exit_valid=false;
    nt->terrain_frame=(local_frame_t){0};
    hal_map_clear();
    gmap_clear_obs(&nt->gm);
    nt->path_valid = false;
    nt->have_map = false;
    g_app.anchor_wait = false;
    hal_map_info_t mi;
    if (hal_map_info(&mi)) { nt->stale = true; nt->stale_seq = mi.seq; }
    target_init(&nt->tgt, 1500000, 700);
    explore_seen_reset(&nt->exp);
}

static void send_ack(void)
{
    uint8_t b[16];
    int n = proto_build_map_ack(b, g_app.prx.next, g_app.prx.state);
    hal_host_send(b, n);
}

static void wait_anchor(void)
{
    hal_map_info_t mi = {0};
    hal_map_info(&mi);
    g_app.anchor_wait = true;
    g_app.anchor_seq = mi.seq;
}

/* 第二遍复用保留的深度帧：只按已接受的采集位姿融合，不重新提取匹配点。 */
static void finish_fusion(bool valid)
{
    if (!g_app.fuse_held) return;
    nav_pose_t at = valid ? g_app.pm_at : g_app.nt.pose.cur;
    if(!g_app.nt.gm.has_prior) local_frame_pose(&g_app.nt.terrain_frame,&at,&at);
    int32_t r[3][3], t[3];
    pose_lidar_tf(&g_app.nt.pose, &at, r, t);
    hal_map_info_t mi; hal_map_info(&mi);
    uint16_t inv = mi.inv_res ? mi.inv_res : 1311;
    int32_t cx = (int32_t)(((int64_t)at.x*inv)>>16), cy = (int32_t)(((int64_t)at.y*inv)>>16);
    hal_lidar_stage(HAL_LIDAR_FUSE_ONLY);
    if (valid) wait_anchor();
    hal_lidar_commit(r, t, cx, cy, valid ? at.z+g_app.zband_lo : 1, valid ? at.z+g_app.zband_hi : 0);
    g_app.fuse_held = false;
}

static void on_msg(nav_task_t *nt, const proto_msg_t *m, uint32_t rx_us)
{
    switch (m->type) {
    case MSG_SET_POSE:
        pose_set(&nt->pose, m->u.pose.x, m->u.pose.y, ANGLE_FROM_CDEG(m->u.pose.yaw_cdeg));
        frame_jumped(nt);
        if (g_app.pm_pend || g_app.pm_run) g_app.pm_stale = true;
        if (nav_app_have_prior() && (m->u.pose.flags & 1))
            loc_start_align(&g_app.loc, m->u.pose.range_mm, (int32_t)m->u.pose.range_cdeg * 182 / 100);
        else if (nav_app_have_prior())
            g_app.loc.st = LOC_OK;                   /* 主机保证位姿准确：直接跟踪 */
        break;
    case MSG_MAP_BEGIN:
        prior_rx_begin(&g_app.prx, &nt->gm, m->u.map_begin.total);
        hal_match_zband(APP_ZLO_ODOM, APP_ZHI);
        loc_init(&g_app.loc);
        if (g_app.pm_pend || g_app.pm_run) g_app.pm_stale = true;
        send_ack();
        break;
    case MSG_MAP_DATA:
        prior_rx_data(&g_app.prx, &nt->gm, m->u.map_data.offset, m->u.map_data.data, m->u.map_data.n);
        send_ack();
        break;
    case MSG_MAP_END:
        if (prior_rx_end(&g_app.prx, &nt->gm) == PRIOR_READY) hal_match_zband(APP_ZLO_PRIOR, APP_ZHI);
        loc_init(&g_app.loc);                        /* 等 SET_POSE */
        send_ack();
        break;
    case MSG_MAP_CLEAR:
        prior_rx_clear(&g_app.prx, &nt->gm);
        hal_match_zband(APP_ZLO_ODOM, APP_ZHI);
        loc_init(&g_app.loc);
        send_ack();
        break;
    default:
        nav_task_on_msg(nt, m, rx_us);
        break;
    }
}

void nav_app_poll(void)
{
    nav_task_t *nt = &g_app.nt;
    loc_t *lc = &g_app.loc;
    uint32_t now = hal_time_us();
    const bool prior = nav_app_have_prior();
    const bool receiving = g_app.prx.state == PRIOR_RECV;

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
        if (g_app.imu.gyro_new) {
            g_app.imu.gyro_new = false;
            pose_imu_gyro(&nt->pose, g_app.imu.gx, g_app.imu.gy, g_app.imu.gyro_t_us);
        }
    }

    /* 3. 雷达帧：先提结构点，匹配门控接受后按修正位姿重读同帧融合。 */
    if (g_app.anchor_wait) {
        hal_map_info_t mi;
        if (hal_map_info(&mi) && mi.seq != g_app.anchor_seq) {
            g_app.anchor_wait = false;
            nav_pose_t local=nt->pose.cur;
            if(!prior) local_frame_pose(&nt->terrain_frame,&local,&local);
            pose_anchor_z_at(&nt->pose,&local,g_app.anchor_hl,g_app.anchor_hw,g_app.anchor_gain);
        }
    }
    xt_link_poll(&g_app.xl, now);
    lidar_cal_poll(&g_app.lcal, g_app.lcal_budget);
    uint32_t ft;
    /* 等到帧时刻之后的姿态已到达（IMU 50 Hz，最多等 100 ms）再处理：否则帧时刻恰逢姿态突变时，
     * 突变还没被检测到，这一帧就会带着旧姿态融合 */
    bool att_ready = false;
    /* 上一帧的匹配未结束时不提交新帧（点缓冲还在用） */
    if (!g_app.pm_pend && !g_app.pm_run && hal_lidar_waiting(&ft))
        att_ready = !nt->pose.have_att || (int32_t)(nt->pose.imu_t_us - ft) >= 0 || (uint32_t)(now - ft) > 100000;
    if (att_ready) {
        nav_pose_t at;
        int32_t r[3][3], tt[3];
        pose_query_t query = pose_at(&nt->pose, ft, &at);
        if (!pose_query_valid(query)) g_app.n_time_skip++;
        if (!pose_query_valid(query)) at = nt->pose.cur; /* 仅用于消费废帧，不融合/匹配 */
        pose_lidar_tf(&nt->pose, &at, r, tt);
        hal_map_info_t mi;
        hal_map_info(&mi);
        uint16_t inv = mi.inv_res ? mi.inv_res : 1311;
        int32_t cx = (int32_t)(((int64_t)at.x * inv) >> 16);
        int32_t cy = (int32_t)(((int64_t)at.y * inv) >> 16);
        /* 姿态突变前后 200 ms 的帧：点云整体倾斜未知，融合会在远处造成大片假台阶。
         * 提交空高度带（zmin > zmax）——帧被消费、不写图、不提结构点，地形扫描照常进行 */
        bool skip = !pose_query_valid(query) || pose_att_unsettled(&nt->pose, ft, 200000);
        bool no_lut = !lidar_cal_ready(&g_app.lcal);      /* 方向表只写了一部分：点的方向不可信 */
        bool matching = !skip && !no_lut && !receiving && (prior ? lc->st != LOC_ODOM : g_app.pm_cfg.enable);
        hal_lidar_stage(matching ? HAL_LIDAR_MATCH_ONLY : HAL_LIDAR_NORMAL);
        if (skip || no_lut) {
            hal_lidar_commit(r, tt, cx, cy, 1, 0);
            if (no_lut) g_app.n_lut_skip++;
            else g_app.n_skip++;
            skip = true;
        } else {
            if (!matching) wait_anchor();
            if(!matching && !prior) {
                nav_pose_t local;
                local_frame_pose(&nt->terrain_frame,&at,&local);
                pose_lidar_tf(&nt->pose,&local,r,tt);
                cx=(int32_t)(((int64_t)local.x*inv)>>16);cy=(int32_t)(((int64_t)local.y*inv)>>16);
            }
            hal_lidar_commit(r, tt, cx, cy, at.z + g_app.zband_lo, at.z + g_app.zband_hi);
        }
        g_app.pm_pend = matching;
        g_app.fuse_held = matching;
        g_app.pm_at = at;
        g_app.pm_stale = false;
        g_app.n_frames++;
    }
    /* 匹配：结构点就绪 -> 异步打分 -> 提案/门控/提交 -> 结构图更新及第二遍高度融合。 */
    const pm_cfg_t *pc = prior ? &g_app.pm_cfg_prior : &g_app.pm_cfg;
    if (g_app.pm_pend && hal_match_ready()) {
        g_app.pm_pend = false;
        g_app.pm_align = false;
        if (g_app.pm_stale || lc->st == LOC_LOST || (lc->st == LOC_ALIGN && g_app.turning)) {
            g_app.pm_k = 0;
        } else if (lc->st == LOC_ALIGN) {
            hal_match_gate(false);
            g_app.pm_k = loc_align_next(lc, g_app.pm_work.cand, PM_MAX_CAND);
            g_app.pm_align = true;
        } else {
            g_app.pm_k = pose_match_cands(pc, g_app.pm_work.cand);
        }
        if (g_app.pm_k > 0 && hal_match_start(g_app.pm_at.x, g_app.pm_at.y, g_app.pm_work.cand, g_app.pm_k))
            g_app.pm_run = true;
        else {
            hal_match_gate(true);
            hal_match_update(0, 0, 0, g_app.pm_at.x, g_app.pm_at.y);
            finish_fusion(!g_app.pm_stale && (!prior || lc->st == LOC_OK || lc->st == LOC_DEGRADED));
        }
    }
    if (g_app.pm_run && hal_match_result(g_app.pm_work.res, g_app.pm_k)) {
        g_app.pm_run = false;
        if (g_app.pm_stale) {
            hal_match_gate(true);
            hal_match_update(0, 0, 0, g_app.pm_at.x, g_app.pm_at.y);
            finish_fusion(false);
        } else if (g_app.pm_align) {
            loc_align_feed_generated(lc, g_app.pm_work.res, g_app.pm_k);
            g_app.pm_k = loc_align_next(lc, g_app.pm_work.cand, PM_MAX_CAND);
            if (g_app.pm_k > 0 && hal_match_start(g_app.pm_at.x, g_app.pm_at.y, g_app.pm_work.cand, g_app.pm_k)) {
                g_app.pm_run = true;                 /* 同一帧的下一批候选 */
            } else {
                g_app.pm_align = false;
                hal_match_gate(true);
                if (loc_align_finish(lc, now)) {
                    pm_stat_t correction = {.dx=lc->res_dx, .dy=lc->res_dy, .dyaw=lc->res_da};
                    pose_match_commit(&nt->pose, g_app.pm_at.x, g_app.pm_at.y, &correction);
                    frame_jumped(nt);
                }
                if (lc->st == LOC_LOST) g_app.t_lost = now;
                hal_match_update(0, 0, 0, g_app.pm_at.x, g_app.pm_at.y);
                finish_fusion(false); /* 全局跳变后只释放旧帧，下一新帧使用新坐标 */
            }
        } else {
            bool ok = pose_match_solve(pc, g_app.pm_work.res, g_app.pm_k, &g_app.pm);
            if (prior && loc_track(lc, &g_app.pm, g_app.pm_work.res, ok, now)) {
                /* 丢失：原地保持，以当前估计为中心重定位 */
                loc_start_align(lc, lc->cfg.reloc_mm, lc->cfg.reloc_ang);
                ok = false;
            }
            int32_t old_x = g_app.pm_at.x, old_y = g_app.pm_at.y;
            if (ok) {
                int64_t x=nt->pose.x_um,y=nt->pose.y_um;
                pose_match_commit(&nt->pose,old_x,old_y,&g_app.pm);
                if(!prior) local_frame_preserve(&nt->terrain_frame,x,y,nt->pose.x_um,nt->pose.y_um,g_app.pm.dyaw);
            }
            hal_match_update(ok ? g_app.pm.dx : 0, ok ? g_app.pm.dy : 0, ok ? g_app.pm.dyaw : 0,
                             old_x, old_y);
            if (ok) {
                g_app.pm_at.x += g_app.pm.dx; g_app.pm_at.y += g_app.pm.dy;
                g_app.pm_at.yaw = (angle_t)(g_app.pm_at.yaw + g_app.pm.dyaw);
            }
            finish_fusion(!prior || lc->st == LOC_OK || lc->st == LOC_DEGRADED);
        }
    }
    /* 对准失败（视野里结构不足或只有平行结构）：原地转约 35° 换视角，稳定后用新帧再对准 */
    if (lc->want_turn) {
        lc->want_turn = false;
        g_app.turning = true;
        g_app.turn_until = now + 1500000;
    }
    if (g_app.turning && (int32_t)(now - g_app.turn_until) > 300000) g_app.turning = false;
    nt->hold_wz = (g_app.turning && (int32_t)(now - g_app.turn_until) < 0 && lc->st == LOC_ALIGN) ? 400 : 0;
    /* 丢失（对准重试用尽）：每 3 s 再试一次 */
    if (prior && lc->st == LOC_LOST && (uint32_t)(now - g_app.t_lost) > 3000000)
        loc_start_align(lc, lc->cfg.reloc_mm, lc->cfg.reloc_ang);

    /* 4. 主机报文 */
    static uint8_t buf[PROTO_MAX_LEN];
    uint32_t rx_us;
    int len;
    while ((len = hal_host_recv(buf, sizeof buf, &rx_us)) > 0) {
        proto_msg_t m;
        if (proto_parse(buf, len, &m)) {
            on_msg(nt, &m, rx_us);
            g_app.n_msgs++;
        }
    }

    /* 5. 规划：有图模式下定位未就绪（等初始位姿 / 对准中 / 丢失）时原地保持，定位退化时限速 */
    nt->hold = prior && lc->st != LOC_OK && lc->st != LOC_DEGRADED;
    nt->slow = prior && lc->st == LOC_DEGRADED;
    if ((uint32_t)(now - g_app.t_plan) >= APP_PLAN_PERIOD_US) {
        g_app.t_plan += APP_PLAN_PERIOD_US;
        if ((uint32_t)(now - g_app.t_plan) >= APP_PLAN_PERIOD_US) g_app.t_plan = now;   /* 落后太多：重新对齐 */
        nav_task_step(nt, now);
    } else nav_task_map_work(nt,4);

    /* 6. 状态回传 */
    if ((uint32_t)(now - g_app.t_status) >= APP_STATUS_PERIOD_US) {
        g_app.t_status = now;
        proto_status_t s;
        nav_task_status(nt, &s, now);
        s.loc_state = prior ? (uint8_t)lc->st : (uint8_t)LOC_ODOM;
        s.loc_quality = (uint8_t)(prior ? loc_quality(lc) : 0);
        int n = proto_build_status(buf, &s);
        hal_host_send(buf, n);
    }
}
