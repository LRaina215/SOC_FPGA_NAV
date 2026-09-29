#include "nav_app.h"
#include "hal.h"

nav_app_t g_app;

bool nav_app_have_prior(void) { return g_app.prx.state == PRIOR_READY; }

void nav_app_init(void)
{
    nav_task_init(&g_app.nt);
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
    prior_rx_init(&g_app.prx);
    loc_default_cfg(&g_app.loc.cfg);
    loc_init(&g_app.loc);
}

/* 位姿进入新坐标系（SET_POSE / 对准完成）：旧坐标系下的局部观测全部作废 */
static void frame_jumped(nav_task_t *nt)
{
    hal_map_clear();
    gmap_clear_obs(&nt->gm);
    nt->path_valid = false;
    nt->have_map = false;
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
    }

    /* 3. 雷达帧：按预测位姿提交（融合 + 提取结构点），窗口中心跟随机器人 */
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
        pose_at(&nt->pose, ft, &at);
        pose_lidar_tf(&nt->pose, &at, r, tt);
        hal_map_info_t mi;
        hal_map_info(&mi);
        uint16_t inv = mi.inv_res ? mi.inv_res : 1311;
        int32_t cx = (int32_t)(((int64_t)at.x * inv) >> 16);
        int32_t cy = (int32_t)(((int64_t)at.y * inv) >> 16);
        /* 姿态突变前后 200 ms 的帧：点云整体倾斜未知，融合会在远处造成大片假台阶。
         * 提交空高度带（zmin > zmax）——帧被消费、不写图、不提结构点，地形扫描照常进行 */
        bool skip = pose_att_unsettled(&nt->pose, now, 200000);
        if (skip) {
            hal_lidar_commit(r, tt, cx, cy, 1, 0);
            g_app.n_skip++;
        } else {
            hal_lidar_commit(r, tt, cx, cy, at.z + g_app.zband_lo, at.z + g_app.zband_hi);
        }
        g_app.pm_pend = !skip && !receiving && (prior ? lc->st != LOC_ODOM : g_app.pm_cfg.enable);
        g_app.pm_at = at;
        g_app.pm_stale = false;
        pose_anchor_z(&nt->pose, 200, 180, g_app.anchor_gain);
        g_app.n_frames++;
    }
    /* 匹配（先融合后匹配）：结构点就绪 -> 发起打分（硬件约 40 ms，主循环不等待）-> 取回结果、修正位姿
     * -> 把结构点按修正量并入结构图（有图模式：只释放点缓冲） */
    const pm_cfg_t *pc = prior ? &g_app.pm_cfg_prior : &g_app.pm_cfg;
    if (g_app.pm_pend && hal_match_ready()) {
        g_app.pm_pend = false;
        g_app.pm_align = false;
        if (g_app.pm_stale || lc->st == LOC_LOST || (lc->st == LOC_ALIGN && g_app.turning)) {
            g_app.pm_k = 0;
        } else if (lc->st == LOC_ALIGN) {
            hal_match_gate(false);
            g_app.pm_k = loc_align_next(lc, g_app.pm_cand, PM_MAX_CAND);
            g_app.pm_align = true;
        } else {
            g_app.pm_k = pose_match_cands(pc, g_app.pm_cand);
        }
        if (g_app.pm_k > 0 && hal_match_start(g_app.pm_at.x, g_app.pm_at.y, g_app.pm_cand, g_app.pm_k))
            g_app.pm_run = true;
        else {
            hal_match_gate(true);
            hal_match_update(0, 0, 0, g_app.pm_at.x, g_app.pm_at.y);
        }
    }
    if (g_app.pm_run && hal_match_result(g_app.pm_res, g_app.pm_k)) {
        g_app.pm_run = false;
        if (g_app.pm_stale) {
            hal_match_gate(true);
            hal_match_update(0, 0, 0, g_app.pm_at.x, g_app.pm_at.y);
        } else if (g_app.pm_align) {
            loc_align_feed(lc, g_app.pm_cand, g_app.pm_res, g_app.pm_k);
            g_app.pm_k = loc_align_next(lc, g_app.pm_cand, PM_MAX_CAND);
            if (g_app.pm_k > 0 && hal_match_start(g_app.pm_at.x, g_app.pm_at.y, g_app.pm_cand, g_app.pm_k)) {
                g_app.pm_run = true;                 /* 同一帧的下一批候选 */
            } else {
                g_app.pm_align = false;
                hal_match_gate(true);
                if (loc_align_finish(lc, now)) {
                    pose_correct_delta(&nt->pose, lc->res_dx, lc->res_dy, 0, lc->res_da);
                    frame_jumped(nt);
                }
                if (lc->st == LOC_LOST) g_app.t_lost = now;
                hal_match_update(0, 0, 0, g_app.pm_at.x, g_app.pm_at.y);
            }
        } else {
            bool ok = pose_match_solve(pc, &nt->pose, g_app.pm_res, g_app.pm_k, &g_app.pm);
            if (prior && loc_track(lc, &g_app.pm, g_app.pm_res, ok, now)) {
                /* 丢失：原地保持，以当前估计为中心重定位 */
                loc_start_align(lc, lc->cfg.reloc_mm, lc->cfg.reloc_ang);
            }
            hal_match_update(ok ? g_app.pm.dx : 0, ok ? g_app.pm.dy : 0, ok ? g_app.pm.dyaw : 0,
                             g_app.pm_at.x, g_app.pm_at.y);
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
    }

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
