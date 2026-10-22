/* 固件主机测试：单元测试 + 仿真世界闭环测试。
 * 用法: fw/test/build.py 编译运行；参数为 traj_eval 交叉验证数据目录（sim/build/tb_traj_eval）
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hal_sim.h"
#include "fixmath.h"
#include "astar.h"
#include "proto.h"
#include "traj_sw.h"
#include "nav_app.h"

static int g_fail = 0;
#define CHECK(c, ...) do { if (!(c)) { g_fail++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ---------------------------------------------------------------- 单元测试 */
static void test_fixmath(void)
{
    printf("[fixmath]\n");
    double worst = 0;
    for (int i = 0; i < 3600; i++) {
        double a = i * M_PI / 1800 - M_PI;
        int32_t x = (int32_t)(cos(a) * 12345), y = (int32_t)(sin(a) * 12345);
        double got = (int16_t)fx_atan2(y, x) * 2 * M_PI / 65536;
        double err = fabs(remainder(got - atan2(y, x), 2 * M_PI)) * 180 / M_PI;
        if (err > worst) worst = err;
    }
    printf("  atan2 最大误差 %.4f°\n", worst);
    CHECK(worst < 0.1, "atan2 误差 %.3f°", worst);
    CHECK(fx_isqrt(1000000) == 1000 && fx_isqrt(999999) == 999, "isqrt");
    CHECK(fx_dist(3000, 4000) == 5000, "dist");
    CHECK(fx_cos(0) == 16384 && fx_sin(16384) == 16384, "sin/cos");
}

static void test_proto(void)
{
    printf("[proto]\n");
    uint8_t b[128];
    proto_msg_t m;
    int n = proto_build_goal(b, 2500, -800, 9000);
    CHECK(proto_parse(b, n, &m) && m.type == MSG_GOAL && m.u.goal.x == 2500 && m.u.goal.y == -800, "goal");
    n = proto_build_target(b, 1, 39, 900, 1500, -200, 300, 45, 7);
    CHECK(proto_parse(b, n, &m) && m.type == MSG_TARGET && m.u.target.x == 1500 && m.u.target.latency_ms == 45, "target");
    CHECK(!proto_parse(b, 3, &m), "短报文应拒绝");
    b[0] = 'X';
    CHECK(!proto_parse(b, n, &m), "错误魔数应拒绝");
    /* 与 RTL 端到端测试中的目标报文一致（gen_top.py） */
    uint8_t g[16] = {'N', 'V', 1, 0x10, 0xC4, 0x09, 0, 0, 0xE0, 0xFC, 0xFF, 0xFF, 0x28, 0x23, 0, 0};
    CHECK(proto_parse(g, 16, &m) && m.u.goal.x == 2500 && m.u.goal.y == -800 && m.u.goal.yaw_cdeg == 9000, "gen_top 报文");
}

static void test_pose(void)
{
    printf("[pose]\n");
    pose_est_t p;
    pose_init(&p, 0, 0, 0);
    uint32_t t = 1000;
    pose_odom(&p, 0, 0, 0, t);
    for (int i = 0; i < 100; i++) { t += 10000; pose_odom(&p, 500, 0, 0, t); }       /* 1 s 前进 0.5 m */
    CHECK(nav_abs32(p.cur.x - 500) <= 2 && nav_abs32(p.cur.y) <= 1, "直行 x=%d y=%d", p.cur.x, p.cur.y);
    for (int i = 0; i < 100; i++) { t += 10000; pose_odom(&p, 0, 0, 1571, t); }      /* 1 s 转 90° */
    int32_t yaw = ANGLE_TO_CDEG(p.cur.yaw);
    CHECK(nav_abs32(yaw - 9000) < 100, "转向 yaw=%d", yaw);
    for (int i = 0; i < 100; i++) { t += 10000; pose_odom(&p, 500, 0, 0, t); }
    CHECK(nav_abs32(p.cur.x - 500) < 20 && nav_abs32(p.cur.y - 500) < 20, "转后直行 x=%d y=%d", p.cur.x, p.cur.y);
    nav_pose_t at;
    pose_at(&p, t - 500000, &at);                                                     /* 0.5 s 前：y≈250 */
    CHECK(nav_abs32(at.y - 250) < 20, "历史插值 y=%d", at.y);
    int32_t ox, oy;
    pose_body_to_odom(&p.cur, 1000, 0, &ox, &oy);                                     /* 朝 +y，前方 1 m */
    CHECK(nav_abs32(ox - p.cur.x) < 30 && nav_abs32(oy - p.cur.y - 1000) < 5, "机身->里程计 %d %d", ox - p.cur.x, oy - p.cur.y);
}

static void test_astar(void)
{
    printf("[astar]\n");
    static uint8_t map[NP_MAP_N * NP_MAP_N];
    memset(map, 0, sizeof map);
    for (int y = 0; y < NP_MAP_N; y++)
        for (int x = 60; x < 66; x++)
            if (y < 100) map[y * NP_MAP_N + x] = 254;         /* 竖墙，上方留口 */
    astar_cfg_t c;
    astar_default_cfg(&c);
    astar_path_t p;
    int rc = astar_plan(&c, map, 20, 20, 110, 20, &p);
    CHECK(rc == 0, "应到达目标 rc=%d", rc);
    int crossed = 0;
    for (int i = 0; i < p.n; i++) {
        if (map[p.wy[i] * NP_MAP_N + p.wx[i]] >= 253) crossed++;
    }
    CHECK(crossed == 0, "路径穿墙 %d 点", crossed);
    CHECK(p.n > 0 && p.wy[p.n / 2] >= 90, "应从上方缺口绕行（中点 y=%d）", p.n ? p.wy[p.n / 2] : -1);
    printf("  路径 %d 点，展开 %d 节点\n", p.n, astar_last_expanded());
    /* 目标被围死：部分路径 */
    for (int y = 0; y < NP_MAP_N; y++) for (int x = 100; x < 104; x++) map[y * NP_MAP_N + x] = 254;
    rc = astar_plan(&c, map, 20, 20, 120, 20, &p);
    CHECK(rc == 1, "不可达应返回部分路径 rc=%d", rc);
}

/* 固定地图与反馈的任务反例，不使用导航的到达判定充当测试真值。 */
static nav_task_t *task_fixture(int32_t gx, int32_t gy)
{
    sim_reset();
    nav_app_init();
    sim_advance(100000);
    nav_task_t *nt = &g_app.nt;
    nt->use_gmap = false;
    hal_map_copy(nt->map, &nt->mi);
    nt->have_map = true;
    memset(nt->map, 0, sizeof nt->map);
    nav_task_set_goal(nt, gx, gy);
    return nt;
}

static void test_goal_contract(void)
{
    printf("[任务：卡住与停稳]\n");
    nav_task_t *nt = task_fixture(550, 0);
    g_sim.now_us += 7000000;
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->state != ST_ARRIVED, "距目标550mm且卡住不能到达");
    CHECK(nt->state == ST_BLOCKED && nt->cmd.vx == 0 && nt->cmd.wz == 0, "近目标卡住应停止并报告受阻");
    g_sim.now_us += 100000;
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->state == ST_BLOCKED && nt->cmd.vx == 0 && nt->cmd.wz == 0, "下一周期不能继续未经检查的恢复");

    nt = task_fixture(100, 0);
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->state != ST_ARRIVED, "无速度反馈不能确认停稳");

    nt = task_fixture(100, 0);
    pose_odom(&nt->pose, 200, 0, 0, g_sim.now_us);
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->state != ST_ARRIVED, "发出零速但反馈仍200mm/s不能到达");
    for (int i = 0; i < 5; i++) {
        g_sim.now_us += 100000;
        pose_odom(&nt->pose, 0, 0, 0, g_sim.now_us);
        nav_task_step(nt, g_sim.now_us);
        CHECK(nt->state != ST_ARRIVED, "尚未完成500ms连续停稳");
    }
    g_sim.now_us += 100000;
    pose_odom(&nt->pose, 0, 0, 0, g_sim.now_us);
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->state == ST_ARRIVED, "新鲜反馈持续停稳后应到达");

    nt = task_fixture(100, 0);
    pose_odom(&nt->pose, 0, 0, 0, g_sim.now_us);
    nav_task_step(nt, g_sim.now_us);
    g_sim.now_us += 600000;
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->state != ST_ARRIVED, "旧零速反馈不能跨过600ms空档证明停稳");

    /* 同一个drive_to也被TRACK调用，定点卡住规则不能让它跳过面对目标。 */
    nt = task_fixture(0, 0);
    nav_task_set_mode(nt, NAV_MODE_TRACK);
    g_sim.now_us += 7000000;
    nt->tgt.valid = true; nt->tgt.x = 1200; nt->tgt.y = 400; nt->tgt.t_us = g_sim.now_us;
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->state == ST_TRACKING && nt->cmd.vx == 0 && nt->cmd.wz > 0,
          "跟踪近站位停滞仍须保留目标朝向对准");
}

static void test_lookahead_contract(void)
{
    printf("[任务：近目标与部分路径]\n");
    nav_task_t *nt = task_fixture(600, 0);
    for (int y = 58; y <= 70; y++)
        for (int x = 67; x <= 68; x++) nt->map[y * NP_MAP_N + x] = 254;
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->look_x != 600 || nt->look_y != 0, "600mm近目标隔墙时应沿可见路径绕行");

    nt = task_fixture(1200, 0);
    for (int y = 0; y < NP_MAP_N; y++) nt->map[y * NP_MAP_N + 67] = 254;
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->look_x != 1200 || nt->look_y != 0, "部分路径末端不能替换成墙后最终目标");

    nt = task_fixture(1200, 0);
    for (int y = 0; y < NP_MAP_N; y++) nt->map[y * NP_MAP_N + 67] = 254;
    nt->path_valid = true;
    nt->path_seq = nt->mi.seq;
    nt->path_t_us = g_sim.now_us;
    nt->path.n = 3;
    nt->path.wx[0] = 64; nt->path.wy[0] = 64;
    nt->path.wx[1] = 68; nt->path.wy[1] = 64;
    nt->path.wx[2] = 72; nt->path.wy[2] = 64;
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->look_x < 150, "不可见的下一路径点不能被强制推进");
}

/* traj_eval_sw 与 Python 定点模型（即 RTL 标准答案）交叉验证 */
static uint32_t *load_hex(const char *path, int *n)
{
    FILE *f = fopen(path, "r");
    if (!f) return NULL;
    uint32_t *v = malloc(sizeof(uint32_t) * 70000);
    *n = 0;
    unsigned x;
    while (*n < 70000 && fscanf(f, "%x", &x) == 1) v[(*n)++] = x;
    fclose(f);
    return v;
}

static void test_traj_crosscheck(const char *dir)
{
    printf("[traj_eval_sw 与定点模型交叉验证]\n");
    char p1[512], p2[512];
    snprintf(p1, sizeof p1, "%s/cost.hex", dir);
    snprintf(p2, sizeof p2, "%s/runs.hex", dir);
    int nc, nr;
    uint32_t *cost = load_hex(p1, &nc), *runs = load_hex(p2, &nr);
    if (!cost || !runs) { printf("  跳过：先运行 sim/run.py tb_traj_eval 生成数据\n"); return; }
    static uint8_t cm[NP_MAP_N * NP_MAP_N];
    for (int i = 0; i < nc && i < NP_MAP_N * NP_MAP_N; i++) cm[i] = (uint8_t)cost[i];
    int p = 0, bad = 0, total = 0;
    while (p + 8 <= nr) {
        int32_t cx = (int32_t)runs[p], cy = (int32_t)runs[p + 1], x0 = (int32_t)runs[p + 2], y0 = (int32_t)runs[p + 3];
        angle_t th0 = (angle_t)runs[p + 4];
        int T = (int)runs[p + 5], K = (int)runs[p + 6];
        uint8_t cmax = (uint8_t)runs[p + 7];
        p += 8;
        hal_cand_t cand[64];
        for (int k = 0; k < K; k++) {
            cand[k].dvx = (int16_t)(runs[p + 2 * k] & 0xFFFF);
            cand[k].dvy = (int16_t)(runs[p + 2 * k] >> 16);
            cand[k].dth = (int16_t)(runs[p + 2 * k + 1] & 0xFFFF);
        }
        p += 2 * K;
        hal_traj_res_t r[64];
        traj_eval_sw(cm, cx, cy, cand, K, T, x0, y0, th0, cmax, r);
        for (int k = 0; k < K; k++) {
            uint32_t w0 = (uint32_t)r[k].max_cost | ((uint32_t)r[k].n_unknown << 8) | ((uint32_t)r[k].coll_step << 16) |
                          ((uint32_t)r[k].collide << 24) | ((uint32_t)r[k].left << 25);
            if (w0 != runs[p] || r[k].sum_cost != runs[p + 1] || (uint32_t)r[k].end_x != runs[p + 2] ||
                (uint32_t)r[k].end_y != runs[p + 3]) {
                if (bad < 5) printf("  不一致 k=%d\n", k);
                bad++;
            }
            total++;
            p += 4;
        }
    }
    printf("  %d 条候选，不一致 %d\n", total, bad);
    CHECK(bad == 0 && total > 0, "traj_eval_sw 与模型不一致");
    free(cost);
    free(runs);
}

/* ---------------------------------------------------------------- 闭环仿真 */
static void run_sim(double seconds)
{
    int steps = (int)(seconds * 1000);          /* 1 ms 一步 */
    for (int i = 0; i < steps; i++) {
        sim_advance(1000);
        nav_app_poll();
    }
}

static void world_default(void)
{
    sim_reset();
    sim_add_obs(1500, -1500, 1800, 1200);       /* 挡在路线上的长墙，上端有缺口 */
    sim_add_obs(3000, 500, 3600, 1100);         /* 箱子 */
    sim_add_obs(-2000, 3000, 6000, 3200);       /* 北墙 */
    sim_add_obs(-2000, -3200, 6000, -3000);     /* 南墙 */
}

static void test_goto(void)
{
    printf("[闭环：定点导航绕障]\n");
    world_default();
    nav_app_init();
    uint8_t b[64];
    int n = proto_build_goal(b, 4500, 0, 0);
    sim_host_inject(b, n);
    double t = 0;
    while (t < 60 && !(g_app.nt.state == ST_ARRIVED)) { run_sim(0.5); t += 0.5; }
    double d = hypot(g_sim.x - 4500, g_sim.y);
    printf("  用时 %.1f s，终点误差 %.0f mm，最小离障 %.0f mm，状态 %d\n", t, d, g_sim.min_clear_mm, g_app.nt.state);
    CHECK(g_app.nt.state == ST_ARRIVED, "未到达");
    CHECK(d < 300, "终点误差 %.0f", d);
    CHECK(g_sim.min_clear_mm > g_sim.robot_r_mm * 0.6, "离障过近 %.0f mm", g_sim.min_clear_mm);
    CHECK(g_sim.n_status > 0, "未回传状态");
}

static void test_waypoints(void)
{
    printf("[闭环：路径点巡航]\n");
    world_default();
    nav_app_init();
    /* 直接走本地接口（报文路径已在 test_goto 覆盖） */
    proto_msg_t m = {.type = MSG_WAYPOINTS};
    int32_t wx[3] = {1000, 1000, 0}, wy[3] = {-2000, 2200, 2200};
    m.u.wp.n = 3; m.u.wp.loop = 0;
    for (int i = 0; i < 3; i++) { m.u.wp.x[i] = wx[i]; m.u.wp.y[i] = wy[i]; }
    nav_task_on_msg(&g_app.nt, &m, hal_time_us());
    double t = 0;
    while (t < 90 && g_app.nt.state != ST_ARRIVED) { run_sim(0.5); t += 0.5; }
    printf("  用时 %.1f s，当前航点 %d，终点 (%.0f, %.0f)，最小离障 %.0f mm\n", t, g_app.nt.wp_i, g_sim.x, g_sim.y, g_sim.min_clear_mm);
    CHECK(g_app.nt.state == ST_ARRIVED && g_app.nt.wp_i == 2, "未走完航点");
    CHECK(hypot(g_sim.x - 0, g_sim.y - 2200) < 300, "终点误差");
}

static void test_search_track(void)
{
    printf("[闭环：探索找目标 -> 跟踪接近]\n");
    world_default();
    sim_add_obs(-2200, -3200, -2000, 3200);     /* 西墙：封闭场地（比赛场地有边界） */
    sim_add_obs(6000, -3200, 6200, 3200);       /* 东墙 */
    g_sim.target_on = true;
    g_sim.tgt_x = 4200;                          /* 目标在墙后 */
    g_sim.tgt_y = -1800;
    nav_app_init();
    uint8_t b[16];
    int n = proto_build_mode(b, NAV_MODE_SEARCH);
    sim_host_inject(b, n);
    double t = 0, t_found = -1;
    while (t < 150) {
        run_sim(0.5);
        t += 0.5;
        if (t_found < 0 && g_app.nt.mode == NAV_MODE_TRACK) t_found = t;
        if (t_found > 0 && hypot(g_sim.x - g_sim.tgt_x, g_sim.y - g_sim.tgt_y) < 1000 && g_sim.cmd.vx == 0) break;
    }
    double d = hypot(g_sim.x - g_sim.tgt_x, g_sim.y - g_sim.tgt_y);
    printf("  发现目标 %.1f s，结束 %.1f s，与目标距离 %.0f mm（站位 700），最小离障 %.0f mm\n",
           t_found, t, d, g_sim.min_clear_mm);
    CHECK(t_found > 0, "未发现目标");
    CHECK(d > 400 && d < 1000, "接近距离 %.0f", d);
    CHECK(g_sim.min_clear_mm > g_sim.robot_r_mm * 0.6, "离障过近 %.0f mm", g_sim.min_clear_mm);
}

static void test_estop(void)
{
    printf("[急停报文]\n");
    world_default();
    nav_app_init();
    uint8_t b[64];
    sim_host_inject(b, proto_build_goal(b, 4500, 0, 0));
    run_sim(3);
    sim_host_inject(b, proto_build_mode(b, NAV_MODE_ESTOP));
    run_sim(0.5);
    CHECK(g_app.nt.mode == NAV_MODE_ESTOP && g_sim.cmd.vx == 0 && g_sim.cmd.wz == 0, "急停后应零速");
}

int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "--nav-contract") == 0) {
        test_goal_contract();
        printf("NAV CONTRACT %s (failures %d)\n", g_fail ? "FAILED" : "PASSED", g_fail);
        return g_fail ? 1 : 0;
    }
    if (argc > 1 && strcmp(argv[1], "--lookahead-known") == 0) {
        test_lookahead_contract();
        printf("KNOWN LOOKAHEAD: %d pending assertions (N2 prerequisite)\n", g_fail);
        return g_fail ? 1 : 0;
    }
    test_fixmath();
    test_proto();
    test_pose();
    test_astar();
    test_goal_contract();
    test_traj_crosscheck(argc > 1 ? argv[1] : "../../sim/build/tb_traj_eval");
    test_goto();
    test_waypoints();
    test_search_track();
    test_estop();
    printf("\n%s（失败 %d）\n", g_fail ? "FW TEST FAILED" : "FW TEST PASSED", g_fail);
    return g_fail ? 1 : 0;
}
