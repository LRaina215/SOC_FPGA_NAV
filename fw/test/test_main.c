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
#include "support_guard.h"

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
    CHECK(pose_xy_at(&p, t - 500000, &at) == POSE_OK, "有效历史查询");                /* 0.5 s 前：y≈250 */
    CHECK(nav_abs32(at.y - 250) < 20, "历史插值 y=%d", at.y);
    int32_t ox, oy;
    pose_body_to_odom(&p.cur, 1000, 0, &ox, &oy);                                     /* 朝 +y，前方 1 m */
    CHECK(nav_abs32(ox - p.cur.x) < 30 && nav_abs32(oy - p.cur.y - 1000) < 5, "机身->里程计 %d %d", ox - p.cur.x, oy - p.cur.y);
}

static void test_pose_time_contract(void)
{
    printf("[pose 时间/历史合同]\n");
    pose_est_t p;
    pose_init(&p, 0, 0, 0);
    uint32_t t = 1000;
    nav_pose_t at = {0};
    CHECK(pose_xy_at(&p, t, &at) == POSE_EMPTY, "未取得反馈不能冒充有效历史");
    pose_odom(&p, 500, 0, 0, t);
    for (int i=0; i<1000; i++) { t+=1000; pose_odom(&p,500,0,0,t); }
    CHECK(pose_xy_at(&p,t-500000,&at)==POSE_OK && nav_abs32(at.x-250)<3,
          "1kHz积分仍须保留0.5秒前历史 x=%ld",(long)at.x);
    CHECK(pose_xy_at(&p,t-900000,&at)==POSE_OLD,"超出历史必须明确拒绝");
    CHECK(pose_xy_at(&p,t+20000,&at)==POSE_PREDICTED && nav_abs32(at.x-510)<3,
          "20ms前向查询须显式预测，不能夹到旧位置");
    CHECK(pose_xy_at(&p,t+31000,&at)==POSE_FUTURE,"不能无限外推");
    int32_t x=p.cur.x;
    pose_odom(&p,0,0,0,t-1000);
    CHECK(p.last_odo_us==t && p.cur.x==x && p.odo_vx==500,"乱序反馈不能回退时钟或篡改停稳反馈");
    CHECK(pose_at(&p,t,&at)==POSE_ATT_EMPTY,"完整位姿须有采集时刻姿态证据");
    pose_imu_att(&p,0,0,0,false,t);
    CHECK(pose_at(&p,t,&at)==POSE_OK,"同步样本应通过");
    CHECK(pose_at(&p,t-10000,&at)==POSE_ATT_OLD,"姿态历史不足不能夹取最旧样本");
    pose_imu_att(&p,0,1000,0,false,t-1000);
    CHECK(p.imu_t_us==t && p.cur.pitch==0,"乱序姿态不能重置滤波器");
    target_t target; target_init(&target,1500000,700);
    target_update(&target,&p,true,1000,0,500,1,0,t);
    CHECK(target.valid && nav_abs32(target.x-1250)<3,"迟到目标使用实际历史而非当前位姿");
    int32_t tx=target.x;
    target_update(&target,&p,true,2000,0,600,2,0,t);
    CHECK(target.x==tx && target.id==1,"旧捕获时间不能覆盖较新目标");
    pose_odom(&p,500,0,0,t+300000);
    CHECK(pose_xy_at(&p,t+100000,&at)==POSE_OLD,"反馈缺口内不插值虚假运动");

    pose_init(&p,0,0,0);t=UINT32_MAX-100000;
    pose_odom(&p,500,0,0,t);
    for(int i=0;i<800;i++){t+=1000;pose_odom(&p,500,0,0,t);}
    CHECK(pose_xy_at(&p,t-300000,&at)==POSE_OK && nav_abs32(at.x-250)<3,
          "32位时间回绕后的历史仍有效");
}

static void test_match_transaction(void)
{
    CHECK(sizeof(hal_match_cand_t)==6 && sizeof(hal_match_res_t)==12,"匹配数组使用硬件字段宽度");
    pm_cfg_t wire;pm_default_cfg(&wire);hal_match_cand_t cand[PM_MAX_CAND];
    int nc=pose_match_cands(&wire,cand);
    CHECK(nc==245 && cand[0].dx==-75 && cand[0].dy==-75 && cand[0].dyaw==-256 &&
          cand[244].dx==75 && cand[244].dy==75 && cand[244].dyaw==256,"压缩不得改变245项默认匹配搜索");
    wire.step_mm=40000;
    CHECK(pose_match_cands(&wire,cand)==0,"超硬件16bit范围应拒绝，不可截断");
    printf("[匹配提案/门控/提交]\n");
    pose_est_t p;
    pose_init(&p,0,0,0); pose_set(&p,1000,2000,0);
    pose_odom(&p,500,0,0,1000); pose_odom(&p,500,0,0,201000);
    pm_cfg_t c; pm_default_cfg(&c); c.half=1; c.half_ang=1; c.gain_q8=256;
    hal_match_res_t r[27];
    for(int a=-1,k=0;a<=1;a++) for(int y=-1;y<=1;y++) for(int x=-1;x<=1;x++,k++)
        r[k]=(hal_match_res_t){.cost=(x-1)*(x-1)*500+y*y*600+a*a*1000,.n_valid=100,.n_in=90,.n_pts=100};
    pm_stat_t st={0};
    CHECK(pose_match_solve(&c,r,27,&st),"曲面应给出提案");
    CHECK(p.cur.x==1100 && p.cur.y==2000 && !st.applied && st.n_applied==0,
          "solve不能先提交位姿");
    loc_t l; loc_default_cfg(&l.cfg); loc_init(&l); l.st=LOC_OK; l.cfg.edge_n=1;
    CHECK(loc_track(&l,&st,r,true,201000),"边界连续门控应拒绝该提案");
    CHECK(p.cur.x==1100 && !st.applied,"拒绝提案不能改变位姿");
    st.dx=30; st.dy=-20; st.dyaw=16384;
    pose_match_commit(&p,1000,2000,&st);
    CHECK(p.cur.x==1030 && p.cur.y==2080 && p.cur.yaw==16384 && st.applied && st.n_applied==1,
          "应绕采集位置而非当前车体旋转 x=%ld y=%ld",(long)p.cur.x,(long)p.cur.y);
    nav_pose_t at;
    CHECK(pose_xy_at(&p,1000,&at)==POSE_OK && at.x==1030 && at.y==1980 && at.yaw==16384,
          "历史必须与当前位姿作同一刚体变换");

    for(int axis=0;axis<2;axis++) for(int sign=-1;sign<=1;sign+=2) {
        pose_init(&p,0,0,0);pose_set(&p,0,0,axis?16384:0);
        pose_odom(&p,7*sign,0,0,1000);
        for(int i=1;i<=500;i++) {
            pose_odom(&p,7*sign,0,0,1000+i*20000);
            if(i%5==0) {pm_stat_t zero={0};pose_match_commit(&p,p.cur.x,p.cur.y,&zero);}
        }
        CHECK((axis?p.y_um:p.x_um)==70000*sign,"匹配不能丢掉小数里程计");
    }
    local_frame_t f={0};
    pose_init(&p,0,0,0);pose_set(&p,1200,-500,1000);
    pose_odom(&p,125,0,0,1000);pose_odom(&p,125,0,0,101000);
    for(int i=0;i<32;i++) {
        nav_pose_t before,after,cap;
        local_frame_pose(&f,&p.cur,&before);
        CHECK(pose_xy_at(&p,1000,&cap)==POSE_OK,"采集历史有效");
        local_frame_pose(&f,&cap,&cap);
        int64_t oldx=p.x_um,oldy=p.y_um;
        pm_stat_t corr={.dx=i&1?18:-24,.dy=i&1?-12:15,.dyaw=i&1?-21:64};
        pose_match_commit(&p,p.cur.x,p.cur.y,&corr);
        local_frame_preserve(&f,oldx,oldy,p.x_um,p.y_um,corr.dyaw);
        local_frame_pose(&f,&p.cur,&after);
        CHECK(fx_dist(after.x-before.x,after.y-before.y)<=2 && after.yaw==before.yaw,
              "匹配修正不应跳动连续地形坐标");
        nav_pose_t hist;
        CHECK(pose_xy_at(&p,1000,&hist)==POSE_OK,"修正后历史有效");
        local_frame_pose(&f,&hist,&hist);
        CHECK(fx_dist(hist.x-cap.x,hist.y-cap.y)<=3,"采集姿态须在同一地形坐标内");
        int32_t lx,ly,gx,gy;
        local_frame_point(&f,2500,1000,&lx,&ly,false);
        local_frame_point(&f,lx,ly,&gx,&gy,true);
        CHECK(fx_dist(gx-2500,gy-1000)<=3,"全局目标往返变换不能累计失真");
    }
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
    static uint8_t unknown[64*64];
    memset(unknown,255,sizeof unknown);
    astar_cfg_t cfg;astar_default_cfg(&cfg);
    rc=astar_grid_bounded(&cfg,unknown,64,2,32,60,32,&p,128);
    CHECK(astar_busy() && rc==1 && p.n>1 && p.wx[0]==2 && p.wy[0]==32 &&
          p.wx[p.n-1]!=60,"预算耗尽应保留前缀和搜索前沿，不能伪装已到目标");
    /* 原输入变动不污染正在搜索的快照；使用路径前任务层仍查当前地图。 */
    memset(unknown,254,sizeof unknown);
    int chunks=1;
    while(astar_busy() && chunks<64) {rc=astar_continue(&p,128);chunks++;}
    CHECK(rc==0 && !astar_busy() && p.wx[p.n-1]==60 && p.wy[p.n-1]==32 && chunks>1,
          "多周期搜索应继续完成而非每周期重启 chunks=%d rc=%d",chunks,rc);
    memset(unknown,255,sizeof unknown);
    astar_grid_bounded(&cfg,unknown,64,2,32,60,32,&p,1);astar_cancel();
    CHECK(!astar_busy() && astar_continue(&p,128)==-1,"取消后不能恢复旧缓存");
    p.n=2;p.wx[0]=1;p.wy[0]=1;p.wx[1]=2;p.wy[1]=1;
    astar_complete_only(true);
    rc=astar_grid_bounded(&cfg,unknown,64,2,32,60,32,&p,128);
    CHECK(rc==2 && p.n==2 && p.wx[0]==1 && p.wy[0]==1,"待完成搜索不得反复替换可用旧路径");
    chunks=1;
    while(astar_busy() && chunks<64) {
        rc=astar_continue(&p,128);chunks++;
        if(astar_busy()) CHECK(rc==2 && p.n==2 && p.wx[0]==1,"搜索期间保持路径发布原子性");
    }
    CHECK(rc==0 && p.wx[p.n-1]==60,"结束后一次替换为完成的新路径");
    memset(map,255,sizeof map);
    for(int mode=0;mode<2;mode++) {
        p.n=2;p.wx[0]=1;p.wy[0]=1;p.wx[1]=2;p.wy[1]=1;
        astar_complete_only(true);
        rc=mode?astar_plan_bounded(&cfg,map,64,64,120,64,&p,128):
                astar_window_bounded(&cfg,map,64,64,90,90,&p,128);
        CHECK(rc==2 && p.wx[0]==1 && p.wy[0]==1,"分辨率/裁窗换算不得修改待保留路径");
        chunks=1;
        while(astar_busy() && chunks<64) {rc=astar_continue(&p,128);chunks++;}
        CHECK(rc==0 && p.wx[p.n-1]==(mode?121:90) && p.wy[p.n-1]==(mode?65:90),
              "续算完成后应使用新搜索的坐标变换 mode=%d",mode);
    }
}

/* 固定地图与反馈的任务反例，不使用导航的到达判定充当测试真值。 */
static nav_task_t *task_fixture(int32_t gx, int32_t gy)
{
    sim_reset();
    nav_app_init();
    sim_advance(100000);
    nav_task_t *nt = &g_app.nt;
    nt->use_gmap = false;
    nt->async_gmap = false; /* 隔离任务测试直接驱动step，不运行应用后台调度。 */
    hal_map_copy(nt->map, &nt->mi);
    nt->have_map = true;
    memset(nt->map, 0, sizeof nt->map);
    sim_map_fixture(nt->map,&nt->mi); /* 此隔离测试明确提供全平地观测，不使用传感器盲区。 */
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
    CHECK(nt->state!=ST_ARRIVED && !nt->n_recover && nt->cmd_source==CMD_LOCAL && nt->cmd.vx>0 &&
          nt->motion_result==MOTION_OK && nt->lp.ground_checked,
          "后续周期须能重新规划已检查的可行接入，不能永久停在550mm处");
    nt=task_fixture(550,0);memset(nt->map,254,sizeof nt->map);sim_map_fixture(nt->map,&nt->mi);
    g_sim.now_us+=7000000;nav_task_step(nt,g_sim.now_us);
    for(int i=0;i<3;i++) {g_sim.now_us+=100000;nav_task_step(nt,g_sim.now_us);}
    CHECK(!nt->cmd.vx && !nt->cmd.wz && !nt->n_recover && nt->state!=ST_ARRIVED,
          "受阻后的重试仍须拦截真实危险，不得变成未经检查的恢复");

    nt = task_fixture(50, 0);
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->state != ST_ARRIVED, "无速度反馈不能确认停稳");

    nt = task_fixture(50, 0);
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

    nt = task_fixture(50, 0);
    pose_odom(&nt->pose, 0, 0, 0, g_sim.now_us);
    nav_task_step(nt, g_sim.now_us);
    g_sim.now_us += 600000;
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->state != ST_ARRIVED, "旧零速反馈不能跨过600ms空档证明停稳");

    nt = task_fixture(130, 0);
    for (int i=0;i<7;i++) {
        g_sim.now_us+=100000;
        pose_odom(&nt->pose,0,0,0,g_sim.now_us);
        nav_task_step(nt,g_sim.now_us);
    }
    CHECK(nt->state!=ST_ARRIVED,"估计130mm尚未预留定位误差预算，不能声明到达");
    CHECK(nt->cmd.vx>0 && nt->cmd.vx<=65,"应有适合130mm接近距离的低速候选 vx=%d",nt->cmd.vx);

    /* 同一个drive_to也被TRACK调用，定点卡住规则不能让它跳过面对目标。 */
    nt = task_fixture(0, 0);
    nav_task_set_mode(nt, NAV_MODE_TRACK);
    g_sim.now_us += 7000000;
    nt->tgt.valid = true; nt->tgt.x = 1200; nt->tgt.y = 400; nt->tgt.t_us = g_sim.now_us;
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->state == ST_TRACKING && nt->cmd.vx == 0 && nt->cmd.wz > 0,
          "跟踪近站位停滞仍须保留目标朝向对准");
}

static void test_gmap_clearance(void)
{
    printf("[地图：粗格保留足迹净空]\n");
    nav_task_t *nt = task_fixture(1500, 0);
    gmap_update(&nt->gm, nt->map, &nt->mi, 0, 0);
    int gx, gy;
    gmap_mm_to_cell(&nt->gm, &nt->mi, 500, 0, &gx, &gy);
    int k = gy * GM_N + gx;
    nt->map[64 * NP_MAP_N + 74] = 253;
    gmap_update(&nt->gm, nt->map, &nt->mi, 0, 0);
    CHECK(nt->gm.grid[k] == GM_COST_PARTIAL, "部分净空粗格保留252高代价");
    CHECK(nt->gm.occ[k] < nt->gm.cfg.occ_thr, "253不能冒充254原始占用证据");
    for (int y = 64; y <= 65; y++) for (int x = 74; x <= 75; x++) nt->map[y * NP_MAP_N + x] = 253;
    gmap_update(&nt->gm, nt->map, &nt->mi, 0, 0);
    CHECK(nt->gm.grid[k] == 253, "整粗格都受限时必须保留253禁行");
    for (int y = 64; y <= 65; y++) for (int x = 74; x <= 75; x++) nt->map[y * NP_MAP_N + x] = 0;
    gmap_update(&nt->gm, nt->map, &nt->mi, 0, 0);
    CHECK(nt->gm.grid[k] == 0, "最新自由观测应能替换净空代价");
    nt->map[64 * NP_MAP_N + 74] = 254;
    for (int i = 0; i < 10; i++) gmap_update(&nt->gm, nt->map, &nt->mi, 0, 0);
    CHECK(nt->gm.grid[k] == 254, "原始危险仍独立累积占用证据");

    /* 500mm直通道，中心x=0两侧各有约250mm净空，210mm圆可直行。
     * 自由细格跨两个100mm粗格边界；取最大253会让两个粗格都带净空代价，
     * 不能据此断言整条通道不可达。 */
    nt = task_fixture(0, 1500);
    nt->use_gmap = true;
    nt->pose.cur.yaw = ANGLE_FROM_CDEG(9000);
    for (int y = 0; y < NP_MAP_N; y++) {
        nt->map[y * NP_MAP_N + 58] = 254;
        nt->map[y * NP_MAP_N + 69] = 254;
        for (int x = 59; x <= 62; x++) nt->map[y * NP_MAP_N + x] = 253;
        for (int x = 65; x <= 68; x++) nt->map[y * NP_MAP_N + x] = 253;
    }
    for (int i = 0; i < 10; i++) gmap_update(&nt->gm, nt->map, &nt->mi, 0, 0);
    nav_cmd_t straight = {125, 0, 0, 0};
    CHECK(motion_check(nt->map, &nt->mi, &nt->pose.cur, &nt->lp_cfg, &straight) == MOTION_OK,
          "独立足迹检查确认500mm通道中心可直行");
    nav_task_step(nt, g_sim.now_us);
    for(int i=0;i<8 && (!nt->path_fine || astar_busy());i++) {
        g_sim.now_us+=100000;nav_task_step(nt,g_sim.now_us);
    }
    CHECK(nt->path_complete && nt->look_y >= 400 && nt->cmd.vx > 0,
          "粗格不能封死细格可通的通道 complete=%d look=%d vx=%d",
          nt->path_complete, nt->look_y, nt->cmd.vx);
    CHECK(nt->path_fine, "已观测近场目标应获得原分辨率路径");

    /* 同一500mm通道，最终目标在64格细化窗口之外。应先接入安全近场
     * 子目标，不能因最终目标在窗外而反复退回不可接入的粗路径。 */
    nt->goal_y=3000;nt->path_valid=false;nt->refine_path=true;
    nav_task_step(nt,g_sim.now_us);
    for(int wait=0;wait<50 && (!nt->path_fine || astar_busy());wait++) {
        g_sim.now_us+=100000;nav_task_step(nt,g_sim.now_us);
    }
    CHECK(nt->path_fine && !nt->path_complete && nt->cmd.vx>0 && nt->goal_y==3000,
          "长目标应使用明确的细路径前缀，不能伪装终点 fine=%d complete=%d vx=%d goal=%ld",
          nt->path_fine,nt->path_complete,nt->cmd.vx,(long)nt->goal_y);

    astar_path_t path;
    astar_cfg_t cfg = nt->as_cfg; cfg.allow_unknown = false;
    memset(nt->map, 0, sizeof nt->map);
    CHECK(astar_window(&cfg, nt->map, 64, 64, 90, 64, &path) == 0 && path.wx[path.n-1] == 90,
          "近场路径终点应保持原窗口坐标");
    CHECK(astar_window(&cfg, nt->map, 64, 64, 110, 64, &path) == -2 && !path.n,
          "裁窗外目标不能伪装成边界到达或复用旧路径");
    for (int y = 0; y < NP_MAP_N; y++) nt->map[y * NP_MAP_N + 70] = 255;
    CHECK(astar_window(&cfg, nt->map, 64, 64, 90, 64, &path) != 0,
          "细路径不能借未知中心格绕过记忆规划");

    nt = task_fixture(1200, 0); nt->use_gmap = true;
    gmap_update(&nt->gm, nt->map, &nt->mi, 0, 0);
    nav_task_step(nt, g_sim.now_us);
    CHECK(!nt->path_fine && !nt->refine_path && nt->cmd.vx > 0,
          "已有可执行粗路径时不应主动换成局部细路径");
    nt->path_seq=(uint16_t)(nt->mi.seq-1);
    g_sim.now_us+=600000;
    static uint8_t cache_grid[GM_N*GM_N];
    memcpy(cache_grid,nt->gm.grid,sizeof cache_grid);
    memset(nt->gm.grid,255,sizeof nt->gm.grid);
    nav_task_step(nt,g_sim.now_us);
    CHECK(!astar_busy() && nt->path_valid && nt->cmd.vx>0,
          "当前图确认仍能执行的路线，不应仅因代际/时间启动全局搜索");
    memcpy(nt->gm.grid,cache_grid,sizeof cache_grid);
    g_sim.traj_blocked = true;
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->refine_path && !nt->path_valid, "粗路径局部执行受阻后应请求细化");
    g_sim.traj_blocked = false;
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->path_fine && nt->cmd.vx > 0, "请求细化后应能接入已观测细路径");
}

static void test_gmap_slices(void)
{
    printf("[地图：分行更新/跨帧去重]\n");
    static gmap_t full,part;
    static uint8_t map[NP_MAP_N*NP_MAP_N];
    gmap_default_cfg(&full.cfg);gmap_init(&full);part=full;
    hal_map_info_t mi={.inv_res=1311,.res_mm=50};
    uint32_t rng=91;
    for(int frame=1;frame<=24;frame++) {
        mi.seq=(uint16_t)frame;mi.cx=frame*5-40;mi.cy=frame*3-36;
        for(int i=0;i<NP_MAP_N*NP_MAP_N;i++) {
            rng=rng*1664525u+1013904223u;
            int v=(rng>>24)%8;map[i]=v<3?255:v==3?254:v==4?253:0;
        }
        int32_t x=mi.cx*50,y=mi.cy*50;
        if(frame==5) {gmap_block_add(&full,&mi,x+300,y,2,4);gmap_block_add(&part,&mi,x+300,y,2,4);}
        gmap_update(&full,map,&mi,x,y);
        int budget=frame%4==0?1:frame%4==1?3:frame%4==2?4:8;
        int chunks=0;
        do {gmap_update_rows(&part,map,&mi,x,y,budget);chunks++;} while(!part.work_done && chunks<70);
        CHECK(part.work_done && chunks<=(GM_N+budget-1)/budget,"每次行数有界且完整扫描收敛");
        CHECK(full.ox==part.ox && full.oy==part.oy && full.n_shift==part.n_shift &&
              !memcmp(full.occ,part.occ,sizeof full.occ) && !memcmp(full.cost,part.cost,sizeof full.cost) &&
              !memcmp(full.grid,part.grid,sizeof full.grid),"分行结果必须与同步结果一致 frame=%d",frame);
        uint32_t n=part.n_upd;
        CHECK(!gmap_update_rows(&part,map,&mi,x,y,4) && part.n_upd==n,"同代完成后不得重复累积");
    }
    gmap_init(&part);memset(map,254,sizeof map);mi.cx=mi.cy=0;mi.seq=100;
    CHECK(!part.ready,"初图完成前不得声明可用于路径引导");
    gmap_update_rows(&part,map,&mi,0,0,4);
    mi.seq=101;
    for(int i=0;i<32 && !part.work_done;i++) gmap_update_rows(&part,map,&mi,0,0,4);
    CHECK(part.work_done && part.ready,"跨帧之后仍能完成最新一代");
    for(int y=0;y<GM_N;y++) CHECK(part.occ[y*GM_N]==(y<4?4:2),"每行每代至多一次证据 y=%d occ=%d",y,part.occ[y*GM_N]);
    gmap_clear_obs(&part);CHECK(!part.ready,"清图后须重新等待初次观测引导");
}

static void test_align_regenerated(void)
{
    printf("[匹配工作区复用：粗/细候选重建等价]\n");
    static hal_match_cand_t ca[PM_MAX_CAND],cb[PM_MAX_CAND];
    static hal_match_res_t res[PM_MAX_CAND];
    for(int seed=1;seed<=3;seed++) {
        loc_t a,b;loc_default_cfg(&a.cfg);loc_init(&a);
        loc_start_align(&a,seed==1?50:seed==2?100:500,10*182);b=a;
        for(int round=0;round<80;round++) {
            int n=loc_align_next(&a,ca,PM_MAX_CAND),nb=loc_align_next(&b,cb,PM_MAX_CAND);
            CHECK(n==nb && !memcmp(ca,cb,(size_t)n*sizeof *ca),"候选批次必须同序同值");
            if(!n) break;
            for(int i=0;i<n;i++) {
                int x=ca[i].dx-seed*10,y=ca[i].dy+seed*15,z=ca[i].dyaw-seed*64;
                res[i]=(hal_match_res_t){.cost=(uint32_t)(x*x+y*y+z*z/32),.n_in=90,.n_valid=100,.n_pts=100};
            }
            loc_align_feed(&a,ca,res,n);loc_align_feed_generated(&b,res,n);
            CHECK(!memcmp(&a,&b,sizeof a),"重建feed的整个搜索状态应与保留候选一致 seed=%d round=%d",seed,round);
        }
        CHECK(a.phase==3 && a.k_next==a.k_total,"粗搜索、两轮细化均须完整覆盖");
    }
}

static void test_ground_scan(void)
{
    printf("[启动地形观察：反馈/超时/统一出口]\n");
    nav_task_t *nt=task_fixture(1500,0);support_reset(350);
    pose_odom(&nt->pose,0,0,0,g_sim.now_us);
    nav_task_step(nt,g_sim.now_us);
    CHECK(nt->cmd_source==CMD_GROUND_SCAN && nt->cmd.wz==600 && !nt->cmd.vx,"先观察再平移");
    for(int i=0;i<255;i++) {
        g_sim.now_us+=100000;pose_odom(&nt->pose,0,0,0,g_sim.now_us);nav_task_step(nt,g_sim.now_us);
    }
    CHECK(nt->scan_failed && support_scan_needed() && !nt->cmd.wz && !nt->cmd.vx,
          "不能用已发600mrad/s假装转完一圈，静止反馈应超时停车");
    nt=task_fixture(1500,0);support_reset(350);pose_odom(&nt->pose,0,0,0,g_sim.now_us);
    nav_task_step(nt,g_sim.now_us);
    for(int i=0;i<115;i++) {
        g_sim.now_us+=100000;pose_odom(&nt->pose,0,0,600,g_sim.now_us);nav_task_step(nt,g_sim.now_us);
    }
    CHECK(nt->scan_turned>=65536 && support_scan_needed() && !nt->cmd.wz,"反馈转满后应先请求停稳");
    for(int i=0;i<8 && support_scan_needed();i++) {
        g_sim.now_us+=100000;pose_odom(&nt->pose,0,0,0,g_sim.now_us);nav_task_step(nt,g_sim.now_us);
    }
    CHECK(!support_scan_needed(),"连续新鲜零速反馈后完成观察");
    nav_task_step(nt,g_sim.now_us);
    CHECK(nt->cmd_source==CMD_GROUND_EXIT && !nt->cmd.vx && nt->cmd.wz<0,
          "扫描惯性/错误反馈转过头时，应先对准已查验的退出走廊，不能直接前进");
    pose_correct(&nt->pose,nt->scan_exit_x,nt->scan_exit_y,0);nav_task_step(nt,g_sim.now_us);
    CHECK(!nt->scan_exit_valid && nt->mode==NAV_MODE_GOTO && nt->goal_x==1500 && nt->state!=ST_ARRIVED,
          "退出前缀不能冒充全局任务到达");
    nt=task_fixture(1500,0);support_reset(350);
    static uint8_t unknown[NP_MAP_N*NP_MAP_N];memset(unknown,255,sizeof unknown);sim_map_fixture(unknown,&nt->mi);
    pose_odom(&nt->pose,0,0,0,g_sim.now_us);nav_task_step(nt,g_sim.now_us);
    for(int i=0;i<115;i++) {
        g_sim.now_us+=100000;pose_odom(&nt->pose,0,0,600,g_sim.now_us);nav_task_step(nt,g_sim.now_us);
    }
    for(int i=0;i<8;i++) {
        g_sim.now_us+=100000;pose_odom(&nt->pose,0,0,0,g_sim.now_us);nav_task_step(nt,g_sim.now_us);
    }
    CHECK(nt->scan_turned>=65536 && support_scan_needed() && !nt->scan_exit_valid,
          "反馈转满一圈但没有实测支撑退出走廊时，不能宣布观察充分");
    nt=task_fixture(1500,-500);support_reset(350);pose_odom(&nt->pose,0,0,0,g_sim.now_us);
    nav_task_step(nt,g_sim.now_us);CHECK(nt->cmd.wz<0,"启动观察方向应先朝向任务所在侧");
    nt=task_fixture(1500,0);support_reset(350);pose_odom(&nt->pose,0,0,0,g_sim.now_us);
    nt->map[64*NP_MAP_N+68]=254;nav_task_step(nt,g_sim.now_us);
    CHECK(nt->cmd_source==CMD_GROUND_SCAN && nt->motion_result==MOTION_HAZARD && !nt->cmd.wz,
          "启动观察仍须经最终足迹检查，不能覆盖真实危险");
    nt=task_fixture(1500,0);support_reset(350);pose_odom(&nt->pose,0,0,0,g_sim.now_us);
    nav_task_step(nt,g_sim.now_us);nav_task_set_mode(nt,NAV_MODE_SEARCH);
    CHECK(nt->scan_active,"自动搜索/跟踪模式切换不应抹掉已取得的观察进度");
    nav_task_set_mode(nt,NAV_MODE_TRACK);nt->tgt.valid=false;nav_task_step(nt,g_sim.now_us);
    CHECK(nt->mode==NAV_MODE_SEARCH && nt->scan_active && !nt->cmd.vx && !nt->cmd.wz,
          "目标失效应先停车转搜索，不能继续过期目标的退出动作");
    nt=task_fixture(1500,0);support_reset(350);support_scan_finish();
    nt->scan_exit_valid=true;nt->scan_exit_x=450;nt->scan_exit_y=0;
    pose_odom(&nt->pose,0,0,0,g_sim.now_us);nt->map[64*NP_MAP_N+69]=254;
    nav_task_step(nt,g_sim.now_us);
    CHECK(nt->cmd_source==CMD_GROUND_EXIT && nt->motion_result==MOTION_HAZARD && !nt->cmd.vx,
          "已查验的退出前缀仍须拦截新图中的危险，不能拿旧结论持续放行");
    support_reset(0);
}

static void test_slow_progress(void)
{
    printf("[任务：低速进展与真实停滞]\n");
    for(int stuck=0;stuck<2;stuck++) {
        nav_task_t *nt=task_fixture(5000,0);
        nt->lp_cfg.v_max=5;
        pose_odom(&nt->pose,0,0,0,g_sim.now_us);
        for(int i=0;i<100;i++) {
            g_sim.now_us+=100000;
            pose_odom(&nt->pose,stuck?0:5,0,0,g_sim.now_us);
            nav_task_step(nt,g_sim.now_us);
        }
        CHECK(stuck ? nt->n_recover>0 : nt->n_recover==0,
              "5mm/s真实进展不应恢复，真正停滞仍须恢复 stuck=%d recover=%lu",stuck,(unsigned long)nt->n_recover);
    }
}

static void test_lookahead_contract(void)
{
    printf("[任务：近目标与部分路径]\n");
    /* 已知中心线的细路径，圆外缘仍有未知。前视只给方向，必须让
     * LP评估微动；细化标志不能抢先把所有受限候选封死。 */
    for(int allow=0;allow<=1;allow++) {
        nav_task_t *q=task_fixture(1200,0);
        q->as_cfg.allow_unknown=allow;
        q->path_valid=q->path_fine=true;q->path_seq=q->mi.seq;
        q->path_cx=q->mi.cx;q->path_cy=q->mi.cy;
        q->path.n=2;q->path.wx[0]=64;q->path.wy[0]=64;
        q->path.wx[1]=80;q->path.wy[1]=64;
        q->map[65*NP_MAP_N+68]=255;
        sim_map_fixture(q->map,&q->mi);
        nav_task_step(q,g_sim.now_us);
        if(allow) CHECK(q->look_x>400 && q->cmd.vx>0 && q->cmd.vx<=60 &&
                        q->motion_result==MOTION_UNKNOWN && q->state!=ST_ARRIVED,
            "细路径应到LP评估受限微动 look=%d vx=%d result=%u",q->look_x,q->cmd.vx,q->motion_result);
        else CHECK(!q->cmd.vx,"禁止未知时仍须拒绝未知前视");
    }
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

    nt = task_fixture(1200, 0);
    nt->path_valid = true; nt->path_seq = nt->mi.seq; nt->path_t_us = g_sim.now_us;
    nt->path.n = 2;
    nt->path.wx[0] = 73; nt->path.wy[0] = 64;
    nt->path.wx[1] = 64; nt->path.wy[1] = 74;
    nt->map[64 * NP_MAP_N + 72] = 254;
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->path_valid && nt->look_y > 400, "最近路径点被真实障碍遮挡时仍应找到后续可行接入点");

    nt = task_fixture(1200, 0);
    nt->path_valid = true; nt->path_seq = nt->mi.seq; nt->path_t_us = g_sim.now_us;
    nt->path.n = 9;
    nt->path.wx[0] = 64; nt->path.wy[0] = 64;
    nt->path.wx[1] = 64; nt->path.wy[1] = 66;
    for (int i = 2; i < 9; i++) { nt->path.wx[i] = (int16_t)(71+i); nt->path.wy[i] = 67; }
    for (int y = 44; y <= 84; y++) nt->map[y * NP_MAP_N + 72] = 254;
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->look_y >= 100, "有界采样不能漏掉唯一可行的相邻下一点而选回自身 look=(%d,%d)", nt->look_x, nt->look_y);

    nt = task_fixture(1200, 0);
    memset(nt->map, 254, sizeof nt->map);
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->cmd.vx == 0 && nt->cmd.wz == 0 && nt->look_x == nt->pose.cur.x,
          "无路径时不能直接指向最终目标");

    /* 超过256点的蛇形通道：返回靠起点的前缀，明确为部分路径。 */
    uint8_t grid[64 * 64];
    memset(grid, 254, sizeof grid);
    for (int x = 1; x <= 61; x += 2) {
        for (int y = 1; y <= 61; y++) grid[y * 64 + x] = 0;
        if (x < 61) grid[((x / 2) % 2 == 0 ? 61 : 1) * 64 + x + 1] = 0;
    }
    astar_path_t path;
    int rc = astar_grid(&nt->as_cfg, grid, 64, 1, 1, 61, 61, &path);
    CHECK(rc == 1 && path.n == ASTAR_MAX_PATH && path.wx[0] == 1 && path.wy[0] == 1,
          "长路径截断必须保留起点并返回部分路径 rc=%d n=%d", rc, path.n);

    nt = task_fixture(1500, 0);
    nt->use_gmap = true; nt->n_recover = 1;
    nt->look_x = nt->look_y = 0;
    g_sim.now_us += 7000000;
    nav_task_step(nt, g_sim.now_us);
    int blocks = 0;
    for (int i = 0; i < GM_VBLK; i++) blocks += nt->gm.vblk[i].ttl != 0;
    CHECK(blocks == 0, "没有有效前视点时不能把机器人当前位置设为虚拟障碍");

    nt = task_fixture(1500, 0);
    nt->use_gmap = true; nt->n_recover = 1;
    nt->look_x = 1000; nt->look_y = 0;
    gmap_update(&nt->gm, nt->map, &nt->mi, 0, 0);
    nt->path_valid = false; /* 局部规划失败会要求重规划，但远处的受阻接入点仍有效 */
    g_sim.now_us += 7000000;
    nav_task_step(nt, g_sim.now_us);
    blocks = 0;
    for (int i = 0; i < GM_VBLK; i++) blocks += nt->gm.vblk[i].ttl != 0;
    CHECK(blocks > 0, "局部规划失败后仍须保留远处受阻点反馈，否则全局反复规划同一窄缝");
}

static void test_motion_contract(void)
{
    printf("[任务：统一运动检查]\n");
    nav_task_t *nt = task_fixture(1500, 0);
    nt->map[64*NP_MAP_N+68]=255;
    nav_cmd_t partial={375,0,0,0};
    CHECK(motion_check(nt->map,&nt->mi,&nt->pose.cur,&nt->lp_cfg,&partial)==MOTION_UNKNOWN,
          "与起始圆部分相交的未知格，其外侧新增扫掠部分仍必须是UNKNOWN");
    CHECK(motion_segment(nt->map,&nt->mi,&nt->pose.cur,&nt->lp_cfg,75,0)==MOTION_UNKNOWN,
          "前视线段同样不能整格豁免起始圆外的未知部分");
    nt->map[64*NP_MAP_N+68]=0;nt->map[64*NP_MAP_N+59]=255;
    CHECK(motion_check(nt->map,&nt->mi,&nt->pose.cur,&nt->lp_cfg,&partial)==MOTION_OK,
          "前进时正在离开的未知后缘不能误标新进入");
    nt=task_fixture(1500,0);
    nav_cmd_t cmd = {-150, 0, 0, 0};
    nt->map[67 * NP_MAP_N + 58] = 254; /* 后侧障碍，中心线看不到 */
    CHECK(motion_check(nt->map, &nt->mi, &nt->pose.cur, &nt->lp_cfg, &cmd) == MOTION_HAZARD,
          "倒车完整足迹必须拦截侧后障碍");
    nt->recovering = true; nt->recover_dir = 1; nt->recover_until_us = g_sim.now_us + 1000000;
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->cmd_source == CMD_RECOVERY && nt->motion_result == MOTION_HAZARD && nt->cmd.vx == 0,
          "恢复倒车必须经过统一检查再下发");

    nt = task_fixture(1500, 0);
    nt->hold = true; nt->hold_wz = 600;
    nt->map[68 * NP_MAP_N + 64] = 254;
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->cmd_source == CMD_HOLD && nt->motion_result == MOTION_HAZARD && !nt->cmd.wz,
          "重定位旋转必须检查完整圆足迹");
    memset(nt->map, 253, sizeof nt->map);
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->motion_result == MOTION_OK && nt->cmd.wz == 600, "原始危险足迹清空时允许退出膨胀区");
    nt->slow = true;
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->cmd.wz == 300 && nt->motion_result == MOTION_OK, "慢速旋转也须按最终角速度评估");
    nt->stale = true; nt->stale_seq = nt->mi.seq;
    nav_task_step(nt, g_sim.now_us);
    CHECK(!nt->cmd.wz && nt->motion_result == MOTION_NO_MAP, "坐标系旧地图不能支持重定位旋转");
    nav_task_set_mode(nt, NAV_MODE_ESTOP);
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->cmd.mode == NAV_MODE_ESTOP && !nt->cmd.wz, "急停独立于地图检查");

    nt = task_fixture(1500, 0);
    nt->hold = true; nt->hold_wz = 600; nt->have_map = false;
    nt->stale = true; nt->stale_seq = nt->mi.seq;
    nav_task_step(nt, g_sim.now_us);
    CHECK(!nt->cmd.wz && nt->motion_result == MOTION_NO_MAP, "无可用地图不盲转");

    nt = task_fixture(1500, 0);
    memset(nt->map, 255, sizeof nt->map);
    cmd = (nav_cmd_t){-150, 0, 0, 0};
    CHECK(motion_check(nt->map, &nt->mi, &nt->pose.cur, &nt->lp_cfg, &cmd) == MOTION_UNKNOWN,
          "近程新足迹未知不冒充已知可行");
    cmd = (nav_cmd_t){0, 0, 600, 0};
    CHECK(motion_check(nt->map, &nt->mi, &nt->pose.cur, &nt->lp_cfg, &cmd) == MOTION_OK,
          "圆足迹不扩大时允许在当前未知足迹内转向观察");

    nt = task_fixture(1500, 0);
    g_sim.traj_stale = true;
    CHECK(!lp_plan(&nt->lp_cfg, &nt->mi, &nt->pose.cur, nt->map, false, true, 1500, 0, &nt->lp) && nt->lp.map_stale && !nt->lp.ground_checked,
          "HAL返回过期不能继续选用结果");
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->motion_result == MOTION_MAP_CHANGED && !nt->cmd.vx && !nt->cmd.wz,
          "评估过期的任务分支不能再用兜底旋转继续运动");
    g_sim.traj_stale = false; g_sim.traj_change_map = true;
    CHECK(!lp_plan(&nt->lp_cfg, &nt->mi, &nt->pose.cur, nt->map, false, true, 1500, 0, &nt->lp) && nt->lp.map_stale,
          "评估后地图代际变化必须拒绝");

    nt = task_fixture(1500, 0);
    nt->map[67 * NP_MAP_N + 78] = 254;
    CHECK(lp_plan(&nt->lp_cfg, &nt->mi, &nt->pose.cur, nt->map, false, true, 1500, 0, &nt->lp) &&
          nt->lp.cmd.vx > 0 && nt->lp.cmd.wz < 0 &&
          motion_check(nt->map, &nt->mi, &nt->pose.cur, &nt->lp_cfg, &nt->lp.cmd) == MOTION_OK,
          "最高分直行足迹受阻时，必须能选出已复核的可行右弯候选");

    nt = task_fixture(1500, 600);
    nt->slow = true;
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->cmd_source == CMD_LOCAL && nt->lp.cmd.vx > 0 && nt->lp.cmd.wz != 0 &&
          nt->cmd.vx == nt->lp.cmd.vx && nt->cmd.wz == nt->lp.cmd.wz &&
          nt->cmd.vx <= nt->lp_cfg.v_max / 2 && nav_abs32(nt->cmd.wz) <= nt->lp_cfg.w_max / 2 &&
          nt->motion_result == MOTION_OK,
          "正常曲线限速须同时缩放线/角速度，并按最终指令通过检查");

    nt = task_fixture(0, 1500);
    nt->map[64 * NP_MAP_N + 68] = 254;
    nav_task_step(nt, g_sim.now_us);
    CHECK(!nt->cmd.vx && !nt->cmd.wz, "足迹已受阻时，不得发路径对准旋转");

    nt = task_fixture(1500, 0);
    g_sim.traj_blocked = true;
    nt->map[64 * NP_MAP_N + 68] = 254;
    nav_task_step(nt, g_sim.now_us);
    CHECK(!nt->cmd.vx && !nt->cmd.wz, "无足迹可行接入时，不得发兜底旋转");

    nt = task_fixture(1500, 0);
    nt->map[68 * NP_MAP_N + 72] = 254;
    CHECK(motion_segment(nt->map, &nt->mi, &nt->pose.cur, &nt->lp_cfg, 800, 0) == MOTION_HAZARD,
          "中心线空闲但侧方200mm有障碍时，前视胶囊必须拒绝");
    nt->map[68 * NP_MAP_N + 72] = 0;
    nt->map[69 * NP_MAP_N + 72] = 254;
    CHECK(motion_segment(nt->map, &nt->mi, &nt->pose.cur, &nt->lp_cfg, 800, 0) == MOTION_OK,
          "250mm侧净空的直线应通过完整足迹检查");

    nt = task_fixture(1500, 0);
    nt->pose.cur.x = 38; nt->map[64 * NP_MAP_N + 69] = 254;
    cmd = (nav_cmd_t){2,0,0,0};
    CHECK(motion_check(nt->map, &nt->mi, &nt->pose.cur, &nt->lp_cfg, &cmd) == MOTION_HAZARD,
          "2mm/s不能被Q8逐步截成零而漏掉前方触碰");
    nt->lp_cfg.safe_steps = 64;
    CHECK(motion_check(nt->map, &nt->mi, &nt->pose.cur, &nt->lp_cfg, &cmd) == MOTION_LIMIT,
          "超过64个50ms采样步的请求须明确拒绝，不能无界运行");

    nt = task_fixture(0, 0);
    nav_task_set_mode(nt, NAV_MODE_TRACK);
    nt->tgt.valid = true; nt->tgt.x = 600; nt->tgt.y = 300; nt->tgt.t_us = g_sim.now_us;
    nt->map[64 * NP_MAP_N + 68] = 254;
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->cmd_source == CMD_TRACK_FACE && nt->motion_result == MOTION_HAZARD && !nt->cmd.wz,
          "跟踪朝向请求仍须检查");

    nt = task_fixture(0, 0);
    nav_task_set_mode(nt, NAV_MODE_SEARCH);
    memset(nt->map, 254, sizeof nt->map);
    nav_task_step(nt, g_sim.now_us);
    CHECK(nt->cmd_source == CMD_SEARCH && nt->motion_result == MOTION_HAZARD && !nt->cmd.wz,
          "没有探索点时旋转仍须检查");

    nt = task_fixture(1500, 0);
    memset(nt->map, 255, sizeof nt->map);
    nt->map[64 * NP_MAP_N + 70] = 254;
    cmd = (nav_cmd_t){500, 0, 0, 0};
    CHECK(motion_check(nt->map, &nt->mi, &nt->pose.cur, &nt->lp_cfg, &cmd) == MOTION_HAZARD,
          "先遇未知仍须继续检查后续已知危险，不能被未知许可短路");

    nt=task_fixture(1500,500);memset(nt->map,255,sizeof nt->map);
    CHECK(lp_plan(&nt->lp_cfg,&nt->mi,&nt->pose.cur,nt->map,false,true,1500,500,&nt->lp) &&
          nt->lp.cmd.vx>0 && nt->lp.cmd.vx<=60 && !nt->lp.cmd.vy &&
          nt->lp.cmd.wz>=0 && nt->lp.cmd.wz<=150 && nt->lp.ground_checked,
          "车体未知但轮支撑明确时，只能选受中心路程上界约束的微动，不能放行375mm/s未知弯道");
    sim_map_fixture(nt->map,&nt->mi);
    CHECK(!lp_plan(&nt->lp_cfg,&nt->mi,&nt->pose.cur,nt->map,false,true,1500,500,&nt->lp) &&
          !nt->lp.ground_checked && !nt->lp.cmd.vx && !nt->lp.cmd.wz,
          "连轮支撑也未知时，不得用慢行或原地转向绕过支撑检查");
    for(int side=-1;side<=1;side+=2) {
        nt=task_fixture(1500,side*500);
        for(int y=0;y<NP_MAP_N;y++) nt->map[y*NP_MAP_N+69]=254;
        CHECK(lp_plan(&nt->lp_cfg,&nt->mi,&nt->pose.cur,nt->map,false,true,1500,side*500,&nt->lp) &&
              !nt->lp.cmd.vx && nt->lp.cmd.wz*side>0,"平移均受阻时转向出口应朝向目标 side=%d",side);
    }
    nt=task_fixture(1500,0);nt->pose.cur.x=10;
    /* 未知支撑始于100mm；50mm/s轮投影会进入，25mm/s不会进入。
     * 原地转向可行不等于平移已经恢复，下一周期应试更低速度。 */
    for(int y=0;y<NP_MAP_N;y++) for(int x=66;x<NP_MAP_N;x++)
        nt->map[y*NP_MAP_N+x]=255;
    sim_map_fixture(nt->map,&nt->mi);
    CHECK(lp_plan(&nt->lp_cfg,&nt->mi,&nt->pose.cur,nt->map,false,true,1500,0,&nt->lp) &&
          !nt->lp.cmd.vx && nt->lp.cmd.wz,"50mm/s支撑不足时先保留可行转向");
    CHECK(lp_plan(&nt->lp_cfg,&nt->mi,&nt->pose.cur,nt->map,false,true,1500,0,&nt->lp) &&
          nt->lp.cmd.vx==25 && nt->lp.ground_checked,
          "转向成功不能重置低速探测，下一轮应找到25mm/s支撑可行接入 vx=%d",nt->lp.cmd.vx);
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
    support_reset(350); /* 本测试场景起点400mm平地由构造保证，显式声明。 */
    uint8_t b[64];
    int n = proto_build_goal(b, 4500, 0, 0);
    sim_host_inject(b, n);
    double t = 0;
    while (t < 60 && !(g_app.nt.state == ST_ARRIVED)) { run_sim(0.5); t += 0.5; }
    double d = hypot(g_sim.x - 4500, g_sim.y);
    printf("  用时 %.1f s，终点误差 %.0f mm，最小离障 %.0f mm，状态 %d\n", t, d, g_sim.min_clear_mm, g_app.nt.state);
    if (g_app.nt.state != ST_ARRIVED)
        printf("  运动诊断 src=%u check=%u cmd=%d,%d look=%ld,%ld pos=%ld,%ld path=%d/%d recover=%lu\n",
               g_app.nt.cmd_source, g_app.nt.motion_result, g_app.nt.cmd.vx, g_app.nt.cmd.wz,
               (long)g_app.nt.look_x, (long)g_app.nt.look_y, (long)g_app.nt.pose.cur.x, (long)g_app.nt.pose.cur.y,
               g_app.nt.path_valid, g_app.nt.path_complete, (unsigned long)g_app.nt.n_recover);
    if (g_app.nt.state != ST_ARRIVED) {
        nav_task_t *nt = &g_app.nt;
        int wx, wy, gx, gy;
        nav_mm_to_win(&nt->mi, nt->pose.cur.x, nt->pose.cur.y, &wx, &wy);
        gmap_mm_to_cell(&nt->gm, &nt->mi, nt->pose.cur.x, nt->pose.cur.y, &gx, &gy);
        printf("  start costs local=%u global=%u path.n=%d first=%d,%d last=%d,%d\n",
               nt->map[wy * NP_MAP_N + wx], nt->gm.grid[gy * GM_N + gx], nt->path.n,
               nt->path.wx[0], nt->path.wy[0], nt->path.wx[nt->path.n-1], nt->path.wy[nt->path.n-1]);
    }
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
    support_reset(350);
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
    support_reset(350);
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
    if(argc>1 && strcmp(argv[1],"--gmap-contract")==0) {
        test_gmap_slices();printf("GMAP CONTRACT: %d failures\n",g_fail);return g_fail?1:0;
    }
    if (argc > 1 && strcmp(argv[1], "--goto-debug") == 0) { test_goto(); return g_fail ? 1 : 0; }
    if (argc > 1 && strcmp(argv[1], "--nav-contract") == 0) {
        test_goal_contract();
        test_gmap_clearance();
        test_lookahead_contract();
        test_motion_contract();
        printf("NAV CONTRACT %s (failures %d)\n", g_fail ? "FAILED" : "PASSED", g_fail);
        return g_fail ? 1 : 0;
    }
    if (argc > 1 && strcmp(argv[1], "--lookahead-known") == 0) {
        test_lookahead_contract();
        printf("LOOKAHEAD CONTRACT: %d failures\n", g_fail);
        return g_fail ? 1 : 0;
    }
    test_fixmath();
    test_proto();
    test_pose();
    test_pose_time_contract();
    test_match_transaction();
    test_astar();
    test_goal_contract();
    test_gmap_clearance();
    test_gmap_slices();
    test_ground_scan();
    test_align_regenerated();
    test_slow_progress();
    test_lookahead_contract();
    test_motion_contract();
    test_traj_crosscheck(argc > 1 ? argv[1] : "../../sim/build/tb_traj_eval");
    test_goto();
    test_waypoints();
    test_search_track();
    test_estop();
    printf("\n%s（失败 %d）\n", g_fail ? "FW TEST FAILED" : "FW TEST PASSED", g_fail);
    return g_fail ? 1 : 0;
}
