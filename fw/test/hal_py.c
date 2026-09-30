/* HAL 的 Python 闭环仿真实现：固件编成动态库，由 sim/closed_loop 通过 ctypes 驱动。
 * Python 侧扮演 FPGA 硬件（感知链路、匹配打分，用与 RTL 逐位一致的模型）和外部世界。
 */
#include <string.h>
#include "hal.h"
#include "traj_sw.h"
#include "nav_app.h"

#define N NP_MAP_N
#define EXPORT __declspec(dllexport)

static uint32_t s_now;
static uint8_t  s_slot[N * N];
static hal_map_info_t s_mi;
static nav_cmd_t s_cmd;
static uint32_t s_ncmd;

typedef struct { int16_t vx, vy, wz; uint32_t t; } odo_t;
static odo_t    s_odo[64];
static int      s_oh, s_ot;

static uint8_t  s_imu[4096];
static uint8_t  s_imu_first[4096];
static uint32_t s_imu_t[4096];
static int      s_ih, s_it;

static bool     s_lw;
static uint32_t s_lw_t;
static bool     s_commit;
static int32_t  s_commit_v[16];

/* 雷达方向查找表：固件写入的表（仿真的点云链路用 Python 模型自己的表，这里只供对比测试） */
static int16_t  s_lut[3 * LCAL_NPIX];
static uint32_t s_lut_writes;
static uint32_t s_lens[9], s_info_seq;

typedef int (*h_cb_t)(int32_t x, int32_t y, int32_t *h);
static h_cb_t s_h_cb;
EXPORT void py_set_h_cb(h_cb_t cb) { s_h_cb = cb; }
bool hal_map_height(int32_t x, int32_t y, int32_t *h) { return s_h_cb ? s_h_cb(x, y, h) != 0 : false; }
typedef void (*mu_cb_t)(int32_t dx, int32_t dy, int32_t dyaw, int32_t cx_mm, int32_t cy_mm);
static mu_cb_t s_mu_cb;
EXPORT void py_set_mu_cb(mu_cb_t cb) { s_mu_cb = cb; }
static bool s_match_ready;
bool hal_match_ready(void) { return s_match_ready; }
void hal_match_update(int32_t dx, int32_t dy, int32_t dyaw, int32_t cx_mm, int32_t cy_mm)
{
    s_match_ready = false;
    if (s_mu_cb) s_mu_cb(dx, dy, dyaw, cx_mm, cy_mm);
}
typedef void (*match_cb_t)(int32_t cx_mm, int32_t cy_mm, const int32_t *cand, int k, uint32_t *out);
static match_cb_t s_match_cb;

static uint8_t  s_hin[16][PROTO_MAX_LEN];
static int      s_hin_len[16], s_hh, s_ht;
static uint32_t s_hin_t[16];
static uint8_t  s_hout[300];
static int      s_hout_len;
static uint32_t s_hout_n;

/* 有图模式 / 清图等控制：统一回调到 Python（op: 0 清高程图 1 先验开始 2 写槽字 3 有图开关 4 门控 5 结构点高度带） */
typedef void (*ctl_cb_t)(int op, int32_t a, int32_t b, int32_t c);
static ctl_cb_t s_ctl_cb;
EXPORT void py_set_ctl_cb(ctl_cb_t cb) { s_ctl_cb = cb; }
static void ctl(int op, int32_t a, int32_t b, int32_t c) { if (s_ctl_cb) s_ctl_cb(op, a, b, c); }
void hal_map_clear(void) { ctl(0, 0, 0, 0); }
void hal_prior_begin(void) { ctl(1, 0, 0, 0); }
void hal_prior_write(uint16_t slot, uint32_t word) { ctl(2, slot, (int32_t)word, 0); }
void hal_prior_enable(bool en, int32_t ox, int32_t oy) { ctl(3, en, ox, oy); }
void hal_match_gate(bool on) { ctl(4, on, 0, 0); }
void hal_match_zband(int32_t lo, int32_t hi) { ctl(5, lo, hi, 0); }

/* ---------------------------------------------------------------- Python 接口 */
EXPORT void py_set_time(uint32_t t) { s_now = t; }
EXPORT void py_set_map(const uint8_t *slot, int32_t cx, int32_t cy, uint16_t seq, uint16_t inv_res)
{
    memcpy(s_slot, slot, sizeof s_slot);
    s_match_ready = true;                  /* 帧已提交处理完：结构点缓冲就绪 */
    s_mi.cx = cx; s_mi.cy = cy; s_mi.seq = seq; s_mi.inv_res = inv_res;
    s_mi.res_mm = inv_res ? 65536 / inv_res : 50;
}
EXPORT void py_push_odom(int16_t vx, int16_t vy, int16_t wz, uint32_t t)
{
    int nx = (s_ot + 1) % 64;
    if (nx == s_oh) return;
    s_odo[s_ot] = (odo_t){vx, vy, wz, t};
    s_ot = nx;
}
EXPORT void py_push_imu(const uint8_t *b, int n, uint32_t t)
{
    for (int i = 0; i < n; i++) {
        int nx = (s_it + 1) % 4096;
        if (nx == s_ih) return;
        s_imu[s_it] = b[i]; s_imu_first[s_it] = (uint8_t)(i == 0); s_imu_t[s_it] = t;
        s_it = nx;
    }
}
EXPORT void py_set_lidar_waiting(uint32_t t) { s_lw = true; s_lw_t = t; s_commit = false; }
/* 返回是否已提交；v: R[9] t[3] cx cy zmin zmax */
EXPORT int py_get_commit(int32_t *v)
{
    if (!s_commit) return 0;
    memcpy(v, s_commit_v, sizeof s_commit_v);
    s_commit = false;
    return 1;
}
EXPORT void py_set_match_cb(match_cb_t cb) { s_match_cb = cb; }
EXPORT void py_push_host(const uint8_t *b, int n, uint32_t t)
{
    int nx = (s_ht + 1) % 16;
    if (nx == s_hh || n > PROTO_MAX_LEN) return;
    memcpy(s_hin[s_ht], b, (size_t)n);
    s_hin_len[s_ht] = n; s_hin_t[s_ht] = t;
    s_ht = nx;
}
EXPORT int py_get_host_out(uint8_t *b) { memcpy(b, s_hout, (size_t)s_hout_len); return s_hout_len; }
EXPORT void py_get_cmd(int16_t *v) { v[0] = s_cmd.vx; v[1] = s_cmd.vy; v[2] = s_cmd.wz; v[3] = s_cmd.mode; }
/* v: x y z yaw pitch roll mode state goal_x goal_y look_x look_y */
EXPORT void py_get_state(int32_t *v)
{
    const nav_task_t *nt = &g_app.nt;
    v[0] = nt->pose.cur.x; v[1] = nt->pose.cur.y; v[2] = nt->pose.cur.z;
    v[3] = nt->pose.cur.yaw; v[4] = (int16_t)nt->pose.cur.pitch; v[5] = (int16_t)nt->pose.cur.roll;
    v[6] = nt->mode; v[7] = nt->state; v[8] = nt->goal_x; v[9] = nt->goal_y;
    v[10] = nt->look_x; v[11] = nt->look_y;
}
/* 定位：v: loc_state quality q_last n_align n_align_fail n_lost res_q res_q2 prior_state prior_next edge_cnt n_valid */
EXPORT void py_get_loc(int32_t *v)
{
    const loc_t *l = &g_app.loc;
    v[0] = nav_app_have_prior() ? (int32_t)l->st : 0; v[1] = loc_quality(l); v[2] = l->q_last;
    v[3] = (int32_t)l->n_align; v[4] = (int32_t)l->n_align_fail; v[5] = (int32_t)l->n_lost;
    v[6] = l->res_q; v[7] = l->res_q2; v[8] = g_app.prx.state; v[9] = (int32_t)g_app.prx.next;
    v[10] = l->edge_cnt; v[11] = (int32_t)g_app.pm.n_valid;
}
/* v: applied dx dy 0 dyaw n_valid cost0 cost_best degen_x degen_y degen_yaw n_applied */
EXPORT void py_get_match(int32_t *v)
{
    const pm_stat_t *s = &g_app.pm;
    v[0] = s->applied; v[1] = s->dx; v[2] = s->dy; v[3] = 0; v[4] = s->dyaw; v[5] = (int32_t)s->n_valid;
    v[6] = (int32_t)s->cost0; v[7] = (int32_t)s->cost_best; v[8] = s->degen_x; v[9] = s->degen_y; v[10] = s->degen_yaw;
    v[11] = (int32_t)s->n_applied;
}
/* 全局记忆栅格：grid[GM_N*GM_N]，v: ox oy n ds；path: 当前 A* 路径（栅格坐标），返回点数 */
EXPORT int py_get_gmap(uint8_t *grid, int32_t *v, int16_t *path)
{
    const nav_task_t *nt = &g_app.nt;
    memcpy(grid, nt->gm.grid, sizeof nt->gm.grid);
    v[0] = nt->gm.ox; v[1] = nt->gm.oy; v[2] = GM_N; v[3] = GM_DS;
    int n = nt->path_valid ? nt->path.n : 0;
    for (int i = 0; i < n; i++) { path[2 * i] = nt->path.wx[i]; path[2 * i + 1] = nt->path.wy[i]; }
    return n;
}
EXPORT void py_get_seen(uint8_t *s) { memcpy(s, g_app.nt.exp.seen, sizeof g_app.nt.exp.seen); }
/* 调试：在给定栅格上用导航同一套参数跑 A*，返回 rc，path 写入栅格坐标 */
EXPORT int py_astar(const uint8_t *grid, int n, int sx, int sy, int gx, int gy, int16_t *path, int *np, int *expanded)
{
    static astar_path_t pth;
    int rc = astar_grid(&g_app.nt.as_cfg_g, grid, n, sx, sy, gx, gy, &pth);
    for (int i = 0; i < pth.n; i++) { path[2 * i] = pth.wx[i]; path[2 * i + 1] = pth.wy[i]; }
    *np = pth.n;
    *expanded = astar_last_expanded();
    return rc;
}
EXPORT void py_set_pose_decomp(int en) { g_app.nt.pose.decomp_pitch = en != 0; }
EXPORT void py_set_gmap_enable(int en) { g_app.nt.use_gmap = en != 0; }
EXPORT void py_set_anchor(int g) { g_app.anchor_gain = g; }
EXPORT void py_set_pm_prior(int gain_q8, int min_curv, int min_curv_ang)
{
    if (gain_q8 > 0) g_app.pm_cfg_prior.gain_q8 = gain_q8;
    if (min_curv > 0) g_app.pm_cfg_prior.min_curv = min_curv;
    if (min_curv_ang > 0) g_app.pm_cfg_prior.min_curv_ang = min_curv_ang;
}
EXPORT void py_set_match_enable(int en) { g_app.pm_cfg.enable = en != 0; }
/* 雷达安装位姿（下俯 1/100°、前伸 mm、离地 mm），覆盖 nav_task_init 里的标定值（安装方案对比用） */
EXPORT void py_set_mount(int pitch_cdeg, int x_mm, int z_mm)
{
    pose_est_t *p = &g_app.nt.pose;
    fx_rot_zyx(&p->r_bs, 0, ANGLE_FROM_CDEG(pitch_cdeg), 0);
    p->t_bs[0] = x_mm;
    p->t_bs[2] = z_mm;
}

EXPORT void py_set_caps(int holonomic, int vx_max, int wz_max, int radius)
{
    g_app.nt.lp_cfg.caps.holonomic = holonomic != 0;
    g_app.nt.lp_cfg.v_max = (int16_t)vx_max;
    g_app.nt.lp_cfg.w_max = (int16_t)wz_max;
    g_app.nt.lp_cfg.caps.radius_mm = radius;
}
EXPORT void py_set_lidar_info(const uint32_t *lens) { memcpy(s_lens, lens, sizeof s_lens); s_info_seq++; }
EXPORT int py_astar_heap_max(void) { return astar_heap_max(); }
EXPORT uint32_t py_get_lut(int16_t *out, int32_t *v)
{
    memcpy(out, s_lut, sizeof s_lut);
    v[0] = lidar_cal_ready(&g_app.lcal); v[1] = (int32_t)g_app.lcal.n_gen; v[2] = g_app.lcal.from_dev;
    return s_lut_writes;
}
/* 按给定内参（float32 原始位；全零 = 缺省）同步生成整张表 */
EXPORT void py_lut_gen(const uint32_t *lens, int cut, int flip, int16_t *out)
{
    static lidar_cal_t lc;
    lidar_cal_init(&lc);
    lc.cut = (uint8_t)cut;
    lc.flip_u = flip != 0;
    lidar_cal_start(&lc, lens, true);
    for (int i = 0; i < LCAL_NPIX; i++) lidar_cal_pixel(&lc, i, &out[3 * i]);
}

EXPORT void py_init(void)
{
    s_now = 1000; memset(&s_mi, 0, sizeof s_mi); memset(s_slot, 255, sizeof s_slot);
    s_oh = s_ot = s_ih = s_it = s_hh = s_ht = 0; s_lw = s_commit = false; s_hout_len = 0; s_ncmd = 0;
    memset(s_lut, 0, sizeof s_lut); s_lut_writes = 0; memset(s_lens, 0, sizeof s_lens); s_info_seq = 0;
    nav_app_init();
    g_app.lcal_budget = LCAL_NPIX;          /* 仿真：第一次轮询就生成整张表 */
}
EXPORT void py_poll(void) { nav_app_poll(); }

/* ---------------------------------------------------------------- HAL */
uint32_t hal_time_us(void) { return s_now; }
bool hal_map_info(hal_map_info_t *mi) { *mi = s_mi; return s_mi.seq != 0; }
uint16_t hal_map_copy(uint8_t *out, hal_map_info_t *mi)
{
    *mi = s_mi;
    for (int wy = 0; wy < N; wy++)
        for (int wx = 0; wx < N; wx++) {
            int32_t ix = s_mi.cx - N / 2 + wx, iy = s_mi.cy - N / 2 + wy;
            out[wy * N + wx] = s_slot[((iy & (N - 1)) << 7) | (ix & (N - 1))];
        }
    return s_mi.seq;
}
bool hal_traj_eval(const hal_cand_t *c, int k, int t, int32_t x0, int32_t y0, angle_t th0, uint8_t cmax, hal_traj_res_t *res)
{
    traj_eval_sw(s_slot, s_mi.cx, s_mi.cy, c, k, t, x0, y0, th0, cmax, res);
    return true;
}
void hal_send_cmd(const nav_cmd_t *cmd) { s_cmd = *cmd; s_ncmd++; }
bool hal_odom(int16_t *vx, int16_t *vy, int16_t *wz, uint32_t *t)
{
    if (s_oh == s_ot) return false;
    *vx = s_odo[s_oh].vx; *vy = s_odo[s_oh].vy; *wz = s_odo[s_oh].wz; *t = s_odo[s_oh].t;
    s_oh = (s_oh + 1) % 64;
    return true;
}
bool hal_lidar_waiting(uint32_t *t) { if (!s_lw) return false; *t = s_lw_t; return true; }
void hal_lidar_init(void) { }
uint32_t hal_lidar_info_seq(void) { return s_info_seq; }
void hal_lidar_lens(uint32_t lens[9]) { memcpy(lens, s_lens, sizeof s_lens); }
void hal_lidar_lut_write(uint32_t idx, const int16_t *rxyz, int n)
{
    for (int i = 0; i < n && idx + (uint32_t)i < LCAL_NPIX; i++)
        for (int j = 0; j < 3; j++) s_lut[3 * (idx + (uint32_t)i) + (uint32_t)j] = rxyz[3 * i + j];
    s_lut_writes += (uint32_t)n;
}
void hal_lidar_commit(const int32_t r[3][3], const int32_t t[3], int32_t cx, int32_t cy, int32_t zmin, int32_t zmax)
{
    for (int i = 0; i < 9; i++) s_commit_v[i] = r[i / 3][i % 3];
    s_commit_v[9] = t[0]; s_commit_v[10] = t[1]; s_commit_v[11] = t[2];
    s_commit_v[12] = cx; s_commit_v[13] = cy; s_commit_v[14] = zmin; s_commit_v[15] = zmax;
    s_commit = true;
    s_lw = false;
}
static hal_match_res_t s_mres[256];
static bool s_mdone;
static uint32_t s_mdone_t;
#define PY_MATCH_LAT_US 40000u      /* 硬件打分耗时（245 候选 × 1024 点 × 约 15 拍 @100 MHz） */
bool hal_match_start(int32_t cx_mm, int32_t cy_mm, const hal_match_cand_t *c, int k)
{
    if (!s_match_cb || k > 256) return false;
    static int32_t cand[3 * 256];
    static uint32_t out[4 * 256];
    for (int i = 0; i < k; i++) { cand[3 * i] = c[i].dx; cand[3 * i + 1] = c[i].dy; cand[3 * i + 2] = c[i].dyaw; }
    s_match_cb(cx_mm, cy_mm, cand, k, out);
    for (int i = 0; i < k; i++) { s_mres[i].n_valid = out[4 * i]; s_mres[i].cost = out[4 * i + 1]; s_mres[i].n_in = out[4 * i + 2]; s_mres[i].n_pts = out[4 * i + 3]; }
    s_mdone = true;
    s_mdone_t = s_now + PY_MATCH_LAT_US;
    return true;
}
bool hal_match_result(hal_match_res_t *res, int k)
{
    if (!s_mdone || (int32_t)(s_now - s_mdone_t) < 0) return false;
    memcpy(res, s_mres, sizeof(hal_match_res_t) * (size_t)k);
    s_mdone = false;
    return true;
}
bool hal_imu_byte(uint8_t *b, bool *first, uint32_t *t)
{
    if (s_ih == s_it) return false;
    *b = s_imu[s_ih]; *first = s_imu_first[s_ih] != 0; *t = s_imu_t[s_ih];
    s_ih = (s_ih + 1) % 4096;
    return true;
}
int hal_host_recv(uint8_t *buf, int maxlen, uint32_t *t)
{
    if (s_hh == s_ht) return 0;
    int n = s_hin_len[s_hh] < maxlen ? s_hin_len[s_hh] : maxlen;
    memcpy(buf, s_hin[s_hh], (size_t)n);
    *t = s_hin_t[s_hh];
    s_hh = (s_hh + 1) % 16;
    return n;
}
bool hal_host_send(const uint8_t *buf, int len)
{
    s_hout_len = len < 300 ? len : 300;
    memcpy(s_hout, buf, (size_t)s_hout_len);
    s_hout_n++;
    return true;
}
