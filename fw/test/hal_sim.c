#include <math.h>
#include <string.h>
#include "hal_sim.h"
#include "traj_sw.h"
#include "proto.h"

#define N NP_MAP_N
#define RES_MM 50
#define INV_RES 1311
#define SEEN_N 1024                 /* 可见记忆：1024×1024 世界栅格（51 m），原点居中 */

sim_t g_sim;

static uint8_t  s_slot[N * N];      /* 槽地址代价图 */
static uint8_t  s_seen[SEEN_N * SEEN_N / 8];
static hal_map_info_t s_mi;
static uint32_t s_t_map, s_t_odo, s_t_cam;
static bool     s_odo_new;
static int16_t  s_odo_vx, s_odo_vy, s_odo_wz;
static uint32_t s_odo_t;
/* 主机报文队列 */
static uint8_t  s_q[16][300];
static int      s_qlen[16], s_qh, s_qt;
static uint32_t s_qts[16];

void sim_reset(void)
{
    memset(&g_sim, 0, sizeof g_sim);
    memset(s_seen, 0, sizeof s_seen);
    memset(s_slot, 255, sizeof s_slot);
    memset(&s_mi, 0, sizeof s_mi);
    s_t_map = s_t_odo = s_t_cam = 0;
    s_odo_new = false;
    s_qh = s_qt = 0;
    g_sim.sense_range_mm = 4000;
    g_sim.fov_cdeg = 12000;
    g_sim.cam_range_mm = 4000;
    g_sim.cam_fov_cdeg = 7000;
    g_sim.infl_cells = 5;          /* 25 cm：底盘半径 */
    g_sim.robot_r_mm = 250;
    g_sim.min_clear_mm = 1e9;
    g_sim.now_us = 1000;
}

void sim_add_obs(int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
    if (g_sim.n_obs < SIM_MAX_OBS)
        g_sim.obs[g_sim.n_obs++] = (sim_rect_t){x0 < x1 ? x0 : x1, y0 < y1 ? y0 : y1, x0 < x1 ? x1 : x0, y0 < y1 ? y1 : y0};
}

double sim_clearance(double x, double y)
{
    double best = 1e9;
    for (int i = 0; i < g_sim.n_obs; i++) {
        const sim_rect_t *r = &g_sim.obs[i];
        double dx = x < r->x0 ? r->x0 - x : (x > r->x1 ? x - r->x1 : 0);
        double dy = y < r->y0 ? r->y0 - y : (y > r->y1 ? y - r->y1 : 0);
        double d = sqrt(dx * dx + dy * dy);
        if (d < best) best = d;
    }
    return best;
}

static int32_t cell_of(double mm) { return (int32_t)floor(mm / RES_MM); }

static bool cell_occ(int32_t ix, int32_t iy)
{
    double x = (ix + 0.5) * RES_MM, y = (iy + 0.5) * RES_MM;
    for (int i = 0; i < g_sim.n_obs; i++) {
        const sim_rect_t *r = &g_sim.obs[i];
        if (x >= r->x0 && x <= r->x1 && y >= r->y0 && y <= r->y1) return true;
    }
    return false;
}

static int seen_idx(int32_t ix, int32_t iy)
{
    int32_t a = ix + SEEN_N / 2, b = iy + SEEN_N / 2;
    if (a < 0 || b < 0 || a >= SEEN_N || b >= SEEN_N) return -1;
    return b * SEEN_N + a;
}

/* 可见性：视场内、距离内，射线不穿过障碍（被遮挡的格不可见） */
static void update_seen(void)
{
    int32_t rx = cell_of(g_sim.x), ry = cell_of(g_sim.y);
    int R = g_sim.sense_range_mm / RES_MM;
    double half = g_sim.fov_cdeg / 100.0 / 2 * M_PI / 180;
    for (int dy = -R; dy <= R; dy++)
        for (int dx = -R; dx <= R; dx++) {
            if (dx * dx + dy * dy > R * R) continue;
            double ang = atan2(dy, dx) - g_sim.yaw;
            while (ang > M_PI) ang -= 2 * M_PI;
            while (ang < -M_PI) ang += 2 * M_PI;
            if (fabs(ang) > half && (dx * dx + dy * dy) > 16) continue;      /* 近身 20 cm 总可见（补盲 ToF） */
            /* 射线检查 */
            int steps = (int)sqrt(dx * dx + dy * dy);
            bool blocked = false;
            for (int s = 1; s < steps; s++) {
                int32_t cx = rx + dx * s / steps, cy = ry + dy * s / steps;
                if (cell_occ(cx, cy)) { blocked = true; break; }
            }
            if (blocked) continue;
            int i = seen_idx(rx + dx, ry + dy);
            if (i >= 0) s_seen[i >> 3] |= (uint8_t)(1 << (i & 7));
        }
}

static void build_map(void)
{
    int32_t cx = cell_of(g_sim.x), cy = cell_of(g_sim.y);
    int K = g_sim.infl_cells;
    for (int wy = 0; wy < N; wy++)
        for (int wx = 0; wx < N; wx++) {
            int32_t ix = cx - N / 2 + wx, iy = cy - N / 2 + wy;
            int si = seen_idx(ix, iy);
            uint8_t c = 255;
            if (si >= 0 && (s_seen[si >> 3] & (1 << (si & 7)))) {
                if (cell_occ(ix, iy)) c = 254;
                else {
                    c = 0;
                    for (int j = -K; j <= K && c == 0; j++)
                        for (int i = -K; i <= K; i++)
                            if (cell_occ(ix + i, iy + j)) { c = 253; break; }
                }
            }
            s_slot[((iy & (N - 1)) << 7) | (ix & (N - 1))] = c;
        }
    s_mi.cx = cx;
    s_mi.cy = cy;
    s_mi.inv_res = INV_RES;
    s_mi.res_mm = RES_MM;
    s_mi.seq++;
}

static void cam_detect(void)
{
    if (!g_sim.target_on) return;
    double dx = g_sim.tgt_x - g_sim.x, dy = g_sim.tgt_y - g_sim.y;
    double d = sqrt(dx * dx + dy * dy);
    double ang = atan2(dy, dx) - g_sim.yaw;
    while (ang > M_PI) ang -= 2 * M_PI;
    while (ang < -M_PI) ang += 2 * M_PI;
    bool vis = d < g_sim.cam_range_mm && fabs(ang) < g_sim.cam_fov_cdeg / 200.0 * M_PI / 180;
    /* 视线遮挡 */
    for (int s = 1; vis && s < 50; s++) {
        double px = g_sim.x + dx * s / 50, py = g_sim.y + dy * s / 50;
        if (cell_occ(cell_of(px), cell_of(py))) vis = false;
    }
    if (!vis) return;
    /* 机身系坐标 */
    double bx = cos(g_sim.yaw) * dx + sin(g_sim.yaw) * dy;
    double by = -sin(g_sim.yaw) * dx + cos(g_sim.yaw) * dy;
    uint8_t buf[64];
    int n = proto_build_target(buf, 1, 39, 900, (int32_t)bx, (int32_t)by, 300, 0, 7);
    sim_host_inject(buf, n);
}

void sim_advance(uint32_t dt_us)
{
    for (uint32_t t = 0; t < dt_us;) {
        uint32_t step = (dt_us - t) < 5000 ? (dt_us - t) : 5000;
        t += step;
        double dt = step * 1e-6;
        g_sim.yaw += g_sim.cmd.wz * 1e-3 * dt;
        g_sim.x += (g_sim.cmd.vx * cos(g_sim.yaw) - g_sim.cmd.vy * sin(g_sim.yaw)) * dt;
        g_sim.y += (g_sim.cmd.vx * sin(g_sim.yaw) + g_sim.cmd.vy * cos(g_sim.yaw)) * dt;
        g_sim.now_us += step;
        double c = sim_clearance(g_sim.x, g_sim.y);
        if (c < g_sim.min_clear_mm) g_sim.min_clear_mm = c;
        if ((uint32_t)(g_sim.now_us - s_t_odo) >= 20000) {        /* 底盘回传 50 Hz */
            s_t_odo = g_sim.now_us;
            s_odo_new = true;
            s_odo_vx = g_sim.cmd.vx; s_odo_vy = g_sim.cmd.vy; s_odo_wz = g_sim.cmd.wz;
            s_odo_t = g_sim.now_us;
        }
        if ((uint32_t)(g_sim.now_us - s_t_map) >= 100000) {       /* 地图 10 Hz */
            s_t_map = g_sim.now_us;
            update_seen();
            build_map();
        }
        if ((uint32_t)(g_sim.now_us - s_t_cam) >= 100000) {       /* 视觉 10 Hz */
            s_t_cam = g_sim.now_us;
            cam_detect();
        }
    }
}

void sim_host_inject(const uint8_t *buf, int len)
{
    int nx = (s_qt + 1) % 16;
    if (nx == s_qh) return;
    memcpy(s_q[s_qt], buf, (size_t)len);
    s_qlen[s_qt] = len;
    s_qts[s_qt] = g_sim.now_us;
    s_qt = nx;
}

/* ---------------------------------------------------------------- HAL */
uint32_t hal_time_us(void) { return g_sim.now_us; }

bool hal_map_info(hal_map_info_t *mi)
{
    *mi = s_mi;
    return s_mi.seq != 0;
}

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

bool hal_traj_eval(const hal_cand_t *c, int k, int t, int32_t x0, int32_t y0, angle_t th0,
                   uint8_t cmax, hal_traj_res_t *res)
{
    traj_eval_sw(s_slot, s_mi.cx, s_mi.cy, c, k, t, x0, y0, th0, cmax, res);
    return true;
}

void hal_send_cmd(const nav_cmd_t *cmd)
{
    g_sim.cmd = *cmd;
    g_sim.n_cmds++;
}

bool hal_odom(int16_t *vx, int16_t *vy, int16_t *wz, uint32_t *t_us)
{
    if (!s_odo_new) return false;
    s_odo_new = false;
    *vx = s_odo_vx; *vy = s_odo_vy; *wz = s_odo_wz; *t_us = s_odo_t;
    return true;
}

bool hal_lidar_waiting(uint32_t *t) { (void)t; return false; }
void hal_lidar_init(void) { }
uint32_t hal_lidar_info_seq(void) { return 0; }
void hal_lidar_lens(uint32_t lens[9]) { for (int i = 0; i < 9; i++) lens[i] = 0; }
void hal_lidar_lut_write(uint32_t idx, const int16_t *rxyz, int n) { (void)idx; (void)rxyz; (void)n; }
void hal_lidar_commit(const int32_t r[3][3], const int32_t t[3], int32_t cx, int32_t cy, int32_t zmin, int32_t zmax)
{ (void)r; (void)t; (void)cx; (void)cy; (void)zmin; (void)zmax; }
bool hal_match_ready(void) { return true; }
bool hal_match_start(int32_t cx_mm, int32_t cy_mm, const hal_match_cand_t *c, int k)
{ (void)cx_mm; (void)cy_mm; (void)c; (void)k; return false; }
bool hal_match_result(hal_match_res_t *res, int k) { (void)res; (void)k; return false; }
void hal_match_update(int32_t dx, int32_t dy, int32_t dyaw, int32_t cx_mm, int32_t cy_mm)
{ (void)dx; (void)dy; (void)dyaw; (void)cx_mm; (void)cy_mm; }
bool hal_map_height(int32_t x, int32_t y, int32_t *h) { (void)x; (void)y; (void)h; return false; }
void hal_map_clear(void) { }
void hal_prior_begin(void) { }
void hal_prior_write(uint16_t slot, uint32_t word) { (void)slot; (void)word; }
void hal_prior_enable(bool en, int32_t ox, int32_t oy) { (void)en; (void)ox; (void)oy; }
void hal_match_gate(bool on) { (void)on; }
void hal_match_zband(int32_t lo, int32_t hi) { (void)lo; (void)hi; }
bool hal_imu_byte(uint8_t *b, bool *f, uint32_t *t) { (void)b; (void)f; (void)t; return false; }

int hal_host_recv(uint8_t *buf, int maxlen, uint32_t *t_us)
{
    if (s_qh == s_qt) return 0;
    int n = s_qlen[s_qh] < maxlen ? s_qlen[s_qh] : maxlen;
    memcpy(buf, s_q[s_qh], (size_t)n);
    *t_us = s_qts[s_qh];
    s_qh = (s_qh + 1) % 16;
    return n;
}

bool hal_host_send(const uint8_t *buf, int len)
{
    g_sim.n_status++;
    g_sim.last_status_len = len < 64 ? len : 64;
    memcpy(g_sim.last_status, buf, (size_t)g_sim.last_status_len);
    return true;
}
