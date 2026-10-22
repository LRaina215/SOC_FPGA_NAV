/* HAL 的上板实现：通过软核总线访问 nav_subsystem 寄存器（地址见 fw/include/nav_regs_map.h）。
 * 软核确定后只需：定义 NAV_BASE（导航子系统在软核地址空间的基址），必要时替换 nav_rd/nav_wr。
 */
#include "hal.h"
#include "nav_regs_map.h"

#ifndef NAV_BASE
#define NAV_BASE 0x40000000u
#endif

static inline uint32_t nav_rd(uint32_t off) { return *(volatile uint32_t *)(uintptr_t)(NAV_BASE + off); }
static inline void nav_wr(uint32_t off, uint32_t v) { *(volatile uint32_t *)(uintptr_t)(NAV_BASE + off) = v; }

#define FIELD(v, reg, f) (((v) & NAV_##reg##_##f##_MASK) >> NAV_##reg##_##f##_SHIFT)

uint32_t hal_time_us(void) { return nav_rd(NAV_SYS_TIME_US); }

bool hal_map_info(hal_map_info_t *mi)
{
    uint32_t st = nav_rd(NAV_MAP_STAT);
    mi->seq = (uint16_t)FIELD(st, MAP_STAT, SEQ);
    mi->cx = (int32_t)nav_rd(NAV_MAP_LAT_CX);
    mi->cy = (int32_t)nav_rd(NAV_MAP_LAT_CY);
    mi->inv_res = (uint16_t)nav_rd(NAV_MAP_INV_RES);
    mi->res_mm = mi->inv_res ? (int32_t)(65536 / mi->inv_res) : 50;
    return mi->seq != 0;
}

uint16_t hal_map_copy(uint8_t *out, hal_map_info_t *mi)
{
    const int N = HAL_MAP_N;
    hal_map_info(mi);
    /* 窗口坐标 wx 对应槽 sx = (cx - N/2 + wx) & (N-1)；按行读出 32 位字（4 格）再重排 */
    int sx0 = (mi->cx - N / 2) & (N - 1);
    int sy0 = (mi->cy - N / 2) & (N - 1);
    static uint8_t row[NP_MAP_N];
    for (int wy = 0; wy < N; wy++) {
        int sy = (sy0 + wy) & (N - 1);
        for (int w = 0; w < N / 4; w++) {
            uint32_t v = nav_rd(NAV_WIN_COSTMAP_BASE + (uint32_t)(sy * N + w * 4));
            row[w * 4 + 0] = (uint8_t)v;
            row[w * 4 + 1] = (uint8_t)(v >> 8);
            row[w * 4 + 2] = (uint8_t)(v >> 16);
            row[w * 4 + 3] = (uint8_t)(v >> 24);
        }
        for (int wx = 0; wx < N; wx++) out[wy * N + wx] = row[(sx0 + wx) & (N - 1)];
    }
    /* 切图时本次复制无效；由下一规划周期重取，不递归占栈或混用两代格。 */
    hal_map_info_t after;
    if (!hal_map_info(&after) || after.seq != mi->seq || after.cx != mi->cx ||
        after.cy != mi->cy || after.inv_res != mi->inv_res) return 0;
    return mi->seq;
}

bool hal_traj_eval(const hal_cand_t *c, int k, int t, int32_t x0_q8, int32_t y0_q8, angle_t th0,
                   uint8_t cmax, hal_traj_res_t *res)
{
    for (int i = 0; i < k; i++) {
        nav_wr(NAV_WIN_TRAJ_CAND_BASE + 8u * i, ((uint32_t)(uint16_t)c[i].dvy << 16) | (uint16_t)c[i].dvx);
        nav_wr(NAV_WIN_TRAJ_CAND_BASE + 8u * i + 4, (uint16_t)c[i].dth);
    }
    nav_wr(NAV_TRAJ_X0, (uint32_t)x0_q8);
    nav_wr(NAV_TRAJ_Y0, (uint32_t)y0_q8);
    nav_wr(NAV_TRAJ_TH0, th0);
    nav_wr(NAV_TRAJ_CFG, ((uint32_t)t << 8) | (uint32_t)k);
    nav_wr(NAV_TRAJ_CMAX, cmax);
    nav_wr(NAV_TRAJ_CMD, 1);
    uint32_t st;
    do st = nav_rd(NAV_TRAJ_STAT); while (!(st & NAV_TRAJ_STAT_DONE_MASK));
    for (int i = 0; i < k; i++) {
        uint32_t w0 = nav_rd(NAV_WIN_TRAJ_RES_BASE + 16u * i);
        res[i].max_cost  = (uint8_t)w0;
        res[i].n_unknown = (uint8_t)(w0 >> 8);
        res[i].coll_step = (uint8_t)(w0 >> 16);
        res[i].collide   = (w0 >> 24) & 1;
        res[i].left      = (w0 >> 25) & 1;
        res[i].sum_cost  = nav_rd(NAV_WIN_TRAJ_RES_BASE + 16u * i + 4);
        res[i].end_x     = (int32_t)nav_rd(NAV_WIN_TRAJ_RES_BASE + 16u * i + 8);
        res[i].end_y     = (int32_t)nav_rd(NAV_WIN_TRAJ_RES_BASE + 16u * i + 12);
    }
    return !(st & NAV_TRAJ_STAT_STALE_MASK);
}

void hal_send_cmd(const nav_cmd_t *cmd)
{
    nav_wr(NAV_SAFE_CMD_V, ((uint32_t)(uint16_t)cmd->vy << 16) | (uint16_t)cmd->vx);
    nav_wr(NAV_SAFE_CMD_W, ((uint32_t)cmd->mode << 16) | (uint16_t)cmd->wz);   /* 写 CMD_W 即提交 */
}

bool hal_odom(int16_t *vx, int16_t *vy, int16_t *wz, uint32_t *t_us)
{
    static uint32_t last_cnt;
    uint32_t cnt = nav_rd(NAV_SAFE_ODO_CNT);
    if (cnt == last_cnt) return false;
    last_cnt = cnt;
    uint32_t v = nav_rd(NAV_SAFE_ODO_V);
    *vx = (int16_t)v;
    *vy = (int16_t)(v >> 16);
    *wz = (int16_t)nav_rd(NAV_SAFE_ODO_W);
    *t_us = nav_rd(NAV_SAFE_ODO_TS);
    return true;
}

bool hal_lidar_waiting(uint32_t *frame_t_us)
{
    if (!(nav_rd(NAV_LIDAR_POSE_STAT) & NAV_LIDAR_POSE_STAT_WAITING_MASK)) return false;
    *frame_t_us = nav_rd(NAV_LIDAR_LAST_TS);
    return true;
}

void hal_lidar_init(void)
{
    nav_wr(NAV_LIDAR_CTRL, nav_rd(NAV_LIDAR_CTRL) | NAV_LIDAR_CTRL_WAIT_POSE_MASK);
}

void hal_lidar_stage(int stage)
{
    uint32_t v = nav_rd(NAV_LIDAR_CTRL) & ~(NAV_LIDAR_CTRL_MATCH_ONLY_MASK | NAV_LIDAR_CTRL_FUSE_ONLY_MASK);
    if (stage == HAL_LIDAR_MATCH_ONLY) v |= NAV_LIDAR_CTRL_MATCH_ONLY_MASK | NAV_LIDAR_CTRL_WAIT_POSE_MASK;
    if (stage == HAL_LIDAR_FUSE_ONLY) v |= NAV_LIDAR_CTRL_FUSE_ONLY_MASK;
    nav_wr(NAV_LIDAR_CTRL, v);
}

uint32_t hal_lidar_frames(void) { return nav_rd(NAV_LIDAR_STAT_FRAMES); }

void hal_netl_cfg(hal_netl_cfg_t *c)
{
    uint32_t lo = nav_rd(NAV_NETL_MAC_LO), hi = nav_rd(NAV_NETL_MAC_HI);
    c->mac[0] = (uint8_t)(hi >> 8);  c->mac[1] = (uint8_t)hi;
    c->mac[2] = (uint8_t)(lo >> 24); c->mac[3] = (uint8_t)(lo >> 16);
    c->mac[4] = (uint8_t)(lo >> 8);  c->mac[5] = (uint8_t)lo;
    c->ip = nav_rd(NAV_NETL_IP);
    c->lidar_ip = nav_rd(NAV_NETL_LIDAR_IP);
    c->tcp_en = (nav_rd(NAV_NETL_CTRL) & NAV_NETL_CTRL_LIDAR_TCP_MASK) != 0;
}

bool hal_lraw_send(const uint8_t *frame, int len)
{
    if (len <= 0 || len > (int)NAV_WIN_LRAW_TX_SIZE) return false;
    uint32_t t0 = hal_time_us();                  /* 发送缓冲只有一份：等上一帧发完（128 字节约 12 us） */
    while (nav_rd(NAV_NETL_RAW_STAT) & NAV_NETL_RAW_STAT_TX_BUSY_MASK)
        if (hal_time_us() - t0 > 200u) return false;
    for (int i = 0; i < len; i += 4) {
        uint32_t w = 0;
        for (int k = 0; k < 4 && i + k < len; k++) w |= (uint32_t)frame[i + k] << (8 * k);
        nav_wr(NAV_WIN_LRAW_TX_BASE + (uint32_t)i, w);
    }
    nav_wr(NAV_NETL_RAW_TX, (uint32_t)len);
    return true;
}

int hal_lraw_recv(uint8_t *buf, int max)
{
    uint32_t st = nav_rd(NAV_NETL_RAW_STAT);
    if (!(st & NAV_NETL_RAW_STAT_RX_N_MASK)) return 0;
    int len = (int)((st & NAV_NETL_RAW_STAT_RX_LEN_MASK) >> NAV_NETL_RAW_STAT_RX_LEN_SHIFT);
    int n = len < max ? len : max;
    if (n > (int)NAV_WIN_LRAW_RX_SIZE) n = (int)NAV_WIN_LRAW_RX_SIZE;
    for (int i = 0; i < n; i += 4) {
        uint32_t w = nav_rd(NAV_WIN_LRAW_RX_BASE + (uint32_t)i);
        for (int k = 0; k < 4 && i + k < n; k++) buf[i + k] = (uint8_t)(w >> (8 * k));
    }
    nav_wr(NAV_NETL_RAW_POP, 1);
    return len;
}

uint32_t hal_lidar_info_seq(void) { return nav_rd(NAV_LIDAR_INFO_SEQ); }

void hal_lidar_lens(uint32_t lens[9])
{
    for (uint32_t i = 0; i < 9; i++) lens[i] = nav_rd(NAV_LIDAR_LENS0 + 4u * i);
}

void hal_lidar_lut_write(uint32_t idx, const int16_t *rxyz, int n)
{
    nav_wr(NAV_LIDAR_LUT_ADDR, idx);
    for (int i = 0; i < n; i++, rxyz += 3) {
        nav_wr(NAV_LIDAR_LUT_XY, ((uint32_t)rxyz[0] & 0xFFFu) | (((uint32_t)rxyz[1] & 0xFFFu) << 12));
        nav_wr(NAV_LIDAR_LUT_Z, (uint32_t)rxyz[2] & 0xFFFu);
    }
}

/* SLF 匹配（MATCH 寄存器块）：提交帧时硬件已提取结构点；打分约 38 ms（245 候选 × 1024 点），并图 + 建似然场约 1 ms */
bool hal_match_ready(void)
{
    return (nav_rd(NAV_MATCH_STAT) & NAV_MATCH_STAT_PTS_READY_MASK) != 0;
}

bool hal_match_start(int32_t cx_mm, int32_t cy_mm, const hal_match_cand_t *c, int k)
{
    if (!(nav_rd(NAV_MATCH_CTRL) & NAV_MATCH_CTRL_ENABLE_MASK) || k > 256) return false;
    nav_wr(NAV_MATCH_CX_MM, (uint32_t)cx_mm);
    nav_wr(NAV_MATCH_CY_MM, (uint32_t)cy_mm);
    for (int i = 0; i < k; i++) {
        nav_wr(NAV_WIN_MATCH_CAND_BASE + 8u * (uint32_t)i, ((uint32_t)(uint16_t)c[i].dy << 16) | (uint16_t)c[i].dx);
        nav_wr(NAV_WIN_MATCH_CAND_BASE + 8u * (uint32_t)i + 4u, (uint16_t)c[i].dyaw);
    }
    nav_wr(NAV_MATCH_NCAND, (uint32_t)k);
    nav_wr(NAV_MATCH_CMD, NAV_MATCH_CMD_EVAL_MASK);
    return true;
}

bool hal_match_result(hal_match_res_t *res, int k)
{
    uint32_t st = nav_rd(NAV_MATCH_STAT);
    if (!(st & NAV_MATCH_STAT_EVAL_DONE_MASK)) return false;
    /* 抽样后的点数：步长 ceil(存量/1024)，与硬件抽样一致 */
    uint32_t np = FIELD(st, MATCH_STAT, NPTS), stride = (np + 1023) / 1024;
    uint32_t ns = stride ? (np + stride - 1) / stride : 0;
    for (int i = 0; i < k; i++) {
        res[i].n_pts = ns;
        uint32_t w1 = nav_rd(NAV_WIN_MATCH_RES_BASE + 8u * (uint32_t)i + 4u);
        res[i].cost = nav_rd(NAV_WIN_MATCH_RES_BASE + 8u * (uint32_t)i);
        res[i].n_valid = w1 & 0xFFFFu;
        res[i].n_in = w1 >> 16;
    }
    return true;
}

void hal_match_update(int32_t dx, int32_t dy, int32_t dyaw, int32_t cx_mm, int32_t cy_mm)
{
    nav_wr(NAV_MATCH_CX_MM, (uint32_t)cx_mm);
    nav_wr(NAV_MATCH_CY_MM, (uint32_t)cy_mm);
    nav_wr(NAV_MATCH_UPD_DX, (uint32_t)dx);
    nav_wr(NAV_MATCH_UPD_DY, (uint32_t)dy);
    nav_wr(NAV_MATCH_UPD_DYAW, (uint32_t)(uint16_t)dyaw);
    nav_wr(NAV_MATCH_CMD, NAV_MATCH_CMD_UPDATE_MASK);
    /* 等并图与建似然场结束（约 1 ms），保证下一帧提取结构点前点缓冲已用完 */
    uint32_t t0 = hal_time_us();
    while ((nav_rd(NAV_MATCH_STAT) & NAV_MATCH_STAT_BUSY_MASK) && (uint32_t)(hal_time_us() - t0) < 5000) { }
}

static void match_wait_idle(uint32_t max_us)
{
    uint32_t t0 = hal_time_us();
    while ((nav_rd(NAV_MATCH_STAT) & NAV_MATCH_STAT_BUSY_MASK) && (uint32_t)(hal_time_us() - t0) < max_us) { }
}

static void match_ctrl(uint32_t set, uint32_t clr)
{
    nav_wr(NAV_MATCH_CTRL, (nav_rd(NAV_MATCH_CTRL) | set) & ~clr);
}

void hal_prior_begin(void)
{
    match_wait_idle(50000);
    match_ctrl(0, NAV_MATCH_CTRL_PRIOR_MASK);
    nav_wr(NAV_MATCH_CMD, NAV_MATCH_CMD_CLEAR_MASK);           /* 清结构图（64K 字，约 0.7 ms） */
    match_wait_idle(5000);
}

void hal_prior_write(uint16_t slot, uint32_t word)
{
    nav_wr(NAV_MATCH_SM_ADDR, slot);
    nav_wr(NAV_MATCH_SM_WDATA, word & 0x3FFFFu);
}

void hal_prior_enable(bool en, int32_t ox, int32_t oy)
{
    match_wait_idle(50000);
    if (en) {
        nav_wr(NAV_MATCH_LFO_X, (uint32_t)ox);
        nav_wr(NAV_MATCH_LFO_Y, (uint32_t)oy);
        match_ctrl(NAV_MATCH_CTRL_PRIOR_MASK, 0);
    } else {
        match_ctrl(0, NAV_MATCH_CTRL_PRIOR_MASK);
        nav_wr(NAV_MATCH_CMD, NAV_MATCH_CMD_CLEAR_MASK);
        match_wait_idle(5000);
    }
}

void hal_match_gate(bool on)
{
    match_ctrl(on ? 0 : NAV_MATCH_CTRL_NOGATE_MASK, on ? NAV_MATCH_CTRL_NOGATE_MASK : 0);
}

void hal_match_zband(int32_t lo, int32_t hi)
{
    nav_wr(NAV_MATCH_ZREL, ((uint32_t)(uint16_t)hi << 16) | (uint16_t)lo);
}

void hal_map_clear(void)
{
    nav_wr(NAV_MAP_CMD, NAV_MAP_CMD_CLEAR_MASK);
}

bool hal_map_height(int32_t x_mm, int32_t y_mm, int32_t *h_mm)
{
    hal_support_t s;
    if(!hal_map_support(x_mm,y_mm,&s) || s.retained_stale ||
       s.age>(nav_rd(NAV_MAP_VALID_CFG)&255) || (int32_t)s.hi-s.lo>40) return false;
    *h_mm=((int32_t)s.lo+s.hi)/2;
    return true;
}

bool hal_map_support(int32_t x_mm,int32_t y_mm,hal_support_t *out)
{
    const int32_t N = HAL_MAP_N, LOG2N = NP_MAP_LOG2N;
    hal_map_info_t before,after;
    if(!hal_map_info(&before) || (nav_rd(NAV_MAP_STAT)&NAV_MAP_STAT_BUSY_MASK)) return false;
    uint32_t fc=nav_rd(NAV_MAP_FRAME_CNT);
    uint32_t inv = nav_rd(NAV_MAP_INV_RES) & 0xFFFFu;
    int32_t ix = (int32_t)(((int64_t)x_mm * inv) >> 16);
    int32_t iy = (int32_t)(((int64_t)y_mm * inv) >> 16);
    if(ix<before.cx-N/2 || iy<before.cy-N/2 || ix>=before.cx+N/2 || iy>=before.cy+N/2) return false;
    uint32_t slot = ((uint32_t)(iy & (N - 1)) << LOG2N) | (uint32_t)(ix & (N - 1));
    uint32_t tag = ((uint32_t)((iy >> LOG2N) & 15) << 4) | (uint32_t)((ix >> LOG2N) & 15);
    nav_wr(NAV_MAP_ELEV_ADDR, slot);
    nav_wr(NAV_MAP_ELEV_CMD, NAV_MAP_ELEV_CMD_READ_MASK);
    uint32_t q1 = 0;
    for (int i = 0; i < 64 && !((q1 = nav_rd(NAV_MAP_ELEV_Q1)) & NAV_MAP_ELEV_Q1_DONE_MASK); i++) { }
    if (!(q1 & NAV_MAP_ELEV_Q1_DONE_MASK)) return false;          /* 地图正忙（扫描中） */
    if (FIELD(q1, MAP_ELEV_Q1, CNT) == 0 || FIELD(q1, MAP_ELEV_Q1, TAG) != tag) return false;
    uint32_t q0 = nav_rd(NAV_MAP_ELEV_Q0);
    int32_t zmax = (int16_t)(q0 & 0xFFFFu), zmin = (int16_t)(q0 >> 16);
    if(!hal_map_info(&after) || before.seq!=after.seq || before.cx!=after.cx ||
       before.cy!=after.cy || before.inv_res!=after.inv_res ||
       fc!=nav_rd(NAV_MAP_FRAME_CNT) || (nav_rd(NAV_MAP_STAT)&NAV_MAP_STAT_BUSY_MASK)) return false;
    *out=(hal_support_t){.lo=(int16_t)zmin,.hi=(int16_t)zmax,.seq=before.seq,
        .count=(uint8_t)FIELD(q1,MAP_ELEV_Q1,CNT),
        .age=(uint8_t)(fc-FIELD(q1,MAP_ELEV_Q1,STAMP)),
        .retained_stale=(q1&NAV_MAP_ELEV_Q1_RETAINED_STALE_MASK)!=0};
    return true;
}

void hal_lidar_commit(const int32_t r[3][3], const int32_t t[3], int32_t cx, int32_t cy, int32_t zmin, int32_t zmax)
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) nav_wr(NAV_LIDAR_R00 + 4u * (3 * i + j), (uint32_t)(uint16_t)(int16_t)r[i][j]);
    nav_wr(NAV_LIDAR_TX, (uint32_t)t[0]);
    nav_wr(NAV_LIDAR_TY, (uint32_t)t[1]);
    nav_wr(NAV_LIDAR_TZ, (uint32_t)t[2]);
    nav_wr(NAV_MAP_CENTER_X, (uint32_t)cx);
    nav_wr(NAV_MAP_CENTER_Y, (uint32_t)cy);
    zmin = nav_clamp32(zmin, -32768, 32767);
    zmax = nav_clamp32(zmax, -32768, 32767);
    nav_wr(NAV_LIDAR_ZBAND, ((uint32_t)(uint16_t)zmax << 16) | (uint16_t)zmin);
    nav_wr(NAV_LIDAR_POSE_COMMIT, 1);
}

bool hal_imu_byte(uint8_t *b, bool *first, uint32_t *t_us)
{
    uint32_t v = nav_rd(NAV_IMU_RX_DATA);           /* 读取即弹出 */
    if (!(v & NAV_IMU_RX_DATA_VALID_MASK)) return false;
    *b = (uint8_t)v;
    *first = (v & NAV_IMU_RX_DATA_FIRST_MASK) != 0;
    if (*first) *t_us = nav_rd(NAV_IMU_RX_TS);      /* 每个突发一个时间戳，同样读取即弹出 */
    return true;
}

int hal_host_recv(uint8_t *buf, int maxlen, uint32_t *t_us)
{
    uint32_t st = nav_rd(NAV_MBX_RX_STAT);
    if (!(st & NAV_MBX_RX_STAT_AVAIL_MASK)) return 0;
    int len = (int)FIELD(st, MBX_RX_STAT, LEN);
    *t_us = nav_rd(NAV_MBX_RX_TS);
    for (int i = 0; i < len; i += 4) {
        uint32_t w = nav_rd(NAV_MBX_RX_DATA);
        for (int k = 0; k < 4 && i + k < len; k++)
            if (i + k < maxlen) buf[i + k] = (uint8_t)(w >> (8 * k));
    }
    nav_wr(NAV_MBX_RX_POP, 1);
    return len < maxlen ? len : maxlen;
}

bool hal_host_send(const uint8_t *buf, int len)
{
    if (nav_rd(NAV_MBX_TX_STAT) & NAV_MBX_TX_STAT_BUSY_MASK) return false;
    for (int i = 0; i < len; i += 4) {
        uint32_t w = 0;
        for (int k = 0; k < 4 && i + k < len; k++) w |= (uint32_t)buf[i + k] << (8 * k);
        nav_wr(NAV_MBX_TX_DATA, w);
    }
    nav_wr(NAV_MBX_TX_LEN, (uint32_t)len);
    nav_wr(NAV_MBX_TX_GO, 1);
    return true;
}
