#include "lidar_cal.h"
#include "hal.h"

#include <string.h>

#define QB     24
#define ONE    (1 << QB)
#define LCAL_ITERS  4               /* 不动点迭代（给初值） */
#define LCAL_NEWTON 3               /* 牛顿步 */
#define LCAL_CHUNK  16

/* fx fy cx cy k1 k2 k3 p1 p2 = 78.212524 79.055527 80.701973 26.804274 -0.302470 0.074865 0 -0.002307 -0.001400 */
const uint32_t LCAL_M60_DEFAULT[9] = {
    0x429C6CD0u, 0x429E1C6Eu, 0x42A16769u, 0x41D66F27u, 0xBE9ADD59u, 0x3D9952D2u, 0x00000000u, 0xBB17310Au, 0xBAB78034u,
};

static inline int32_t qmul(int32_t a, int32_t b) { return (int32_t)(((int64_t)a * b) >> QB); }

/* float32 原始位 -> round(v * 2^frac)，饱和到 int32；非规格化数、NaN 以外的极小值按 0 */
static int32_t f32_to_q(uint32_t b, int frac)
{
    int e = (int)((b >> 23) & 0xFF);
    if (e == 0) return 0;
    int64_t m = (int64_t)((b & 0x7FFFFFu) | 0x800000u);
    int sh = e - 150 + frac;
    int64_t q;
    if (sh >= 0) q = sh < 38 ? m << sh : INT32_MAX;
    else if (sh > -40) q = (m + ((int64_t)1 << (-sh - 1))) >> -sh;
    else q = 0;
    if (q > INT32_MAX) q = INT32_MAX;
    return (b >> 31) ? -(int32_t)q : (int32_t)q;
}

/* 厂家 setcutcorner 的倒角（标定坐标系，与 model/lidar_lut.py corner_mask 相同） */
static bool cut_px(int u, int v, int r)
{
    const int w = LCAL_W, h = LCAL_H, r2 = r * r;
    if (u < r) {
        if (v < r && (r - u) * (r - u) + (r - v) * (r - v) > r2) return true;
        if (v > h - r && (r - u) * (r - u) + (v - (h - r)) * (v - (h - r)) > r2) return true;
    }
    if (u > w - r) {
        if (v < r && (u - (w - r)) * (u - (w - r)) + (r - v) * (r - v) > r2) return true;
        if (v > h - r && (u - (w - r)) * (u - (w - r)) + (v - (h - r)) * (v - (h - r)) > r2) return true;
    }
    return false;
}

static int16_t to_q11(int32_t v)
{
    int32_t q = (v + (1 << (QB - 12))) >> (QB - 11);
    if (q > 2047) q = 2047;
    if (q < -2047) q = -2047;
    return (int16_t)q;
}

void lidar_cal_start(lidar_cal_t *lc, const uint32_t lens[9], bool from_dev)
{
    bool zero = true;
    for (int i = 0; i < 9; i++) zero = zero && lens[i] == 0;
    const uint32_t *L = zero ? LCAL_M60_DEFAULT : lens;
    int32_t fx = f32_to_q(L[0], 16), fy = f32_to_q(L[1], 16), cx = f32_to_q(L[2], 16), cy = f32_to_q(L[3], 16);
    if (fx < (1 << 16) || fy < (1 << 16)) {           /* 内参不合理：用缺省值 */
        L = LCAL_M60_DEFAULT;
        zero = true;
        fx = f32_to_q(L[0], 16); fy = f32_to_q(L[1], 16); cx = f32_to_q(L[2], 16); cy = f32_to_q(L[3], 16);
    }
    memcpy(lc->lens, L, sizeof lc->lens);
    lc->from_dev = from_dev && !zero;
    lc->t_start = hal_time_us();
    for (int i = 0; i < 5; i++) lc->k[i] = f32_to_q(L[4 + i], QB);
    for (int q = 0; q < LCAL_W; q++) {
        int us = lc->flip_u ? LCAL_W - 1 - q : q;
        int64_t d = ((int64_t)us << 16) - cx;
        lc->xd[q] = (int32_t)((d << QB) / fx);
    }
    for (int v = 0; v < LCAL_H; v++) {
        int64_t d = ((int64_t)v << 16) - cy;
        lc->yd[v] = (int32_t)((d << QB) / fy);
    }
    lc->next = 0;
}

void lidar_cal_init(lidar_cal_t *lc)
{
    memset(lc, 0, sizeof *lc);
    lc->cut = LCAL_CUT_R;
    lc->flip_u = true;
    static const uint32_t none[9] = {0};
    lidar_cal_start(lc, none, false);
}

void lidar_cal_pixel(const lidar_cal_t *lc, int idx, int16_t out[3])
{
    int v = idx / LCAL_W, q = idx - v * LCAL_W;
    int us = lc->flip_u ? LCAL_W - 1 - q : q;
    if (lc->cut && cut_px(us, v, lc->cut)) {
        out[0] = out[1] = out[2] = 0;
        return;
    }
    const int32_t *k = lc->k;
    const int32_t xd = lc->xd[q], yd = lc->yd[v];
    int32_t x = xd, y = yd, icd = ONE;
    /* 不动点迭代给初值（1/rad 用牛顿迭代跟踪，省掉除法） */
    for (int it = 0; it < LCAL_ITERS; it++) {
        int32_t x2 = qmul(x, x), y2 = qmul(y, y), xy = qmul(x, y);
        int32_t r2 = x2 + y2, r4 = qmul(r2, r2), r6 = qmul(r4, r2);
        int32_t rad = ONE + qmul(k[0], r2) + qmul(k[1], r4) + qmul(k[2], r6);
        int32_t dx = qmul(2 * k[3], xy) + qmul(k[4], r2 + 2 * x2);
        int32_t dy = qmul(k[3], r2 + 2 * y2) + qmul(2 * k[4], xy);
        icd = qmul(icd, 2 * ONE - qmul(rad, icd));
        icd = qmul(icd, 2 * ONE - qmul(rad, icd));
        x = qmul(xd - dx, icd);
        y = qmul(yd - dy, icd);
    }
    /* 牛顿法收敛（畸变大时四角的不动点迭代收敛很慢） */
    for (int it = 0; it < LCAL_NEWTON; it++) {
        int32_t x2 = qmul(x, x), y2 = qmul(y, y), xy = qmul(x, y);
        int32_t r2 = x2 + y2, r4 = qmul(r2, r2), r6 = qmul(r4, r2);
        int32_t rad  = ONE + qmul(k[0], r2) + qmul(k[1], r4) + qmul(k[2], r6);
        int32_t drad = k[0] + 2 * qmul(k[1], r2) + 3 * qmul(k[2], r4);
        int32_t fx = qmul(x, rad) + qmul(2 * k[3], xy) + qmul(k[4], r2 + 2 * x2) - xd;
        int32_t fy = qmul(y, rad) + qmul(k[3], r2 + 2 * y2) + qmul(2 * k[4], xy) - yd;
        int32_t jxx = rad + 2 * qmul(x2, drad) + qmul(2 * k[3], y) + qmul(6 * k[4], x);
        int32_t jxy = 2 * qmul(xy, drad) + qmul(2 * k[3], x) + qmul(2 * k[4], y);
        int32_t jyy = rad + 2 * qmul(y2, drad) + qmul(6 * k[3], y) + qmul(2 * k[4], x);
        int32_t det = qmul(jxx, jyy) - qmul(jxy, jxy);
        if (det < ONE / 16) break;                         /* 畸变模型在这里不可逆：保留当前值 */
        int64_t nx = (int64_t)jyy * fx - (int64_t)jxy * fy;  /* Q48 */
        int64_t ny = (int64_t)jxx * fy - (int64_t)jxy * fx;
        x -= (int32_t)(nx / det);
        y -= (int32_t)(ny / det);
    }
    /* 1/sqrt(1 + x^2 + y^2)：线性初值 + 牛顿迭代 */
    int32_t s = ONE + qmul(x, x) + qmul(y, y);
    int32_t g = (int32_t)(1.1862 * ONE) - qmul((int32_t)(0.1862 * ONE), s);
    if (g < ONE / 4) g = ONE / 4;
    for (int it = 0; it < 5; it++) g = qmul(g, 3 * ONE - qmul(s, qmul(g, g))) >> 1;
    out[0] = to_q11(g);                       /* 前 */
    out[1] = to_q11(qmul(x, g));              /* 左：列号翻转后标定坐标 x 即向左，与厂家 SDK 相同（2026-11-01 实车核对） */
    out[2] = to_q11(-qmul(y, g));             /* 上：标定坐标 y 向下 */
}

int lidar_cal_poll(lidar_cal_t *lc, int budget)
{
    uint32_t seq = hal_lidar_info_seq();
    if (seq != 0 && seq != lc->info_seq) {
        uint32_t lens[9];
        lc->info_seq = seq;
        hal_lidar_lens(lens);
        bool zero = true;
        for (int i = 0; i < 9; i++) zero = zero && lens[i] == 0;
        if (!zero && memcmp(lens, lc->lens, sizeof lens) != 0) lidar_cal_start(lc, lens, true);
    }
    int n = 0;
    int16_t buf[3 * LCAL_CHUNK];
    while (n < budget && lc->next < LCAL_NPIX) {
        int m = LCAL_NPIX - lc->next;
        if (m > LCAL_CHUNK) m = LCAL_CHUNK;
        if (m > budget - n) m = budget - n;
        for (int i = 0; i < m; i++) lidar_cal_pixel(lc, lc->next + i, &buf[3 * i]);
        hal_lidar_lut_write(lc->next, buf, m);
        lc->next = (uint16_t)(lc->next + m);
        n += m;
        if (lidar_cal_ready(lc)) {
            lc->n_gen++;
            lc->gen_us = hal_time_us() - lc->t_start;
        }
    }
    return n;
}
