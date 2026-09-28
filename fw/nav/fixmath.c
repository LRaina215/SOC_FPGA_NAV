#include "fixmath.h"
#include "nav_sin_lut.h"

int32_t fx_sin(angle_t a) { return NAV_SIN_LUT[a >> 6]; }
int32_t fx_cos(angle_t a) { return NAV_SIN_LUT[((a >> 6) + 256) & 1023]; }

/* CORDIC 向量模式。atan(2^-i)，单位 angle（65536 = 一周） */
static const int16_t CORDIC_ATAN[14] = {8192, 4836, 2555, 1297, 651, 326, 163, 81, 41, 20, 10, 5, 3, 1};

angle_t fx_atan2(int32_t y, int32_t x)
{
    if (x == 0 && y == 0) return 0;
    int64_t xx = x, yy = y;
    int32_t a = 0;
    if (xx < 0) {                       /* 转到右半平面 */
        xx = -xx; yy = -yy; a = 32768;
    }
    /* 放大以保留精度 */
    while (xx < (1LL << 30) && yy < (1LL << 30) && yy > -(1LL << 30)) {
        xx <<= 1; yy <<= 1;
    }
    for (int i = 0; i < 14; i++) {
        int64_t nx, ny;
        if (yy > 0) { nx = xx + (yy >> i); ny = yy - (xx >> i); a += CORDIC_ATAN[i]; }
        else        { nx = xx - (yy >> i); ny = yy + (xx >> i); a -= CORDIC_ATAN[i]; }
        xx = nx; yy = ny;
    }
    return (angle_t)a;
}

uint32_t fx_isqrt(uint64_t v)
{
    uint64_t r = 0, b = (uint64_t)1 << 62;
    while (b > v) b >>= 2;
    while (b) {
        if (v >= r + b) { v -= r + b; r = (r >> 1) + b; }
        else r >>= 1;
        b >>= 2;
    }
    return (uint32_t)r;
}

int32_t fx_dist(int32_t dx, int32_t dy)
{
    return (int32_t)fx_isqrt((uint64_t)((int64_t)dx * dx + (int64_t)dy * dy));
}

void fx_mat_mul(fx_mat3_t *out, const fx_mat3_t *a, const fx_mat3_t *b)
{
    fx_mat3_t t;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            int64_t s = 0;
            for (int k = 0; k < 3; k++) s += (int64_t)a->m[i][k] * b->m[k][j];
            t.m[i][j] = (int32_t)((s + (Q14 / 2)) >> 14);
        }
    *out = t;
}

void fx_rot_zyx(fx_mat3_t *r, angle_t yaw, angle_t pitch, angle_t roll)
{
    fx_mat3_t rz = {{{fx_cos(yaw), -fx_sin(yaw), 0}, {fx_sin(yaw), fx_cos(yaw), 0}, {0, 0, Q14}}};
    fx_mat3_t ry = {{{fx_cos(pitch), 0, fx_sin(pitch)}, {0, Q14, 0}, {-fx_sin(pitch), 0, fx_cos(pitch)}}};
    fx_mat3_t rx = {{{Q14, 0, 0}, {0, fx_cos(roll), -fx_sin(roll)}, {0, fx_sin(roll), fx_cos(roll)}}};
    fx_mat3_t t;
    fx_mat_mul(&t, &rz, &ry);
    fx_mat_mul(r, &t, &rx);
}

void fx_mat_vec(int32_t out[3], const fx_mat3_t *r, const int32_t v[3])
{
    int32_t t[3];
    for (int i = 0; i < 3; i++) {
        int64_t s = 0;
        for (int k = 0; k < 3; k++) s += (int64_t)r->m[i][k] * v[k];
        t[i] = (int32_t)(s >> 14);
    }
    out[0] = t[0]; out[1] = t[1]; out[2] = t[2];
}
