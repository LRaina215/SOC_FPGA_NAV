/* 定点数学：正余弦查表（与 traj_eval 同一张表）、atan2、整数开方、Q14 旋转矩阵。 */
#ifndef NAV_FIXMATH_H
#define NAV_FIXMATH_H

#include "nav_types.h"

#define Q14 16384

/* 正余弦，Q14 */
int32_t fx_sin(angle_t a);
int32_t fx_cos(angle_t a);
/* atan2(y, x) -> angle（误差 < 0.1°） */
angle_t fx_atan2(int32_t y, int32_t x);
/* 整数平方根 */
uint32_t fx_isqrt(uint64_t v);
/* 两点距离 mm */
int32_t fx_dist(int32_t dx, int32_t dy);
/* 有符号角度差 b - a，范围 [-32768, 32767] */
static inline int32_t fx_angle_diff(angle_t a, angle_t b) { return (int16_t)(uint16_t)(b - a); }

/* 3x3 Q14 矩阵 */
typedef struct { int32_t m[3][3]; } fx_mat3_t;
void fx_rot_zyx(fx_mat3_t *r, angle_t yaw, angle_t pitch, angle_t roll);   /* Rz*Ry*Rx */
void fx_mat_mul(fx_mat3_t *out, const fx_mat3_t *a, const fx_mat3_t *b);
void fx_mat_vec(int32_t out[3], const fx_mat3_t *r, const int32_t v[3]);  /* 结果 = R*v（v 单位任意） */

#endif
