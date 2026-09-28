#include "traj_sw.h"
#include "nav_sin_lut.h"

#define N NP_MAP_N

void traj_eval_sw(const uint8_t *cost_slot, int32_t cx, int32_t cy, const hal_cand_t *c, int k, int t,
                  int32_t x0_q8, int32_t y0_q8, angle_t th0, uint8_t cmax, hal_traj_res_t *res)
{
    for (int i = 0; i < k; i++) {
        int32_t x = x0_q8, y = y0_q8;
        uint16_t th = th0;
        hal_traj_res_t r = {0};
        for (int s = 1; s <= t; s++) {
            th = (uint16_t)(th + (uint16_t)c[i].dth);
            int si = th >> 6;
            int64_t sn = NAV_SIN_LUT[si], cs = NAV_SIN_LUT[(si + 256) & 1023];
            int64_t ddx = (int64_t)c[i].dvx * cs - (int64_t)c[i].dvy * sn;
            int64_t ddy = (int64_t)c[i].dvx * sn + (int64_t)c[i].dvy * cs;
            /* 算术右移 = 向下取整，与 RTL >>> 一致 */
            x = (int32_t)((uint32_t)x + (uint32_t)(int32_t)(ddx >> 14));
            y = (int32_t)((uint32_t)y + (uint32_t)(int32_t)(ddy >> 14));
            int32_t ix = x >> 8, iy = y >> 8;
            int32_t wx = ix - (cx - N / 2), wy = iy - (cy - N / 2);
            if (wx < 0 || wx >= N || wy < 0 || wy >= N) {
                r.left = true;
                if (r.n_unknown != 255) r.n_unknown++;
                continue;
            }
            uint8_t cc = cost_slot[((iy & (N - 1)) << 7) | (ix & (N - 1))];
            if (cc == 255) {
                if (r.n_unknown != 255) r.n_unknown++;
            } else if (cc >= cmax) {
                r.collide = true;
                r.coll_step = (uint8_t)s;
                break;
            } else {
                r.sum_cost += cc;
                if (cc > r.max_cost) r.max_cost = cc;
            }
        }
        r.end_x = x;
        r.end_y = y;
        res[i] = r;
    }
}
