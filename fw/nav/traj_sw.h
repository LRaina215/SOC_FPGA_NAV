/* traj_eval 的软件实现，与 RTL / model/nav_fixed.py 逐位一致。
 * 用途：主机测试的 HAL；以及无硬件加速时的回退（性能低约两个数量级）。
 * cost 按槽地址（slot = (iy&127)*128 + (ix&127)）。
 */
#ifndef NAV_TRAJ_SW_H
#define NAV_TRAJ_SW_H

#include "hal.h"

void traj_eval_sw(const uint8_t *cost_slot, int32_t cx, int32_t cy, const hal_cand_t *c, int k, int t,
                  int32_t x0_q8, int32_t y0_q8, angle_t th0, uint8_t cmax, hal_traj_res_t *res);

#endif
