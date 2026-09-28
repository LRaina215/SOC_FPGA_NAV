/* 目标跟踪：RK3588 给出目标在机身系的位置（采图时刻），
 * 用采图时刻的历史位姿换算到里程计系并保存；超时判为丢失。
 * 采图时刻 = FPGA 收到报文的时间 - 报文中的 latency_ms（两边时钟无需同步）。
 */
#ifndef NAV_TARGET_H
#define NAV_TARGET_H

#include "nav_types.h"
#include "pose.h"

typedef struct {
    bool     valid;
    int32_t  x, y;           /* 里程计系 mm */
    uint32_t t_us;           /* 采图时刻（FPGA 时基） */
    uint16_t id;
    uint8_t  cls;
    uint32_t timeout_us;
    int32_t  standoff_mm;    /* 接近时与目标保持的距离 */
} target_t;

void target_init(target_t *t, uint32_t timeout_us, int32_t standoff_mm);
void target_update(target_t *t, const pose_est_t *pe, bool found, int32_t bx, int32_t by,
                   uint16_t latency_ms, uint16_t id, uint8_t cls, uint32_t rx_us);
bool target_alive(const target_t *t, uint32_t now_us);
/* 接近目标的站位点：沿"机器人->目标"方向，距目标 standoff */
void target_approach_goal(const target_t *t, const nav_pose_t *robot, int32_t *gx, int32_t *gy);

#endif
