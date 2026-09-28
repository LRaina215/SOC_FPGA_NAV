/* 导航应用主循环（软核上跑的全部逻辑的入口）。
 * 上板：main() { nav_app_init(); for (;;) nav_app_poll(); }
 * 主机测试：由测试程序推进仿真时间并调用 nav_app_poll()。
 */
#ifndef NAV_APP_H
#define NAV_APP_H

#include "nav_task.h"
#include "imu_parse.h"
#include "pose_match.h"

#define APP_PLAN_PERIOD_US   100000    /* 规划 10 Hz */
#define APP_STATUS_PERIOD_US 200000    /* 状态回传 5 Hz */

typedef struct {
    nav_task_t   nt;
    imu_parser_t imu;
    uint32_t     t_plan, t_status;
    uint32_t     n_frames, n_msgs;
    bool         use_imu_yaw;           /* IMU 航向可信时置 1（磁干扰大时用里程计航向） */
    pm_cfg_t     pm_cfg;
    pm_stat_t    pm;
    int32_t      zband_lo, zband_hi;
    int32_t      anchor_gain;           /* 足迹高度锚定增益 /256（0 = 关闭） */    /* 保留点高度带（相对机器人，mm）：地面以下 / 机身以上的点不进地图 */
} nav_app_t;

extern nav_app_t g_app;

void nav_app_init(void);
void nav_app_poll(void);

#endif
