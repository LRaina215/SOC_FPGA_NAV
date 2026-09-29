/* 导航应用主循环（软核上跑的全部逻辑的入口）。
 * 上板：main() { nav_app_init(); for (;;) nav_app_poll(); }
 * 主机测试：由测试程序推进仿真时间并调用 nav_app_poll()。
 */
#ifndef NAV_APP_H
#define NAV_APP_H

#include "nav_task.h"
#include "imu_parse.h"
#include "pose_match.h"
#include "loc.h"
#include "prior.h"
#include "lidar_cal.h"

#define APP_PLAN_PERIOD_US   100000    /* 规划 10 Hz */
#define APP_STATUS_PERIOD_US 200000    /* 状态回传 5 Hz */
/* 结构点高度带（相对足底 mm）：无图 / 有图（有图放宽下限，见 model/slf_match.py Z_LO_PRIOR） */
#define APP_ZLO_ODOM   60
#define APP_ZLO_PRIOR  (-300)
#define APP_ZHI        1500

typedef struct {
    nav_task_t   nt;
    imu_parser_t imu;
    uint32_t     t_plan, t_status;
    uint32_t     n_frames, n_msgs, n_skip;
    bool         use_imu_yaw;           /* IMU 航向可信时置 1（磁干扰大时用里程计航向） */
    pm_cfg_t     pm_cfg;
    pm_stat_t    pm;
    bool         pm_pend;               /* 已提交、待匹配的帧 */
    bool         pm_run;                /* 打分进行中 */
    int          pm_k;
    hal_match_cand_t pm_cand[PM_MAX_CAND];
    hal_match_res_t  pm_res[PM_MAX_CAND];
    nav_pose_t   pm_at;                 /* 该帧提交时的位姿 */
    int32_t      zband_lo, zband_hi;    /* 保留点高度带（相对机器人，mm）：地面以下 / 机身以上的点不进地图 */
    int32_t      anchor_gain;           /* 足迹高度锚定增益 /256（0 = 关闭） */
    int32_t      anchor_hl, anchor_hw;  /* 足迹采样点相对机身中心的前后 / 左右偏移 mm（应落在轮子着地处） */
    /* 有图模式 */
    prior_rx_t   prx;                   /* 先验地图包接收 */
    loc_t        loc;                   /* 对准 / 跟踪质量 / 重定位 */
    bool         pm_align;              /* 当前打分属于对准搜索 */
    bool         pm_stale;              /* 位姿已跳变：进行中的打分结果作废 */
    uint32_t     t_lost;                /* 进入丢失的时刻（定期重试对准） */
    pm_cfg_t     pm_cfg_prior;          /* 有图跟踪用的匹配参数（先验图可信，增益更高） */
    uint32_t     turn_until;            /* 对准换视角：原地转到这个时刻（之后再稳定 300 ms） */
    bool         turning;
    /* 雷达方向查找表（按雷达自带内参在板上生成；生成期间的帧丢弃） */
    lidar_cal_t  lcal;
    int          lcal_budget;           /* 每次主循环最多生成的像素数 */
    uint32_t     n_lut_skip;            /* 因查找表未就绪丢弃的帧 */
} nav_app_t;

/* 先验地图已加载（有图模式） */
bool nav_app_have_prior(void);

extern nav_app_t g_app;

void nav_app_init(void);
void nav_app_poll(void);

#endif
