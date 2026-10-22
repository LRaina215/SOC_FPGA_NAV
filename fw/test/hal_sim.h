/* 主机测试用 HAL：二维仿真世界 + 运动学机器人 + 模拟 RK3588 目标检测。
 * 代价图生成规则模仿硬件：窗口 128×128，中心跟随机器人，看见过的格才已知；
 * 障碍内 254，膨胀区 253，其余 0；轨迹评估用 traj_eval_sw（与硬件逐位一致）。
 */
#ifndef HAL_SIM_H
#define HAL_SIM_H

#include "hal.h"

#define SIM_MAX_OBS 32

typedef struct { int32_t x0, y0, x1, y1; } sim_rect_t;

typedef struct {
    /* 世界 */
    sim_rect_t obs[SIM_MAX_OBS];
    int        n_obs;
    bool       target_on;
    int32_t    tgt_x, tgt_y;
    /* 机器人真值 */
    double     x, y, yaw;           /* mm, rad */
    nav_cmd_t  cmd;
    uint32_t   now_us;
    /* 传感器模型 */
    int32_t    sense_range_mm;      /* 地图可见距离 */
    int32_t    fov_cdeg;            /* 雷达水平视场（整幅） */
    int32_t    cam_range_mm, cam_fov_cdeg;
    int32_t    infl_cells;
    /* 统计 */
    double     min_clear_mm;        /* 机器人中心到障碍物的最小距离 */
    int32_t    robot_r_mm;
    uint32_t   n_cmds, n_status;
    bool       traj_stale, traj_change_map, traj_blocked; /* 确定性HAL故障注入 */
    uint8_t    last_status[64];
    int        last_status_len;
} sim_t;

extern sim_t g_sim;

void sim_reset(void);
void sim_add_obs(int32_t x0, int32_t y0, int32_t x1, int32_t y1);
/* 推进仿真时间 dt（机器人运动、里程计、地图、目标检测） */
void sim_advance(uint32_t dt_us);
/* 向"RK3588 -> FPGA"队列注入一个报文 */
void sim_host_inject(const uint8_t *buf, int len);
void sim_map_fixture(const uint8_t *map,const hal_map_info_t *mi);
/* 真值点到障碍物最近距离 */
double sim_clearance(double x, double y);

#endif
