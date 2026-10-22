#ifndef NAV_MOTION_GUARD_H
#define NAV_MOTION_GUARD_H

#include "local_planner.h"

/* 内部诊断值，不改变串口协议。零速请求不等于底盘已经停稳。 */
enum { MOTION_ZERO, MOTION_OK, MOTION_NO_MAP, MOTION_MAP_CHANGED,
       MOTION_HAZARD, MOTION_UNKNOWN, MOTION_OUTSIDE, MOTION_LIMIT,
       MOTION_SUPPORT_UNKNOWN,MOTION_SUPPORT_BLOCK };
enum { CMD_STOP, CMD_LOCAL, CMD_RECOVERY, CMD_TURN, CMD_FALLBACK,
       CMD_TRACK_FACE, CMD_SEARCH, CMD_HOLD, CMD_ESTOP,CMD_GROUND_SCAN,CMD_GROUND_EXIT };

bool motion_map_same(const hal_map_info_t *a, const hal_map_info_t *b);
/* 同一缓存地图上检查最终指令：原始254格+完整外接圆；253不再次膨胀。
 * 近200ms新进入未知足迹单独返回UNKNOWN，仍检查后续已知危险；由任务层明确处理。
 * 这是名义匀速检查，响应/刹停包络属于N3。 */
uint8_t motion_check(const uint8_t *map, const hal_map_info_t *mi,
                     const nav_pose_t *pose, const lp_cfg_t *cfg, const nav_cmd_t *cmd);
/* 前视线段的连续圆形扫掠；逐格检查胶囊与方格，不使用253作半径代理。 */
uint8_t motion_segment(const uint8_t *map, const hal_map_info_t *mi,
                       const nav_pose_t *pose, const lp_cfg_t *cfg, int32_t gx, int32_t gy);
#endif
