/* 硬件抽象层：导航算法（fw/nav）只通过这些函数访问硬件。
 *   上板：fw/drv/hal_fpga.c（寄存器访问 nav_subsystem）
 *   主机测试：fw/test/hal_sim.c（仿真世界）
 */
#ifndef NAV_HAL_H
#define NAV_HAL_H

#include "nav_types.h"
#include "nav_params.h"

#define HAL_MAP_N NP_MAP_N

/* ---- 时间 ---- */
uint32_t hal_time_us(void);

/* ---- 代价图（前台） ---- */
typedef struct {
    uint16_t seq;           /* 每次新代价图加 1 */
    int32_t  cx, cy;        /* 窗口中心世界栅格 */
    int32_t  res_mm;        /* 栅格分辨率 */
    uint16_t inv_res;       /* 65536 / res_mm */
} hal_map_info_t;

bool    hal_map_info(hal_map_info_t *mi);
/* 把前台代价图按"窗口坐标"复制出来：out[wy*N + wx]，wx = ix - (cx - N/2)。返回复制时的 seq */
uint16_t hal_map_copy(uint8_t *out, hal_map_info_t *mi);

/* ---- 轨迹评估（硬件加速）---- */
typedef struct { int16_t dvx, dvy, dth; } hal_cand_t;      /* 格 Q8/步，angle/步 */
typedef struct {
    uint8_t  max_cost, n_unknown, coll_step;
    bool     collide, left;
    uint32_t sum_cost;
    int32_t  end_x, end_y;                                  /* 世界栅格 Q8 */
} hal_traj_res_t;
/* 起点为世界栅格 Q8 与航向；阻塞直到完成。返回 false 表示评估期间地图已更新（结果仍可用） */
bool hal_traj_eval(const hal_cand_t *c, int k, int t, int32_t x0_q8, int32_t y0_q8, angle_t th0,
                   uint8_t cmax, hal_traj_res_t *res);

/* ---- 底盘 ---- */
void hal_send_cmd(const nav_cmd_t *cmd);
/* 读取底盘回传（里程计速度）。返回是否有新数据 */
bool hal_odom(int16_t *vx, int16_t *vy, int16_t *wz, uint32_t *t_us);

/* ---- 雷达帧位姿握手 ---- */
/* 有一帧点云在等待位姿时返回 true，并给出该帧到达时间 */
bool hal_lidar_waiting(uint32_t *frame_t_us);
/* 写入 雷达->里程计 变换（R Q14，t mm）、本帧地图窗口中心、保留点高度带（里程计系 mm），并放行该帧 */
void hal_lidar_commit(const int32_t r_q14[3][3], const int32_t t_mm[3], int32_t cx, int32_t cy,
                      int32_t zmin, int32_t zmax);

/* ---- 匹配定位（硬件并行打分）----
 * 对等待中的这一帧点云（按预测位姿 R/t 变换后）施加每个候选位姿增量：
 *   p' = Rz(dyaw)·(p - c) + c + (dx, dy)，c = 机器人位置
 * SLF 后端（默认，model/slf_match.py）：只用竖直结构点，与结构图比较；
 *   n_valid = 参与打分的点数，cost = Σ 点到结构线法向距离（1/16 格，截断），n_in = 近邻命中点数，sum_dz = 0。
 * EICM 后端（对照）：与"本帧融合之前"的高程图区间 [zmin-TOL, zmax+TOL] 比较，cost += min(e, EMAX)。
 * 返回 false 表示硬件不支持（固件退回纯航位推算）。 */
typedef struct { int32_t dx, dy; int32_t dyaw; } hal_match_cand_t;       /* mm, mm, angle */
typedef struct { uint32_t n_valid, cost, n_in; int32_t sum_dz; } hal_match_res_t;
bool hal_match_eval(const int32_t r_q14[3][3], const int32_t t_mm[3], int32_t cx_mm, int32_t cy_mm,
                    const hal_match_cand_t *c, int k, hal_match_res_t *res);
/* 关键帧（仅 EICM 打分后端使用；SLF 的结构图由硬件在每次提交时自动累积，不需要）。 */
void hal_match_keyframe(int32_t z_ref_mm);
/* 读高程图某点（里程计系 mm）所在栅格的地面高度：[zmin, zmax] 区间中点（格内高差 > 40 mm 视为非地面）。
 * 栅格有效返回 true。上板实现走 MAP.ELEV_ADDR/ELEV_CMD/ELEV_Q0/Q1 */
bool hal_map_height(int32_t x_mm, int32_t y_mm, int32_t *h_mm);

/* ---- IMU 原始字节 ---- */
/* 取一个字节；first 表示突发首字节，t_us 为其到达时间（仅 first 时有效）。无数据返回 false */
bool hal_imu_byte(uint8_t *b, bool *first, uint32_t *t_us);

/* ---- 主机（RK3588）报文 ---- */
/* 取一个报文，返回长度（0 表示没有）；t_us 为到达时间 */
int  hal_host_recv(uint8_t *buf, int maxlen, uint32_t *t_us);
bool hal_host_send(const uint8_t *buf, int len);

#endif
