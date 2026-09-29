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

/* ---- 匹配定位（SLF，硬件并行打分）----
 * 提交帧（hal_lidar_commit）时硬件同时提取竖直结构点存入点缓冲（按提交时的位姿变换）。
 * hal_match_ready：本帧结构点已就绪、可以打分。
 * hal_match_start：发起打分——对缓冲点施加每个候选增量 p' = Rz(dyaw)·(p - c) + c + (dx, dy)，c = (cx_mm, cy_mm)，
 *   与结构图比较。返回 false 表示硬件不支持（固件退回纯航位推算）。
 * hal_match_result：打分完成时返回 true 并填结果：n_valid = 参与打分的点数，
 *   cost = Σ 点到结构线法向距离（1/16 格，截断），n_in = 近邻命中点数，n_pts = 抽样后的结构点总数（门控前）。
 * hal_match_update：把缓冲点按最终修正量（同样的变换形式）并入结构图（修正量为 0 也要调用）。 */
typedef struct { int32_t dx, dy; int32_t dyaw; } hal_match_cand_t;       /* mm, mm, angle */
typedef struct { uint32_t n_valid, cost, n_in, n_pts; } hal_match_res_t;   /* n_pts：本帧抽样后的结构点总数 */
bool hal_match_ready(void);
bool hal_match_start(int32_t cx_mm, int32_t cy_mm, const hal_match_cand_t *c, int k);
bool hal_match_result(hal_match_res_t *res, int k);
void hal_match_update(int32_t dx, int32_t dy, int32_t dyaw, int32_t cx_mm, int32_t cy_mm);
/* 读高程图某点（里程计系 mm）所在栅格的地面高度：[zmin, zmax] 区间中点（格内高差 > 40 mm 视为非地面）。
 * 栅格有效返回 true。上板实现走 MAP.ELEV_ADDR/ELEV_CMD/ELEV_Q0/Q1 */
bool hal_map_height(int32_t x_mm, int32_t y_mm, int32_t *h_mm);
/* 清空高程图（位姿跳变到地图系后，旧坐标系下融合的内容作废） */
void hal_map_clear(void);

/* ---- 有图模式（先验定位图）----
 * hal_prior_begin：清空结构图、暂停在线并图（下载期间匹配不可用）。
 * hal_prior_write：写一个结构图槽字（23 位，格式见 model/slf_match.py StructMap.words）。
 * hal_prior_enable(true, ox, oy)：进入有图模式——似然场窗口原点固定为 (ox, oy)（全局 5 cm 格），
 *   UPDATE 不再并图（只释放点缓冲），打分时槽字标签与点所在格不符视为无结构。false：清空并回到在线建图。
 * hal_match_gate(false)：打分不做门控（初始对准 / 重定位：位姿误差大时门控会挡掉正确的点）。 */
void hal_prior_begin(void);
void hal_prior_write(uint16_t slot, uint32_t word);
void hal_prior_enable(bool en, int32_t ox, int32_t oy);
void hal_match_gate(bool on);
/* 结构点高度带（相对机器人足底 mm）：有图模式放宽下限（脚下的路沿/平台侧面也在先验图里） */
void hal_match_zband(int32_t lo, int32_t hi);

/* ---- IMU 原始字节 ---- */
/* 取一个字节；first 表示突发首字节，t_us 为其到达时间（仅 first 时有效）。无数据返回 false */
bool hal_imu_byte(uint8_t *b, bool *first, uint32_t *t_us);

/* ---- 主机报文（RK3588 / PC，网口或串口注入，见 proto.h）---- */
/* 取一个报文，返回长度（0 表示没有）；t_us 为到达时间 */
int  hal_host_recv(uint8_t *buf, int maxlen, uint32_t *t_us);
bool hal_host_send(const uint8_t *buf, int len);

#endif
