/* 导航固件公共类型与单位约定。
 *
 * 坐标系：里程计系（上电位置为原点）X 前 Y 左 Z 上；机身系同向。
 * 单位：长度 mm（int32），速度 mm/s，角速度 mrad/s，
 *       角度 "angle"：uint16，65536 = 一周（与 traj_eval 的 TH0 相同），
 *       时间 us（uint32，按模运算比较），
 *       栅格：世界栅格整数坐标 = floor(mm * INV_RES / 65536)，轨迹用 Q8 栅格。
 * 全部整数/定点运算：软核有无浮点单元都能运行。
 */
#ifndef NAV_TYPES_H
#define NAV_TYPES_H

#include <stdint.h>
#include <stdbool.h>

typedef uint16_t angle_t;

#define ANGLE_FROM_CDEG(cd) ((angle_t)(((int32_t)(cd) * 65536 / 36000)))
#define ANGLE_TO_CDEG(a)    ((int32_t)(int16_t)(a) * 36000 / 65536)

typedef struct {
    int32_t x, y, z;        /* mm */
    angle_t yaw, pitch, roll;
    uint32_t t_us;
} nav_pose_t;

typedef struct {
    int32_t x, y;           /* mm，里程计系 */
} nav_pt_t;

/* 速度指令（发往 safety_guard） */
typedef struct {
    int16_t vx, vy, wz;     /* mm/s, mm/s, mrad/s */
    uint8_t mode;
} nav_cmd_t;

/* 底盘能力（底盘调好后填写） */
typedef struct {
    bool     holonomic;     /* 能否直接横移（vy） */
    int16_t  vx_max, vy_max, wz_max;
    int32_t  radius_mm;     /* 外接圆半径，用于代价图膨胀与安全距离 */
} nav_chassis_caps_t;

/* 运行模式（PDF：定点导航 / 目标跟踪 / 自主探索） */
typedef enum {
    NAV_MODE_IDLE   = 0,
    NAV_MODE_GOTO   = 1,
    NAV_MODE_TRACK  = 2,
    NAV_MODE_SEARCH = 3,
    NAV_MODE_ESTOP  = 4,
} nav_mode_t;

static inline int32_t nav_abs32(int32_t a) { return a < 0 ? -a : a; }
static inline int32_t nav_min32(int32_t a, int32_t b) { return a < b ? a : b; }
static inline int32_t nav_max32(int32_t a, int32_t b) { return a > b ? a : b; }
static inline int32_t nav_clamp32(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

#endif
