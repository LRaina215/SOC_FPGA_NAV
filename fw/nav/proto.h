/* RK3588 <-> FPGA 主机报文（UDP 负载，小端）。完整定义见 docs/icd.md。
 *
 * 公共头 4 字节: 'N' 'V' | 版本(1) | 类型(1)
 * RK3588 -> FPGA
 *   0x10 GOAL      int32 x_mm, int32 y_mm, int32 yaw_cdeg          （里程计系；yaw 可填 0x7FFFFFFF 表示不关心）
 *   0x11 WAYPOINTS uint8 n, uint8 loop, uint16 rsv, n × {int32 x_mm, int32 y_mm}  （n <= 32）
 *   0x20 TARGET    uint8 found, uint8 cls, uint16 conf_permille, int32 x_mm, int32 y_mm, int32 z_mm,
 *                  uint16 latency_ms, uint16 track_id              （机身系；latency = 采图到发送的延时）
 *   0x30 MODE      uint8 mode(0 空闲 1 定点 2 跟踪 3 探索 4 急停), uint8 rsv[3]
 * FPGA -> RK3588
 *   0x80 STATUS    uint8 mode, uint8 state, uint16 map_seq, int32 x_mm, int32 y_mm, int32 yaw_cdeg,
 *                  int32 goal_x, int32 goal_y, int16 vx, int16 wz, uint16 flags, uint16 rsv, uint32 t_us
 *   0x81 PATH      uint8 n, uint8 rsv[3], n × {int32 x_mm, int32 y_mm}（n <= 32，已降采样）
 */
#ifndef NAV_PROTO_H
#define NAV_PROTO_H

#include "nav_types.h"

#define PROTO_VER 1
#define PROTO_MAX_WP 32

enum {
    MSG_GOAL = 0x10, MSG_WAYPOINTS = 0x11, MSG_TARGET = 0x20, MSG_MODE = 0x30,
    MSG_STATUS = 0x80, MSG_PATH = 0x81,
};

/* STATUS.state */
enum { ST_IDLE = 0, ST_MOVING = 1, ST_ARRIVED = 2, ST_BLOCKED = 3, ST_TRACKING = 4, ST_SEARCHING = 5, ST_ESTOP = 6 };
/* STATUS.flags */
#define STF_TARGET_VALID 0x0001
#define STF_MAP_STALE    0x0002
#define STF_LIDAR_OK     0x0004
#define STF_ODOM_OK      0x0008

typedef struct {
    uint8_t type;
    union {
        struct { int32_t x, y, yaw_cdeg; } goal;
        struct { uint8_t n, loop; int32_t x[PROTO_MAX_WP], y[PROTO_MAX_WP]; } wp;
        struct { uint8_t found, cls; uint16_t conf; int32_t x, y, z; uint16_t latency_ms, track_id; } target;
        struct { uint8_t mode; } mode;
    } u;
} proto_msg_t;

typedef struct {
    uint8_t mode, state;
    uint16_t map_seq;
    int32_t x, y, yaw_cdeg, goal_x, goal_y;
    int16_t vx, wz;
    uint16_t flags;
    uint32_t t_us;
} proto_status_t;

/* 解析：成功返回 true */
bool proto_parse(const uint8_t *buf, int len, proto_msg_t *m);
/* 组包：返回长度 */
int proto_build_status(uint8_t *buf, const proto_status_t *s);
int proto_build_path(uint8_t *buf, int n, const int32_t *x, const int32_t *y);
/* 主机侧（测试/上位机参考）组包 */
int proto_build_goal(uint8_t *buf, int32_t x, int32_t y, int32_t yaw_cdeg);
int proto_build_target(uint8_t *buf, uint8_t found, uint8_t cls, uint16_t conf, int32_t x, int32_t y, int32_t z,
                       uint16_t latency_ms, uint16_t track_id);
int proto_build_mode(uint8_t *buf, uint8_t mode);

#endif
