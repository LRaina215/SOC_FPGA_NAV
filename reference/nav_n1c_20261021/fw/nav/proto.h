/* 主机 <-> FPGA 报文（小端）。主机可以是 RK3588（UDP，SFP 口）或 PC（UDP；或板载 USB 串口经
 * uart_bus_bridge 注入邮箱）——报文完全相同，导航不依赖 RK3588。完整定义见 docs/2_interface/icd.md。
 *
 * 公共头 4 字节: 'N' 'V' | 版本(1) | 类型(1)
 * RK3588 -> FPGA
 *   0x10 GOAL      int32 x_mm, int32 y_mm, int32 yaw_cdeg          （无图：里程计系；有图：地图系；yaw 可填 0x7FFFFFFF 表示不关心）
 *   0x11 WAYPOINTS uint8 n, uint8 loop, uint16 rsv, n × {int32 x_mm, int32 y_mm}  （n <= 32）
 *   0x20 TARGET    uint8 found, uint8 cls, uint16 conf_permille, int32 x_mm, int32 y_mm, int32 z_mm,
 *                  uint16 latency_ms, uint16 track_id              （机身系；latency = 采图到发送的延时）
 *   0x30 MODE      uint8 mode(0 空闲 1 定点 2 跟踪 3 探索 4 急停), uint8 rsv[3]
 *   0x12 SET_POSE  int32 x_mm, int32 y_mm, int32 yaw_cdeg, uint16 flags(bit0 在先验图上对准), uint16 range_mm,
 *                  uint16 range_cdeg, uint16 rsv          （有图：地图系初始位姿与不确定范围；无图：重置里程计原点）
 *   0x40 MAP_BEGIN uint32 total_len                      （先验地图包 .nvmp，见 model/prior_map.py）
 *   0x41 MAP_DATA  uint32 offset, 字节…（<= 1024）        （须按顺序；FPGA 以 MAP_ACK 回报下一个期望偏移）
 *   0x42 MAP_END   -                                     （校验通过即进入有图模式，等待 SET_POSE 对准）
 *   0x43 MAP_CLEAR -                                     （回到无图模式）
 * FPGA -> 主机
 *   0x80 STATUS    uint8 mode, uint8 state, uint16 map_seq, int32 x_mm, int32 y_mm, int32 yaw_cdeg,
 *                  int32 goal_x, int32 goal_y, int16 vx, int16 wz, uint16 flags, uint16 rsv, uint32 t_us
 *                  uint8 loc_state(0 无图 1 对准中 2 正常 3 退化 4 丢失), uint8 loc_quality(%), uint16 rsv
 *   0x81 PATH      uint8 n, uint8 rsv[3], n × {int32 x_mm, int32 y_mm}（n <= 32，已降采样）
 *   0x82 MAP_ACK   uint32 next_offset, uint8 map_state(0 无 1 接收中 2 已加载 3 校验错 4 格式错), uint8 rsv[3]
 */
#ifndef NAV_PROTO_H
#define NAV_PROTO_H

#include "nav_types.h"

#define PROTO_VER 1
#define PROTO_MAX_WP 32
#define PROTO_MAX_LEN 1100          /* 最长报文（MAP_DATA：8 + 1024） */

enum {
    MSG_GOAL = 0x10, MSG_WAYPOINTS = 0x11, MSG_SET_POSE = 0x12, MSG_TARGET = 0x20, MSG_MODE = 0x30,
    MSG_MAP_BEGIN = 0x40, MSG_MAP_DATA = 0x41, MSG_MAP_END = 0x42, MSG_MAP_CLEAR = 0x43,
    MSG_STATUS = 0x80, MSG_PATH = 0x81, MSG_MAP_ACK = 0x82,
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
        struct { int32_t x, y, yaw_cdeg; uint16_t flags, range_mm, range_cdeg; } pose;
        struct { uint32_t total; } map_begin;
        struct { uint32_t offset; uint16_t n; const uint8_t *data; } map_data;   /* data 指向报文缓冲 */
    } u;
} proto_msg_t;

typedef struct {
    uint8_t mode, state;
    uint16_t map_seq;
    int32_t x, y, yaw_cdeg, goal_x, goal_y;
    int16_t vx, wz;
    uint16_t flags;
    uint32_t t_us;
    uint8_t  loc_state, loc_quality;
} proto_status_t;

/* 解析：成功返回 true */
bool proto_parse(const uint8_t *buf, int len, proto_msg_t *m);
/* 组包：返回长度 */
int proto_build_status(uint8_t *buf, const proto_status_t *s);
int proto_build_path(uint8_t *buf, int n, const int32_t *x, const int32_t *y);
int proto_build_map_ack(uint8_t *buf, uint32_t next_offset, uint8_t state);
/* 主机侧（测试/上位机参考）组包 */
int proto_build_goal(uint8_t *buf, int32_t x, int32_t y, int32_t yaw_cdeg);
int proto_build_target(uint8_t *buf, uint8_t found, uint8_t cls, uint16_t conf, int32_t x, int32_t y, int32_t z,
                       uint16_t latency_ms, uint16_t track_id);
int proto_build_mode(uint8_t *buf, uint8_t mode);

#endif
