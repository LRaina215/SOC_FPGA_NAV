/* IMU 串口协议解析（可替换）。
 * 硬件只提供原始字节 + 突发时间戳，协议全部在这里实现；换 IMU 型号只改本文件。
 * IMU088 的协议文档尚未拿到，这里先实现常见的 0x55 帧协议（WIT 维特智能系列：
 * 0x55 0x53 + roll/pitch/yaw 各 int16（×180/32768 度）+ 温度 + 校验和；
 * 0x55 0x52 + wx/wy/wz 各 int16（×2000/32768 °/s）角速度帧）作为样例。
 * 拿到 IMU088 手册后：新增一个 imu_proto_t 分支即可，接口不变。
 */
#ifndef NAV_IMU_PARSE_H
#define NAV_IMU_PARSE_H

#include "nav_types.h"

typedef enum { IMU_PROTO_WIT55 = 0 } imu_proto_t;

typedef struct {
    imu_proto_t proto;
    uint8_t  buf[16];
    int      n;
    uint32_t t_burst;           /* 当前突发首字节时间 */
    /* 输出 */
    bool     att_new;
    angle_t  roll, pitch, yaw;
    uint32_t att_t_us;
    bool     gyro_new;
    int16_t  gx, gy, gz;        /* 原始角速度（×2000/32768 °/s），机身系 x 前 y 左 z 上 */
    uint32_t gyro_t_us;
    uint32_t n_ok, n_bad;
} imu_parser_t;

void imu_parser_init(imu_parser_t *p, imu_proto_t proto);
/* 喂一个字节；返回 true 表示解析出新的姿态（角速度帧只置 gyro_new，返回 false） */
bool imu_parser_feed(imu_parser_t *p, uint8_t b, bool first, uint32_t t_us);

#endif
