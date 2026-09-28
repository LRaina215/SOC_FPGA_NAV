#include "imu_parse.h"

void imu_parser_init(imu_parser_t *p, imu_proto_t proto)
{
    *p = (imu_parser_t){0};
    p->proto = proto;
}

/* 度 ×32768/180 -> angle（65536/360）：angle = raw * 65536/360 * 180/32768 = raw（量纲恰好一致） */
static angle_t wit_angle(uint8_t lo, uint8_t hi) { return (angle_t)(int16_t)(lo | (hi << 8)); }

bool imu_parser_feed(imu_parser_t *p, uint8_t b, bool first, uint32_t t_us)
{
    if (first) p->t_burst = t_us;
    if (p->n == 0 && b != 0x55) return false;
    p->buf[p->n++] = b;
    if (p->n < 11) return false;
    p->n = 0;
    uint8_t sum = 0;
    for (int i = 0; i < 10; i++) sum = (uint8_t)(sum + p->buf[i]);
    if (sum != p->buf[10]) { p->n_bad++; return false; }
    p->n_ok++;
    if (p->buf[1] != 0x53) return false;         /* 只用角度帧 */
    p->roll  = wit_angle(p->buf[2], p->buf[3]);
    p->pitch = wit_angle(p->buf[4], p->buf[5]);
    p->yaw   = wit_angle(p->buf[6], p->buf[7]);
    p->att_t_us = p->t_burst;
    p->att_new = true;
    return true;
}
