#include "proto.h"

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static int32_t rd32(const uint8_t *p) { return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24)); }
static uint8_t *wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); return p + 2; }
static uint8_t *wr32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i)); return p + 4; }
static uint8_t *hdr(uint8_t *p, uint8_t type) { p[0] = 'N'; p[1] = 'V'; p[2] = PROTO_VER; p[3] = type; return p + 4; }

bool proto_parse(const uint8_t *b, int len, proto_msg_t *m)
{
    if (len < 4 || b[0] != 'N' || b[1] != 'V' || b[2] != PROTO_VER) return false;
    m->type = b[3];
    const uint8_t *p = b + 4;
    int n = len - 4;
    switch (m->type) {
    case MSG_GOAL:
        if (n < 12) return false;
        m->u.goal.x = rd32(p); m->u.goal.y = rd32(p + 4); m->u.goal.yaw_cdeg = rd32(p + 8);
        return true;
    case MSG_WAYPOINTS: {
        if (n < 4) return false;
        int k = p[0];
        if (k > PROTO_MAX_WP || n < 4 + 8 * k) return false;
        m->u.wp.n = (uint8_t)k; m->u.wp.loop = p[1];
        for (int i = 0; i < k; i++) { m->u.wp.x[i] = rd32(p + 4 + 8 * i); m->u.wp.y[i] = rd32(p + 8 + 8 * i); }
        return true;
    }
    case MSG_TARGET:
        if (n < 20) return false;
        m->u.target.found = p[0]; m->u.target.cls = p[1]; m->u.target.conf = rd16(p + 2);
        m->u.target.x = rd32(p + 4); m->u.target.y = rd32(p + 8); m->u.target.z = rd32(p + 12);
        m->u.target.latency_ms = rd16(p + 16); m->u.target.track_id = rd16(p + 18);
        return true;
    case MSG_MODE:
        if (n < 1) return false;
        m->u.mode.mode = p[0];
        return true;
    default:
        return false;
    }
}

int proto_build_status(uint8_t *buf, const proto_status_t *s)
{
    uint8_t *p = hdr(buf, MSG_STATUS);
    *p++ = s->mode; *p++ = s->state; p = wr16(p, s->map_seq);
    p = wr32(p, (uint32_t)s->x); p = wr32(p, (uint32_t)s->y); p = wr32(p, (uint32_t)s->yaw_cdeg);
    p = wr32(p, (uint32_t)s->goal_x); p = wr32(p, (uint32_t)s->goal_y);
    p = wr16(p, (uint16_t)s->vx); p = wr16(p, (uint16_t)s->wz);
    p = wr16(p, s->flags); p = wr16(p, 0);
    p = wr32(p, s->t_us);
    return (int)(p - buf);
}

int proto_build_path(uint8_t *buf, int n, const int32_t *x, const int32_t *y)
{
    if (n > PROTO_MAX_WP) n = PROTO_MAX_WP;
    uint8_t *p = hdr(buf, MSG_PATH);
    *p++ = (uint8_t)n; *p++ = 0; *p++ = 0; *p++ = 0;
    for (int i = 0; i < n; i++) { p = wr32(p, (uint32_t)x[i]); p = wr32(p, (uint32_t)y[i]); }
    return (int)(p - buf);
}

int proto_build_goal(uint8_t *buf, int32_t x, int32_t y, int32_t yaw_cdeg)
{
    uint8_t *p = hdr(buf, MSG_GOAL);
    p = wr32(p, (uint32_t)x); p = wr32(p, (uint32_t)y); p = wr32(p, (uint32_t)yaw_cdeg);
    return (int)(p - buf);
}

int proto_build_target(uint8_t *buf, uint8_t found, uint8_t cls, uint16_t conf, int32_t x, int32_t y, int32_t z,
                       uint16_t latency_ms, uint16_t track_id)
{
    uint8_t *p = hdr(buf, MSG_TARGET);
    *p++ = found; *p++ = cls; p = wr16(p, conf);
    p = wr32(p, (uint32_t)x); p = wr32(p, (uint32_t)y); p = wr32(p, (uint32_t)z);
    p = wr16(p, latency_ms); p = wr16(p, track_id);
    return (int)(p - buf);
}

int proto_build_mode(uint8_t *buf, uint8_t mode)
{
    uint8_t *p = hdr(buf, MSG_MODE);
    *p++ = mode; *p++ = 0; *p++ = 0; *p++ = 0;
    return (int)(p - buf);
}
