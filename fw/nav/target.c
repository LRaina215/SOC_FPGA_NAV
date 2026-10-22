#include "target.h"
#include "fixmath.h"

void target_init(target_t *t, uint32_t timeout_us, int32_t standoff_mm)
{
    *t = (target_t){0};
    t->timeout_us = timeout_us;
    t->standoff_mm = standoff_mm;
}

void target_update(target_t *t, const pose_est_t *pe, bool found, int32_t bx, int32_t by,
                   uint16_t latency_ms, uint16_t id, uint8_t cls, uint32_t rx_us)
{
    if (!found) return;          /* 未找到：保持旧值，交给超时判断 */
    uint32_t cap = rx_us - (uint32_t)latency_ms * 1000u;
    if (t->valid && (int32_t)(cap - t->t_us) < 0) return;
    nav_pose_t at;
    if (!pose_query_valid(pose_xy_at(pe, cap, &at))) return;
    pose_body_to_odom(&at, bx, by, &t->x, &t->y);
    t->t_us = cap;
    t->id = id;
    t->cls = cls;
    t->valid = true;
}

bool target_alive(const target_t *t, uint32_t now_us)
{
    return t->valid && (uint32_t)(now_us - t->t_us) < t->timeout_us;
}

void target_approach_goal(const target_t *t, const nav_pose_t *robot, int32_t *gx, int32_t *gy)
{
    int32_t dx = t->x - robot->x, dy = t->y - robot->y;
    int32_t d = fx_dist(dx, dy);
    if (d <= t->standoff_mm) { *gx = robot->x; *gy = robot->y; return; }
    *gx = t->x - (int32_t)((int64_t)dx * t->standoff_mm / d);
    *gy = t->y - (int32_t)((int64_t)dy * t->standoff_mm / d);
}
