#ifndef NAV_STOP_GUARD_H
#define NAV_STOP_GUARD_H
#include "nav_types.h"
/* Navigation supervisor, not the original4500 low-level controller.
 * Experimental bounds: reaction200ms, deceleration100mm/s2 and1000mrad/s2,
 * hold excursion25mm. These are offline assumptions, not vehicle calibration. */
enum { STOP_UNCONFIRMED, STOP_MOVING, STOP_HOLDING, STOP_CONFIRMED, STOP_FEEDBACK, STOP_ENVELOPE };
typedef struct {
    int64_t error_um, integral_um_s;
    int64_t yaw_urad,yaw_integral_urad_s,lateral_um;
    int32_t filtered_q8,filtered_w_q8;
    uint32_t stamp, stable_since;
    int16_t vx,vy,wz;
    uint8_t active,have,stable,status,fault,admitted;
} stop_guard_t;
void stop_feedback(stop_guard_t *s,int vx,int vy,int wz,uint32_t t);
bool stop_fresh(const stop_guard_t *s,uint32_t now);
void stop_release(stop_guard_t *s);
nav_cmd_t stop_hold(stop_guard_t *s,uint32_t now,bool estop);
int stop_margin(const stop_guard_t *s,const nav_cmd_t *c,uint32_t now);
void stop_admission(stop_guard_t *s,bool admitted);
#endif
