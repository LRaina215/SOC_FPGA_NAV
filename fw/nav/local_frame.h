/* 连续地形里程计系 <- 全局匹配定位系，仅用于无先验在线导航。 */
#ifndef NAV_LOCAL_FRAME_H
#define NAV_LOCAL_FRAME_H
#include "nav_types.h"
typedef struct { int64_t tx_um, ty_um; angle_t yaw; } local_frame_t;
void local_frame_point(const local_frame_t *f,int32_t x,int32_t y,int32_t *ox,int32_t *oy,bool inverse);
void local_frame_pose(const local_frame_t *f,const nav_pose_t *p,nav_pose_t *out);
void local_frame_preserve(local_frame_t *f,int64_t old_x,int64_t old_y,int64_t new_x,int64_t new_y,int32_t dyaw);
#endif
