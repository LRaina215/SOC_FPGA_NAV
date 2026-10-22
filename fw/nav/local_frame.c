#include "local_frame.h"
#include "fixmath.h"
static int32_t sn(angle_t a) {
    int b=a&63;
    return (fx_sin(a)*(64-b)+fx_sin((angle_t)(a+64))*b)/64;
}
static void rotate(int64_t x,int64_t y,angle_t a,int64_t *ox,int64_t *oy) {
    int32_t s=sn(a),c=sn((angle_t)(a+16384));
    *ox=(x*c-y*s)/Q14;*oy=(x*s+y*c)/Q14;
}
void local_frame_point(const local_frame_t *f,int32_t x,int32_t y,int32_t *ox,int32_t *oy,bool inverse) {
    int64_t a,b;
    if(inverse) rotate((int64_t)x*1000-f->tx_um,(int64_t)y*1000-f->ty_um,(angle_t)-f->yaw,&a,&b);
    else {rotate((int64_t)x*1000,(int64_t)y*1000,f->yaw,&a,&b);a+=f->tx_um;b+=f->ty_um;}
    *ox=(int32_t)(a/1000);*oy=(int32_t)(b/1000);
}
void local_frame_pose(const local_frame_t *f,const nav_pose_t *p,nav_pose_t *out) {
    *out=*p;local_frame_point(f,p->x,p->y,&out->x,&out->y,false);
    out->yaw=(angle_t)(p->yaw+f->yaw);
}
void local_frame_preserve(local_frame_t *f,int64_t old_x,int64_t old_y,int64_t new_x,int64_t new_y,int32_t dyaw) {
    if(old_x==new_x && old_y==new_y && !dyaw) return;
    int64_t ax,ay,bx,by;
    rotate(old_x,old_y,f->yaw,&ax,&ay);
    f->yaw=(angle_t)(f->yaw-dyaw);
    rotate(new_x,new_y,f->yaw,&bx,&by);
    f->tx_um+=ax-bx;f->ty_um+=ay-by;
}
