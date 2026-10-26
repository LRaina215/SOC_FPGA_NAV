/* Actual frozen HAL cuts plus explicit MMIO test substitutes; not a CPU run. */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include "hal.h"
#include "fixmath.h"
#include "nav_regs_map.h"
#include "nav_sin_lut.h"
static uint32_t inv_value,got_x,got_y;
static unsigned read_inv,writes_lf,writes_map,unexpected_forward_write;
static bool forwarding;
static uint32_t nav_rd(uint32_t a){
 if(a==NAV_MAP_INV_RES){read_inv++;return inv_value;}
 if(a==NAV_SYS_TIME_US)return 1000000;
 if(a==NAV_LIDAR_CTRL)return 0;
 fprintf(stderr,"unexpected read %x\n",a);exit(2);
}
static void nav_wr(uint32_t a,uint32_t v){
 if(a==NAV_MATCH_LF_CX){got_x=v;writes_lf++;}
 else if(a==NAV_MATCH_LF_CY){got_y=v;writes_lf++;}
 else if(a==NAV_MAP_CENTER_X||a==NAV_MAP_CENTER_Y){writes_map++;if(forwarding)unexpected_forward_write++;}
 else if(forwarding)unexpected_forward_write++;
}
#include "actual_hal_cuts.h"
static int64_t oracle(int64_t n){
 /* n is below2^42 and denominator2^30, so this double division and0.5
    addition are exact over the tested domain.  Independent floor/ceil rule. */
 double q=(double)n/1073741824.0;
 return (int64_t)(q<0?ceil(q-0.5):floor(q+0.5));
}
static void ck(bool x,const char*s){if(!x){fprintf(stderr,"FAIL %s\n",s);exit(1);}}
int main(void){
 const int ds[]={0,1,1000,1250,1500,3000};
 const uint32_t invs[]={0,1,1311,65535};
 const int32_t centers[][2]={{0,0},{-37,-82},{200000,-300000},{INT_MAX-1,INT_MIN+1}};
 const int32_t rot[3][3]={{16384,0,0},{0,16384,0},{0,0,16384}},tr[3]={0,0,0};
 uint64_t calls=0,ties=0,trig=0;
 for(int k=0;k<4;k++)for(int sg=-1;sg<=1;sg+=2)for(int off=-1;off<=1;off++){
  int64_t n=sg*((int64_t)k*1073741824+536870912+off);
  ck(lf_round_cell(n)==oracle(n),"explicit signed half tie");ties++;
 }
 ck(lf_round_cell(0)==0,"zero rounding");ties++;
 for(unsigned a=0;a<65536;a++){
  ck(fx_sin((angle_t)a)==NAV_SIN_LUT[a>>6],"sin LUT addressing");
  ck(fx_cos((angle_t)a)==NAV_SIN_LUT[((a>>6)+256)&1023],"cos LUT addressing");trig+=2;
 }
 for(unsigned c=0;c<sizeof centers/sizeof centers[0];c++){
  forwarding=false;unsigned map_before=writes_map;
  hal_lidar_commit(rot,tr,centers[c][0],centers[c][1],60,350,1000000);
  ck(writes_map==map_before+2,"original MAP_CENTER commit writes");
  ck(lf_vehicle_cx==centers[c][0]&&lf_vehicle_cy==centers[c][1],"last actual commit center");
  for(unsigned d=0;d<sizeof ds/sizeof ds[0];d++)for(unsigned i=0;i<sizeof invs/sizeof invs[0];i++)for(unsigned a=0;a<65536;a++){
   inv_value=0xa5a50000u|invs[i];read_inv=0;writes_lf=0;forwarding=true;
   hal_match_lf_forward(ds[d],(angle_t)a);forwarding=false;
   int64_t nx=(int64_t)ds[d]*invs[i]*NAV_SIN_LUT[((a>>6)+256)&1023];
   int64_t ny=(int64_t)ds[d]*invs[i]*NAV_SIN_LUT[a>>6];
   uint32_t ex=(uint32_t)centers[c][0]+(uint32_t)oracle(nx),ey=(uint32_t)centers[c][1]+(uint32_t)oracle(ny);
   if(got_x!=ex||got_y!=ey){fprintf(stderr,"FAIL d%d inv%u yaw%u center%u got%x/%x expected%x/%x\n",ds[d],invs[i],a,c,got_x,got_y,ex,ey);return 1;}
   ck(read_inv==1&&writes_lf==2&&!unexpected_forward_write,"exact LF-only MMIO access contract");calls++;
  }
 }
 printf("TEST PASSED actual_forward_HAL calls=%llu all_yaw=65536 distances=6 inverse_values=4 center_pairs=4 trig_checks=%llu tie_checks=%llu map_center_commits=%u forward_MAP_CENTER_writes=0\n",(unsigned long long)calls,(unsigned long long)trig,(unsigned long long)ties,writes_map);
 return 0;
}
