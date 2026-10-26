/* Real original1024-word table is an explicit proposed CPU-local CSR model.
 * No peripheral timing or physical FPGA behavior is simulated here. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "fixmath.h"
#include "traj_sw.h"
#include "nav_sin_lut.h"
#include "nav_fw_sin_lookup.h"
static unsigned checks,reads,angles,rotations,trajectories;
#define CHECK(x)do{checks++;if(!(x)){fprintf(stderr,"FAIL line%d %s\n",__LINE__,#x);exit(1);}}while(0)
int32_t nav_fw_sin_csr_model(uint32_t address)
{
    CHECK(address>=NAV_FW_SIN_MMIO_BASE&&address<=NAV_FW_SIN_MMIO_BASE+4092u);
    CHECK((address&3u)==0);reads++;
    return NAV_SIN_LUT[(address-NAV_FW_SIN_MMIO_BASE)>>2];
}
int32_t reference_fx_sin(angle_t a);
int32_t reference_fx_cos(angle_t a);
void reference_fx_rot_zyx(fx_mat3_t *r,angle_t yaw,angle_t pitch,angle_t roll);
void reference_traj_eval_sw(const uint8_t*,int32_t,int32_t,const hal_cand_t*,int,int,int32_t,int32_t,angle_t,uint8_t,hal_traj_res_t*);
static uint8_t map[128*128];
int main(void)
{
    for(unsigned i=0;i<1024;i++){unsigned n=reads;CHECK(nav_fw_sin_lookup(i)==NAV_SIN_LUT[i]);CHECK(reads==n+1);}
    /* Entire16bit angle domain covers every index and all wrap/quadrant edges. */
    for(unsigned a=0;a<65536;a++){
        unsigned n=reads;CHECK(fx_sin((angle_t)a)==reference_fx_sin((angle_t)a));
        CHECK(fx_cos((angle_t)a)==reference_fx_cos((angle_t)a));CHECK(reads==n+2);angles++;
    }
    for(unsigned a=0;a<4096;a++){
        angle_t y=(angle_t)(a*17u),p=(angle_t)(a*997u),r=(angle_t)(a*32749u);
        fx_mat3_t x,z;fx_rot_zyx(&x,y,p,r);reference_fx_rot_zyx(&z,y,p,r);
        for(unsigned j=0;j<3;j++)for(unsigned k=0;k<3;k++){CHECK(x.m[j][k]==z.m[j][k]);}
        rotations++;
    }
    for(unsigned scenario=0;scenario<32;scenario++){
        for(unsigned i=0;i<sizeof map;i++)map[i]=(uint8_t)((i*17u+scenario*19u)%201);
        if(scenario&1u)for(unsigned i=0;i<128;i++)map[i*128+64]=254;
        if(scenario&2u)for(unsigned i=0;i<128;i++)map[96*128+i]=255;
        hal_cand_t c[16];hal_traj_res_t actual[16],expected[16];
        for(unsigned i=0;i<16;i++)c[i]=(hal_cand_t){(int16_t)(15*(int)i-100),(int16_t)(20*(int)(i%3)-20),(int16_t)(scenario*1021u+i*3277u)};
        traj_eval_sw(map,64,64,c,16,32,64*256,64*256,(angle_t)(scenario*4093u),253,actual);
        reference_traj_eval_sw(map,64,64,c,16,32,64*256,64*256,(angle_t)(scenario*4093u),253,expected);
        for(unsigned i=0;i<16;i++){
            CHECK(actual[i].max_cost==expected[i].max_cost);CHECK(actual[i].n_unknown==expected[i].n_unknown);
            CHECK(actual[i].coll_step==expected[i].coll_step);CHECK(actual[i].collide==expected[i].collide);CHECK(actual[i].left==expected[i].left);
            CHECK(actual[i].sum_cost==expected[i].sum_cost);CHECK(actual[i].end_x==expected[i].end_x);CHECK(actual[i].end_y==expected[i].end_y);
        }trajectories++;
    }
    printf("PASS FW_SIN_MMIO checks=%u table_words=1024 angles=%u rotations=%u trajectory_scenarios=%u modeled_MMIO_reads=%u hardware_timing_not_tested=1\n",checks,angles,rotations,trajectories,reads);
    return 0;
}
