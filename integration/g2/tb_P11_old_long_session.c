#include "p11_command_guard.h"
#include <stdio.h>
#include <stdint.h>
static uint16_t crc(const uint8_t *p,unsigned n){uint16_t c=65535;while(n--){c^=(uint16_t)*p++<<8;for(unsigned i=0;i<8;i++)c=(uint16_t)((c&32768)?(c<<1)^0x1021:c<<1);}return c;}
static bool frame(p11_command_guard_t *g,int16_t v,uint32_t t)
{uint8_t b[10]={0xa5,0x5a,0x10,4,(uint8_t)v,(uint8_t)((uint16_t)v>>8),0,0,0,0};uint16_t c=crc(b+2,6);b[8]=(uint8_t)c;b[9]=(uint8_t)(c>>8);bool ok=false;for(unsigned i=0;i<10;i++)ok=p11_guard_byte(g,b[i],i==0,t,t+44*(i+1),true)||ok;return ok;}
int main(void){p11_command_guard_t g;p11_guard_init(&g);uint32_t auth=1000000;for(unsigned i=0;i<=50;i++)p11_guard_feedback(&g,0,0,auth-500000+i*10000,auth-500000+i*10000,true);if(p11_guard_manage(&g,P11_AUTHORIZE,1,auth,true,true,true)!=P11_OK||!frame(&g,0,auth+1000))return 2;for(unsigned i=0;i<110000;i++){uint32_t t=auth+2000+20000u*i;p11_guard_feedback(&g,0,0,t,t,false);if(!frame(&g,100,t)){printf("EXPECTED_OLD_LONG_SESSION_FAILURE frame=%u auth=%u current=%u age=%u fresh=1 state=%u\n",i,auth,t,t-auth,(unsigned)g.state);return 1;}}puts("UNEXPECTED_NO_COUNTEREXAMPLE");return 3;}
