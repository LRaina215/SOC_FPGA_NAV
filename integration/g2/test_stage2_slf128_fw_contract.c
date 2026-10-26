/* Component tests of unchanged frozen parser/prior/setter C with explicit HAL
 * substitutes. No DLL, CPU execution, timing or physical stop claim. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "nav_app.h"
#include "host_link_guard.h"
#include "hal.h"
#include "nav_params.h"

static uint32_t ram[16384];static unsigned writes,clears,enables,zsets,cases;
static bool enabled;
void hal_prior_begin(void){memset(ram,0,sizeof ram);enabled=false;clears++;}
void hal_prior_write(uint16_t s,uint32_t w){assert(s<16384);ram[s]=w;writes++;}
void hal_prior_enable(bool on,int32_t x,int32_t y){(void)x;(void)y;enabled=on;if(on)enables++;else{memset(ram,0,sizeof ram);clears++;}}
void hal_match_zband(int32_t lo,int32_t hi){assert(lo==60&&hi>=60&&hi<=1500);zsets++;}
void gmap_prior_begin(gmap_t*g,int32_t x,int32_t y,int nx,int ny){(void)x;(void)y;(void)nx;(void)ny;g->has_prior=false;}
void gmap_prior_put(gmap_t*g,int i,uint8_t v){(void)g;(void)i;(void)v;}
void gmap_prior_done(gmap_t*g){g->has_prior=true;}
void gmap_prior_drop(gmap_t*g){g->has_prior=false;}
static void u16(unsigned char*p,unsigned v){p[0]=v;p[1]=v>>8;}
static void u32(unsigned char*p,uint32_t v){for(int i=0;i<4;i++)p[i]=v>>(8*i);}
static unsigned char pkg[80];
static unsigned make(int zhi){
 memset(pkg,0,sizeof pkg);memcpy(pkg,"NVMP",4);u16(pkg+4,3);u16(pkg+6,0x0107);
 u32(pkg+8,(uint32_t)-64);u32(pkg+12,(uint32_t)-200);u32(pkg+16,3);
 u32(pkg+36,15);u32(pkg+40,0x33580001);u16(pkg+44,zhi);u16(pkg+46,(uint16_t)-300);
 unsigned slots[3]={0,16383,77},words[3]={0x3ffff,0x15555,0x2aaaa};
 for(int i=0;i<3;i++){u16(pkg+48+5*i,slots[i]);pkg[50+5*i]=words[i];pkg[51+5*i]=words[i]>>8;pkg[52+5*i]=words[i]>>16;}
 u32(pkg+32,prior_crc32_update(0,pkg+48,15));return 63;
}
static void empty(void){for(int i=0;i<16384;i++)assert(ram[i]==0);assert(!enabled);}
static void feed(prior_rx_t*r,gmap_t*g,unsigned n,unsigned chunk){
 for(unsigned p=0;p<n;){unsigned k=n-p;if(k>chunk)k=chunk;assert(prior_rx_data(r,g,p,pkg+p,(int)k));p+=k;if(r->state!=PRIOR_RECV)break;}
}
static void packet_cases(void){
 prior_rx_t r;gmap_t g;memset(&g,0,sizeof g);
 const unsigned chunks[]={1,2,3,5,17,48,63};
 for(unsigned i=0;i<sizeof(chunks)/sizeof(chunks[0]);i++){
  unsigned n=make(350);prior_rx_begin_profile(&r,&g,n,350);feed(&r,&g,n,chunks[i]);
  assert(prior_rx_end(&r,&g)==PRIOR_READY&&enabled&&g.has_prior);
  assert(r.n_words==3&&ram[0]==0x3ffff&&ram[16383]==0x15555&&ram[77]==0x2aaaa);cases++;
 }
 for(int failure=0;failure<8;failure++){
  unsigned n=make(350);prior_rx_begin_profile(&r,&g,n,350);
  switch(failure){
   case 0:u16(pkg+4,2);break;case 1:u16(pkg+6,0x0108);break;
   case 2:u32(pkg+40,0);break;case 3:u16(pkg+53,16384);break;
   case 4:pkg[57]=0x80;break;case 5:pkg[32]^=1;break;
   case 6:u16(pkg+44,1500);break;case 7:u16(pkg+53,65535);break;
  }
  if(failure==3||failure==4||failure==7)u32(pkg+32,prior_crc32_update(0,pkg+48,15));
  feed(&r,&g,n,5);assert(prior_rx_end(&r,&g)!=PRIOR_READY);empty();assert(!g.has_prior);cases++;
 }
 unsigned n=make(350);prior_rx_begin_profile(&r,&g,n,350);unsigned before=writes;
 feed(&r,&g,53,53);assert(writes==before+1&&ram[0]!=0);
 assert(prior_rx_end(&r,&g)==PRIOR_EFMT);empty();cases++;
 n=make(1500);prior_rx_begin_profile(&r,&g,n,1500);feed(&r,&g,n,7);assert(prior_rx_end(&r,&g)==PRIOR_READY);cases++;
 n=make(350);prior_rx_begin_profile(&r,&g,n,350);assert(prior_rx_data(&r,&g,0,pkg,48));
 assert(!prior_rx_data(&r,&g,49,pkg+48,5)&&r.state==PRIOR_RECV); // retry stays blocked in RECV
 assert(prior_rx_data(&r,&g,48,pkg+48,15));assert(prior_rx_end(&r,&g)==PRIOR_READY);cases++;
}
static void config_cases(void){
 unsigned char b[24]={'N','V',1,0x44};proto_msg_t m;
 u32(b+4,1);u32(b+8,1250);u32(b+12,1500);assert(proto_parse(b,16,&m)&&m.type==MSG_SLF_CONFIG&&m.u.slf.forward_mm==1250&&m.u.slf.zhi_mm==1500);cases++;
 u32(b+4,2);assert(!proto_parse(b,16,&m));u32(b+4,1);assert(!proto_parse(b,15,&m));cases++;
 assert(proto_build_slf_ack(b,1,1250,350,0x534c0107)==20&&b[3]==0x84);assert(MSG_CONTROL_V2==0x83);cases++;
 memset(&g_app,0,sizeof g_app);g_app.nt.mode=NAV_MODE_IDLE;g_app.lf_forward_mm=1500;g_app.match_zhi=350;
 unsigned clear0=clears;assert(nav_app_set_slf_window(1250,1500)&&g_app.lf_forward_mm==1250&&g_app.match_zhi==1500&&clears==clear0+1);cases++;
 for(int why=0;why<4;why++){
  g_app.pm_pend=why==0;g_app.pm_run=why==1;g_app.prx.state=why==2?PRIOR_RECV:PRIOR_NONE;g_app.nt.mode=why==3?NAV_MODE_GOTO:NAV_MODE_IDLE;
  clear0=clears;assert(!nav_app_set_slf_window(1000,350)&&clears==clear0);cases++;
 }
 g_app.pm_pend=g_app.pm_run=false;g_app.nt.mode=NAV_MODE_IDLE;g_app.prx.state=PRIOR_READY;assert(!nav_app_set_slf_window(1000,350));cases++;
 g_app.prx.state=PRIOR_NONE;assert(!nav_app_set_slf_window(-1,350)&&!nav_app_set_slf_window(1500,59)&&!nav_app_set_slf_window(1500,1501));cases++;
 host_link_guard_t guard;host_link_guard_init(&guard,100);m.type=MSG_SLF_CONFIG;
 assert(host_link_guard_observe(&guard,&m,100,100)&&guard.locked);cases++;
}
int main(void){packet_cases();config_cases();printf("TEST PASSED SLF FW contracts cases=%u writes=%u clears=%u enables=%u zsets=%u\n",cases,writes,clears,enables,zsets);return 0;}
