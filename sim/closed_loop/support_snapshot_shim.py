"""Host-only bulk HAL bridge. No navigation, board HAL or target RAM changes."""
from pathlib import Path


def apply(fw):
    fw=Path(fw);p=fw/'test/hal_py.c';s=p.read_text(encoding='utf-8')
    assert s.count('bool hal_map_support(int32_t x,int32_t y,hal_support_t *out)')==1
    s=s.replace('bool hal_map_support(int32_t x,int32_t y,hal_support_t *out)','bool snapshot_callback_support(int32_t x,int32_t y,hal_support_t *out)',1)
    s=s.replace('void hal_map_clear(void) { ctl(0, 0, 0, 0); }','extern void support_snapshot_clear(void);\nvoid hal_map_clear(void) {support_snapshot_clear();ctl(0,0,0,0);}',1)
    assert 'support_snapshot_clear();ctl' in s
    p.write_text(s,encoding='utf-8')
    (fw/'test/support_snapshot_host.c').write_text(SOURCE,encoding='utf-8')


SOURCE=r'''#include <string.h>
#include "hal.h"
#define EXPORT __declspec(dllexport)
extern bool snapshot_callback_support(int32_t,int32_t,hal_support_t*);
static int16_t low[16384],high[16384];
static uint8_t count[16384],age[16384],stale[16384];
static uint16_t known[1024],fresh[1024];
static int32_t cx,cy,ccx,ccy;
static uint16_t seq,inv;
static uint32_t epoch,mismatches,queries;
static bool enabled,cov_enabled,verify;
void support_snapshot_clear(void){enabled=cov_enabled=false;}
EXPORT void py_set_support_snapshot(const int16_t *a,const int16_t *b,const uint8_t *c,const uint8_t *d,const uint8_t *e,int32_t x,int32_t y,uint16_t generation,uint16_t resolution){
 memcpy(low,a,sizeof low);memcpy(high,b,sizeof high);memcpy(count,c,sizeof count);memcpy(age,d,sizeof age);memcpy(stale,e,sizeof stale);
 cx=x;cy=y;seq=generation;inv=resolution;enabled=true;
}
EXPORT void py_set_coverage_snapshot(const uint16_t *k,const uint16_t *f,int32_t x,int32_t y,uint32_t ep){
 memcpy(known,k,sizeof known);memcpy(fresh,f,sizeof fresh);ccx=x;ccy=y;epoch=ep;cov_enabled=true;
}
EXPORT void py_set_support_snapshot_verify(int on){verify=on!=0;mismatches=queries=0;}
EXPORT void py_get_support_snapshot_stats(uint32_t *v){v[0]=queries;v[1]=mismatches;}
bool hal_map_support(int32_t x,int32_t y,hal_support_t *out){
 hal_map_info_t mi;
 if(!enabled || !hal_map_info(&mi) || mi.seq!=seq || mi.cx!=cx || mi.cy!=cy || mi.inv_res!=inv)return snapshot_callback_support(x,y,out);
 int32_t gx=(int32_t)(((int64_t)x*inv)>>16),gy=(int32_t)(((int64_t)y*inv)>>16);
 int ix=gx-cx+64,iy=gy-cy+64;bool ok=ix>=0&&ix<128&&iy>=0&&iy<128;int index=ok?iy*128+ix:0;ok=ok&&count[index]!=0;
 if(ok){
  int mx=gx-ccx+16,my=gy-ccy+16;uint16_t k=0,f=0;
  if(cov_enabled && epoch==hal_time_us()/200000u && mx>=0&&mx<32&&my>=0&&my<32){k=known[my*32+mx];f=fresh[my*32+mx];}
  *out=(hal_support_t){.lo=low[index],.hi=high[index],.seq=seq,.count=count[index],.age=age[index],.retained_stale=stale[index]!=0,.coverage=k,.coverage_retained_stale=(k&~f)!=0};
 }
 queries++;
 if(verify){hal_support_t ref={0};bool a=snapshot_callback_support(x,y,&ref);
  if(a!=ok || (ok&&(ref.lo!=out->lo||ref.hi!=out->hi||ref.seq!=out->seq||ref.count!=out->count||ref.age!=out->age||ref.retained_stale!=out->retained_stale||ref.coverage!=out->coverage||ref.coverage_retained_stale!=out->coverage_retained_stale)))mismatches++;
 }
 return ok;
}
'''
