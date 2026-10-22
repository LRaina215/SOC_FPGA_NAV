"""Actual board and host HALs must not confuse fine age with height retention."""
import argparse
import ctypes
import json
from pathlib import Path
import subprocess


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--fw',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    fw=a.fw.resolve();out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    board=out/'board.c';board.write_text('''#include <windows.h>
#include "hal.h"
#include "nav_regs_map.h"
__declspec(dllexport) int init_mmio(void){return VirtualAlloc((void*)0x40000000,65536,MEM_RESERVE|MEM_COMMIT,PAGE_READWRITE)==(void*)0x40000000;}
__declspec(dllexport) int probe(int raw_old,int coverage_old){
 volatile uint32_t *r=(volatile uint32_t*)0x40000000;
 r[NAV_MAP_STAT/4]=7u<<16;r[NAV_MAP_INV_RES/4]=1311;r[NAV_MAP_FRAME_CNT/4]=10;r[NAV_MAP_VALID_CFG/4]=60;
 r[NAV_MAP_ELEV_Q0/4]=20;r[NAV_MAP_ELEV_Q1/4]=NAV_MAP_ELEV_Q1_DONE_MASK|(1u<<NAV_MAP_ELEV_Q1_CNT_SHIFT)|(10u<<NAV_MAP_ELEV_Q1_STAMP_SHIFT)|(raw_old?NAV_MAP_ELEV_Q1_RETAINED_STALE_MASK:0);
 r[NAV_MAP_SUPPORT_Q/4]=coverage_old?65535u:0xffffffffu;
 int32_t h=-999;return hal_map_height(0,0,&h)?h:-999;
}
__declspec(dllexport) int support_bytes(void){return sizeof(hal_support_t);}
''',encoding='utf-8')
    host=out/'host.c';host.write_text('''#include "hal.h"
static int old,coverage_old;
extern void py_set_h_cb(int (*cb)(int32_t,int32_t,int32_t*));
extern void py_set_support_cb(int (*cb)(int32_t,int32_t,int32_t*));
extern void py_set_coverage_cb(int (*cb)(int32_t,int32_t,int32_t*));
static int height(int32_t x,int32_t y,int32_t *h){*h=10;return !old;}
static int raw(int32_t x,int32_t y,int32_t *v){v[0]=0;v[1]=20;v[2]=7;v[3]=1;v[4]=0;v[5]=old;return 1;}
static int cov(int32_t x,int32_t y,int32_t *v){v[0]=65535;v[1]=coverage_old?0:65535;return 1;}
__declspec(dllexport) int probe(int a,int b){old=a;coverage_old=b;py_set_h_cb(height);py_set_support_cb(raw);py_set_coverage_cb(cov);int32_t h=-999;return hal_map_height(0,0,&h)?h:-999;}
__declspec(dllexport) int support_bytes(void){return sizeof(hal_support_t);}
''',encoding='utf-8')
    inc=['-I'+str(fw/d) for d in ['include','nav','hal','app','test']]
    for name,files in [('board',[board,fw/'drv/hal_fpga.c']),('host',[host,fw/'test/hal_py.c',fw/'app/nav_app.c']+list((fw/'nav').glob('*.c')))]:
        subprocess.run(['gcc','-std=c99','-O2','-shared','-o',str(out/(name+'.dll'))]+inc+list(map(str,files)),check=True)
    board_d,host_d=ctypes.CDLL(str(out/'board.dll')),ctypes.CDLL(str(out/'host.dll'));assert board_d.init_mmio()
    rows=[]
    assert board_d.support_bytes()==host_d.support_bytes()==12,'Must use existing HAL padding'
    for raw in [0,1]:
        for fine in [0,1]:
            b,h=board_d.probe(raw,fine),host_d.probe(raw,fine);expected=-999 if raw else 10
            rows.append(dict(raw_retained_stale=raw,fine_retained_stale=fine,board=b,host=h,expected=expected));assert b==h==expected
    (out/'result.json').write_text(json.dumps(dict(cases=rows,support_struct_bytes=12,passed=4,reference='actual board/host HAL; same raw-age/span contract; coverage age affects support separately'),indent=2),encoding='utf-8');print('4 board/host age-separation cases passed; HAL structure12bytes')


if __name__=='__main__':main()
