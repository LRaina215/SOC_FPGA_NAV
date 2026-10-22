"""Exercise the actual board HAL capture checks through an allocated MMIO page."""
import argparse
import ctypes
import json
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[2]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--fw',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    fw=a.fw.resolve();a.out.mkdir(parents=True,exist_ok=False)
    src=a.out/'probe.c'
    src.write_text('''#include <windows.h>
#include "hal.h"
#include "nav_regs_map.h"
__declspec(dllexport) int init_mmio(void) {
 return VirtualAlloc((void*)0x40000000,65536,MEM_RESERVE|MEM_COMMIT,PAGE_READWRITE)==(void*)0x40000000;
}
__declspec(dllexport) unsigned probe(unsigned cap,unsigned now,int stage,int clear) {
 volatile unsigned *r=(volatile unsigned*)0x40000000;
 if(clear)hal_map_clear();r[NAV_SYS_TIME_US/4]=now;hal_lidar_stage(stage);
 int32_t R[3][3]={{16384,0,0},{0,16384,0},{0,0,16384}},t[3]={0,0,155};
 hal_lidar_commit(R,t,0,0,-500,1500,cap);
 return r[NAV_MAP_SUPPORT_STAMP/4];
}
''',encoding='utf-8')
    dll=(a.out/'hal.dll').resolve()
    subprocess.run(['gcc','-O2','-std=c99','-shared','-o',str(dll)]+['-I'+str(fw/d) for d in ['include','nav','hal']]+[str(src),str(fw/'drv/hal_fpga.c')],check=True)
    d=ctypes.CDLL(str(dll));assert d.init_mmio(),'MMIO address unavailable; trial not run'
    d.probe.argtypes=[ctypes.c_uint32,ctypes.c_uint32,ctypes.c_int,ctypes.c_int];d.probe.restype=ctypes.c_uint32
    rows=[]
    def check(name,cap,now,stage,clear,expected):
        actual=d.probe(cap,now,stage,clear);rows.append(dict(name=name,actual=actual,expected=expected));assert actual==expected,name
    check('first MATCH_ONLY valid',200000,250000,1,1,129)
    check('FUSE retained same capture valid',200000,280000,2,0,129)
    check('duplicate FUSE invalid',200000,290000,2,0,0)
    check('older capture invalid',100000,300000,0,0,0)
    check('new capture same epoch valid',250000,310000,0,0,129)
    check('future capture invalid',400000,350000,0,0,0)
    check('late capture invalid',400000,1400001,0,0,0)
    check('fresh delayed capture valid',500000,1400001,0,0,130)
    check('clear permits older valid capture',400000,1400000,0,1,130)
    check('prewrap capture valid',0xffff0000,0xfffff000,0,1,128|((0xffff0000//200000)&127))
    check('timer wrap permits new capture',1000,2000,0,0,128)
    check('prewrap delayed rejected',0xfffff000,3000,0,0,0)
    (a.out/'result.json').write_text(json.dumps(dict(contracts=rows,passed=len(rows),reference='actual hal_fpga.c MMIO; no copied timestamp implementation'),indent=2),encoding='utf-8')
    print(f'{len(rows)} board HAL capture contracts passed')


if __name__=='__main__':main()
