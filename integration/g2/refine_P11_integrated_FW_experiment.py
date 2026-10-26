"""Preserve integrated1 and freeze its corrected real-HAL binding in a new copy.

No RTL, selected firmware image, serial device or EDA action is performed.
"""
from pathlib import Path
import argparse,difflib,json,shutil
from datetime import datetime
from prepare_baseline_fusion_retime import metadata,save

BASE=Path('F:/nav_integration_g2_20261023')
SRC=BASE/'P11_shared_guard_FW_integrated_experiment_prepared1'

def freeze(out):
    assert not out.exists(),out
    shutil.copytree(SRC,out)
    def replace(rel,a,b):
        p=out/'fw'/rel;t=p.read_text(encoding='utf-8-sig')
        assert t.count(a)==1,(rel,a,t.count(a))
        p.write_text(t.replace(a,b),encoding='utf8')
    wrappers='''uint32_t p11_raw_hal_count(void) { return nav_rd(NAV_IMU_RX_COUNT)&1023u; }
uint32_t p11_raw_hal_data(void) { return nav_rd(NAV_IMU_RX_DATA); }
uint32_t p11_raw_hal_timestamp(void) { return nav_rd(NAV_IMU_RX_TS); }
uint32_t p11_raw_hal_overflow(void) { return nav_rd(NAV_IMU_STAT_OVF); }
uint32_t p11_raw_hal_time_us(void) { return hal_time_us(); }
'''
    replace('app/nav_app.c',wrappers,'')
    replace('app/nav_app.c','nav_wr(NAV_IMU_BAUD_DIV,selected?217u:868u);nav_wr(NAV_IMU_IDLE_US,500u);',
            'hal_raw_uart_config(selected?217u:868u,500u);')
    replace('drv/hal_fpga.c','#include "host_link_guard.h"',
            '#include "host_link_guard.h"\n#include "p11_raw_service.h"')
    replace('drv/hal_fpga.c','uint32_t hal_time_us(void) { return nav_rd(NAV_SYS_TIME_US); }',
            'uint32_t hal_time_us(void) { return nav_rd(NAV_SYS_TIME_US); }\n'+wrappers+
            'void hal_raw_uart_config(uint32_t baud_div,uint32_t idle_us)\n{\n    nav_wr(NAV_IMU_BAUD_DIV,baud_div);nav_wr(NAV_IMU_IDLE_US,idle_us);\n}\n')
    replace('hal/hal.h','uint32_t hal_time_us(void);',
            'uint32_t hal_time_us(void);\nvoid hal_raw_uart_config(uint32_t baud_div,uint32_t idle_us);')
    replace('app/nav_app.c','bool edge;p11_raw_service(&p11,&p11_epoch,!g_host_link_guard.locked,&edge);',
            'bool edge;unsigned n=p11_raw_service(&p11,&p11_epoch,!g_host_link_guard.locked,&edge);\n        if(n)p11_last_raw_us=hal_time_us();')
    replace('app/nav_app.c','m->u.uart.session,now,true,true,p11_epoch.clean_epoch);',
            'm->u.uart.session,now,true,true,p11_epoch.clean_epoch&&\n                       p11_raw_hal_count()==0&&(uint32_t)(now-p11_last_raw_us)>=2000u);')
    modified=[];patch=[]
    for p in sorted((out/'fw').rglob('*')):
        if not p.is_file():continue
        rel=p.relative_to(out/'fw');before=SRC/'fw'/rel
        if p.read_bytes()!=before.read_bytes():
            modified.append(str(rel));patch.extend(difflib.unified_diff(
                before.read_text().splitlines(True),p.read_text().splitlines(True),
                fromfile='integrated1/fw/'+str(rel),tofile='integrated2/fw/'+str(rel)))
    (out/'binding_correction.patch').write_text(''.join(patch),encoding='utf8')
    m=json.loads((SRC/'manifest.json').read_text());m.update(
        at=datetime.now().astimezone().isoformat(),source=str(SRC),
        predecessor_preserved=True,correction_files=modified,
        corrections=['MMIO adapters live in actual HAL rather than calling private nav_rd from app',
                     'management uses current real byte count and last raw work time in addition to clean epoch'],
        stage='integrated experimental FW; HAL binding corrected, no validation/link yet',
        all_FW_bindings=[metadata(p)for p in sorted((out/'fw').rglob('*'))if p.is_file()],
        new_ELF_BIN_INIT=False,deployment_ready=False)
    save(out/'manifest.json',m)
    print(json.dumps(dict(output=str(out),corrected=modified),ensure_ascii=False))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=BASE/'P11_shared_guard_FW_integrated_experiment_prepared2')
    freeze(p.parse_args().out)
