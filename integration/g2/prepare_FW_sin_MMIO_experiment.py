"""Freeze a software-only LUT relocation experiment and a hardware JSON plan.

This deliberately has no RTL materialization operation. The selected hardware
does not implement the proposed CPU-local ROM window.
"""
from pathlib import Path
from datetime import datetime
import argparse,difflib,json,re,shutil
from prepare_baseline_fusion_retime import metadata,save
BASE=Path('F:/nav_integration_g2_20261023');REPO=Path(__file__).resolve().parents[2]
SRC=BASE/'P11_quiesce_shared_guard_FW_prepared1'
HW=BASE/'SLF128_storage_safeFW_SYN43_prepared1'

def values(path):
    t=path.read_text();a=t[t.index('NAV_SIN_LUT[1024] = {')+len('NAV_SIN_LUT[1024] = {'):t.index('};')]
    v=[int(x)for x in re.findall(r'-?\d+',a)];assert len(v)==1024 and min(v)==-16384 and max(v)==16384
    return v

def prepare(out):
    assert not out.exists();shutil.copytree(SRC,out)
    v=values(SRC/'fw/include/nav_sin_lut.h')
    hexfile=HW/'sin_lut.hex';hv=[int(x,16)for x in hexfile.read_text().split()]
    assert len(hv)==1024 and all((a&65535)==b for a,b in zip(v,hv))
    soc=HW/'nav/rtl/soc/nav_soc.v';pico=HW/'nav/rtl/soc/third_party/picorv32.v'
    st=soc.read_text();pt=pico.read_text()
    assert 'wire sel_ram = (mem_addr[31:28] == 4\'h0)' in st
    assert "wire sel_nav = (mem_addr[31:28] == 4'h4);" in st
    assert ': {reg_op1[31:2], 2\'b00};' in pt
    assert "m_addr <= mem_addr[15:0]" in st
    helper='''/* Future CPU-local BSRAM window. Current selected FPGA has no such ROM.
 * Host tests must explicitly supply the CSR substitute; no silent table fallback.
 */
#ifndef NAV_FW_SIN_LOOKUP_H
#define NAV_FW_SIN_LOOKUP_H
#include <stdint.h>
#define NAV_FW_SIN_MMIO_BASE 0x90000000u
#ifdef NAV_FW_SIN_CSR_MODEL
int32_t nav_fw_sin_csr_model(uint32_t address);
#endif
static inline int32_t nav_fw_sin_lookup(uint32_t index)
{
    uint32_t address=NAV_FW_SIN_MMIO_BASE+((index&1023u)<<2);
#ifdef NAV_FW_SIN_CSR_MODEL
    return nav_fw_sin_csr_model(address);
#else
    return *(volatile const int32_t *)(uintptr_t)address;
#endif
}
#endif
'''
    (out/'fw/include/nav_fw_sin_lookup.h').write_text(helper,encoding='utf8')
    p=out/'fw/nav/fixmath.c';t=p.read_text();assert t.count('#include "nav_sin_lut.h"')==1
    t=t.replace('#include "nav_sin_lut.h"','#include "nav_fw_sin_lookup.h"')
    t=t.replace('NAV_SIN_LUT[a >> 6]','nav_fw_sin_lookup(a >> 6)').replace('NAV_SIN_LUT[((a >> 6) + 256) & 1023]','nav_fw_sin_lookup(((a >> 6) + 256) & 1023)')
    assert 'NAV_SIN_LUT' not in t;p.write_text(t,encoding='utf8')
    p=out/'fw/nav/traj_sw.c';t=p.read_text().replace('#include "nav_sin_lut.h"','#include "nav_fw_sin_lookup.h"')
    t=t.replace('NAV_SIN_LUT[si]','nav_fw_sin_lookup((uint32_t)si)').replace('NAV_SIN_LUT[(si + 256) & 1023]','nav_fw_sin_lookup((uint32_t)((si + 256) & 1023))')
    assert 'NAV_SIN_LUT' not in t;p.write_text(t,encoding='utf8')
    production=[p for p in (out/'fw/nav').glob('*.c')]+[out/'fw/app/nav_app.c',out/'fw/app/main.c',out/'fw/drv/hal_fpga.c']
    assert all('NAV_SIN_LUT[' not in p.read_text()for p in production)
    direct=[dict(file=str(p.relative_to(SRC)),line=i,text=line.strip())for p in (SRC/'fw').rglob('*')if p.suffix in('.c','.h')for i,line in enumerate(p.read_text().splitlines(),1)if('fx_sin('in line or'fx_cos('in line or'NAV_SIN_LUT['in line)]
    plan=dict(at=datetime.now().astimezone().isoformat(),phase='software source frozen; hardware plan only',
        source_parent=str(SRC),baseline_metadata=[metadata(p)for p in(soc,pico,hexfile,SRC/'fw/include/nav_sin_lut.h')],
        readonly_table=dict(words=1024,signed_bits=16,Q_fraction=14,byte_count=2048,all_words_equal_to_existing_hex=True),
        direct_source_call_inventory=direct,
        address_contract=dict(base='0x90000000',last='0x90000FFF',slots=1024,stride_bytes=4,
             read='aligned32 bit signed sign-extension of original int16 table[index]; one volatile lw per lookup',
             writes='ignore and ACK zero; never assert a ROM write; no command/address CSR or shared mutable query index',
             decoder='nav_soc exact mem_addr[31:12]==20h90000, before its old generic sel_per read logic',
             why_not_2byte_NAV_window='actual PicoRV32 mem_la_addr word-aligns reg_op1, so downstream cannot distinguish adjacent halfword indices; no false 0x0800 halfword scheme',
             public_overlap='documented CPU ranges0RAM/4NAV/8peripheral; selected repo/HW has no documented9 window. Old generic unmapped reads alias per_q status at low byte04/08/0c: this undocumented behavior changes inside the new exact ROM range',
             no_existing_matcher_or_host_bus_port_shared=True),
        proposed_cycle_contract=[
             'Dedicated1024x16 single read-port BSRAM, CLK=existing clk50; address=mem_addr[11:2], no new clock/global resources',
             'Demand read at E0: req && exactwindow && no pending && !cpu_rst && mem_wstrb==0; capture ROM output at E1 into per_q and assert original per_ready for one cycle',
             'Hold pending to prevent duplicate reads while mem_valid stays asserted; original per/ram/nav acceptance must be mutually exclusive for this exact range',
             'ROM output holds when read enable0, has no protocol-reset data clearing; INIT supplies the original1024 values and no runtime writer exists',
             'cpu_rst/rst/HOLD cancels only pending response/ready; no stale ACK. ROM content and q persist; first postreset ACK must follow a fresh accepted read',
             'Existing RAM128KiB, console posted writes/debug ACK, NAV arbiter and original four trigROM instances/INIT/ports remain unchanged'],
        new_resource_prediction=dict(BSRAM_source=1,physical_expected=1,data_width=16,no_32_to_16_split_expected=True,
             previous_actual_PNR53=280,predicted_total=281,limit=300,
             actual_RAM_geometry_INIT_timing_and_FF_LUT_unknown=True),
        generator_recipe=['only after root permits and both current PNR runs naturally finish: copy immutable selected source, add one dedicated nav_fw_sin_rom module and nav_soc decode/pending state',
             'Generate its INIT from original exact1024 values; reuse/check existing hex, never floating-point recompute or change old trig modules',
             'Add the new source/path and CPU-window ICD/test entries; old source bindings unchanged except this explicit patch',
             'Run actual read-hold/reset/back-to-back/ignored-write/wholeROM/Pico address miter and mapping before deployment'],
        performance_risks=['Every former RAM table load becomes a volatile CPU-local I/O load; compiler may no longer merge identical sin/cos calls, increasing read count',
             'Proposed E0-to-E1 ACK is not a measured fullCPU load latency. No NAV bus arbitration for this plan, but per_q/mem_ready integration must be proven',
             'motion_guard/support_guard and local-frame conversions execute between search checkpoints; extra reads can lengthen time before a stop service',
             'R0/P11 effective-time and350ms zero/500ms physical-stop gates remain required, including busy A*; numerical identity alone proves none of them'],
        RTL_generated_or_modified=False,EDA_started=False,selected_image_changed=False,deployment_ready=False)
    save(out/'hardware_ROM_plan.json',plan)
    patch=[];changed=[]
    for f in sorted((out/'fw').rglob('*')):
        if not f.is_file():continue
        r=f.relative_to(out/'fw');original=SRC/'fw'/r
        if not original.exists()or f.read_bytes()!=original.read_bytes():
            changed.append(str(r));patch.extend(difflib.unified_diff(original.read_text().splitlines(True)if original.exists()else [],f.read_text().splitlines(True),fromfile='quiesce/'+str(r),tofile='MMIO/'+str(r)))
    assert set(changed)=={'include\\nav_fw_sin_lookup.h','nav\\fixmath.c','nav\\traj_sw.c'},changed
    (out/'FW_sin_MMIO.patch').write_text(''.join(patch),encoding='utf8')
    save(out/'FW_sin_MMIO_manifest.json',dict(at=datetime.now().astimezone().isoformat(),source=str(SRC),changed_files=changed,
        all_FW_bindings=[metadata(p)for p in sorted((out/'fw').rglob('*'))if p.is_file()],hardware_plan=metadata(out/'hardware_ROM_plan.json'),
        same_owner_guard_and_quiesce_FW=True,RTL_generated=False,EDA=False,selected_image_changed=False,new_BIN_INIT=False,deployment_ready=False))
    print(json.dumps(dict(output=str(out),changed=changed,new_ROM_expected=1,hardware_generated=False),ensure_ascii=False))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=BASE/'P11_quiesce_FW_sin_MMIO_prepared1');prepare(p.parse_args().out)
