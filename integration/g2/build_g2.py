"""Reproducible joint original4500 / navigation resource build; never programs.

Both variants retain the real policy ROM, BMI, motors, servos and nav feedback.
G2 is a resource gate, not complete G3 attitude integration or vehicle acceptance.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'sim'))
from tests import ALL_RTL,SOC


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--nav',choices=['as','coverage'],required=True)
    ap.add_argument('--stage',choices=['syn','all'],default='syn')
    ap.add_argument('--packing',choices=['none','nav','control','both','ssram','hybrid','planes'],default='none')
    ap.add_argument('--command-lease',action='store_true',help='isolated command-age/stop gate, not accepted vehicle behavior')
    ap.add_argument('--wide-split',type=int,choices=[0,1],default=0,help='explicit PNR wide-RAM split; 1 duplicates every 32/36-bit block')
    a=ap.parse_args();out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    nav=out/'nav';nav.mkdir()
    shutil.copytree(ROOT/'rtl',nav/'rtl')
    if a.packing in ('nav','both'):
        shutil.copyfile(ROOT/'integration/g2/nav_ram_sdp_banked.v',nav/'rtl/common/nav_ram_sdp.v')
    elif a.packing=='ssram':
        shutil.copyfile(ROOT/'integration/g2/nav_ram_sdp_ssram.v',nav/'rtl/common/nav_ram_sdp.v')
    elif a.packing=='hybrid':
        shutil.copyfile(ROOT/'integration/g2/nav_ram_sdp_hybrid.v',nav/'rtl/common/nav_ram_sdp.v')
        shutil.copyfile(ROOT/'integration/g2/nav_ram_banked_hybrid.v',nav/'rtl/common/nav_ram_banked.v')
    elif a.packing=='planes':
        shutil.copyfile(ROOT/'integration/g2/nav_ram_sdp_planes.v',nav/'rtl/common/nav_ram_sdp.v')
    fw=out/'fw';shutil.copytree(ROOT/'fw',fw,ignore=shutil.ignore_patterns('build','__pycache__'))
    if a.nav=='coverage':
        for p in fw.rglob('*'):
            if p.suffix in ('.c','.h'):p.write_bytes(p.read_text(encoding='utf-8').encode('utf-8'))
        patch=ROOT/'sim/closed_loop/candidates/n2_support_coverage.patch'
        cmd=['git','apply','--unsafe-paths','--directory='+out.as_posix()]
        subprocess.run(cmd+['--check',str(patch)],cwd=ROOT,check=True)
        subprocess.run(cmd+[str(patch)],cwd=ROOT,check=True)
    with (out/'firmware.log').open('w',encoding='utf-8') as log:
        subprocess.run([sys.executable,'-X','utf8',str(fw/'bsp/build_rv.py'),'--output-dir',str(out/'firmware'),
                        '--copy-to',str(out),'--ram-kb','128'],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True)
    control=ROOT/'integration/control4500/original_src'
    shutil.copytree(control/'mem',out/'src/mem')
    shutil.copyfile(ROOT/'rtl/plan/sin_lut.hex',out/'sin_lut.hex')
    top=(ROOT/'integration/g2/g2_top.v').read_text(encoding='utf-8')
    top=top.replace('`include "soc_cfg.vh"','`define NAV_SOC_RAM_KB 128\n`define G2_COVERAGE '+str(int(a.nav=='coverage')))
    if a.command_lease:top='`define G2_COMMAND_LEASE\n'+top
    (out/'g2_top.v').write_text(top,encoding='utf-8')
    shutil.copyfile(ROOT/'integration/g2/g2.cst',out/'g2.cst')
    shutil.copyfile(ROOT/'integration/g2/g2.sdc',out/'g2.sdc')
    shutil.copyfile(ROOT/'integration/g2/control4500_core.v',out/'control4500_core.v')
    shutil.copyfile(ROOT/'integration/g2/command_lease_guard.v',out/'command_lease_guard.v')
    shutil.copytree(control,out/'control')
    files=[nav/x for x in ALL_RTL+SOC+['rtl/net/rgmii_mac100.v']]
    files += [p for p in (out/'control').glob('*.v') if p.name!='main.v']
    files += [out/'control4500_core.v',out/'command_lease_guard.v',out/'g2_top.v']
    if a.packing in ('control','both','ssram','hybrid'):
        from optimize_storage import prepare
        prepare(out)
        files=[p for p in files if p!=out/'control/fc_v4_top.v']
        files += [out/'fc_v4_top.v',out/'fixed4500_weight_rom.v']
    tcl=['set_device -device_version C GW5AT-LV138PG484AC1/I0']
    tcl += [f'add_file -type verilog {{{p.as_posix()}}}' for p in files]
    tcl += [f'add_file -type cst {{{(out/"g2.cst").as_posix()}}}',
            f'add_file -type sdc {{{(out/"g2.sdc").as_posix()}}}',
            f'set_option -include_path {{{(nav/"rtl/regs").as_posix()}}}',
            'set_option -top_module g2_top','set_option -output_base_name g2_top',
            'set_option -verilog_std v2001','set_option -frequency 50','set_option -print_all_synthesis_warning 1',
            'set_option -looplimit 100000','set_option -convert_sdp32_36_to_sdp16_18 '+str(a.wide_split)]
    tcl += ['set_option -'+x+' 1' for x in ['use_sspi_as_gpio','use_mspi_as_gpio','use_ready_as_gpio','use_done_as_gpio','use_i2c_as_gpio','use_cpu_as_gpio']]
    tcl += ['set_option -place_option 1','set_option -route_option 0','run '+a.stage]
    (out/'run.tcl').write_text('\n'.join(tcl)+'\n',encoding='utf-8')
    manifest=dict(nav_variant=a.nav,coverage_enabled=a.nav=='coverage',stage=a.stage,packing=a.packing,command_lease=a.command_lease,wide_split=a.wide_split,
        device='GW5AT-LV138PG484AC1/I0',device_version='C',clock_core_mhz=50,synthesis_frequency_hint_mhz=50,
        control_commit='6443f5678b4de222c22cbf331b8396f7c8df6a41',control_weights_changed=False,
        source_files=[str(p) for p in files],programmed=False,
        scope='G2 resource gate, not complete G3 nav IMU adaptation or real vehicle acceptance')
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    tool=os.environ.get('GW_SH',r'D:\Gowin2\Gowin_V1.9.12.04_x64\IDE\bin\gw_sh.exe')
    print('joint build prepared: '+str(out),flush=True)
    with (out/'gw_sh.log').open('w',encoding='utf-8') as log:
        result=subprocess.run([tool,'run.tcl'],cwd=out,stdout=log,stderr=subprocess.STDOUT)
    text=(out/'gw_sh.log').read_text(encoding='utf-8',errors='replace')
    errors=[line for line in text.splitlines() if 'ERROR' in line]
    print('\n'.join(errors[:15]) if errors else text[-1000:])
    print('Gowin process return code: '+str(result.returncode))
    return result.returncode or bool(errors)


if __name__=='__main__':raise SystemExit(main())
