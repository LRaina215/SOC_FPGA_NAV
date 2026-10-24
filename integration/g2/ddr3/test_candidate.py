"""Bounded, sequential functional checks. No PNR or board programming."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import subprocess
import shutil

HERE=Path(__file__).resolve().parent
VIV=Path('E:/2026.1/Vivado/bin')


def run(out,top,files):
    for f in HERE.glob('*.sv'):shutil.copyfile(f,out/f.name)
    def tool(n,args):
        with (out/(n+'_stdout.log')).open('w') as log:
            subprocess.run(['cmd','/c',str(VIV/(n+'.bat')),*args],cwd=out,stdout=log,
                stderr=subprocess.STDOUT,timeout=180,check=True)
    tool('xvlog',[str(out/f) for f in files if f.endswith('.v')])
    tool('xvlog',['-sv',str(out/'ddr_native_model.sv'),str(out/(top+'.sv'))])
    tool('xelab',['-debug','off','-timescale','1ns/1ps',top,'-s','snap'])
    tool('xsim',['snap','-runall'])
    s=(out/'xsim_stdout.log').read_text(errors='replace')
    assert 'TEST PASSED' in s and not re.search(r'Fatal|FATAL|ERROR:',s),s[-3000:]
    return re.search(r'TEST PASSED[^\r\n]*',s)[0]


def cpu_program(out):
    # Exercise the real PicoRV32 load/store, subword enables and MMIO path.
    asm='''.section .text
.global _start
_start:
    li t0,0x1f000
    li t1,1024
    li t2,0x4500138c
1:  sw t2,0(t0)
    sb t1,1(t0)
    sh t1,2(t0)
    addi t0,t0,4
    addi t2,t2,1
    addi t1,t1,-1
    bnez t1,1b
    li t0,0x1f000
    li t1,1024
    li t3,0
2:  lw t2,0(t0)
    xor t3,t3,t2
    addi t0,t0,4
    addi t1,t1,-1
    bnez t1,2b
    li t0,0x40000124
    sw t3,0(t0)
    lw t2,0(t0)
    xor t2,t2,t3
    li t0,0x80000008
    sw t2,0(t0)
    li t0,0x80000004
    li t1,15
    sw t1,0(t0)
    ebreak
3:  j 3b
'''
    (out/'cpu_program.S').write_text(asm)
    prefix='D:/tools/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-'
    subprocess.run([prefix+'gcc.exe','-march=rv32imc','-mabi=ilp32','-nostdlib','-Wl,-Ttext=0',
        str(out/'cpu_program.S'),'-o',str(out/'cpu_program.elf')],check=True,capture_output=True,timeout=30)
    subprocess.run([prefix+'objcopy.exe','-O','binary',str(out/'cpu_program.elf'),str(out/'cpu_program.bin')],check=True,timeout=30)
    image=(out/'cpu_program.bin').read_bytes();image+=bytes((-len(image))%4)
    words=[int.from_bytes(image[k:k+4],'little') for k in range(0,len(image),4)]
    assert len(words)<255
    (out/'cpu_program.hex').write_text(''.join(f'{w:08x}\n' for w in words+[0]*(255-len(words))+[len(words)]))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('candidate',type=Path)
    p.add_argument('--soc-only',action='store_true');a=p.parse_args()
    out=a.candidate.resolve()
    r=dict(tested_at=datetime.now().astimezone().isoformat(),programmed=False,
        scope='functional app-interface model; no DDR PHY, physical timing or joint RL/nav acceptance',checks=[])
    if a.soc_only:
        for f in ('nav_ddr_cache.v','nav_ddr_native.v','nav_ddr_boot_rom.v'):
            assert (out/f).read_bytes()==(HERE/f).read_bytes(),'stale tested backend'
        r['checks']=json.loads((out/'memory_result.json').read_text())['checks']
    else:
        r['checks'].append(run(out,'tb_ddr_memory',['nav_ddr_boot_rom.v','nav_ddr_cache.v','nav_ddr_native.v']))
        (out/'memory_result.json').write_text(json.dumps(r,indent=2),encoding='utf-8')
        for f in ('xsim_stdout.log','xvlog_stdout.log','xelab_stdout.log'):
            shutil.copyfile(out/f,out/('memory_'+f))
    cpu_program(out)
    r['checks'].append(run(out,'tb_ddr_soc',['nav_soc_baseline.v','nav_soc_ddr.v',
        'nav_ddr_boot_rom.v','nav_ddr_cache.v','nav_ddr_native.v','nav_ram_sp.v','nav_ram_sdp.v','picorv32.v']))
    (out/'functional_result.json').write_text(json.dumps(r,indent=2),encoding='utf-8')
    print(json.dumps(r,indent=2))
