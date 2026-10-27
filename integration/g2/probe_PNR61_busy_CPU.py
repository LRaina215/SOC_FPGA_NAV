"""Real PicoRV32/current FW functions under finite busy-frontier lease inputs.

Instrumentation wraps checkpoints; timer/mailbox/SAFE CSRs are explicit test
fixtures. This is not a full UART/G2 or mechanical stopping simulation.
"""
from datetime import datetime
import json
from pathlib import Path
import shutil
import subprocess
import sys

from probe_frontier_checkpoint_cpu import BENCH,TB,PARSER
from prepare_cpu250_segment_probe import RUNNER
from retry_frozen_synthesis import memory_status

ROOT=Path(__file__).resolve().parents[2]
BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'K_timing_syn49'
OUT=BASE/'PNR61_current_busy_CPU_probe1'
GCC=Path('D:/tools/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-gcc.exe')


def main():
    assert not OUT.exists();OUT.mkdir()
    fw=OUT/'fw';shutil.copytree(SOURCE/'fw',fw,ignore=shutil.ignore_patterns('build','__pycache__'))
    suite=OUT/'frontier';suite.mkdir();(OUT/'rtl').mkdir()
    bench=BENCH.replace('kind<3','kind<4').replace('kind==2?now-249000u:now','kind>=2?now-249000u:now')
    bench=bench.replace('diag(0x7ff0,kind==2?2:1);','diag(0x7ff0,kind==2?2:kind==3?3:1);')
    bench=bench.replace('if(kind==2 &&','if(kind>=2 &&')
    assert bench!=BENCH and 'kind<4' in bench
    tb=TB.replace("read_word==0?32'h2001564e:0","read_word==0?(profile==3?32'h7f01564e:32'h2001564e):0")
    tb=tb.replace("avail<=0;read_word<=0;moving_seen<=0;zero_seen<=0;","avail<=wd==3;rx_ts<=now_us;read_word<=0;moving_seen<=0;zero_seen<=0;")
    tb=tb.replace('profile==2&&moving_seen','(profile==2||profile==3)&&moving_seen')
    parser=PARSER.replace('len(rows)==3 and len(zero)==1','len(rows)==4 and len(zero)==2')
    (suite/'bench.c').write_text(bench);(suite/'tb_cpu250_segments.sv').write_text(tb)
    (OUT/'parse_segments.py').write_text(parser);(OUT/'run_xsim.py').write_text(RUNNER)
    bindings=[]
    for rel in ['common/nav_ram_sp.v','common/nav_ram_sdp.v','soc/third_party/picorv32.v','soc/nav_soc.v']:
        src=SOURCE/'nav/rtl'/rel;dst=OUT/'rtl'/src.name
        shutil.copyfile(src,dst);assert dst.read_bytes()==src.read_bytes()
        bindings.append(dict(source=str(src),bytes=src.stat().st_size))
    flags=['-flto','-march=rv32imc_zicsr','-mabi=ilp32','-Os','-std=c99','-ffunction-sections','-fdata-sections',
        '-fno-common','-nostartfiles','-specs=nano.specs','-DNAV_BASE=0x40000000u','-DGM_N=64','-DASTAR_MAXN=64','-DGM_DS=2',
        '-DAPP_IMU_PROTO=1','-DAPP_IMU_BAUD_DIV=434']
    flags+=['-I'+str(fw/n) for n in ('include','hal','nav','app','bsp')]
    app=suite/'actual_app.o'
    cmds=[[str(GCC),*flags,'-Dnav_app_search_checkpoint=nav_app_search_checkpoint_impl','-c',str(fw/'app/nav_app.c'),'-o',str(app)],
        [str(GCC),*flags,'-Wl,--gc-sections','-Wl,--defsym=RAM_SIZE=131072','-Wl,--defsym=STACK_RESERVE=6144',
        '-T',str(fw/'bsp/link.ld'),str(suite/'bench.c'),str(app),*[str(p) for p in sorted((fw/'nav').glob('*.c'))],
        str(fw/'drv/hal_fpga.c'),str(fw/'bsp/console.c'),str(fw/'bsp/start.S'),'-o',str(suite/'bench.elf'),'-lgcc']]
    for i,cmd in enumerate(cmds):
        r=subprocess.run(cmd,capture_output=True,text=True);(suite/f'compile{i}.log').write_text(r.stdout+r.stderr)
        assert r.returncode==0,r.stderr
    subprocess.run([str(GCC.with_name('riscv-none-elf-objcopy.exe')),'-O','binary',str(suite/'bench.elf'),str(suite/'bench.bin')],check=True)
    image=(suite/'bench.bin').read_bytes();assert len(image)<131072
    image=image.ljust(131072,b'\0')
    for n in range(4):(suite/f'ram_b{n}.hex').write_text('\n'.join(f'{v:02x}' for v in image[n::4])+'\n')
    assert all((fw/n).read_bytes()==(SOURCE/'fw'/n).read_bytes() for n in [
        'app/nav_app.c','nav/explore.c','nav/host_link_guard.c','nav/proto.c','drv/hal_fpga.c'])
    m=memory_status();assert m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3
    record=dict(started_at=datetime.now().astimezone().isoformat(),source=str(SOURCE),prepared=True,
        actual_current_app_guard_HAl_explore_bytes_unchanged=True,actual_current_CPU_RAM_RTL=bindings,
        scenarios=['healthy known map','healthy eight blacklists','lease loss in busy frontier','invalid arrival cannot renew busy lease'],
        physical_memory=m.avail_phys,commit_memory=m.avail_page,board_access=False,not_a_full_APP_WCET_proof=True)
    (OUT/'preparation.json').write_text(json.dumps(record,indent=2))
    r=subprocess.run([sys.executable,'-B','-X','utf8',str(OUT/'run_xsim.py'),'--suite','frontier'],cwd=OUT)
    assert r.returncode==0
    result=json.loads((suite/'result.json').read_text());assert result['passed']
    print(json.dumps(dict(passed=True,out=str(OUT),rows=result['rows'],zero_commit=result['zero_commit'])))


if __name__=='__main__':main()
