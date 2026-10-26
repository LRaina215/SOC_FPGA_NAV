"""One real-CPU comparison/timing run for the fixed-radius candidate.

The three frontier inputs and timer/mailbox service are the prior checkpoint
probe's exact inputs. A simulation-only wrapper exposes the actual private
predicate for target-compiler extreme-input observations before timed searches.
"""
import argparse,json,subprocess,shutil
from pathlib import Path
from probe_frontier_checkpoint_cpu import BENCH,TB,PARSER,SOURCE_RTL,GCC,RUNNER

WRAPPER='''\n/* Simulation-only observation entry; private predicate body is unchanged. */
__attribute__((noinline)) bool radius_probe_candidate(int32_t fx,int32_t fy,int32_t bx,int32_t by)
{ return blacklist_within_1000(fx,fy,bx,by); }
'''

TARGET=r'''#include "fixmath.h"
#include <limits.h>
extern bool radius_probe_candidate(int32_t,int32_t,int32_t,int32_t);
static volatile int32_t radius_inputs[4];
static unsigned radius_checked;
__attribute__((noinline)) static bool radius_original(int32_t fx,int32_t fy,int32_t bx,int32_t by)
{return fx_dist(fx-bx,fy-by)<1000;}
static void radius_one(int32_t fx,int32_t fy,int32_t bx,int32_t by)
{
 radius_inputs[0]=fx;radius_inputs[1]=fy;radius_inputs[2]=bx;radius_inputs[3]=by;
 bool old=radius_original(radius_inputs[0],radius_inputs[1],radius_inputs[2],radius_inputs[3]);
 bool candidate=radius_probe_candidate(radius_inputs[0],radius_inputs[1],radius_inputs[2],radius_inputs[3]);
 if(old!=candidate)fail(1000+radius_checked);
 radius_checked++;
}
static void radius_target_observations(void)
{
 const int32_t e[]={INT32_MIN,INT32_MIN+1,-1073741825,-1073741824,-1073741823,
 -1001,-1000,-999,-1,0,1,999,1000,1001,1073741823,1073741824,1073741825,INT32_MAX-1,INT32_MAX};
 for(unsigned i=0;i<19;i++)for(unsigned j=0;j<19;j++) {
  radius_one(e[i],e[j],0,0);radius_one(e[i],0,e[j],0);radius_one(0,e[i],0,e[j]);
 }
 for(int x=-5;x<=5;x++)for(int y=-5;y<=5;y++)radius_one(600+x,800+y,0,0);
 unsigned v=0x345193ba;
 for(unsigned n=0;n<32;n++) {
  int32_t p[4];for(unsigned k=0;k<4;k++){v^=v<<13;v^=v>>17;v^=v<<5;p[k]=(int32_t)v;}
  radius_one(p[0],p[1],p[2],p[3]);
 }
 con_printf("RADIUS_TARGET_OBSERVATIONS passed checks=%u;extremes are target-compiler observations,not portable UB proof\n",radius_checked);
}
'''


def prepare(candidate,out):
    review=json.loads((candidate/'firmware_review.json').read_text());assert review['full_fw_compile_passed']
    proof=candidate/'host_proofs_radius/result.json';assert json.loads(proof.read_text())['passed']
    assert not out.exists();out.mkdir();shutil.copytree(candidate/'fw',out/'fw');fw=out/'fw';suite=out/'frontier';suite.mkdir()
    original=(fw/'nav/explore.c').read_bytes();(fw/'nav/explore.c').write_bytes(original+WRAPPER.encode())
    bench=BENCH.replace('int main(void) {',TARGET+'\nint main(void) {\n radius_target_observations();',1)
    assert bench.count('radius_target_observations();')==1
    (suite/'bench.c').write_text(bench);(suite/'tb_cpu250_segments.sv').write_text(TB)
    parser=PARSER.replace("(p/'result.json').write_text",'''obs=re.findall(r'RADIUS_TARGET_OBSERVATIONS passed checks=(\\d+)',t)
assert obs==['1236'],obs
r['radius_target_compiler_observations']={'passed':True,'checks':int(obs[0]),'portable_C_undefined_extremes_proven':False,'actual_private_helper_called_via_simulation_only_wrapper':True}
(p/'result.json').write_text''')
    (out/'parse_segments.py').write_text(parser);(out/'run_xsim.py').write_text(RUNNER);(out/'rtl').mkdir()
    for n in ['common/nav_ram_sp.v','common/nav_ram_sdp.v','soc/third_party/picorv32.v','soc/nav_soc.v']:
        shutil.copy2(SOURCE_RTL/n,out/'rtl'/Path(n).name)
    flags=['-flto','-march=rv32imc_zicsr','-mabi=ilp32','-Os','-std=c99','-ffunction-sections','-fdata-sections',
        '-fno-common','-nostartfiles','-specs=nano.specs','-DNAV_BASE=0x40000000u','-DGM_N=64','-DASTAR_MAXN=64','-DGM_DS=2']
    flags+=['-I'+str(fw/n)for n in ['include','hal','nav','app','bsp']]
    app=suite/'actual_app.o'
    commands=[[str(GCC),*flags,'-Dnav_app_search_checkpoint=nav_app_search_checkpoint_impl','-c',str(fw/'app/nav_app.c'),'-o',str(app)],
        [str(GCC),*flags,'-Wl,--gc-sections','-Wl,--defsym=RAM_SIZE=131072','-Wl,--defsym=STACK_RESERVE=6144',
        '-T',str(fw/'bsp/link.ld'),str(suite/'bench.c'),str(app),*[str(p)for p in sorted((fw/'nav').glob('*.c'))],
        str(fw/'drv/hal_fpga.c'),str(fw/'bsp/console.c'),str(fw/'bsp/start.S'),'-o',str(suite/'bench.elf'),'-lgcc']]
    for i,cmd in enumerate(commands):
        r=subprocess.run(cmd,capture_output=True,text=True);(suite/f'compile{i}.log').write_text(r.stdout+'\n'+r.stderr);assert r.returncode==0,r.stderr
    subprocess.run([str(GCC.with_name('riscv-none-elf-objcopy.exe')),'-O','binary',str(suite/'bench.elf'),str(suite/'bench.bin')],check=True)
    image=(suite/'bench.bin').read_bytes();assert len(image)<131072;image=image.ljust(131072,b'\0')
    for n in range(4):(suite/f'ram_b{n}.hex').write_text('\n'.join(f'{v:02x}'for v in image[n::4])+'\n')
    bound={n:(fw/n).read_bytes()==(candidate/'fw'/n).read_bytes()for n in ['app/nav_app.c','nav/host_link_guard.c','drv/hal_fpga.c','nav/fixmath.c','nav/scratch.h']}
    assert all(bound.values()) and (fw/'nav/explore.c').read_bytes()==original+WRAPPER.encode()
    record={'source_candidate':str(candidate),'out':str(out),'proof':str(proof),'commands':commands,
        'CPU_RTL4_bytes_bound_to_source':str(SOURCE_RTL),'actual_candidate_function_bytes_unchanged':bound,
        'explore_only_simulation_wrapper_appended':True,'target_observation_count':1236,
        'same_three_timing_inputs_and_MMIO_service_as_checkpoint_CPU_probe1':True,
        'simulators_started_by_preparation':False,'image_bytes':(suite/'bench.bin').stat().st_size,
        'whole_loop_WCET_proven':False,'physical_stop500ms_proven':False}
    (out/'preparation.json').write_text(json.dumps(record,indent=2));print(json.dumps(record,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();prepare(a.candidate.resolve(),a.out.resolve())
