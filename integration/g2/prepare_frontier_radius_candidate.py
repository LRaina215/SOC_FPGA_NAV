"""Prepare/prove a fixed1000mm blacklist predicate with a strict fast domain.

Only the blacklist boolean changes. Large coordinate differences go to the
literal original expression; this does not redefine original signed-overflow
or uint32-to-int32 conversion corners as portable C behavior.
"""
import argparse,json,subprocess,shutil,os,sys,struct
from pathlib import Path
from datetime import datetime
from prepare_frontier_checkpoint_candidate import BASE,GCC
from prove_frontier_checkpoint_candidate import ALGORITHM,PUBLIC

SOURCE=Path('F:/nav_integration_g2_20261023/CPU250_frontier_checkpoint_candidate2')
ORIGINAL=Path('F:/nav_integration_g2_20261023/baseline_fusion_preview_safety250_emptyIMU_prepared3')
HELPER='''/* Fast domain guarantees the original signed arithmetic and result are
 * representable. Outside it retain the original expression,including its
 * implementation-specific/extreme-input behavior;do not change global flags. */
static bool blacklist_within_1000(int32_t fx,int32_t fy,int32_t bx,int32_t by)
{
    int64_t dx=(int64_t)fx-bx,dy=(int64_t)fy-by;
    const int64_t bound=(int64_t)1<<30;
    if(dx>=-bound && dx<=bound && dy>=-bound && dy<=bound) {
        if(dx<=-1000 || dx>=1000 || dy<=-1000 || dy>=1000) return false;
        int32_t x=(int32_t)dx,y=(int32_t)dy;
        return x*x+y*y<1000000;
    }
    return fx_dist(fx-bx,fy-by)<1000;
}

'''

MATH=r'''#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include "nav_search_checkpoint.h"
bool g_nav_search_interrupted;
bool nav_app_search_checkpoint(void) {return true;}
#include "@EXPLORE@"
static volatile int32_t input[4];
static unsigned checked,fast,slow;
__attribute__((noinline)) static bool old_pred(int32_t fx,int32_t fy,int32_t bx,int32_t by) {
 return fx_dist(fx-bx,fy-by)<1000;
}
static void one(int32_t fx,int32_t fy,int32_t bx,int32_t by) {
 input[0]=fx;input[1]=fy;input[2]=bx;input[3]=by;
 bool a=old_pred(input[0],input[1],input[2],input[3]);
 bool b=blacklist_within_1000(input[0],input[1],input[2],input[3]);
 if(a!=b) {fprintf(stderr,"Mismatch %d %d %d %d %u %u\n",fx,fy,bx,by,a,b);exit(1);}
 int64_t dx=(int64_t)fx-bx,dy=(int64_t)fy-by,B=(int64_t)1<<30;
 if(dx>=-B&&dx<=B&&dy>=-B&&dy<=B)fast++;else slow++;
 checked++;
}
static uint32_t random32(void) {static uint32_t v=0x345193ba;v^=v<<13;v^=v>>17;v^=v<<5;return v;}
int main(void) {
 const int32_t edge[]={INT32_MIN,INT32_MIN+1,-1073741825,-1073741824,-1073741823,
 -1001,-1000,-999,-1,0,1,999,1000,1001,1073741823,1073741824,1073741825,INT32_MAX-1,INT32_MAX};
 for(unsigned i=0;i<sizeof edge/sizeof edge[0];i++)for(unsigned j=0;j<sizeof edge/sizeof edge[0];j++) {
  one(edge[i],edge[j],0,0);one(edge[i],0,edge[j],0);one(0,edge[i],0,edge[j]);
 }
 /* Circle equality and both adjacent integer rows;strict radius never changes. */
 for(int dx=-1001;dx<=1001;dx++) {
  unsigned xx=(unsigned)(dx*dx);
  int y=xx<=1000000?(int)fx_isqrt(1000000-xx):0;
  for(int d=-1;d<=1;d++) {one(dx,y+d,0,0);one(dx,-y+d,0,0);}
 }
 for(unsigned i=0;i<4096;i++)one((int32_t)random32(),(int32_t)random32(),(int32_t)random32(),(int32_t)random32());
 printf("RADIUS_NATIVE passed checked=%u fast=%u fallback=%u;extreme samples are compiler-specific observations,not portable UB proof\n",checked,fast,slow);
 return 0;
}
'''


def prove(out):
    fw=out/'fw';p=out/'radius_proofs';p.mkdir();cc=shutil.which('gcc');assert cc
    env=os.environ.copy();env['PATH']=str(Path(cc).parent)+';'+env.get('PATH','')
    flags=['-std=c99','-O2','-ffunction-sections','-fdata-sections','-Wl,--gc-sections']
    flags+=['-I'+str(fw/n)for n in ['include','hal','nav','app','bsp']]
    def run(cmd,name):
        r=subprocess.run(cmd,cwd=p,capture_output=True,text=True,env=env)
        (p/(name+'.json')).write_text(json.dumps({'command':cmd,'returncode':r.returncode,'stdout':r.stdout,'stderr':r.stderr},indent=2))
        assert r.returncode==0,(name,r.stderr,r.stdout)
        return r.stdout
    # Include the actual new helper text,not a reimplementation of its math.
    (p/'math.c').write_text(MATH.replace('@EXPLORE@',(fw/'nav/explore.c').as_posix()))
    deps=[str(fw/'nav'/n)for n in ['scratch.c','astar.c','gmap.c','fixmath.c']]
    run([cc,*flags,str(p/'math.c'),*deps,'-o',str(p/'math.exe')], 'compile_math')
    math=run([str(p/'math.exe')],'math_result')
    ref=p/'reference_explore.o'
    run([cc,*flags,*['-D'+n+'=ref_'+n for n in PUBLIC],'-c',str(ORIGINAL/'fw/nav/explore.c'),'-o',str(ref)],'compile_reference')
    # Retain the existing full-map/tie/queue/cancel proof;lease code is byte-
    # identical to checkpoint2 so its separate protocol proof is reused.
    alg=ALGORITHM[:ALGORITHM.index(' proto_msg_t heartbeat=')]
    alg+=' printf("RADIUS_ALGORITHM passed cases=%u cancel_restart=%u checks=%u scratch_bytes=%u\\n",cases,cancel_cases,checks,(unsigned)sizeof g_scratch);return 0;\n}\n'
    (p/'algorithm.c').write_text(alg)
    run([cc,*flags,str(p/'algorithm.c'),str(ref),str(fw/'nav/explore.c'),*deps,'-o',str(p/'algorithm.exe')],'compile_algorithm')
    algorithm=run([str(p/'algorithm.exe')],'algorithm_result')
    unchanged={n:(fw/n).read_bytes()==(SOURCE/'fw'/n).read_bytes()for n in ['app/nav_app.c','nav/nav_task.c','nav/host_link_guard.c','drv/hal_fpga.c','nav/scratch.h','nav/nav_search_checkpoint.h']}
    assert all(unchanged.values())
    existing=json.loads((SOURCE/'host_proofs_pending_payload_pose/result.json').read_text());assert existing['passed']
    result={'passed':True,'math_native':math.strip(),'original_algorithm':algorithm.strip(),
        'checkpoint_guard_and_FIFO_protocol_proof_reused_from':str(SOURCE/'host_proofs_pending_payload_pose/result.json'),
        'reused_source_byte_identity':unchanged,'source_candidate':str(out),'actual_CPU_timing_pending':True,
        'portable_C_defined_fast_domain_proven':True,'original_undefined_C_extremes_made_defined':False,
        'outside_fast_domain_literal_original_expression':True}
    (p/'result.json').write_text(json.dumps(result,indent=2))
    # Existing CPU preparation accepts a completed focused proof in this prefix.
    (out/'host_proofs_radius').mkdir();(out/'host_proofs_radius/result.json').write_text(json.dumps(result,indent=2))
    return result


def review_existing_firmware(out):
    """Review the already-built ELF using its exact section alignment.

    The earlier reader assumed all BSS padding was below four bytes. This
    candidate has eight-byte BSS alignment,so validate the exact ELF geometry
    instead of tolerating an arbitrary larger gap. No compiler invocation.
    """
    fw=out/'fw';built=out/'firmware';elf=built/'nav_fw.elf'
    compiled=json.loads((built/'compile.json').read_text());assert compiled['returncode']==0
    data=elf.read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',data)
    assert h[0][:7]==b'\x7fELF\x01\x01\x01' and h[1:4]==(2,243,1)
    loads=[struct.unpack_from('<IIIIIIII',data,h[5]+i*h[9])for i in range(h[10])]
    loads=[p for p in loads if p[0]==1 and p[4]]
    assert len(loads)==1 and loads[0][2]==loads[0][3]==0
    p=loads[0];image=data[p[1]:p[1]+p[4]];assert len(image)==p[4]
    sections=[struct.unpack_from('<IIIIIIIIII',data,h[6]+i*h[11])for i in range(h[12])]
    symbols={}
    for s in sections:
        if s[1]!=2:continue
        st=sections[s[6]];names=data[st[4]:st[4]+st[5]]
        for off in range(s[4],s[4]+s[5],s[9]):
            name,value,size,info,other,index=struct.unpack_from('<IIIBBH',data,off)
            symbols[names[name:].split(b'\0',1)[0].decode()]=(value,size,index)
    v={k:symbols[k][0]for k in ['RAM_SIZE','STACK_RESERVE','_bss_start','_bss_end','_stack_top']}
    assert v['RAM_SIZE']==v['_stack_top']==131072 and v['STACK_RESERVE']==6144
    bss=sections[symbols['_bss_start'][2]];align=bss[8]
    assert bss[1]==8 and align>0 and align&(align-1)==0
    assert bss[3]==v['_bss_start'] and bss[3]+bss[5]==v['_bss_end']
    allocated_file_sections=[s for s in sections if s[2]&2 and s[1]!=8 and s[5]]
    assert max(s[3]+s[5]for s in allocated_file_sections)==len(image)
    assert v['_bss_start']==(len(image)+align-1)&~(align-1)
    assert p[5]==v['_bss_end'] and v['_bss_end']<=131072-6144
    assert v['_bss_start']%4==v['_bss_end']%4==0
    scratch=symbols['g_scratch'];assert scratch[1]==20992 and scratch[2]==symbols['_bss_start'][2]
    (built/'nav_fw.bin').write_bytes(image)
    sys.path.insert(0,str(fw/'bsp'));from analyze_stack import analyze
    stack=analyze(elf,GCC.with_name('riscv-none-elf-objdump.exe'),next(built.glob('*.ltrans.su')),
                  next(built.glob('*.ltrans.s')),'.ENABLE_IRQ(0)' in (BASE/'nav/rtl/soc/nav_soc.v').read_text())
    (built/'stack_analysis.json').write_text(json.dumps(stack,indent=2))
    assert stack['complete'] and stack['static_bound_bytes']<=3072,stack
    layout={'linked_symbols':v,'load_file_bytes':p[4],'load_memory_bytes':p[5],
        'bss_section_alignment':align,'exact_alignment_padding_bytes':v['_bss_start']-len(image),
        'bss_bytes':bss[5],'aligned_remaining_bytes':131072-6144-v['_bss_end']}
    result={'image_bytes':len(image),'layout':layout,'stack':stack,'full_fw_compile_passed':True,
        'source_only_not_hardware_candidate':True,'new_snapshot_arrays':False,'scratch_bytes':scratch[1],
        'legacy_reader_rejection_preserved':(built/'first_image_reader_rejection.json').exists(),
        'BSS_padding_verified_from_exact_ELF_section_alignment':True}
    (out/'firmware_review.json').write_text(json.dumps(result,indent=2));return result


def compile_firmware(out):
    fw=out/'fw';built=out/'firmware';built.mkdir()
    prior=json.loads((BASE/'firmware_compile.json').read_text())['command'];command=[];next_output=False
    for arg in prior:
        if next_output:command.append(str(built/'nav_fw.elf'));next_output=False;continue
        if arg=='-o':command.append(arg);next_output=True;continue
        if arg.startswith('-Wl,-Map='):command.append('-Wl,-Map='+str(built/'nav_fw.map'));continue
        normalized=arg.replace('\\','/')
        if '/fw/' in normalized:command.append(('-I' if arg.startswith('-I')else '')+str(fw/normalized.split('/fw/',1)[1]))
        else:command.append(arg)
    run=subprocess.run(command,cwd=built,capture_output=True,text=True,encoding='utf-8')
    (built/'compile.json').write_text(json.dumps({'command':command,'returncode':run.returncode,'stdout':run.stdout,'stderr':run.stderr},indent=2))
    assert run.returncode==0,run.stderr
    return review_existing_firmware(out)


def finish_existing(out):
    assert (out/'radius_proofs/result.json').exists()
    assert (out/'radius_proofs/result.json').read_bytes()==(out/'host_proofs_radius/result.json').read_bytes()
    changed=[p.relative_to(out/'fw').as_posix()for p in (out/'fw').rglob('*')if p.is_file() and p.read_bytes()!=(SOURCE/'fw'/p.relative_to(out/'fw')).read_bytes()]
    assert changed==['nav/explore.c'],changed
    old=(SOURCE/'fw/nav/explore.c').read_text(encoding='utf-8-sig')
    expected=old.replace('bool explore_frontier(explore_t *e,',HELPER+'bool explore_frontier(explore_t *e,',1).replace(
        'fx_dist(fx - e->bl_x[b], fy - e->bl_y[b]) < 1000','blacklist_within_1000(fx,fy,e->bl_x[b],e->bl_y[b])',1)
    assert (out/'fw/nav/explore.c').read_text(encoding='utf-8')==expected
    result={'recorded_at':datetime.now().astimezone().isoformat(),'source':str(SOURCE),'out':str(out),
        'only_existing_FW_changed':'nav/explore.c','only_predicate_changed':'blacklist fixed1000mm',
        'frontier_min_predicate_unchanged':True,'new_arrays_or_coordinate_tables':0,
        'mathematical_basis':json.loads((out/'mathematical_basis.json').read_text()),
        'focused_proof':json.loads((out/'radius_proofs/result.json').read_text()),
        'no_original_FW_board_or_joint_build_mutations':True,'full_FW':review_existing_firmware(out)}
    (out/'preparation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))


def prepare(out,compile_full=True):
    assert not out.exists();out.mkdir();shutil.copytree(SOURCE/'fw',out/'fw')
    f=out/'fw/nav/explore.c';old=f.read_text(encoding='utf-8-sig')
    anchor='bool explore_frontier(explore_t *e,';assert old.count(anchor)==1
    new=old.replace(anchor,HELPER+anchor,1)
    call='fx_dist(fx - e->bl_x[b], fy - e->bl_y[b]) < 1000';assert new.count(call)==1
    new=new.replace(call,'blacklist_within_1000(fx,fy,e->bl_x[b],e->bl_y[b])',1);f.write_text(new,encoding='utf-8')
    import difflib
    (out/'candidate.patch').write_text(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='checkpoint2/nav/explore.c',tofile='radius/nav/explore.c')),encoding='utf-8')
    math={'safe_difference':'int64 differences of two int32 values are representable',
      'fast_domain':'abs(dx)<=2^30 and abs(dy)<=2^30',
      'domain_bound':'dx^2+dy^2<=2^61<INT64_MAX;floor sqrt<=floor(sqrt(2^61))<INT32_MAX',
      'identity':'For integer R=1000 and nonnegative q:floor(sqrt(q))<R iff q<R^2',
      'AABB_reject':'Any abs(dx)>=1000 or abs(dy)>=1000 implies q>=1000000',
      'inner_arithmetic':'Remaining x,y are within[-999,999],so x*x+y*y<=1996002<INT32_MAX;32bit products/sum exact',
      'fallback':'Literal original fx_dist(fx-bx,fy-by)<1000 for all values outside the fast domain',
      'original_C_limits':'Original coordinate subtraction and signed64 sum can overflow;large uint32 sqrt cast to int32 is implementation-defined. No claim of portable-C all-input equivalence at those undefined corners;fallback retains original target expression.',
      'global_compiler_wrap_flags_changed':False}
    (out/'mathematical_basis.json').write_text(json.dumps(math,indent=2))
    proof=prove(out)
    result={'recorded_at':datetime.now().astimezone().isoformat(),'source':str(SOURCE),'out':str(out),
      'only_existing_FW_changed':'nav/explore.c','only_predicate_changed':'blacklist fixed1000mm',
      'frontier_min_predicate_unchanged':True,'new_arrays_or_coordinate_tables':0,'mathematical_basis':math,
      'focused_proof':proof,'no_original_FW_board_or_joint_build_mutations':True}
    if compile_full:result['full_FW']=compile_firmware(out)
    (out/'preparation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--review-existing',action='store_true');a=p.parse_args()
    (finish_existing if a.review_existing else prepare)(a.out.resolve())
