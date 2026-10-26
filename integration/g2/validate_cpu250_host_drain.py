"""Isolated cross compile and finite native validation of the frozen drain patch.

Only a new output directory is written. No CPU/EDA/synthesis/PNR is launched.
Native tests use explicit HAL substitutes and do not establish hardware timing.
"""
import argparse
import difflib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

BASE=Path('F:/nav_integration_g2_20261023')
ORIGINAL=BASE/'CPU250_frontier_radius_candidate1'
PATCHROOT=BASE/'CPU250_host_drain_static_review1'
REPO=Path(__file__).resolve().parents[2]
RISCV=Path('D:/tools/xpack-riscv-none-elf-gcc-15.2.0-1/bin')
NATIVE=Path('C:/msys64/mingw64/bin/gcc.exe')


def save(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def meta(p):
    s=p.stat();return dict(path=str(p),bytes=s.st_size,mtime_ns=s.st_mtime_ns)


def execute(out,label,cmd,cwd=None):
    base=label;attempt=1
    while (out/(label+'_command.json')).exists():
        attempt+=1;label=base+f'_attempt{attempt}'
    save(out/(label+'_command.json'),[str(x) for x in cmd])
    env=os.environ.copy()
    # cc1 is in lib/gcc, so locate this exact MinGW toolchain's dependent DLLs
    # before unrelated PATH DLLs. No install or runtime extraction is involved.
    env['PATH']=str(NATIVE.parent)+os.pathsep+env.get('PATH','')
    r=subprocess.run([str(x) for x in cmd],cwd=cwd,env=env,capture_output=True,text=True,encoding='utf-8',errors='replace')
    (out/(label+'.log')).write_text(r.stdout+r.stderr,encoding='utf-8')
    assert r.returncode==0,(label,r.returncode,r.stdout[-1500:],r.stderr[-3000:])
    return r.stdout


def compile_fw(out):
    original=ORIGINAL/'fw/app/nav_app.c';candidate=PATCHROOT/'candidate/fw/app/nav_app.c';patch=PATCHROOT/'bounded_host_drain.patch'
    original_text=original.read_text(encoding='utf-8');candidate_text=candidate.read_text(encoding='utf-8')
    actual=''.join(difflib.unified_diff(original_text.splitlines(True),candidate_text.splitlines(True),fromfile='a/fw/app/nav_app.c',tofile='b/fw/app/nav_app.c'))
    assert actual==patch.read_text(encoding='utf-8'),'Isolated candidate is not exactly the one frozen unified patch'
    assert (ORIGINAL/'firmware/nav_fw.bin').stat().st_size==52796
    fw=out/'isolated/fw';shutil.copytree(ORIGINAL/'fw',fw,ignore=shutil.ignore_patterns('build','__pycache__'))
    shutil.copy2(candidate,fw/'app/nav_app.c')
    changed=[p.relative_to(ORIGINAL/'fw').as_posix() for p in (ORIGINAL/'fw').rglob('*') if p.is_file() and 'build' not in p.relative_to(ORIGINAL/'fw').parts and '__pycache__' not in p.parts and p.read_bytes()!=(fw/p.relative_to(ORIGINAL/'fw')).read_bytes()]
    assert changed==['app/nav_app.c'],changed
    irq=REPO/'rtl/soc/nav_soc.v';copy=out/'isolated/rtl/soc/nav_soc.v';copy.parent.mkdir(parents=True);shutil.copy2(irq,copy)
    assert '.ENABLE_IRQ(0)' in copy.read_text(encoding='utf-8')
    save(out/'patch_binding.json',dict(original=str(original),candidate=str(candidate),patch=str(patch),exact_patch_equal=True,only_isolated_FW_change=changed,IRQ_source=meta(irq),IRQ_copy_bytes_equal=copy.read_bytes()==irq.read_bytes()))
    versions={}
    for label,tool in [('cross',RISCV/'riscv-none-elf-gcc.exe'),('native',NATIVE)]:
        versions[label]=execute(out,label+'_version',[tool,'--version']).splitlines()[0]
    # Run the exact frozen build driver in-process only to retain the complete
    # compiler stderr and exact commands, which that driver otherwise abbreviates.
    spec=importlib.util.spec_from_file_location('isolated_build_rv',fw/'bsp/build_rv.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    calls=[];run=subprocess.run
    def recorded(cmd,*args,**kwargs):
        r=run(cmd,*args,**kwargs)
        def text(x):return x.decode('utf-8',errors='replace') if isinstance(x,bytes) else (x or '')
        calls.append(dict(command=[str(x) for x in cmd],returncode=r.returncode,stdout=text(r.stdout),stderr=text(r.stderr)))
        return r
    cwd=Path.cwd();argv=sys.argv[:];prior=sys.path[:]
    buildout=out/'firmware';buildout.mkdir()
    try:
        os.chdir(out/'isolated');sys.path.insert(0,str(fw/'bsp'))
        sys.argv=[str(fw/'bsp/build_rv.py'),'--ram-kb','128','--stack-kb','6','--stack-usage','--grid','64,2','--output-dir',str(buildout)]
        subprocess.run=recorded;status=module.main()
    finally:
        subprocess.run=run;os.chdir(cwd);sys.argv=argv;sys.path[:]=prior
        save(out/'cross_tool_calls.json',calls)
    assert status==0,('isolated cross build',status)
    elf=buildout/'nav_fw.elf';symbols=execute(out,'linked_symbols',[RISCV/'riscv-none-elf-nm.exe','-n',elf])
    sizes=execute(out,'linked_sections',[RISCV/'riscv-none-elf-size.exe','-A',elf])
    wanted={'RAM_SIZE','STACK_RESERVE','STACK_SIZE','_bss_start','_bss_end','_stack_top'}
    syms={parts[-1]:int(parts[0],16) for line in symbols.splitlines() if len(parts:=line.split())>=3 and parts[-1] in wanted}
    bss=int(re.search(r'^\.bss\s+(\d+)\s+',sizes,re.M)[1]);image=(buildout/'nav_fw.bin').stat().st_size
    old=json.loads((ORIGINAL/'firmware_review.json').read_text());stack=json.loads((buildout/'stack_analysis.json').read_text())
    remaining=syms['_stack_top']-syms['_bss_end']-syms['STACK_RESERVE']
    result=dict(passed=remaining>=0 and stack['complete'] and stack['static_bound_bytes']<=3072,tool_versions=versions,
        linked_symbols=syms,image_bytes=image,bss_bytes=bss,load_memory_bytes=syms['_bss_end'],aligned_remaining_after_6KiB_stack=remaining,
        delta=dict(image_bytes=image-52796,bss_bytes=bss-old['layout']['bss_bytes'],bss_end=syms['_bss_end']-old['layout']['linked_symbols']['_bss_end'],
                   remaining_bytes=remaining-old['layout']['aligned_remaining_bytes'],static_stack_bound=stack['static_bound_bytes']-old['stack']['static_bound_bytes']),
        stack=stack,all_compiler_stderr=[c['stderr'] for c in calls if c['command'][0].endswith(('gcc','gcc.exe'))],
        scope='Actual RV32IMC linked image and existing conservative static stack analyzer, IRQ disabled. No runtime watermark, Pico execution or timing proof.')
    save(out/'cross_compile_result.json',result);return fw,result


def native_tests(out,fw):
    includes=[f'-I{fw/d}' for d in ('include','hal','nav','app','test','bsp')]
    common=[NATIVE,'-std=c99','-O2','-Wall','-Wextra','-Wno-unused-parameter','-ffunction-sections','-fdata-sections','-D_USE_MATH_DEFINES','-DGM_N=64','-DASTAR_MAXN=64','-DGM_DS=2','-Wl,--gc-sections']+includes
    nav=sorted((fw/'nav').glob('*.c'))
    old=BASE/'CPU250_frontier_checkpoint_candidate2/host_proofs_pending_payload_pose/app.c'
    app=out/'original_checkpoint_cases.c';shutil.copy2(old,app)
    log=out/'original_checkpoint_cases.log'
    if log.exists() and 'APP_CHECKPOINT_PROOF passed' in log.read_text():
        first=log.read_text()
    else:
        execute(out,'compile_original_checkpoint_cases',common+[app]+nav+[fw/'app/nav_app.c',fw/'test/hal_sim.c','-o',out/'original_checkpoint_cases.exe','-lm'])
        first=execute(out,'original_checkpoint_cases',[out/'original_checkpoint_cases.exe'])
    harness=Path(__file__).with_name('test_cpu250_host_drain_schedule.c').read_text(encoding='utf-8').replace('@FW@',fw.as_posix())
    c=out/'schedule.c';c.write_text(harness,encoding='utf-8')
    execute(out,'compile_schedule',common+[c]+nav+['-o',out/'schedule.exe','-lm'])
    second=execute(out,'schedule',[out/'schedule.exe'])
    result=dict(passed=True,original_checkpoint_cases_output=first,schedule_output=second,
        real_C='Unchanged candidate nav_app drain/checkpoint/on_msg/proto/host guard, all frozen nav C including explore_frontier/prior/gmap/target; no alternate scheduler reimplementation',
        substitutes='Explicit capacity16 HAL FIFO, chosen arrival timestamps, chosen logical clock/copy increments, captured HAL command requests; surrounding non-RX sensors from frozen hal_sim. Two planner entry hooks count service and inject one real checkpoint for pending-unwind ordering only.',
        limits=['Finite native traces, not real UART/Pico/FPGA throughput, cycle or 350ms acceptance.','Captured zero requests reach only HAL substitute, not physical SAFE_CMD_W or standstill.','No handler WCET, MMIO wait bound or complete nav_app_poll latency bound established.'])
    save(out/'native_result.json',result);return result


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--stage',choices=['all','compile','native'],default='all');a=ap.parse_args()
    a.out=a.out.resolve()
    if a.stage!='native':a.out.mkdir(parents=True,exist_ok=False)
    watch=sorted(p for p in (ORIGINAL/'fw').rglob('*') if p.is_file() and p.suffix in ('.c','.h','.py','.ld','.S'))
    watch += [ORIGINAL/'firmware/nav_fw.bin',ORIGINAL/'firmware/nav_fw.elf',PATCHROOT/'bounded_host_drain.patch',PATCHROOT/'candidate/fw/app/nav_app.c']
    before=[meta(p) for p in watch]
    fw,compile_result=compile_fw(a.out) if a.stage!='native' else (a.out/'isolated/fw',json.loads((a.out/'cross_compile_result.json').read_text()))
    native=native_tests(a.out,fw) if a.stage!='compile' else None
    result=dict(passed=compile_result['passed'] and (native is None or native['passed']),source=str(ORIGINAL),frozen_binary_bytes=52796,patch=str(PATCHROOT/'bounded_host_drain.patch'),
                cross_compile='cross_compile_result.json',native='native_result.json' if native else 'pending',
                frozen_inputs_unchanged=before==[meta(p) for p in watch],metadata_before=before,
                no_budget_exhaustion_fault_policy_added=True,no_EDA_or_CPU_simulation=True,no_SHA=True)
    save(a.out/'result.json',result);print(json.dumps({k:v for k,v in result.items() if k!='metadata_before'},ensure_ascii=False))
    assert result['passed'] and result['frozen_inputs_unchanged']


if __name__=='__main__':main()
