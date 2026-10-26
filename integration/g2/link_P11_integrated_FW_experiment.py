"""Link an isolated P11 experiment with the selected FW's original RV flags.

The output is ELF/map/SU only: no BIN, RAM INIT, selected-source replacement,
serial access or EDA. Link failure and all actual diagnostics are retained.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,os,psutil,subprocess,time,traceback
from prepare_baseline_fusion_retime import metadata,save

BASE=Path('F:/nav_integration_g2_20261023')
REPO=Path(__file__).resolve().parents[2]
PREFIX=Path('D:/tools/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-')

def run(source,out):
    assert not out.exists(),out;out.mkdir()
    fw=source/'fw';firmware=out/'firmware';firmware.mkdir()
    inputs=[metadata(p)for p in sorted(fw.rglob('*'))if p.is_file()]
    state=dict(source=str(source),output=str(out),phase='preparing',passed=False,
               controller_pid=os.getpid(),controller_created_epoch=psutil.Process().create_time(),
               memory_at_start=dict(psutil.virtual_memory()._asdict()),
               production_image_unchanged=True,new_BIN_INIT=False,EDA=False)
    save(out/'job.json',state)
    def invoke(cmd,label):
        cmd=[str(x)for x in cmd];tool_env=os.environ.copy()
        tool_env['PATH']=str(PREFIX.parent)+';'+tool_env.get('PATH','')
        with (out/(label+'.log')).open('w',encoding='utf8')as log:
            p=subprocess.Popen(cmd,cwd=firmware,env=tool_env,stdout=log,stderr=subprocess.STDOUT)
            runtime=dict(pid=p.pid,created_epoch=psutil.Process(p.pid).create_time(),command=cmd,
                         at=datetime.now().astimezone().isoformat(),log=str(out/(label+'.log')))
            save(out/(label+'_runtime.json'),runtime);state.update(phase=label,tool=runtime);save(out/'job.json',state)
            code=p.wait();runtime.update(exit_code=code,ended=datetime.now().astimezone().isoformat())
            save(out/(label+'_exit.json'),runtime)
        return code
    try:
        srcs=sorted((fw/'nav').glob('*.c'))+[fw/'app/nav_app.c',fw/'app/main.c',fw/'drv/hal_fpga.c',fw/'bsp/console.c',fw/'bsp/start.S']
        cmd=[str(PREFIX)+'gcc.exe','-fstack-usage','-save-temps=obj','-flto',
             '-march=rv32imc_zicsr','-mabi=ilp32','-Os','-g','-std=c99',
             '-Wall','-Wno-unused-function','-ffunction-sections','-fdata-sections','-fno-common',
             '-nostartfiles','-specs=nano.specs','-DNAV_BASE=0x40000000u',
             '-DGM_N=64','-DASTAR_MAXN=64','-DGM_DS=2',
             '-Wl,--defsym=RAM_SIZE=131072','-Wl,--defsym=STACK_RESERVE=6144',
             '-Wl,--gc-sections','-Wl,-Map='+str(firmware/'nav_fw.map'),'-T',fw/'bsp/link.ld']
        cmd += ['-I'+str(fw/d)for d in('include','hal','nav','app','bsp')]
        cmd += srcs+['-o',firmware/'nav_fw.elf','-lgcc']
        save(out/'actual_link_request.json',dict(command=[str(x)for x in cmd],source_files=[str(x)for x in srcs],
             flags_match_frozen_build_rv=True,extra_compiler_flags=[],
             note='Only new CRC function has no-optimize-crc attribute; original global flags retained.'))
        code=invoke(cmd,'RV_original_flags_LTO_link');state['link_exit_code']=code
        if code:
            state.update(phase='link_failed_preserved',error=(out/'RV_original_flags_LTO_link.log').read_text(errors='replace'))
        else:
            assert (firmware/'nav_fw.elf').is_file()
            invoke([str(PREFIX)+'size.exe','-A',firmware/'nav_fw.elf'],'RV_actual_sections')
            invoke([str(PREFIX)+'nm.exe','-S','--size-sort',firmware/'nav_fw.elf'],'RV_actual_symbols')
            invoke([str(PREFIX)+'objdump.exe','-d',firmware/'nav_fw.elf'],'RV_actual_disassembly')
            stackout=out/'strict_stack_review'
            soc=BASE/'stage3_SLF128_storage_RTL_prepared6/nav/rtl/soc/nav_soc.v'
            assert soc.is_file(),soc
            sc=invoke(['D:/Anaconda/python.exe','-B','-X','utf8',REPO/'integration/g2/review_slf128_linked_stack.py',
                       firmware,'--soc',soc,'--output',stackout],'strict_linked_stack')
            state['strict_stack_exit_code']=sc
            sr=json.loads((stackout/'result.json').read_text())if(stackout/'result.json').is_file()else None
            state.update(stack_review=sr,phase='linked_stack_pending_or_failed'if sc else'linked_stack_passed',
                         passed=sc==0 and bool(sr and sr.get('passed')))
        assert inputs==[metadata(Path(x['path']))for x in inputs],'frozen input mutation'
        state['frozen_inputs_unchanged']=True
    except Exception as exc:
        state.update(phase='failed_preserved',error=str(exc),traceback=traceback.format_exc())
    state.update(at=datetime.now().astimezone().isoformat(),input_bindings=inputs,
                 actual_outputs=[metadata(p)for p in sorted(firmware.rglob('*'))if p.is_file()])
    save(out/'result.json',state);save(out/'job.json',state)
    print(json.dumps(dict(passed=state['passed'],phase=state['phase'],output=str(out),error=state.get('error')),ensure_ascii=False))
    return 0 if state['passed']else 1

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=BASE/'P11_shared_guard_FW_integrated_experiment_prepared2')
    p.add_argument('--out',type=Path,required=True);a=p.parse_args();raise SystemExit(run(a.source,a.out))
