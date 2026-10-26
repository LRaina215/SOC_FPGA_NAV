"""Single selected native component run and RV object-only size budget.

No full FW link/INIT, RTL simulation, synthesis, PNR or serial access.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,shutil,subprocess,psutil,traceback,os
from prepare_baseline_fusion_retime import metadata,save
BASE=Path('F:/nav_integration_g2_20261023');REPO=Path(__file__).resolve().parents[2]
CC=Path('C:/msys64/mingw64/bin/gcc.exe');RV=Path('D:/tools/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-gcc.exe')

def run(source,out):
 assert not out.exists();out.mkdir();state=dict(source=str(source),output=str(out),controller_pid=psutil.Process().pid,controller_created_epoch=psutil.Process().create_time(),passed=False,EDA=False)
 inputs=[metadata(source/'fw/nav'/n)for n in('p11_command_guard.c','p11_command_guard.h','p11_raw_service.c','p11_raw_service.h')]+[metadata(REPO/'integration/g2/tb_P11_shared_FW_component.c')]
 save(out/'job.json',state)
 def invoke(cmd,label):
  log=out/(label+'.log')
  tool_env=os.environ.copy();tool_env['PATH']=str(CC.parent)+';'+str(RV.parent)+';'+tool_env.get('PATH','')
  with log.open('w',encoding='utf8')as f:
   p=subprocess.Popen([str(x)for x in cmd],cwd=out,env=tool_env,stdout=f,stderr=subprocess.STDOUT)
   ident=dict(pid=p.pid,created_epoch=psutil.Process(p.pid).create_time(),command=[str(x)for x in cmd],log=str(log));save(out/(label+'_runtime.json'),ident)
   code=p.wait();ident.update(exit_code=code,at=datetime.now().astimezone().isoformat());save(out/(label+'_exit.json'),ident)
  assert code==0,(label,code,str(log))
  return log
 try:
  nav=source/'fw/nav';fixture=out/'tb.c';shutil.copyfile(REPO/'integration/g2/tb_P11_shared_FW_component.c',fixture)
  exe=out/'component.exe';invoke([CC,'-std=c99','-O2','-Wall','-Wextra','-Werror','-I',nav,fixture,nav/'p11_command_guard.c',nav/'p11_raw_service.c','-o',exe],'native_compile')
  log=invoke([exe],'native_actual_C');text=log.read_text();assert 'PASS P11_COMPONENT 'in text and'FAIL'not in text,text
  inv=out/'instance.c';inv.write_text('#include "p11_raw_service.h"\np11_command_guard_t instance_guard;\np11_raw_epoch_t instance_epoch;\n',encoding='utf8')
  for p in[nav/'p11_command_guard.c',nav/'p11_raw_service.c',inv]:
   invoke([RV,'-std=c99','-Os','-march=rv32im','-mabi=ilp32','-ffreestanding','-fno-builtin','-fdata-sections','-ffunction-sections','-fstack-usage','-Wall','-Wextra','-Werror','-I',nav,'-c',p,'-o',out/(p.stem+'.o')],'RV_object_'+p.stem)
  size=RV.with_name('riscv-none-elf-size.exe');nm=RV.with_name('riscv-none-elf-nm.exe')
  objects=sorted(out.glob('*.o'));invoke([size,'-A',*objects],'RV_section_sizes');invoke([nm,'-S','--size-sort',out/'instance.o'],'RV_instance_sizes')
  assert inputs==[metadata(Path(x['path']))for x in inputs]
  state.update(passed=True,phase='component_passed_full_FW_integration_pending',native_output=text.strip(),input_bindings=inputs,full_FW_linked=False,new_BIN_INIT=False,physical_stop_verified=False)
 except Exception as e:state.update(phase='failed_preserved',error=str(e),traceback=traceback.format_exc(),input_bindings=inputs)
 save(out/'result.json',state);save(out/'job.json',state);print(json.dumps(dict(passed=state['passed'],phase=state['phase'],output=str(out),error=state.get('error')),ensure_ascii=False));return 0 if state['passed']else 1

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=BASE/'P11_shared_guard_FW_experiment_prepared2');p.add_argument('--out',type=Path,required=True);a=p.parse_args();raise SystemExit(run(a.source,a.out))
