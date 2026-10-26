"""Finite native C component contracts, not a CPU/DLL or physical test."""
from pathlib import Path
from datetime import datetime
import argparse,json,os,psutil,re,shutil,subprocess,time,traceback
from prepare_stage2_slf128_forward import OUT,BASE,load,save
from prepare_baseline_fusion_retime import metadata
from retry_frozen_synthesis import memory_status
DEST=BASE/'stage2_SLF128_FW_components_probe1';GIB=1024**3
def run(setter_cut=False):
 global DEST
 if setter_cut:DEST=BASE/'stage2_SLF128_FW_components_probe2'
 assert not DEST.exists();DEST.mkdir();fw=OUT/'fw';review=load(OUT/'prepared_source_review.json');bindings=review['all_FW_bindings']
 def frozen():
  for x in bindings:assert metadata(Path(x['path']))==x
 frozen();shutil.copyfile(Path(__file__).with_name('test_stage2_slf128_fw_contract.c'),DEST/'test.c')
 app_input=fw/'app/nav_app.c'
 if setter_cut:
  old=load(BASE/'stage2_SLF128_FW_components_probe1/result.json');assert not old['passed']
  text=app_input.read_text();body=text.split('bool nav_app_have_prior(void)',1)[1].split('void nav_app_init(void)',1)[0]
  body='bool nav_app_have_prior(void)'+body
  app_input=DEST/'actual_public_setter_cut.c';app_input.write_text('#include "nav_app.h"\n#include "hal.h"\nnav_app_t g_app;\n'+body,encoding='utf8')
  save(DEST/'source_cut.json',dict(source=str(fw/'app/nav_app.c'),byte_identical_function_body=True,cut_body=body,prior_failure_preserved=str(BASE/'stage2_SLF128_FW_components_probe1'),reason='PE native linker kept unrelated app functions despite section GC, giving missing full-app dependencies; no test executed. Test only the actual exported setter/have_prior bodies, not host dispatch.'))
 me=psutil.Process();state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),frozen_FW=str(fw),component_native_test=True,DLL_rebuilt=False,CPU_replaced_or_run=False,RTL_changed=False,SYN_started=False,PNR_started=False,programmed=False)
 def phase(n,**kw):state.update(phase=n,updated_at=datetime.now().astimezone().isoformat(),**kw);save(DEST/'job.json',state)
 def tool(label,cmd):
  while True:
   mem=memory_status();phase('waiting_memory_'+label,physical=mem.avail_phys,commit=mem.avail_page)
   if mem.avail_phys>=2*GIB and mem.avail_page>=4*GIB:break
   time.sleep(17)
  env=os.environ.copy();env['PATH']='C:/msys64/mingw64/bin'+os.pathsep+env.get('PATH','')
  with(DEST/(label+'.log')).open('w')as log:
   p=subprocess.Popen([str(x)for x in cmd],cwd=DEST,env=env,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
   rt=dict(command=[str(x)for x in cmd],pid=p.pid,created_epoch=psutil.Process(p.pid).create_time(),admission_physical_bytes=mem.avail_phys,admission_commit_bytes=mem.avail_page,started_at=datetime.now().astimezone().isoformat());phase('running_'+label,runtime=rt);p.wait();rt.update(exit_code=p.returncode,ended_at=datetime.now().astimezone().isoformat());save(DEST/(label+'_runtime.json'),rt)
  assert p.returncode==0,(label,p.returncode)
  return rt
 try:
  inc=[f'-I{fw/d}'for d in ['include','hal','nav','app','bsp']]
  cmd=['C:/msys64/mingw64/bin/gcc.exe','-std=c99','-O2','-Wall','-Wextra','-ffunction-sections','-fdata-sections','-Wl,--gc-sections','-DGM_N=64','-DGM_DS=2','-DASTAR_MAXN=64',*inc,DEST/'test.c',fw/'nav/prior.c',fw/'nav/proto.c',fw/'nav/host_link_guard.c',app_input,'-o',DEST/'contracts.exe']
  runtimes=[tool('compile',cmd),tool('execute',[DEST/'contracts.exe'])]
  text=(DEST/'execute.log').read_text();assert 'TEST PASSED SLF FW contracts' in text
  line=text.strip();counts={k:int(v)for k,v in re.findall(r'(\w+)=(\d+)',line)}
  frozen();r=dict(state,phase='native_components_complete',passed=True,counts=counts,actual_PASS_line=line,runtimes=runtimes,explicit_HAL_and_gmap_substitutes=True,actual_production_C_parser_prior_setter_guard_used=True,not_proven=['full nav_app host dispatch/real RK end-to-end','Pico firmware/busy stop latency','actual MMIO side-effect latency','physical mapping/INIT/timing/board'])
  save(DEST/'result.json',r);save(DEST/'job.json',r);print(json.dumps(dict(passed=True,counts=counts,output=str(DEST)),ensure_ascii=False));return 0
 except Exception as e:
  r=dict(state,phase='failed_preserved',passed=False,error=str(e),traceback=traceback.format_exc());save(DEST/'result.json',r);save(DEST/'job.json',r);return 1
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');p.add_argument('--setter-cut',action='store_true');a=p.parse_args();assert a.run;raise SystemExit(run(a.setter_cut))
