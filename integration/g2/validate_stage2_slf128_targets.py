"""Run the frozen frontend,128 bitwise and256 validation-only profiles in order."""
from pathlib import Path
from datetime import datetime
import argparse,json,os,psutil,re,shutil,subprocess,time,traceback
from prepare_stage2_slf128_forward import OUT,BASE,load,save
from prepare_baseline_fusion_retime import metadata
from retry_frozen_synthesis import memory_status

DEST=BASE/'stage2_SLF128_targeted_validation1';VIV=Path('E:/2026.1/Vivado/bin');GIB=1024**3
def run(parameter_fix=False):
 global DEST
 if parameter_fix:DEST=BASE/'stage2_SLF128_targeted_validation2'
 assert not DEST.exists();DEST.mkdir();val=OUT/'validation';entry=load(val/'entry.json')
 bindings=load(val/'frozen_bindings.json');rows=bindings['inputs']+bindings['hardware_inputs']
 def frozen():
  for x in rows:assert metadata(Path(x['path']))==x,('frozen input changed',x)
 if parameter_fix:
  first=load(BASE/'stage2_SLF128_targeted_validation1/result.json')
  assert not first['passed'] and first['completed_profiles'][0]['passed']
  error=(BASE/'stage2_SLF128_bitwise_probe1/xelab_stdout.log').read_text()
  assert 'work.7' in error and '-generic_top SLF_LOG2N 7' in error
  for lg in (7,8):
   p=DEST/f'tb_fixed_{lg}.sv';p.write_text(f'module tb_slf{1<<lg}_fixed;tb_slf #(.SLF_LOG2N({lg})) actual_fixture();endmodule\n',encoding='utf8');rows.append(metadata(p))
  save(DEST/'parameter_fix.json',dict(prior_failure_preserved=str(BASE/'stage2_SLF128_bitwise_probe1'),classification='Windows BAT split generic NAME=VALUE into two arguments; no128 simulation entered',fixed_wrappers_only=True,original_TB_RTL_checks_and_expressions_unchanged=True,frontend_PASS_not_repeated=True))
 frozen();me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
 state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),candidate=str(OUT),automatic_retry=False,RTL_tools_sequential=True,SYN_started=False,PNR_started=False,programmed=False)
 def status(phase,**kw):state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw);save(DEST/'job.json',state)
 def launch(folder,label,cmd,env=None):
  while True:
   mem=memory_status();status('waiting_memory_'+label,available_physical_bytes=mem.avail_phys,available_commit_bytes=mem.avail_page)
   if mem.avail_phys>=2*GIB and mem.avail_page>=4*GIB:break
   time.sleep(17)
  frozen()
  with(folder/(label+'_stdout.log')).open('w')as log:
   p=subprocess.Popen(cmd,cwd=folder,env=env,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW|subprocess.BELOW_NORMAL_PRIORITY_CLASS)
   rt=dict(command=[str(x)for x in cmd],wrapper_pid=p.pid,wrapper_created_epoch=psutil.Process(p.pid).create_time(),started_at=datetime.now().astimezone().isoformat(),admission_physical_bytes=mem.avail_phys,admission_commit_bytes=mem.avail_page)
   status('running_'+label,current_output=str(folder),runtime=rt)
   while p.poll()is None:
    try:rt['processes']=[dict(pid=q.pid,created_epoch=q.create_time(),name=q.name(),command=q.cmdline())for q in psutil.Process(p.pid).children(recursive=True)]
    except(psutil.NoSuchProcess,psutil.AccessDenied):pass
    save(folder/(label+'_runtime.json'),rt);time.sleep(1)
   rt.update(exit_code=p.returncode,ended_at=datetime.now().astimezone().isoformat());save(folder/(label+'_runtime.json'),rt)
  assert p.returncode==0,(label,p.returncode,str(folder/(label+'_stdout.log')))
  return rt
 results=[]
 try:
  profiles=[('SLF128','stage2_SLF128_bitwise_probe2',7),('SLF256_control_only','stage2_SLF256_bitwise_control_probe1',8)] if parameter_fix else [('frontend','stage2_SLF128_frontend_probe1',None),('SLF128','stage2_SLF128_bitwise_probe1',7),('SLF256_control_only','stage2_SLF256_bitwise_control_probe1',8)]
  for name,folder_name,log2 in profiles:
   folder=BASE/folder_name;assert not folder.exists();folder.mkdir();runtimes=[]
   if log2 is None:
    files=['frontend_cut.v','tb_frontend.sv']
    for n in files:shutil.copyfile(val/n,folder/n)
    top='tb_stage2_slf_frontend';options=[]
   else:
    env=os.environ.copy();env.update(SLF_SN_LOG2=str(log2),SLF_W18='1')
    if parameter_fix and log2==7:
     old=BASE/'stage2_SLF128_bitwise_probe1'
     for n in ['pts.hex','frm.hex','cand.hex','res.hex','sm.hex','smm.hex','prior.hex','pres.hex','counts.txt']:shutil.copyfile(old/n,folder/n)
     save(folder/'input_reuse.json',dict(source=str(old),generator_exit=0,exact_bytes_copied=True,reason='same frozen128 integer inputs already generated successfully before xelab argument failure'))
    else:runtimes.append(launch(folder,'generate_integer_inputs',[str(Path('D:/Anaconda/python.exe')),'-B','-X','utf8',entry['vector_generator'],str(folder)],env))
    shutil.copyfile(val/'tb_slf128.sv',folder/'tb.sv');shutil.copyfile(Path(entry['sin_lut']),folder/'sin_lut.hex')
    files=[*entry['RTL_files'],'tb.sv'];top='tb_slf';options=['-generic_top',f'SLF_LOG2N={log2}']
    if parameter_fix:
     shutil.copyfile(DEST/f'tb_fixed_{log2}.sv',folder/'wrapper.sv');files.append('wrapper.sv');top=f'tb_slf{1<<log2}_fixed';options=[]
   for tool,args in [('xvlog',['-sv',*files]),('xelab',['-debug','off','-timescale','1ns/1ps',*options,top,'-s','stage2_target']),('xsim',['stage2_target','-runall'])]:
    runtimes.append(launch(folder,tool,['cmd','/c',str(VIV/(tool+'.bat')),*args]))
   text=(folder/'xsim.log').read_text(errors='replace');marker='TEST PASSED SLF FRONTEND' if log2 is None else 'TEST PASSED'
   assert marker in text and not re.search(r'\bFatal:|TEST FAILED',text)
   markers=[x for x in text.splitlines()if x.startswith(('TEST PASSED','SLF_TIMING','$finish called')) or '一致'in x]
   frozen();result=dict(profile=name,passed=True,output=str(folder),production_SLF128=log2==7,SLF256_validation_only=log2==8,markers=markers,runtimes=runtimes,physical_mapping_timing_or_board_proof=False)
   save(folder/'result.json',result);results.append(result);status('profile_complete',profiles=results)
  result=dict(state,phase='all_targeted_profiles_complete',passed=True,profiles=results,frozen_inputs_unchanged=True,no_model_DLL_rebuilt=True);save(DEST/'result.json',result);save(DEST/'job.json',result);return 0
 except Exception as e:
  result=dict(state,phase='failed_preserved_no_retry',passed=False,error=str(e),traceback=traceback.format_exc(),completed_profiles=results);save(DEST/'result.json',result);save(DEST/'job.json',result);return 1

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');p.add_argument('--continue-parameter-fix',action='store_true');a=p.parse_args();assert a.run;raise SystemExit(run(a.continue_parameter_fix))
