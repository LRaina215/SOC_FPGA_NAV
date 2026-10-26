"""Compile the selected frozen RV firmware to a new directory, never a DLL/EDA."""
from pathlib import Path
from datetime import datetime
import argparse,importlib.util,json,os,psutil,subprocess,sys,threading,time,traceback
from prepare_baseline_fusion_retime import metadata
from prepare_stage2_slf128_forward import OUT,SRC,load,save
from retry_frozen_synthesis import memory_status

BASE=OUT.parent;DEST=BASE/'stage2_SLF128_safeFW_RV_compile1';GIB=1024**3
def run():
 assert not DEST.exists();DEST.mkdir();review=load(OUT/'prepared_source_review.json')
 bindings=review['hardware_bindings']+review['all_FW_bindings']
 def frozen():
  for x in bindings:assert metadata(Path(x['path']))==x,('frozen input changed',x)
 frozen();me=psutil.Process();state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),candidate=str(OUT),output=str(DEST),FW_only=True,model_DLL_untouched=True,EDA_started=False,automatic_retry=False)
 def status(phase,**kw):state.update(phase=phase,at=datetime.now().astimezone().isoformat(),**kw);save(DEST/'job.json',state)
 try:
  while True:
   mem=memory_status();status('waiting_memory',available_physical_bytes=mem.avail_phys,available_commit_bytes=mem.avail_page)
   if mem.avail_phys>=2*GIB and mem.avail_page>=4*GIB:break
   time.sleep(17)
  cwd=DEST/'isolated';soc=cwd/'rtl/soc/nav_soc.v';soc.parent.mkdir(parents=True);soc.write_bytes((OUT/'nav/rtl/soc/nav_soc.v').read_bytes());assert '.ENABLE_IRQ(0)' in soc.read_text()
  build=OUT/'fw/bsp/build_rv.py';spec=importlib.util.spec_from_file_location('frozen_stage2_build',build);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
  calls=[];original_run=subprocess.run;observed={};done=threading.Event()
  def observe():
   while not done.is_set():
    try:
     for p in me.children(recursive=True):
      try:observed[(p.pid,p.create_time())]=dict(pid=p.pid,created_epoch=p.create_time(),name=p.name(),command=p.cmdline())
      except(psutil.NoSuchProcess,psutil.AccessDenied):pass
    except(psutil.NoSuchProcess,psutil.AccessDenied):pass
    done.wait(.1)
  thread=threading.Thread(target=observe,daemon=True);thread.start()
  def recorded(cmd,*args,**kw):
   start=datetime.now().astimezone().isoformat();r=original_run(cmd,*args,**kw)
   def txt(x):return x.decode('utf8',errors='replace') if isinstance(x,bytes) else (x or '')
   calls.append(dict(command=[str(v)for v in cmd],started_at=start,ended_at=datetime.now().astimezone().isoformat(),returncode=r.returncode,stdout=txt(r.stdout),stderr=txt(r.stderr)));save(DEST/'tool_calls.json',calls);return r
  oldcwd=Path.cwd();oldargv=sys.argv[:];oldpath=sys.path[:]
  status('RV_compile_running',admission_physical_bytes=mem.avail_phys,admission_commit_bytes=mem.avail_page)
  try:
   os.chdir(cwd);sys.path.insert(0,str(build.parent));sys.argv=[str(build),'--ram-kb','128','--stack-kb','6','--stack-usage','--grid','64,2','--output-dir',str(DEST/'firmware')]
   subprocess.run=recorded;exitcode=module.main()
  finally:
   subprocess.run=original_run;os.chdir(oldcwd);sys.argv=oldargv;sys.path[:]=oldpath;done.set();thread.join()
   save(DEST/'observed_processes.json',list(observed.values()))
  assert exitcode==0,('frozen RV build failed',exitcode)
  tool=Path('D:/tools/xpack-riscv-none-elf-gcc-15.2.0-1/bin');elf=DEST/'firmware/nav_fw.elf'
  nm=recorded([tool/'riscv-none-elf-nm.exe','-n',elf],capture_output=True,text=True);assert nm.returncode==0
  wanted={'RAM_SIZE','STACK_RESERVE','STACK_SIZE','_bss_start','_bss_end','_stack_top'}
  syms={a[-1]:int(a[0],16)for line in nm.stdout.splitlines()if len(a:=line.split())>=3 and a[-1]in wanted}
  image=(DEST/'firmware/nav_fw.bin').read_bytes();full=image+bytes(131072-len(image));assert len(full)==131072
  lanes=[bytes(int(x,16) for x in (DEST/f'firmware/ram_b{b}.hex').read_text().split())for b in range(4)]
  assert all(len(x)==32768 for x in lanes)
  assert bytes(v for word in zip(*lanes)for v in word)==full
  words=[int(x,16)for x in (DEST/'firmware/ram_w.hex').read_text().split()]
  assert b''.join(x.to_bytes(4,'little')for x in words)==full
  stack=load(DEST/'firmware/stack_analysis.json');remaining=131072-syms['_bss_end']-6144
  assert remaining>=0 and stack['complete'] and stack['static_bound_bytes']<=3072
  frozen();result=dict(state,phase='RV_compile_complete',passed=True,image_bytes=len(image),linked_symbols=syms,remaining_after_6KiB_stack_bytes=remaining,stack=stack,CPU128KiB_lane_and_word_reconstruction_exact=True,selected_old_safe_image_bytes=52824,old_SYN40_image_bytes=51712,observed_processes=list(observed.values()),tool_calls=str(DEST/'tool_calls.json'),frozen_FW_RTL_unchanged=True,not_proven=['native application/protocol behavior','Pico busy host-loss latency/physical standstill','new firmware INIT actual joint netlist binding','physical resources/timing/CDC/board'],SYN_started=False,PNR_started=False,programmed=False)
  save(DEST/'result.json',result);save(DEST/'job.json',result);print(json.dumps(dict(passed=True,image_bytes=len(image),remaining_bytes=remaining,static_stack=stack['static_bound_bytes'],output=str(DEST)),ensure_ascii=False));return 0
 except Exception as e:
  result=dict(state,phase='failed_preserved',passed=False,error=str(e),traceback=traceback.format_exc());save(DEST/'result.json',result);save(DEST/'job.json',result);print(result['traceback']);return 1

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');a=p.parse_args();assert a.run;raise SystemExit(run())
