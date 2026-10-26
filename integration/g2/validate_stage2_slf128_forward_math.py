"""Native proof of the actual frozen HAL arithmetic and MMIO access roles."""
from pathlib import Path
from datetime import datetime
import argparse,json,os,psutil,re,shutil,subprocess,time,traceback
from prepare_baseline_fusion_retime import metadata,same_bytes,save
from retry_frozen_synthesis import memory_status
BASE=Path('F:/nav_integration_g2_20261023');SRC=BASE/'stage2_SLF128_forward_safeFW_prepared8';DEST=BASE/'stage2_SLF128_forward_HAL_math_probe1';GIB=1024**3
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def function(text,signature):
 start=text.index(signature);b=text.index('{',start);level=1;pos=b+1
 while level:
  level+=(text[pos]=='{')-(text[pos]=='}');pos+=1
 return text[start:pos]
def run():
 assert not DEST.exists();DEST.mkdir();fw=SRC/'fw';bindings=load(SRC/'prepared_source_review.json')['all_FW_bindings']
 def frozen():
  for x in bindings:assert metadata(Path(x['path']))==x,x
 frozen();text=(fw/'drv/hal_fpga.c').read_text();start=text.index('static uint32_t support_last_capture');end=text.index('\nuint32_t hal_time_us',start)
 cuts=text[start:end]+'\n'+function(text,'uint32_t hal_time_us(void)')+'\n'+function(text,'void hal_lidar_commit(')+'\n'
 (DEST/'actual_hal_cuts.h').write_text(cuts,encoding='utf8');shutil.copyfile(Path(__file__).with_name('test_stage2_slf128_forward_math.c'),DEST/'test.c')
 save(DEST/'source_cut.json',dict(actual_source=metadata(fw/'drv/hal_fpga.c'),literal_function_cuts=True,functions=['lf_round_cell','hal_match_lf_forward','hal_time_us','hal_lidar_commit'],unchanged_fixmath_and_LUT=[metadata(fw/'nav/fixmath.c'),metadata(fw/'include/nav_sin_lut.h')],explicit_substitutes=['nav_rd/nav_wr MMIO access observations only'],independent_oracle='IEEE double exact numerator<=2^42, power-of-two denominator2^30, floor/ceil symmetric ties-away; whole16-bit yaw with actual1024 lookup addressing',not_full_CPU_or_bus_timing=True))
 me=psutil.Process();state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),candidate=str(SRC),output=str(DEST),SYN_started=False,PNR_started=False,RTL_tool_started=False,DLL_rebuilt=False,programmed=False)
 def phase(n,**kw):state.update(phase=n,at=datetime.now().astimezone().isoformat(),**kw);save(DEST/'job.json',state)
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
  cmd=['C:/msys64/mingw64/bin/gcc.exe','-std=c99','-O2','-Wall','-Wextra',*[f'-I{fw/d}'for d in ['include','hal','nav','app','bsp']],DEST/'test.c',fw/'nav/fixmath.c','-lm','-o',DEST/'forward_math.exe']
  runs=[tool('compile',cmd),tool('execute',[DEST/'forward_math.exe'])];line=(DEST/'execute.log').read_text().strip();assert 'TEST PASSED actual_forward_HAL' in line
  counts={k:int(v)for k,v in re.findall(r'(\w+)=(\d+)',line)};assert counts['calls']==6291456 and counts['trig_checks']==131072 and counts['forward_MAP_CENTER_writes']==0
  frozen();r=dict(state,phase='complete',passed=True,counts=counts,PASS_line=line,runtimes=runs,actual_HAL_C_and_LUT_unchanged=True,not_proven=['full CPU/RK path or busy stop latency','actual bus order/timing beyond observed MMIO call sequence','physical mapping/timing/board'])
  save(DEST/'result.json',r);save(DEST/'job.json',r);print(json.dumps(dict(passed=True,counts=counts,output=str(DEST)),ensure_ascii=False));return 0
 except Exception as e:
  r=dict(state,phase='failed_preserved',passed=False,error=str(e),traceback=traceback.format_exc());save(DEST/'result.json',r);save(DEST/'job.json',r);print(r['traceback']);return 1
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');a=p.parse_args();assert a.run;raise SystemExit(run())
