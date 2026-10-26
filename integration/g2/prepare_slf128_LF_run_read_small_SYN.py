"""One LOG2N7 LF-only C synthesis after frozen RTL proofs; no full G2/PNR."""
from pathlib import Path
from datetime import datetime
import argparse,json,shutil,subprocess,time,traceback
import psutil
from prepare_baseline_fusion_retime import load,save,metadata,same_bytes
from retry_frozen_synthesis import memory_status

GW=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe');GIB=1024**3
WRAPPER='''module g2_top(input clk50,rst,start,input signed[31:0]cx,cy,input[17:0]sm_q,
 output busy,done,sm_re,output[13:0]sm_raddr,output sm_we,output[13:0]sm_waddr,output[17:0]sm_wdata);
slf_lf #(.LOG2N(7),.HIT_MIN(2))u_lf(.clk(clk50),.rst(rst),.start(start),.cx(cx),.cy(cy),
 .busy(busy),.done(done),.sm_re(sm_re),.sm_raddr(sm_raddr),.sm_q(sm_q),.sm_we(sm_we),.sm_waddr(sm_waddr),.sm_wdata(sm_wdata));
endmodule
'''
def stamp():return datetime.now().astimezone().isoformat()
def unchanged(rows):
 for r in rows:assert metadata(Path(r['path']))==r,('frozen input changed',r)

def prepare(source,out,proofs):
 source=source.resolve();out=out.resolve();assert not out.exists()and len(proofs)==2
 prep=load(source.parent/'preparation.json');assert metadata(source)==prep['repaired_source']
 assert all(load(p)['passed']for p in proofs)
 for row in prep['frozen_validation_inputs']:assert metadata(Path(row['path']))==row
 for p in psutil.process_iter(['name']):
  assert(p.info['name']or'').lower()not in('xvlog.exe','xelab.exe','xsim.exe','xsimk.exe','vsim.exe','vsimk.exe'),'RTL tool still active; do not create a new RTL input directory'
 out.mkdir(parents=True);shutil.copyfile(source,out/'slf_lf.v');assert same_bytes(source,out/'slf_lf.v')
 (out/'top.v').write_text(WRAPPER,encoding='utf8',newline='\n')
 (out/'clock.sdc').write_text('create_clock -name clk50 -period 20 [get_ports {clk50}]\n',encoding='utf8')
 (out/'run.tcl').write_text(f'''set_device -device_version C GW5AT-LV138PG484AC1/I0
add_file -type verilog {{{(out/'slf_lf.v').as_posix()}}}
add_file -type verilog {{{(out/'top.v').as_posix()}}}
add_file -type sdc {{{(out/'clock.sdc').as_posix()}}}
set_option -top_module g2_top
set_option -output_base_name g2_top
set_option -verilog_std v2001
set_option -global_freq 50
run syn
exit
''',encoding='utf8',newline='\n')
 entry=dict(source=str(source),source_preparation=str(source.parent/'preparation.json'),RTL_proofs=[str(p)for p in proofs],RTL_proofs_passed=True,
  LOG2N=7,HIT_MIN=2,full_LF_ports_observable=True,source_files=2,assets=0,CPU=False,device='GW5AT-LV138PG484AC1/I0',device_version='C',
  SYN_only=True,PNR=False,automatic_start=False,automatic_retry=False,minimum_physical_bytes=2*GIB,minimum_commit_bytes=4*GIB,max_runtime_seconds=2400,
  required_mapping_review=['Actual array read CEB plus BLKSELB, never OCE alone','All mapped row widths/READ_MODE/RESET_MODE and data concatenation','Closed actual control trace for reset,start,16900 scan and drain; physical18bit chunk noncollision'])
 save(out/'entry.json',entry)
 paths=[source,source.parent/'preparation.json',Path(__file__).resolve(),*proofs]+[out/x for x in ('slf_lf.v','top.v','clock.sdc','run.tcl','entry.json')]
 paths+=[Path(r['path'])for r in prep['frozen_validation_inputs']]
 save(out/'frozen_inputs.json',dict(inputs=[metadata(p)for p in paths]))
 save(out/'job.json',dict(phase='LF_source_wrapper_and_proofs_frozen_not_started',SYN_started=False,PNR_started=False))
 return entry

def start(out):
 out=out.resolve();proof=load(out/'frozen_inputs.json');unchanged(proof['inputs']);entry=load(out/'entry.json')
 assert entry['RTL_proofs_passed']and not(out/'run_claim.json').exists()and not(out/'impl').exists()
 me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
 state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),build=str(out),scope='LOG2N7 LF-only C SYN',PNR_started=False,programmed=False,automatic_retry=False)
 with(out/'run_claim.json').open('x',encoding='utf8')as f:json.dump(state,f)
 def status(phase,**kw):state.update(phase=phase,updated_at=stamp(),**kw);save(out/'job.json',state)
 try:
  while True:
   mem=memory_status();status('waiting_2GiB_4GiB_admission',physical_bytes=mem.avail_phys,commit_bytes=mem.avail_page)
   if mem.avail_phys>=2*GIB and mem.avail_page>=4*GIB:break
   time.sleep(15)
  unchanged(proof['inputs']);rt=dict(command=[str(GW),'run.tcl'],admission_physical_bytes=mem.avail_phys,admission_commit_bytes=mem.avail_page,max_runtime_seconds=2400)
  with(out/'small_SYN_stdout.log').open('w',encoding='utf8')as log:
   child=subprocess.Popen(rt['command'],cwd=out,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW|subprocess.BELOW_NORMAL_PRIORITY_CLASS)
   proc=psutil.Process(child.pid);rt.update(PID=child.pid,created_epoch=proc.create_time(),started_at=stamp());seen={};began=time.monotonic();sampled=0;low=0
   while child.poll()is None:
    try:owned=[proc]+proc.children(recursive=True)
    except psutil.NoSuchProcess:owned=[]
    for p in owned:
     try:r=dict(pid=p.pid,created_epoch=p.create_time(),name=p.name(),command=p.cmdline());seen[(p.pid,r['created_epoch'])]=r
     except(psutil.NoSuchProcess,psutil.AccessDenied):pass
    now=time.monotonic()
    if now-sampled>=2:
     mem=memory_status();low=low+1 if mem.avail_page<512*1024**2 or(mem.avail_phys<256*1024**2 and mem.avail_page<GIB)else 0;sampled=now
     rt.update(physical_bytes=mem.avail_phys,commit_bytes=mem.avail_page,consecutive_low_memory_samples=low)
    reason='sustained_host_memory'if low>=2 else 'runtime_2400s'if now-began>=2400 else None
    rt.update(elapsed_seconds=now-began,processes=list(seen.values()));save(out/'tool_runtime.json',rt);status('running_LF_only_small_SYN',runtime=rt)
    if reason:
     for p in reversed(owned):
      try:seen[(p.pid,p.create_time())];p.kill()
      except(psutil.NoSuchProcess,psutil.AccessDenied,KeyError):pass
     child.wait();rt.update(cancelled_reason=reason,exit_code=child.returncode,ended_at=stamp());save(out/'small_SYN_runtime.json',rt);raise RuntimeError(reason)
    time.sleep(.5)
   rt.update(exit_code=child.returncode,ended_at=stamp());save(out/'small_SYN_runtime.json',rt);assert child.returncode==0,child.returncode
  unchanged(proof['inputs']);assert(out/'impl/gwsynthesis/g2_top.vg').exists()
  result=dict(state,runtime=rt,phase='small_SYN_complete_actual_mapping_review_pending',tool_passed=True,mapping_review_passed=None,completed_at=stamp(),frozen_inputs_unchanged=True)
  save(out/'tool_result.json',result);save(out/'job.json',result);return 0
 except Exception as error:
  result=dict(state,tool_passed=False,phase='failed_preserved_no_retry',error=str(error),traceback=traceback.format_exc());save(out/'tool_result.json',result);save(out/'job.json',result);return 1

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);p.add_argument('--source',type=Path);p.add_argument('--proof',type=Path,action='append',default=[])
 m=p.add_mutually_exclusive_group(required=True);m.add_argument('--prepare',action='store_true');m.add_argument('--start',action='store_true');a=p.parse_args()
 if a.prepare:assert a.source;print(json.dumps(prepare(a.source,a.out,a.proof),ensure_ascii=False,indent=2))
 else:raise SystemExit(start(a.out))
