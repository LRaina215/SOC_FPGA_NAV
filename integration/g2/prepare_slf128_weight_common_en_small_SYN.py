"""One frozen weights-only C synthesis with the existing 2/4 and40min guards."""
from pathlib import Path
from datetime import datetime
import argparse,json,re,shutil,subprocess,time,traceback
import psutil
from prepare_baseline_fusion_retime import load,save,metadata,same_bytes
from retry_frozen_synthesis import memory_status

GW=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe');GIB=1024**3
WRAPPER='''module g2_top(input clk50,en,input[15:0]addr,output[15:0]value);
fixed4500_weight_rom u_weights(.clk(clk50),.en(en),.addr(addr),.value(value));
endmodule
'''
def stamp():return datetime.now().astimezone().isoformat()
def unchanged(rows):
 for row in rows:assert metadata(Path(row['path']))==row,('frozen input changed',row)

def prepare(source,proof,out):
 source=source.resolve();proof=proof.resolve();out=out.resolve();assert not out.exists()
 prep=load(source.parent/'preparation.json');assert metadata(source)==prep['repaired_source']and load(proof)['passed']
 ancestor=Path(prep['ancestor']['path']);assert metadata(ancestor)==prep['ancestor']
 old=ancestor.read_text(encoding='utf8');candidate=source.read_text(encoding='utf8')
 pattern=r"always@\(posedge clk\)if\(en && addr\[15:13\]==3'd[0-4]\)(high_q[012]\[[0-4]\])"
 expected,count=re.subn(pattern,r'always@(posedge clk)if(en)\1',old);assert count==15 and expected==candidate
 unchanged(prep['validation_inputs'])
 for p in psutil.process_iter(['name']):assert(p.info['name']or'').lower()not in('xvlog.exe','xelab.exe','xsim.exe','xsimk.exe','vsim.exe','vsimk.exe'),'RTL tool still active'
 assets=re.findall(r'\$readmemh\("([^"]+)"',candidate);assert len(assets)==len(set(assets))==35
 out.mkdir(parents=True);shutil.copyfile(source,out/'fixed4500_weight_rom.v')
 for rel in assets+['src/mem/weights.mem']:
  src=ancestor.parent/rel;dst=out/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(src,dst);assert same_bytes(src,dst)
 (out/'top.v').write_text(WRAPPER,encoding='utf8',newline='\n')
 (out/'clock.sdc').write_text('create_clock -name clk50 -period 20 [get_ports {clk50}]\n',encoding='utf8')
 (out/'run.tcl').write_text(f'''set_device -device_version C GW5AT-LV138PG484AC1/I0
add_file -type verilog {{{(out/'fixed4500_weight_rom.v').as_posix()}}}
add_file -type verilog {{{(out/'top.v').as_posix()}}}
add_file -type sdc {{{(out/'clock.sdc').as_posix()}}}
set_option -top_module g2_top
set_option -output_base_name g2_top
set_option -verilog_std v2001
set_option -global_freq 50
run syn
exit
''',encoding='utf8',newline='\n')
 entry=dict(scope='weights-only common high enable, all16 output bits',source=str(source),source_ancestor=str(ancestor),RTL_proof=str(proof),RTL_proof_passed=True,
  source_files=2,initialization_payloads=35,additional_reference_gold='src/mem/weights.mem (not instantiated)',CPU=False,PNR=False,
  device='GW5AT-LV138PG484AC1/I0',device_version='C',minimum_physical_bytes=2*GIB,minimum_commit_bytes=4*GIB,max_runtime_seconds=2400,
  automatic_start=False,automatic_retry=False,only15_high_en_conditions_changed=True,no_public_pipeline_added=True)
 save(out/'entry.json',entry)
 paths=[source,source.parent/'preparation.json',ancestor,proof,Path(__file__).resolve()]+[Path(r['path'])for r in prep['validation_inputs']]
 paths+=[out/x for x in ['fixed4500_weight_rom.v','top.v','clock.sdc','run.tcl','entry.json',*assets,'src/mem/weights.mem']]
 save(out/'frozen_inputs.json',dict(inputs=[metadata(p)for p in paths]));save(out/'job.json',dict(phase='weights_inputs_frozen_not_started',SYN_started=False,PNR_started=False))
 return entry

def start(out):
 out=out.resolve();proof=load(out/'frozen_inputs.json');unchanged(proof['inputs']);entry=load(out/'entry.json')
 assert entry['RTL_proof_passed']and not(out/'run_claim.json').exists()and not(out/'impl').exists()
 me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
 state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),build=str(out),scope=entry['scope'],PNR_started=False,programmed=False,automatic_retry=False)
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
    rt.update(elapsed_seconds=now-began,processes=list(seen.values()));save(out/'tool_runtime.json',rt);status('running_weights_only_small_SYN',runtime=rt)
    if reason:
     for p in reversed(owned):
      try:seen[(p.pid,p.create_time())];p.kill()
      except(psutil.NoSuchProcess,psutil.AccessDenied,KeyError):pass
     child.wait();rt.update(cancelled_reason=reason,exit_code=child.returncode,ended_at=stamp());save(out/'small_SYN_runtime.json',rt);raise RuntimeError(reason)
    time.sleep(.5)
   rt.update(exit_code=child.returncode,ended_at=stamp());save(out/'small_SYN_runtime.json',rt);assert child.returncode==0,child.returncode
  unchanged(proof['inputs']);assert(out/'impl/gwsynthesis/g2_top.vg').exists()
  result=dict(state,runtime=rt,phase='small_SYN_complete_actual_ROM_and_official_model_review_pending',tool_passed=True,mapping_review_passed=None,completed_at=stamp(),frozen_inputs_unchanged=True)
  save(out/'tool_result.json',result);save(out/'job.json',result);return 0
 except Exception as error:
  result=dict(state,tool_passed=False,phase='failed_preserved_no_retry',error=str(error),traceback=traceback.format_exc());save(out/'tool_result.json',result);save(out/'job.json',result);return 1

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);p.add_argument('--source',type=Path);p.add_argument('--proof',type=Path)
 m=p.add_mutually_exclusive_group(required=True);m.add_argument('--prepare',action='store_true');m.add_argument('--start',action='store_true');a=p.parse_args()
 if a.prepare:assert a.source and a.proof;print(json.dumps(prepare(a.source,a.proof,a.out),ensure_ascii=False,indent=2))
 else:raise SystemExit(start(a.out))
