"""One explicitMAX-only full SYN37 with original2/4GiB/40minute protection.
No dependency on SYN36 or production probe exit, no global tool mutex, no PNR.
"""
from pathlib import Path
from datetime import datetime
import json,os,subprocess,sys,time,traceback
import psutil
from retry_frozen_synthesis import memory_status
from prepare_baseline_fusion_retime import metadata,same_bytes
BASE=Path('F:/nav_integration_g2_20261023')
ORIGIN=BASE/'baseline_fusion_model25_NAV50_syn34'
PREP=BASE/'baseline_fusion_model25_bounded_MAX_prepared1'
OUTPUT=BASE/'baseline_fusion_model25_bounded_MAX_syn37'
CONTROL=BASE/'baseline_fusion_model25_bounded_MAX_one_shot'
GIB=1024**3

def source_check():
 proof=json.loads((PREP/'prepared_source_review.json').read_text());assert proof['prepared']and proof['only_changed_RTL']==['command_lease_guard.v']
 for row in proof['bindings']:assert metadata(Path(row['path']))==row
 names=lambda p:{Path(n).relative_to(p).as_posix()for n in json.loads((p/'manifest.json').read_text())['source_files']}
 a,b=names(ORIGIN),names(PREP);assert a==b and len(a)==66
 assert[n for n in sorted(a)if not same_bytes(ORIGIN/n,PREP/n)]==['command_lease_guard.v']
 text=(PREP/'run.tcl').read_text()
 assert all('{'+(PREP/n).as_posix()+'}'in text for n in a)
 assert ORIGIN.as_posix()not in text and str(ORIGIN)not in text
 for suffixes,count in[({'.hex','.mem','.bin','.mif'},69),({'.h','.vh'},31)]:
  src={p.relative_to(ORIGIN)for p in ORIGIN.rglob('*')if p.is_file()and'impl'not in p.relative_to(ORIGIN).parts and'__pycache__'not in p.parts and p.suffix.lower()in suffixes}
  assert len(src)==count and all(same_bytes(ORIGIN/n,PREP/n)for n in src)
 for folder in('fw','firmware'):
  for p in(ORIGIN/folder).rglob('*'):
   if p.is_file()and'__pycache__'not in p.parts:assert same_bytes(p,PREP/p.relative_to(ORIGIN))
 assert all(same_bytes(ORIGIN/n,PREP/n)for n in('g2.cst','g2.sdc'))

def main():
 source_check();assert not OUTPUT.exists();CONTROL.mkdir(exist_ok=False)
 me=psutil.Process();me.nice(psutil.NORMAL_PRIORITY_CLASS);me.cpu_affinity(list(range(psutil.cpu_count())))
 state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),build_label='SYN37',source=str(PREP),output=str(OUTPUT),automatic_PNR=False,programmed=False,
  SYN36_exit_dependency=False,production_probe3_exit_dependency=False,global_tool_mutex=False,minimum_physical_bytes=2*GIB,minimum_commit_bytes=4*GIB,SYN_max_minutes=40,original_runtime_memory_protection=True)
 with(CONTROL/'launch_claim.json').open('x')as file:json.dump(state,file,indent=2)
 def status(phase,**kw):
  state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw)
  tmp=CONTROL/(str(me.pid)+'.tmp');tmp.write_text(json.dumps(state,indent=2));tmp.replace(CONTROL/'job.json')
 try:
  while True:
   m=memory_status();status('waiting_for_actual_memory_admission',free_bytes=m.avail_phys,available_commit_bytes=m.avail_page)
   if m.avail_phys>=2*GIB and m.avail_page>=4*GIB:break
   time.sleep(13)
  source_check();assert not OUTPUT.exists()
  # The existing bounded SYN runner independently rechecks admission after
  # input/settings validation, so a concurrent36 allocation is measured again.
  cmd=[sys.executable,'-X','utf8',str(Path(__file__).with_name('retry_frozen_synthesis.py')),str(PREP),'--out',str(OUTPUT),'--start-now','--tool-priority','normal']
  with(CONTROL/'full_SYN37_stdout.log').open('w')as log:
   child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
   status('running_unique_full_SYN37',runner_pid=child.pid,runner_created_epoch=psutil.Process(child.pid).create_time(),admission_physical_bytes=m.avail_phys,admission_available_commit_bytes=m.avail_page,command=cmd)
   code=child.wait()
  assert code==0,('full_SYN37_failed_preserved',code)
  status('actual_full_SYN37_complete_mapping_RAM_clock_review_pending',actual_netlist_review_complete=False,wholeG2_prefix_removed_verified=False,wholeG2_timing_gain_accepted=False)
  return 0
 except Exception as exc:
  status('failed_preserved_no_automatic_retry',error=str(exc),traceback=traceback.format_exc());return 1
if __name__=='__main__':raise SystemExit(main())
