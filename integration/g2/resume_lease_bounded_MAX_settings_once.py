"""One existing settings-only resume; SYN37 has never run in this directory."""
from pathlib import Path
from datetime import datetime
import json,os,re,subprocess,sys,time,traceback
import psutil
from run_lease_bounded_MAX_syn37 import source_check,PREP,OUTPUT,CONTROL,GIB
from retry_frozen_synthesis import memory_status
from prepare_baseline_fusion_retime import same_bytes,metadata

def main():
 source_check()
 old=json.loads((OUTPUT/'job.json').read_text());assert old['phase']=='validating_frozen_settings'and'tool_pid'not in old
 assert not(OUTPUT/'impl').exists()and not(OUTPUT/'gw_sh.log').exists()
 oldcontroller=json.loads((CONTROL/'job.json').read_text());assert oldcontroller['phase']=='failed_preserved_no_automatic_retry'
 dump=(OUTPUT/'effective_options.tcl').read_text()
 assert re.search(r'^set_option -global_freq 50\.000$',dump,re.M)and re.search(r'^set_option -convert_sdp32_36_to_sdp16_18 1$',dump,re.M)
 names={Path(p).relative_to(OUTPUT)for p in json.loads((OUTPUT/'manifest.json').read_text())['source_files']}
 assert len(names)==66 and all(same_bytes(PREP/n,OUTPUT/n)for n in names)
 me=psutil.Process();me.nice(psutil.NORMAL_PRIORITY_CLASS);me.cpu_affinity(list(range(psutil.cpu_count())))
 state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),source=str(PREP),output=str(OUTPUT),
  settings_only_resume_once=True,previous_SYN_started=False,automatic_PNR=False,programmed=False,previous_owned_controller=oldcontroller,
  no_SYN36_exit_dependency=True,no_global_tool_mutex=True,existing_effective_dump_metadata=metadata(OUTPUT/'effective_options.tcl'))
 with(CONTROL/'settings_resume_once_claim.json').open('x')as f:json.dump(state,f,indent=2)
 with(CONTROL/'first_controller_settings_timeout_job.json').open('x')as f:json.dump(oldcontroller,f,indent=2)
 def status(phase,**kw):
  state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw)
  tmp=CONTROL/(str(me.pid)+'.tmp');tmp.write_text(json.dumps(state,indent=2));tmp.replace(CONTROL/'job.json')
 try:
  while True:
   m=memory_status();status('waiting_memory_for_one_settings_only_resume',free_bytes=m.avail_phys,available_commit_bytes=m.avail_page,minimum_physical_bytes=2*GIB,minimum_commit_bytes=4*GIB)
   if m.avail_phys>=2*GIB and m.avail_page>=4*GIB:break
   time.sleep(13)
  source_check()
  cmd=[sys.executable,'-X','utf8',str(Path(__file__).with_name('retry_frozen_synthesis.py')),str(PREP),'--out',str(OUTPUT),'--start-now','--resume-settings-only','--tool-priority','normal']
  with(CONTROL/'settings_resume_once_stdout.log').open('w')as log:
   child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
   status('running_one_settings_only_resume',runner_pid=child.pid,runner_created_epoch=psutil.Process(child.pid).create_time(),command=cmd)
   code=child.wait()
  assert code==0,('one_settings_resume_failed_preserved_no_more_automatic_retries',code)
  status('actual_full_SYN37_complete_mapping_RAM_clock_review_pending');return 0
 except Exception as exc:
  status('failed_after_single_settings_only_resume_preserved',error=str(exc),traceback=traceback.format_exc());return 1
if __name__=='__main__':raise SystemExit(main())
