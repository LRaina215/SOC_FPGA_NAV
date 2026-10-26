"""Read-only follow-through of the unique SYN37, never constructs or programs."""
from pathlib import Path
from datetime import datetime
import json,os,subprocess,sys,time,traceback
import psutil
from retry_frozen_synthesis import memory_status
BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'baseline_fusion_model25_NAV50_syn34'
NEW=BASE/'baseline_fusion_model25_bounded_MAX_syn37'
PREP=BASE/'baseline_fusion_model25_bounded_MAX_prepared1'
BUILD_CONTROL=BASE/'baseline_fusion_model25_bounded_MAX_one_shot'
CONTROL=BASE/'baseline_fusion_model25_bounded_MAX_actual_review_one_shot'

def main():
 CONTROL.mkdir(exist_ok=False);me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
 state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),actual_SYN37=str(NEW),new_SYN_PNR=False,automatic_PNR=False,programmed=False)
 with(CONTROL/'claim.json').open('x')as f:json.dump(state,f,indent=2)
 def status(phase,**kw):
  state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw)
  tmp=CONTROL/(str(me.pid)+'.tmp');tmp.write_text(json.dumps(state,indent=2));tmp.replace(CONTROL/'job.json')
 def command(label,argv,expected):
  m=memory_status()
  with(CONTROL/(label+'_stdout.log')).open('w')as log:
   proc=subprocess.Popen(argv,cwd=Path(__file__).parent,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW|subprocess.BELOW_NORMAL_PRIORITY_CLASS)
   tool=psutil.Process(proc.pid);created=tool.create_time();low=0
   status('running_'+label,reviewer_pid=proc.pid,reviewer_created_epoch=created,command=argv,physical_bytes=m.avail_phys,available_commit_bytes=m.avail_page)
   while proc.poll()is None:
    m=memory_status();danger=m.avail_page<512*1024**2 or(m.avail_phys<256*1024**2 and m.avail_page<1024**3);low=low+1 if danger else 0
    if low>=2:
     assert abs(tool.create_time()-created)<.01
     try:children=tool.children(recursive=True)
     except psutil.NoSuchProcess:children=[]
     for c in reversed(children):
      try:c.kill()
      except psutil.NoSuchProcess:pass
     proc.kill();proc.wait();raise RuntimeError('stopped_only_owned_readonly_reviewer_for_sustained_host_memory')
    time.sleep(2)
  assert proc.returncode==0,(label,proc.returncode)
  r=json.loads(expected.read_text());assert r['passed'];return r
 try:
  while True:
   build=json.loads((BUILD_CONTROL/'job.json').read_text());j=json.loads((NEW/'job.json').read_text())
   if j['phase']=='synthesis_complete':assert j['tool_exit']==0;break
   if build['phase']in('failed_after_single_settings_only_resume_preserved','failed_preserved_no_automatic_retry'):
    status('not_started_unique_SYN37_did_not_complete',build_phase=build['phase']);return 1
   status('waiting_unique_actual_SYN37_result',build_control_phase=build['phase'],actual_build_phase=j['phase']);time.sleep(15)
  while True:
   m=memory_status()
   if m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3:break
   status('waiting_memory_for_readonly_actual_review',physical_bytes=m.avail_phys,available_commit_bytes=m.avail_page);time.sleep(15)
  here=Path(__file__).parent
  a=CONTROL/'actual_mapping'
  mapping=command('actual_mapping',[sys.executable,'-X','utf8',str(here/'review_lease_bounded_MAX_joint_mapping.py'),'--old',str(SOURCE),'--new',str(NEW),'--prepared',str(PREP),'--out',str(a)],a/'final_review.json')
  b=CONTROL/'actual_memories_clocks'
  other=command('actual_memories_clocks',[sys.executable,'-X','utf8',str(here/'review_lease_tree_memories_clocks.py'),'--new',str(NEW),'--out',str(b)],b/'result.json')
  result=dict(passed=True,scope='Actual completeSYN37 review, still not physical timing or deployment acceptance',mapping_result=str(a/'final_review.json'),memory_clock_result=str(b/'result.json'),
   explicit_MAX_mapping_goal_passed=mapping['mapping_goal_achieved'],allRAM_INIT_CPU_three_clocks_original26_passed=other['all_memory_parameters_INIT_CPU_and_clock_checks_passed'],
   before_first_estimated_path=mapping['old_reported_paths'][0]['summary'],after_first_estimated_path=mapping['new_reported_paths'][0]['summary'],
   global_resources_before=mapping['global_resources_before'],global_resources_after=mapping['global_resources_after'],
   new_PNR_started=False,programmed=False,actual_PNR_decision_pending_root_review=True)
  (CONTROL/'result.json').write_text(json.dumps(result,indent=2));status('actual_SYN37_review_complete_root_timing_gain_and_PNR_decision_pending');return 0
 except Exception as exc:
  status('actual_review_failed_preserved_no_automatic_retry',error=str(exc),traceback=traceback.format_exc());return 1
if __name__=='__main__':raise SystemExit(main())
