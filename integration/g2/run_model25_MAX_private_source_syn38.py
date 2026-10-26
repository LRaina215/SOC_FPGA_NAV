"""Unique optional SYN38 controller.  Preparing its entry starts no tool.

Explicit --start is required.  A later authorized run waits for the original
2GiB physical/4GiB commit gate and reuses 40minute/runtime memory protection.
No dependency on PNR50 exit and no automatic PNR or combined acceptance.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,subprocess,sys,time,traceback,psutil
from retry_frozen_synthesis import memory_status
from prepare_model25_MAX_private_source import PREP,OUTPUT,CONTROL,check,save
GIB=1024**3
def main():
 preflight=check();assert not OUTPUT.exists()and not CONTROL.exists()
 CONTROL.mkdir(exist_ok=False);me=psutil.Process();me.nice(psutil.NORMAL_PRIORITY_CLASS);me.cpu_affinity(list(range(psutil.cpu_count())))
 state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),source=str(PREP),output=str(OUTPUT),source_review=preflight,automatic_PNR=False,PNR50_exit_dependency=False,minimum_physical_bytes=2*GIB,minimum_commit_bytes=4*GIB,SYN_max_minutes=40,original_runtime_memory_protection=True,programmed=False)
 with(CONTROL/'launch_claim.json').open('x')as f:json.dump(state,f,indent=2)
 def status(phase,**kw):state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw);save(CONTROL/'job.json',state)
 try:
  while True:
   m=memory_status();status('waiting_for_actual_memory_admission',free_bytes=m.avail_phys,available_commit_bytes=m.avail_page)
   if m.avail_phys>=2*GIB and m.avail_page>=4*GIB:break
   time.sleep(13)
  check();assert not OUTPUT.exists()
  cmd=[sys.executable,'-X','utf8',str(Path(__file__).with_name('retry_frozen_synthesis.py')),str(PREP),'--out',str(OUTPUT),'--start-now','--tool-priority','normal']
  with(CONTROL/'full_SYN38_stdout.log').open('w')as log:
   child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
   status('running_unique_full_SYN38',runner_pid=child.pid,runner_created_epoch=psutil.Process(child.pid).create_time(),command=cmd,admission_physical_bytes=m.avail_phys,admission_available_commit_bytes=m.avail_page);code=child.wait()
  assert code==0,('full_SYN38_failed_preserved',code)
  status('actual_full_SYN38_complete_combined_mapping_review_pending',actual_new_netlist_review_passed=False);return 0
 except BaseException as exc:status('failed_preserved_no_automatic_retry',error=str(exc),traceback=traceback.format_exc());return 1
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--start',action='store_true');ap.add_argument('--preflight',action='store_true');a=ap.parse_args();assert a.start!=a.preflight
 if a.start:raise SystemExit(main())
 print(json.dumps(check(),ensure_ascii=False,indent=2))
