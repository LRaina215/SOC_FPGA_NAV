"""Targeted exact-cycle guard test, separate from the running production model.

Uses a small independent xsim after the ordinary2/4GiB admission. It never
occupies ModelSim, edits PNR49, or starts synthesis/PnR.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,re,shutil,subprocess,time,traceback
import psutil
from retry_frozen_synthesis import memory_status
from prepare_baseline_fusion_retime import metadata,save
from prepare_lease_deadline_compare import OLD,NEW,rewrite

ROOT=Path(__file__).resolve().parents[2]
VIV=Path('E:/2026.1/Vivado/bin')

def run(source,out,wait_memory):
 assert not out.exists();out.mkdir(parents=True)
 me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
 state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),source=str(source),output=str(out),
  full_production_model_untouched=True,logical_clock_HZ_parameter=4000,logical_DIV=4,
  actual_default_200ms_timeout_cycles_preserved_under_scaled_clock=True,
  no_internal_force_or_hierarchical_state_write=True,new_SYN_PNR_started=False,programmed=False)
 def status(phase,**updates):
  state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**updates)
  target=out/'job.tmp';save(target,state);target.replace(out/'job.json')
 def admission(label):
  while True:
   mem=memory_status()
   if mem.avail_phys>=2*1024**3 and mem.avail_page>=4*1024**3:return dict(available_physical_bytes=mem.avail_phys,available_commit_bytes=mem.avail_page,checked_at=datetime.now().astimezone().isoformat())
   assert wait_memory,('actual memory admission',label,mem.avail_phys,mem.avail_page)
   status('waiting_for_actual_memory_admission',next_tool=label,available_physical_bytes=mem.avail_phys,available_commit_bytes=mem.avail_page);time.sleep(15)
 def command(index,tool,args):
  entry=admission(tool);cmd=['cmd','/c',str(VIV/(tool+'.bat')),*args]
  with(out/f'{index}_{tool}.log').open('w')as log:
   p=subprocess.Popen(cmd,cwd=out,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW|subprocess.BELOW_NORMAL_PRIORITY_CLASS)
   wrapper=psutil.Process(p.pid);status('running_'+tool,command=cmd,wrapper_pid=p.pid,wrapper_created_epoch=wrapper.create_time(),tool_start_admission=entry)
   runtime=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),tool=tool,command=cmd,wrapper_pid=p.pid,wrapper_created_epoch=wrapper.create_time(),started_at=datetime.now().astimezone().isoformat(),max_runtime_s=0,max_no_observable_progress_s=0,absolute_deadline=None)
   while p.poll()is None:
    rows=[]
    try:
     for child in[wrapper]+wrapper.children(recursive=True):
      try:rows.append(dict(pid=child.pid,parent_pid=child.ppid(),created_epoch=child.create_time(),name=child.name(),command=child.cmdline(),cwd=child.cwd()))
      except(psutil.NoSuchProcess,psutil.AccessDenied):pass
    except psutil.NoSuchProcess:pass
    runtime.update(updated_at=datetime.now().astimezone().isoformat(),processes=rows);save(out/'tool_runtime.json',runtime);status('running_'+tool,current_tool_processes=rows);time.sleep(1)
   runtime.update(completed_at=datetime.now().astimezone().isoformat(),exit_code=p.returncode,processes_exited=True);save(out/f'{index}_{tool}_runtime.json',runtime);save(out/'tool_runtime.json',runtime)
  assert p.returncode==0,(tool,p.returncode)
 try:
  status('preparing_targeted_source_bound_fixture')
  review=json.loads((source/'prepared_source_review.json').read_text());original=Path(review['source'])/'command_lease_guard.v'
  a=original.read_text();b=(source/'command_lease_guard.v').read_text();assert rewrite(a)==b
  (out/'original.v').write_text(a.replace('module command_lease_guard #(','module original_guard #(',1),encoding='utf-8',newline='\n')
  shutil.copyfile(source/'command_lease_guard.v',out/'candidate.v')
  (out/'deadline_math_candidate.vh').write_text(NEW,encoding='utf-8',newline='\n')
  tb=ROOT/'integration/g2/tb_lease_deadline_compare.sv';shutil.copyfile(tb,out/tb.name)
  save(out/'fixture_binding.json',dict(original=metadata(original),candidate=metadata(out/'candidate.v'),testbench=metadata(out/tb.name),
   exact_candidate_NEW_text_reused_for_math_fixture=True,old_post_ramp_test_not_repeated=True))
  command(0,'xvlog',[str(out/'original.v'),str(out/'candidate.v')])
  command(1,'xvlog',['-sv',str(out/tb.name)])
  command(2,'xelab',['tb_lease_deadline_compare','-s','deadline_compare_snap'])
  command(3,'xsim',['deadline_compare_snap','-runall'])
  log=(out/'xsim.log').read_text(errors='replace');summary=re.search(r'LEASE_DEADLINE_COMPARE_PASS[^\r\n]+',log)
  assert summary and'Fatal:'not in log,'Targeted fixture did not pass; inspect original log'
  check_labels=[line[5:]for line in log.splitlines()if line.startswith('PASS ')]
  result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),summary=summary[0],
   contracts=check_labels,check_count=len(check_labels),distinct_check_labels=list(dict.fromkeys(check_labels)),
   distinct_check_count=len(set(check_labels)),
   compared_cycles=int(re.search(r'compared_cycles=(\d+)',summary[0])[1]),math_parameter_XZ_cases=int(re.search(r'math_parameter_XZ_cases=(\d+)',summary[0])[1]),
   output_and_registered_state_every_cycle=True,full_production_model_untouched=True,
   physical_200ms_stopping_or_setup_hold_not_verified=True,new_SYN_PNR_started=False,programmed=False)
  save(out/'result.json',result);status('completed_targeted_deadline_cycle_comparison',passed=True,result=str(out/'result.json'));return 0
 except Exception as exc:
  (out/'controller_error.log').write_text(traceback.format_exc(),encoding='utf-8')
  save(out/'result.json',dict(passed=False,reason=str(exc),failed_at=datetime.now().astimezone().isoformat(),no_SYN_PNR_or_hardware_action=True));status('failed',passed=False,error=str(exc));return 1

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--source',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);ap.add_argument('--wait-for-memory',action='store_true');a=ap.parse_args()
 raise SystemExit(run(a.source,a.out,a.wait_for_memory))
