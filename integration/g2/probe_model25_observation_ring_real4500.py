"""Unique memory-admitted real4500 observer/policy old-vs-ring comparison.

Uses original production25MHz/125Hz hardware time. No fusion calibration,
ModelSim, inference substitute, joint synthesis, PNR, or board operations.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,re,shutil,subprocess,time,traceback,psutil
from prepare_model25_observation_ring import SOURCE,PREP,FILES
from prepare_baseline_fusion_retime import metadata,save,same_bytes
from retry_frozen_synthesis import memory_status
VIV=Path('E:/2026.1/Vivado/bin');HERE=Path(__file__).parent
BASE=Path('F:/nav_integration_g2_20261023')
def run(out,wait):
 assert not out.exists();out.mkdir(parents=True);me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
 state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),source=str(PREP),reference=str(SOURCE),output=str(out),real4500=True,inference_substitute=False,force_or_private_initialization=False,clock_MHz=25,PERIOD_CLKS=200000,period_us=8000,scope='Original observer + scheduler + real4500, fusion calibration and wholeG2 excluded',automatic_PNR=False,new_SYN_PNR=False,programmed=False)
 with(out/'launch_claim.json').open('x')as f:json.dump(state,f,indent=2)
 def status(phase,**kw):
  state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw);save(out/'job.tmp',state);(out/'job.tmp').replace(out/'job.json')
 def admission(tool):
  while True:
   m=memory_status()
   if m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3:return dict(physical=m.avail_phys,commit=m.avail_page,checked_at=datetime.now().astimezone().isoformat())
   assert wait,('actual memory gate',m.avail_phys,m.avail_page)
   status('waiting_for_actual_memory_admission',next_tool=tool,available_physical_bytes=m.avail_phys,available_commit_bytes=m.avail_page,minimum_physical_bytes=2*1024**3,minimum_commit_bytes=4*1024**3);time.sleep(15)
 def command(index,tool,args):
  entered=admission(tool);cmd=['cmd','/c',str(VIV/(tool+'.bat')),*args]
  with(out/f'{index}_{tool}.log').open('w')as log:
   p=subprocess.Popen(cmd,cwd=out,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW|subprocess.BELOW_NORMAL_PRIORITY_CLASS);parent=psutil.Process(p.pid)
   runtime=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),tool=tool,command=cmd,wrapper_pid=p.pid,wrapper_created_epoch=parent.create_time(),started_at=datetime.now().astimezone().isoformat(),admission=entered,max_runtime_s=0,max_no_observable_progress_s=0,absolute_deadline=None,processes=[])
   while p.poll()is None:
    rows=[]
    try:
     for proc in[parent]+parent.children(recursive=True):
      try:rows.append(dict(pid=proc.pid,parent_pid=proc.ppid(),created_epoch=proc.create_time(),name=proc.name(),command=proc.cmdline(),rss=proc.memory_info().rss,priority=proc.nice()))
      except(psutil.NoSuchProcess,psutil.AccessDenied):pass
    except psutil.NoSuchProcess:pass
    m=memory_status();runtime.update(updated_at=datetime.now().astimezone().isoformat(),processes=rows,available_physical_bytes=m.avail_phys,available_commit_bytes=m.avail_page);save(out/'tool_runtime.json',runtime)
    status('running_'+tool,tool_command=cmd,wrapper_pid=p.pid,wrapper_created_epoch=parent.create_time(),current_tool_processes=rows);time.sleep(3)
   runtime.update(completed_at=datetime.now().astimezone().isoformat(),exit_code=p.returncode,processes_exited=True);save(out/f'{index}_{tool}_runtime.json',runtime);save(out/'tool_runtime.json',runtime)
  assert p.returncode==0,(tool,p.returncode)
 try:
  status('freezing_source_and_real4500_fixture')
  review=json.loads((PREP/'prepared_source_review.json').read_text());assert review['minimal_source_model_and_load_protocol_passed']
  for row in review['bindings']:assert metadata(Path(row['path']))==row
  for rel in('fc_v4_top.v','fixed4500_weight_rom.v','control/fc_v4_round_sat.v'):assert same_bytes(SOURCE/rel,PREP/rel)
  for root,suffix in((SOURCE,'ref'),(PREP,'candidate')):
   for name in('rl_observation_v5','rl_policy_tick_v5'):
    t=(root/'control'/(name+'.v')).read_text();t=re.sub(r'\b'+name+r'\b',name+'_'+suffix,t);t=t.replace(name.upper()+'_V_INCLUDED',name.upper()+'_V_INCLUDED_'+suffix.upper());(out/(name+'_'+suffix+'.v')).write_text(t)
  for rel in('fc_v4_top.v','fixed4500_weight_rom.v','control/fc_v4_round_sat.v'):
   p=out/rel;p.parent.mkdir(exist_ok=True,parents=True);shutil.copyfile(PREP/rel,p);assert same_bytes(PREP/rel,p)
  assets=[p for p in PREP.rglob('*')if p.is_file()and p.suffix.lower()in{'.hex','.mem','.mif','.bin'}];assert len(assets)==69
  for p in assets:
   target=out/p.relative_to(PREP);target.parent.mkdir(exist_ok=True,parents=True);shutil.copyfile(p,target);assert same_bytes(p,target)
  tb=HERE/'tb_model25_observation_ring_real4500.sv';shutil.copyfile(tb,out/tb.name)
  frozen=[metadata(PREP/rel)for rel in FILES]+[metadata(PREP/rel)for rel in('fc_v4_top.v','fixed4500_weight_rom.v','control/fc_v4_round_sat.v')]
  spec=dict(frozen_at=datetime.now().astimezone().isoformat(),actual_source_bindings=frozen,fixture_binding=metadata(out/tb.name),all69_assets_byte_identical=True,source_files_name_and_include_guard_only_changed_in_simulation_copies=True,real_core_input_mem_and425_weights_outputs_unmodified=True,normal_clock_ns=40,normal_period_clocks=200000,normal_period_us=8000,real_inference_substitute=False,compare='Every168write/address/data/start,425layer-index-data records,six actions,frame IDs,publish cycle and cadence; at least2ring wraps plus reset/disable/sensor interruption and rearm',scope_excludes=['Unmodified fusion2second calibration','wholeG2 CPU/UART/RK/real sensors','Physical place-route timing/CDC/IO and vehicle stopping'],watchdog_simulated_ms=400)
  save(out/'input_and_observation_spec.json',spec);status('frozen_waiting_for_first_tool_admission',spec=str(out/'input_and_observation_spec.json'))
  files=[str(out/(name+'_'+suffix+'.v'))for suffix in('ref','candidate')for name in('rl_observation_v5','rl_policy_tick_v5')]+[str(out/n)for n in('fc_v4_top.v','fixed4500_weight_rom.v','control/fc_v4_round_sat.v')]
  for i,(tool,args)in enumerate([('xvlog',files),('xvlog',['-sv',str(out/tb.name)]),('xelab',['tb_observation_ring_real4500','-s','ring_real4500_snap']),('xsim',['ring_real4500_snap','-runall'])]):command(i,tool,args)
  log=(out/'xsim.log').read_text(errors='replace');match=re.search(r'OBSERVATION_RING_REAL4500_PASS[^\r\n]+',log);assert match and'Fatal:'not in log
  assert all(metadata(Path(r['path']))==r for r in frozen),'running source changed'
  result=dict(passed=True,completed_at=datetime.now().astimezone().isoformat(),summary=match[0],numeric_frames=re.findall(r'RING_REAL4500_FRAME_MATCH[^\r\n]+',log),spec=spec,scope=state['scope'],new_SYN_PNR_or_board_action=False,physical_timing_or_vehicle_acceptance=False)
  save(out/'result.json',result);status('completed_real4500_finite_comparison',passed=True,result=str(out/'result.json'));return 0
 except BaseException as e:
  (out/'controller_error.log').write_text(traceback.format_exc());save(out/'result.json',dict(passed=False,error=str(e),recorded_at=datetime.now().astimezone().isoformat()));status('failed_preserved',passed=False,error=str(e));return 1
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--wait-for-memory',action='store_true');a=ap.parse_args();raise SystemExit(run(a.out,a.wait_for_memory))
