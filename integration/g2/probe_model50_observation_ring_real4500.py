"""One memory-admitted real4500 comparison at physical50MHz/400000 clocks.

Reuses the proved25MHz ring fixture with only clock/period/header changes.
No inference substitute, RAM initialization, wholeG2, SYN/PNR or board action.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,re,shutil,subprocess,time,traceback,psutil
from prepare_baseline_fusion_retime import hardware_inputs,metadata,save,same_bytes
from retry_frozen_synthesis import memory_status

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'baseline_fusion_only_syn23'
VIV=Path('E:/2026.1/Vivado/bin')
HERE=Path(__file__).parent
CHANGED={'control/rl_observation_v5.v','control/rl_policy_tick_v5.v','control/mem_policy.v','command_lease_guard.v'}

def load(path):return json.loads(path.read_text(encoding='utf-8-sig'))

def fixture():
 old=(HERE/'tb_model25_observation_ring_real4500.sv').read_text()
 changes=[('Both model clocks are physical25MHz;','Both model clocks are physical50MHz;'),
          ('both periods are200000 cycles/8ms.','both periods are400000 cycles/8ms.'),
          ('always #20 clk=~clk;','always #10 clk=~clk;'),
          ('.PERIOD_CLKS(200000)', '.PERIOD_CLKS(400000)'),
          ('clock_MHz=25 period_us=8000','clock_MHz=50 period_us=8000')]
 result=old
 for before,after in changes:
  assert result.count(before)==(2 if before.startswith('.PERIOD')else 1),(before,result.count(before))
  result=result.replace(before,after)
 return result,dict(reference=str(HERE/'tb_model25_observation_ring_real4500.sv'),
                    exact_changes=changes,input_and_comparison_logic_unchanged=True,
                    only_clock_halfperiod_20_to10ns_and_period_200000_to400000=True)

def run(out,wait,prep):
 assert not out.exists();out.mkdir(parents=True)
 me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
 state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),source=str(prep),reference=str(SOURCE),
            output=str(out),real4500=True,inference_substitute=False,force_or_private_initialization=False,
            clock_MHz=50,PERIOD_CLKS=400000,period_us=8000,
            scope='Original observer + scheduler + real4500 at50MHz; fusion calibration, actuator and wholeG2 excluded',
            automatic_PNR=False,new_SYN_PNR=False,programmed=False)
 with(out/'launch_claim.json').open('x')as f:json.dump(state,f,indent=2)
 def status(phase,**kw):
  state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw)
  save(out/'job.tmp',state);(out/'job.tmp').replace(out/'job.json')
 def admission(tool):
  while True:
   m=memory_status()
   if m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3:
    return dict(physical=m.avail_phys,commit=m.avail_page,checked_at=datetime.now().astimezone().isoformat())
   assert wait,('actual memory gate',m.avail_phys,m.avail_page)
   status('waiting_for_actual_memory_admission',next_tool=tool,available_physical_bytes=m.avail_phys,
          available_commit_bytes=m.avail_page,minimum_physical_bytes=2*1024**3,minimum_commit_bytes=4*1024**3)
   time.sleep(15)
 def command(index,tool,args):
  entered=admission(tool);cmd=['cmd','/c',str(VIV/(tool+'.bat')),*args]
  with(out/f'{index}_{tool}.log').open('w')as log:
   p=subprocess.Popen(cmd,cwd=out,stdout=log,stderr=subprocess.STDOUT,
        creationflags=subprocess.CREATE_NO_WINDOW|subprocess.BELOW_NORMAL_PRIORITY_CLASS)
   parent=psutil.Process(p.pid)
   runtime=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),tool=tool,command=cmd,
        wrapper_pid=p.pid,wrapper_created_epoch=parent.create_time(),started_at=datetime.now().astimezone().isoformat(),
        admission=entered,max_runtime_s=0,max_no_observable_progress_s=0,absolute_deadline=None,processes=[])
   while p.poll()is None:
    rows=[]
    try:
     for proc in[parent]+parent.children(recursive=True):
      try:rows.append(dict(pid=proc.pid,parent_pid=proc.ppid(),created_epoch=proc.create_time(),name=proc.name(),
              command=proc.cmdline(),rss=proc.memory_info().rss,priority=proc.nice()))
      except(psutil.NoSuchProcess,psutil.AccessDenied):pass
    except psutil.NoSuchProcess:pass
    m=memory_status();runtime.update(updated_at=datetime.now().astimezone().isoformat(),processes=rows,
        available_physical_bytes=m.avail_phys,available_commit_bytes=m.avail_page)
    save(out/'tool_runtime.json',runtime)
    status('running_'+tool,tool_command=cmd,wrapper_pid=p.pid,wrapper_created_epoch=parent.create_time(),current_tool_processes=rows)
    time.sleep(3)
   runtime.update(completed_at=datetime.now().astimezone().isoformat(),exit_code=p.returncode,processes_exited=True)
   save(out/f'{index}_{tool}_runtime.json',runtime);save(out/'tool_runtime.json',runtime)
  assert p.returncode==0,(tool,p.returncode)
 try:
  status('freezing_original50MHz_source_and_real4500_fixture')
  old=hardware_inputs(SOURCE,load(SOURCE/'manifest.json'));new=hardware_inputs(prep,load(prep/'manifest.json'))
  assert len(old)==len(new)==63 and old.keys()==new.keys()
  assert {n for n in old if not same_bytes(old[n],new[n])}==CHANGED
  for rel in('fc_v4_top.v','fixed4500_weight_rom.v','control/fc_v4_round_sat.v','control/imu_fusion.v',
             'control/policy_action_map.v','control/oled.v','control/ir_remote_control.v'):
   assert same_bytes(SOURCE/rel,prep/rel),rel
  policy=(prep/'control/mem_policy.v').read_text()
  assert re.search(r'parameter integer PERIOD_CLKS\s*=\s*400000\b',policy)
  assert re.search(r'parameter integer SENSOR_TIMEOUT_CLKS\s*=\s*2500000\b',policy)
  assert re.search(r'parameter integer OBS_RING_STORAGE\s*=\s*1\b',policy)
  assert policy.count('.OBS_RING_STORAGE(OBS_RING_STORAGE)')==2
  for root,suffix in((SOURCE,'ref'),(prep,'candidate')):
   for name in('rl_observation_v5','rl_policy_tick_v5'):
    t=(root/'control'/(name+'.v')).read_text()
    t=re.sub(r'\b'+name+r'\b',name+'_'+suffix,t)
    t=t.replace(name.upper()+'_V_INCLUDED',name.upper()+'_V_INCLUDED_'+suffix.upper())
    (out/(name+'_'+suffix+'.v')).write_text(t)
  for rel in('fc_v4_top.v','fixed4500_weight_rom.v','control/fc_v4_round_sat.v'):
   p=out/rel;p.parent.mkdir(exist_ok=True,parents=True);shutil.copyfile(prep/rel,p);assert same_bytes(prep/rel,p)
  assets={p.relative_to(prep):p for p in prep.rglob('*')if p.is_file()and p.suffix.lower()in{'.hex','.mem','.mif','.bin'}}
  original_assets={p.relative_to(SOURCE):p for p in SOURCE.rglob('*')if p.is_file()and p.suffix.lower()in{'.hex','.mem','.mif','.bin'}}
  assert assets.keys()==original_assets.keys()and len(assets)==69
  for rel,p in assets.items():
   assert same_bytes(original_assets[rel],p);target=out/rel;target.parent.mkdir(exist_ok=True,parents=True)
   shutil.copyfile(p,target);assert same_bytes(p,target)
  tb=out/'tb_model50_observation_ring_real4500.sv';text,delta=fixture();tb.write_text(text)
  frozen=[metadata(p)for p in old.values()]+[metadata(p)for p in new.values()]
  spec=dict(frozen_at=datetime.now().astimezone().isoformat(),actual_source_bindings=frozen,
       fixture_binding=metadata(tb),fixture_change_review=delta,all69_assets_byte_identical=True,
       source_files_name_and_include_guard_only_changed_in_simulation_copies=True,
       real_core_input_mem_and425_weights_outputs_unmodified=True,normal_clock_ns=20,
       normal_period_clocks=400000,normal_period_us=8000,real_inference_substitute=False,
       compare='Every168write/address/data/start,425layer-index-data records,six actions,frame IDs,publish cycle and cadence; at least2ring wraps plus reset/disable/sensor interruption and rearm',
       scope_excludes=['Unmodified fusion2second calibration','wholeG2 CPU/UART/RK/real sensors',
                       'Actuator mapping and command lease','Physical place-route timing/CDC/IO and vehicle stopping'],
       watchdog_simulated_ms=400)
  save(out/'input_and_observation_spec.json',spec);status('frozen_waiting_for_first_tool_admission',spec=str(out/'input_and_observation_spec.json'))
  files=[str(out/(n+'_'+suffix+'.v'))for suffix in('ref','candidate')for n in('rl_observation_v5','rl_policy_tick_v5')]
  files += [str(out/n)for n in('fc_v4_top.v','fixed4500_weight_rom.v','control/fc_v4_round_sat.v')]
  for i,(tool,args)in enumerate([('xvlog',files),('xvlog',['-sv',str(tb)]),
       ('xelab',['tb_observation_ring_real4500','-s','ring50_real4500_snap']),('xsim',['ring50_real4500_snap','-runall'])]):command(i,tool,args)
  log=(out/'xsim.log').read_text(errors='replace');match=re.search(r'OBSERVATION_RING_REAL4500_PASS[^\r\n]+',log)
  assert match and'Fatal:'not in log and'clock_MHz=50 period_us=8000'in match[0]
  assert all(metadata(Path(r['path']))==r for r in frozen),'running source changed'
  result=dict(passed=True,completed_at=datetime.now().astimezone().isoformat(),summary=match[0],
       numeric_frames=re.findall(r'RING_REAL4500_FRAME_MATCH[^\r\n]+',log),spec=spec,scope=state['scope'],
       new_SYN_PNR_or_board_action=False,physical_timing_or_vehicle_acceptance=False)
  save(out/'result.json',result);status('completed_real4500_finite_comparison',passed=True,result=str(out/'result.json'));return 0
 except BaseException as e:
  (out/'controller_error.log').write_text(traceback.format_exc())
  save(out/'result.json',dict(passed=False,error=str(e),recorded_at=datetime.now().astimezone().isoformat()))
  status('failed_preserved',passed=False,error=str(e));return 1

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--candidate',type=Path,required=True)
 ap.add_argument('--out',type=Path,required=True);ap.add_argument('--wait-for-memory',action='store_true')
 a=ap.parse_args();raise SystemExit(run(a.out,a.wait_for_memory,a.candidate))
