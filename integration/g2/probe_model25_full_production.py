"""Single memory-admitted production-time real4500/wrapper simulation.

Does not edit the frozen prepared4, use ModelSim, start SYN/PnR, or access board
or RK. job.json/tool_runtime.json record the actual new process tree. The only
simulation deadline is the five-second stimulus watchdog inside the fixture.
"""
from datetime import datetime
from pathlib import Path
import argparse,json,re,shutil,subprocess,time,traceback
import psutil
from prepare_baseline_fusion_retime import metadata,save,same_bytes
from retry_frozen_synthesis import memory_status

ROOT=Path(__file__).resolve().parents[2]
BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'baseline_fusion_model25_NAV50_prepared4'
ORIGIN=BASE/'baseline_fusion_only_syn23'
VIV=Path('E:/2026.1/Vivado/bin')
LIBRARY=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
FILES=['control/fc_v4_round_sat.v','control/imu_fusion.v','control/imu_isqrt.v','control/imu_recip.v',
 'control/leg_ik_ff.v','control/policy_action_map.v','control/rl_observation_v5.v','control/rl_policy_tick_v5.v',
 'fc_v4_top.v','fixed4500_weight_rom.v','control/mem_policy.v','control/mem_policy_related25.v','model25_related_mailbox.v',
 'model25_pll/model25_pll.v']

def run(out,wait_for_memory):
 assert not out.exists();out.mkdir(parents=True)
 me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
 state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),source=str(SOURCE),output=str(out),
   real4500=True,inference_substitute=False,force_internal_state=False,production_BIAS_SAMPLES=200,
   source_sensor_hz=100,model_clock_mhz=25,reference_clock_mhz=50,policy_hz=125,period_us=8000,
   testbench_watchdog_simulated_seconds=5,new_SYN_PNR=False,programmed=False,physical_stop_or_deployment_accepted=False)
 def status(phase,**updates):
  state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**updates)
  target=out/'job.tmp';save(target,state);target.replace(out/'job.json')
 def admission(label):
  while True:
   mem=memory_status()
   if mem.avail_phys>=2*1024**3 and mem.avail_page>=4*1024**3:
    return dict(available_physical_bytes=mem.avail_phys,available_commit_bytes=mem.avail_page,checked_at=datetime.now().astimezone().isoformat())
   assert wait_for_memory,('memory admission',mem.avail_phys,mem.avail_page)
   status('waiting_for_actual_memory_admission',next_tool=label,available_physical_bytes=mem.avail_phys,available_commit_bytes=mem.avail_page,minimum_physical_bytes=2*1024**3,minimum_commit_bytes=4*1024**3)
   time.sleep(15)
 def command(index,tool,args):
  entry=admission(tool)
  cmd=['cmd','/c',str(VIV/(tool+'.bat')),*args]
  with(out/f'{index}_{tool}.log').open('w')as log:
   p=subprocess.Popen(cmd,cwd=out,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW|subprocess.BELOW_NORMAL_PRIORITY_CLASS)
   wrapper=psutil.Process(p.pid)
   status('running_'+tool,tool_command=cmd,wrapper_pid=p.pid,wrapper_created_epoch=wrapper.create_time(),tool_start_admission=entry)
   runtime=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),tool=tool,command=cmd,wrapper_pid=p.pid,wrapper_created_epoch=wrapper.create_time(),started_at=datetime.now().astimezone().isoformat(),max_runtime_s=0,max_no_observable_progress_s=0,absolute_deadline=None,processes=[])
   while p.poll()is None:
    rows=[]
    try:
     for child in [wrapper]+wrapper.children(recursive=True):
      try:rows.append(dict(pid=child.pid,parent_pid=child.ppid(),created_epoch=child.create_time(),name=child.name(),command=child.cmdline(),cwd=child.cwd(),priority=child.nice(),rss=child.memory_info().rss))
      except(psutil.NoSuchProcess,psutil.AccessDenied):pass
    except psutil.NoSuchProcess:pass
    mem=memory_status();runtime.update(updated_at=datetime.now().astimezone().isoformat(),processes=rows,available_physical_bytes=mem.avail_phys,available_commit_bytes=mem.avail_page)
    save(out/'tool_runtime.json',runtime)
    status('running_'+tool,current_tool_processes=rows)
    time.sleep(3)
   runtime.update(completed_at=datetime.now().astimezone().isoformat(),tool_exit=p.returncode,processes_exited=True)
   save(out/f'{index}_{tool}_runtime.json',runtime);save(out/'tool_runtime.json',runtime)
  assert p.returncode==0,(tool,p.returncode,str(out/f'{index}_{tool}.log'))
 try:
  status('checking_and_freezing_real_source_and_production_fixture')
  review=json.loads((SOURCE/'prepared_source_review.json').read_text())
  assert review['prepared'] and review['out']==str(SOURCE) and not review['new_SYN_PNR_started']
  for row in review['bindings']:assert metadata(Path(row['path']))==row
  source_bindings=[metadata(SOURCE/n)for n in FILES]
  for n in FILES:
   p=out/n;p.parent.mkdir(exist_ok=True,parents=True)
   shutil.copyfile(SOURCE/n,p)
  # Only module/include-guard names differ in the frozen reference copy.
  old_fusion=(ORIGIN/'control/imu_fusion.v').read_text()
  reference_fusion=old_fusion.replace('IMU_FUSION_V_INCLUDED','PRODUCTION_REFERENCE_IMU_FUSION_V_INCLUDED').replace('module imu_fusion #(','module imu_fusion_reference #(',1).replace('imu_div100_3','imu_div100_3_reference')
  reverse=reference_fusion.replace('PRODUCTION_REFERENCE_IMU_FUSION_V_INCLUDED','IMU_FUSION_V_INCLUDED').replace('module imu_fusion_reference #(','module imu_fusion #(',1).replace('imu_div100_3_reference','imu_div100_3')
  assert reverse==old_fusion
  old_policy=(ORIGIN/'control/mem_policy.v').read_text()
  reference_policy=old_policy.replace('module mem_policy #(','module mem_policy_reference #(',1).replace('imu_fusion #(','imu_fusion_reference #(',1)
  assert reference_policy.replace('module mem_policy_reference #(','module mem_policy #(',1).replace('imu_fusion_reference #(','imu_fusion #(',1)==old_policy
  (out/'fusion_reference.v').write_text(reference_fusion,newline='\n');(out/'policy_reference.v').write_text(reference_policy,newline='\n')
  assert all(same_bytes(SOURCE/n,ORIGIN/n)for n in['fc_v4_top.v','fixed4500_weight_rom.v','control/rl_policy_tick_v5.v','control/rl_observation_v5.v','control/policy_action_map.v'])
  assets=[p for p in SOURCE.rglob('*')if p.is_file()and p.suffix.lower()in{'.hex','.mem','.mif','.bin'}]
  for p in assets:
   q=out/p.relative_to(SOURCE);q.parent.mkdir(exist_ok=True,parents=True);shutil.copyfile(p,q)
   assert same_bytes(p,q)
  library=LIBRARY.read_text();models=[]
  for name in('GSR','PLL'):
   m=re.search(r'\bmodule '+name+r'\s*\(.*?\bendmodule',library,re.S);assert m;models.append(m[0])
  (out/'unmodified_vendor_public_PLL.v').write_text('`timescale 1ns/1ps\n'+'\n'.join(models),newline='\n')
  tb=ROOT/'integration/g2/tb_model25_full_production.sv';shutil.copyfile(tb,out/tb.name)
  spec=dict(frozen_at=datetime.now().astimezone().isoformat(),source_bindings=source_bindings,reference_module_names_only_changed=True,
   reference_source_bindings=[metadata(ORIGIN/n)for n in['control/imu_fusion.v','control/mem_policy.v']],
   fixture_binding=metadata(out/tb.name),source_asset_count=len(assets),assets_byte_identical=True,public_PLL_library=metadata(LIBRARY),
   input_scenario='100Hz stationary six-axis data at productionBIAS200; three physical command plateaus changed after completed publication, zero encoder deltas; then fault/rearm/pending reset/full recalibration/clock halt',
   comparison='actual core frame IDs +168words; all425 layer-index/data results, six core actions and336bit actuator/publication bundle; compare only same explicitly tracked epoch/frame with exact captured-input equality; explicitly reject unknown bits before key/index/data comparisons and publication acceptance',
   unknown_value_rejection=True,
   observation='read-only actual core ports and output bridges; no force, no inference substitute, no internal state initialization',
   normal_timing='actual PLL25 model clock and original50; capture/publication intervals8000us and capture-to-publication<=8000us',
   scope_excludes=['full G2 CPU/A*/UART/RK/real IIC sensor','all motion/sensor phases/numerical domain equivalence','post-route clocks/CDC/IO/pulsewidth/physical stopping','50MHz reference-clock failure'])
  save(out/'input_and_observation_spec.json',spec)
  status('prepared_and_waiting_for_admission',input_and_observation_spec=str(out/'input_and_observation_spec.json'))
  files=[str(out/'unmodified_vendor_public_PLL.v'),str(out/'fusion_reference.v'),str(out/'policy_reference.v')]+[str(out/n)for n in FILES]
  command(0,'xvlog',files)
  command(1,'xvlog',['-sv',str(out/tb.name)])
  command(2,'xelab',['tb_model25_full_production','-s','full_production_snap'])
  command(3,'xsim',['full_production_snap','-runall'])
  log=(out/'xsim.log').read_text(errors='replace')
  success=re.search(r'MODEL25_FULL_PRODUCTION_PASS[^\r\n]+',log)
  assert success and'Fatal:'not in log,'production stimulus/functional proof did not pass; inspect actual log'
  frames=re.findall(r'FRAME_NUMERIC_MATCH[^\r\n]+',log)
  result=dict(passed=True,completed_at=datetime.now().astimezone().isoformat(),summary=success[0],numeric_frame_rows=frames,
   real_full4500_production_BIAS200_and100Hz=True,inference_substitute=False,force_internal_state=False,
   original_frozen_prepared4_modified=False,physical_setup_hold_CDC_or_vehicle_stop_acceptance=False,input_and_observation_spec=spec)
  save(out/'result.json',result);status('completed_finite_production_proof',passed=True,result=str(out/'result.json'))
  return 0
 except Exception as exc:
  (out/'controller_error.log').write_text(traceback.format_exc(),encoding='utf-8')
  result=dict(passed=False,failed_at=datetime.now().astimezone().isoformat(),reason=str(exc),log=str(out/'controller_error.log'),no_SYN_PNR_or_hardware_action=True)
  save(out/'result.json',result);status('failed',passed=False,error=str(exc));return 1

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--wait-for-memory',action='store_true');a=p.parse_args()
 raise SystemExit(run(a.out,a.wait_for_memory))
