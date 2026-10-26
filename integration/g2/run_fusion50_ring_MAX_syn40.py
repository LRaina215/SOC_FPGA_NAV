"""Unique two-clock SYN40: routed SYN23 anchor + proven ring and MAX tree.

63 selected inputs; no model25 PLL/transport or new PNR. The candidate's
mem_policy explicitly defaults ring on; original frozen baselines stay intact.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,os,re,shutil,subprocess,sys,time,traceback,difflib
import psutil
from prepare_baseline_fusion_retime import metadata,same_bytes
from prepare_model25_observation_ring import derive_observation,derive_tick,once
from prepare_lease_bounded_MAX import rewrite as derive_MAX
from run_model25_joint_syn import hardware,inputs
from retry_frozen_synthesis import memory_status

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'baseline_fusion_only_syn23'
PREP=BASE/'baseline_fusion50_ring_MAX_prepared1'
OUTPUT=BASE/'baseline_fusion50_ring_MAX_syn40'
CONTROL=BASE/'baseline_fusion50_ring_MAX_one_shot'
ROUTED35=BASE/'baseline_fusion32_CDC52_pnr35'
FILES=['control/rl_observation_v5.v','control/rl_policy_tick_v5.v','control/mem_policy.v','command_lease_guard.v']
GIB=1024**3
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
def derive_policy50(text):
 text=once(text,'parameter integer WEIGHT_BITS=16,','parameter integer OBS_RING_STORAGE=1,\n    parameter integer WEIGHT_BITS=16,')
 text=once(text,'wire [3359:0] history_flat;','''wire [3359:0] history_flat;
    wire observation_freeze;
    wire [7:0] observation_read_index;
    wire signed [23:0] observation_read_data;''')
 text=once(text,'rl_observation_v5 #(.LEFT_ENCODER_SIGN(-1),','rl_observation_v5 #(.OBS_RING_STORAGE(OBS_RING_STORAGE),.LEFT_ENCODER_SIGN(-1),')
 text=once(text,'.valid(frame_valid),.initialized(history_initialized),','''.valid(frame_valid),.initialized(history_initialized),
        .frame_freeze(observation_freeze),.frame_read_index(observation_read_index),.frame_read_data(observation_read_data),''')
 text=once(text,'rl_policy_tick_v5 #(.WEIGHT_BITS(WEIGHT_BITS),','rl_policy_tick_v5 #(.OBS_RING_STORAGE(OBS_RING_STORAGE),.WEIGHT_BITS(WEIGHT_BITS),')
 text=once(text,'.frame_valid(frame_valid),.current_flat(current_flat),.history_flat(history_flat),','''.frame_valid(frame_valid),.current_flat(current_flat),.history_flat(history_flat),
        .frame_freeze(observation_freeze),.frame_read_index(observation_read_index),.frame_read_data(observation_read_data),''')
 return text
DERIVE=[derive_observation,derive_tick,derive_policy50,derive_MAX]
def source_check(bind=True):
 a,b=hardware(SOURCE),hardware(PREP);assert a.keys()==b.keys()and len(a)==63
 changed=sorted(n for n in a if not same_bytes(a[n],b[n]));assert changed==sorted(FILES),changed
 for n,f in zip(FILES,DERIVE):assert f((SOURCE/n).read_text())==(PREP/n).read_text(),n
 for suffixes,count in[({'.hex','.mem','.bin','.mif'},69),({'.h','.vh'},31)]:
  x,y=inputs(SOURCE,suffixes),inputs(PREP,suffixes);assert x.keys()==y.keys()and len(x)==count and all(same_bytes(x[n],y[n])for n in x)
 for directory in('fw','firmware'):
  a0={p.relative_to(SOURCE):p for p in(SOURCE/directory).rglob('*')if p.is_file()and'__pycache__'not in p.parts};b0={p.relative_to(PREP):p for p in(PREP/directory).rglob('*')if p.is_file()and'__pycache__'not in p.parts};assert a0.keys()==b0.keys()and all(same_bytes(a0[n],b0[n])for n in a0)
 for n in('g2.cst','control4500_core.v','g2_top.v','control/imu_fusion.v','control/policy_action_map.v','control/fc_v4_round_sat.v','fc_v4_top.v','fixed4500_weight_rom.v','control/oled.v','control/ir_remote_control.v'):
  assert same_bytes(SOURCE/n,PREP/n),n
 assert same_bytes(PREP/'g2.sdc',ROUTED35/'g2.sdc')and same_bytes(PREP/'g2.cst',ROUTED35/'g2.cst')
 sdc=(PREP/'g2.sdc').read_text();assert len(re.findall(r'^create_clock ',sdc,re.M))==2
 assert'create_clock -name clk50 -period 20 [get_ports {clk50}]'in sdc and'create_clock -name rgmii_rxc -period 10 [get_ports {rgmii_rxc}]'in sdc
 assert len(re.findall(r'^set_false_path .* -hold$',sdc,re.M))==26 and len(re.findall(r'^set_max_delay ',sdc,re.M))==26
 assert not re.search(r'^create_generated_clock|^set_clock_groups|^set_multicycle_path',sdc,re.M)
 from synthesis_connectivity import read
 original26=load(BASE/'model25_SYN34_actual_review1/actual_review4/actual_original_radar26.json');assert original26['passed']and len(original26['paths'])==26
 needed={x[k]for x in original26['paths']for k in('source_register','first_register','second_register')}
 cells=read(SOURCE/'impl/gwsynthesis/g2_top.vg',lambda n:n in needed or any(v.startswith(n+'/')for v in needed))
 assert set(cells)==needed
 for x in original26['paths']:
  src,first,second=[cells[x[k]]for k in('source_register','first_register','second_register')]
  assert src['ports']['Q']==first['ports']['D']and first['ports']['Q']==second['ports']['D']
  assert first['ports']['CLK']==second['ports']['CLK']
  assert '{'+src['name']+'/Q}'in sdc and'{'+first['name']+'/D}'in sdc
 for func in('sat24','clip_current'):
  body=lambda t:re.search(r'function signed \[23:0\] '+func+r';.*?endfunction',t,re.S)[0]
  assert body((SOURCE/FILES[0]).read_text())==body((PREP/FILES[0]).read_text())
 for n in('control/mem_policy.v','control/rl_policy_tick_v5.v'):assert 'PERIOD_CLKS=400000'in(PREP/n).read_text().replace(' ','')
 assert 'SPEED_GATE_CYCLES=500000'in(PREP/FILES[0]).read_text().replace(' ','')
 assert 'OBS_RING_STORAGE=1'in(PREP/FILES[2]).read_text().replace(' ','')
 assert all('OBS_RING_STORAGE = 0'in(PREP/n).read_text()for n in FILES[:2])
 assert 'mem_policy_related25.v'not in a and'model25_related_mailbox.v'not in a and'model25_pll/model25_pll.v'not in a
 t=(PREP/'run.tcl').read_text();selected=re.findall(r'^add_file -type verilog \{(.+)\}$',t,re.M)
 assert len(selected)==63 and{Path(p).resolve()for p in selected}=={p.resolve()for p in b.values()}
 assert all(PREP.resolve()in Path(p).resolve().parents for p in selected)
 assert SOURCE.as_posix()not in t and str(SOURCE)not in t
 assert[x.strip()for x in t.splitlines()if x.strip().startswith('run ')]==['run syn']
 m=load(PREP/'manifest.json');assert m['actual_clock_plan_mhz']==dict(all_NAV_CPU_fusion_RL_peripherals=50,radar_constraint=100)
 if bind:
  for row in load(PREP/'prepared_source_review.json')['bindings']:assert metadata(Path(row['path']))==row
 return dict(prepared=True,recorded_at=datetime.now().astimezone().isoformat(),reference=str(SOURCE),out=str(PREP),selected_RTL=63,changed_RTL=changed,other59_RTL_identical=True,assets69_headers31_FW_all_weights_core_input_mem_math_OLED_IR_preserved=True,two_original_true_clocks_50_100=True,SDC_single20_10ns_52audited_constraints_byte_identical_PNR35=True,original26_SYN23_actual_source_first_second_connectivity_checked=True,new_actual_SYN40_26_revalidation_pending=True,no_model25_PLL_or_five_bridges=True,period_clocks=400000,speed_gate_cycles=500000,control_rate_Hz=125,candidate_mem_policy_selects_ring1=True,observer_tick_default_legacy0=True,external_mem_policy_ports_unchanged=True,internal_ring_ports='freeze1/index8/data24',MAX_explicit_tree_source_matches_proved_SYN37=same_bytes(PREP/'command_lease_guard.v',BASE/'baseline_fusion_model25_bounded_MAX_syn37/command_lease_guard.v'),ring_algebra_protocol_real25MHz4500_previously_proved=True,new50MHz_real4500_validation_pending=True,actual_mapping_and_physical_timing_pending=True,automatic_PNR=False,bindings=[metadata(p)for p in b.values()]+[metadata(PREP/n)for n in('g2.sdc','g2.cst')])
def prepare():
 assert not PREP.exists()and not OUTPUT.exists()and not CONTROL.exists()
 for p in psutil.process_iter(['pid','cmdline']):
  if p.pid!=os.getpid():assert not any(OUTPUT.name in str(x)or CONTROL.name in str(x)for x in(p.info.get('cmdline')or[])),(p.pid,p.info['cmdline'])
 shutil.copytree(SOURCE,PREP,ignore=shutil.ignore_patterns('impl','__pycache__','job.json','tool_runtime.json','*.log'))
 for n,f in zip(FILES,DERIVE):
  old=(SOURCE/n).read_text();new=f(old);(PREP/n).write_text(new,encoding='utf8');(PREP/(Path(n).name+'.delta.patch')).write_text(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='SYN23/'+n,tofile='SYN40/'+n)),encoding='utf8')
 assert(PREP/'command_lease_guard.v').read_text()==(BASE/'baseline_fusion_model25_bounded_MAX_syn37/command_lease_guard.v').read_text()
 shutil.copyfile(BASE/'baseline_fusion_model25_bounded_MAX_syn37/command_lease_guard.v',PREP/'command_lease_guard.v')
 assert same_bytes(SOURCE/'g2.cst',ROUTED35/'g2.cst')
 shutil.copyfile(ROUTED35/'g2.sdc',PREP/'g2.sdc')
 m=load(SOURCE/'manifest.json');m['source_files']=[str(PREP/Path(p).relative_to(SOURCE))for p in m['source_files']]
 m.update(candidate='fusion50_NAV50_ring6_explicitMAX',frozen_source_build=str(SOURCE),actual_clock_plan_mhz=dict(all_NAV_CPU_fusion_RL_peripherals=50,radar_constraint=100),OBS_RING_STORAGE=1,period_clocks=400000,speed_gate_cycles=500000,automatic_pnr=False,automatic_PNR=False,synthesis_complete=False,programmed=False,source_preparation_only=True);save(PREP/'manifest.json',m)
 t=(SOURCE/'run.tcl').read_text().replace(str(SOURCE),str(PREP)).replace(SOURCE.as_posix(),PREP.as_posix());(PREP/'run.tcl').write_text(t,encoding='utf8')
 result=source_check(False);save(PREP/'prepared_source_review.json',result);return result
def start():
 source_check();assert not OUTPUT.exists();CONTROL.mkdir(exist_ok=False)
 me=psutil.Process();me.nice(psutil.NORMAL_PRIORITY_CLASS);me.cpu_affinity(list(range(psutil.cpu_count())))
 state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),source=str(PREP),output=str(OUTPUT),build_label='SYN40',automatic_PNR=False,programmed=False,SYN_max_runtime_minutes=40,minimum_physical_bytes=2*GIB,minimum_commit_bytes=4*GIB,original_runtime_memory_protection=True,other_tool_exit_dependency=False)
 with(CONTROL/'launch_claim.json').open('x')as f:json.dump(state,f,indent=2)
 def status(phase,**kw):
  state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw);tmp=CONTROL/(str(me.pid)+'.tmp');save(tmp,state);tmp.replace(CONTROL/'job.json')
 try:
  while True:
   m=memory_status();status('waiting_for_actual_memory_admission',free_bytes=m.avail_phys,available_commit_bytes=m.avail_page)
   if m.avail_phys>=2*GIB and m.avail_page>=4*GIB:break
   time.sleep(13)
  source_check()
  cmd=[sys.executable,'-X','utf8',str(Path(__file__).with_name('retry_frozen_synthesis.py')),str(PREP),'--out',str(OUTPUT),'--start-now','--tool-priority','normal']
  with(CONTROL/'full_SYN40_stdout.log').open('w')as f:
   child=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
   status('running_unique_full_SYN40',runner_pid=child.pid,runner_created_epoch=psutil.Process(child.pid).create_time(),admission_physical_bytes=m.avail_phys,admission_commit_bytes=m.avail_page,command=cmd);code=child.wait()
  assert code==0,('SYN40 failed; preserve without automatic retry',code)
  status('SYN40_complete_actual_two_clock_RING_MAX_mapping_review_pending',actual_new_netlist_review_passed=False);return 0
 except Exception as e:status('failed_preserved_no_automatic_retry',error=str(e),traceback=traceback.format_exc());return 1
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--prepare',action='store_true');p.add_argument('--start',action='store_true');a=p.parse_args();assert a.prepare!=a.start
 if a.prepare:print(json.dumps(prepare(),ensure_ascii=False,indent=2))
 else:raise SystemExit(start())
