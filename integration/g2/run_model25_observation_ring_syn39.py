"""One complete SYN39 of the numerically validated observation ring source.

Original2GiB/4GiB admission,40minute and runtime memory policy. No automatic
PNR, SYN38 start, new source changes, or dependency on unrelated tool exits.
"""
from pathlib import Path
from datetime import datetime
import json,re,shutil,os,psutil
import run_model25_joint_syn as run
from prepare_model25_observation_ring import SOURCE,PREP as PROVED,FILES,derive_observation,derive_tick,derive_policy,derive_wrapper
BASE=Path('F:/nav_integration_g2_20261023')
REAL=BASE/'observation_ring_real4500_probe1'
run.PREP=BASE/'baseline_fusion_model25_observation_ring_prepared2'
run.OUTPUT=BASE/'baseline_fusion_model25_observation_ring_syn39'
run.CONTROL=BASE/'baseline_fusion_model25_observation_ring_one_shot'
run.BUILD_LABEL='SYN39'
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
def rewrite(v,old,new):
 if isinstance(v,str):return v.replace(str(old),str(new)).replace(old.as_posix(),new.as_posix())
 if isinstance(v,list):return[rewrite(x,old,new)for x in v]
 if isinstance(v,dict):return{k:rewrite(x,old,new)for k,x in v.items()}
 return v
def source_check(bind=True):
 old,new=run.hardware(SOURCE),run.hardware(run.PREP);assert len(old)==len(new)==66 and old.keys()==new.keys()
 changed=sorted(n for n in old if not run.same_bytes(old[n],new[n]));assert changed==sorted(FILES),changed
 assert all(run.same_bytes(PROVED/n,new[n])for n in new)
 for suffixes,count in[({'.hex','.mem','.bin','.mif'},69),({'.h','.vh'},31)]:
  a,b=run.inputs(SOURCE,suffixes),run.inputs(run.PREP,suffixes);assert a.keys()==b.keys()and len(a)==count and all(run.same_bytes(a[n],b[n])for n in a)
 for folder in('fw','firmware'):
  a={p.relative_to(SOURCE):p for p in(SOURCE/folder).rglob('*')if p.is_file()and'__pycache__'not in p.parts}
  b={p.relative_to(run.PREP):p for p in(run.PREP/folder).rglob('*')if p.is_file()and'__pycache__'not in p.parts};assert a.keys()==b.keys()and all(run.same_bytes(a[n],b[n])for n in a)
 for n in('g2.cst','g2.sdc','fc_v4_top.v','fixed4500_weight_rom.v','control/fc_v4_round_sat.v','control/imu_fusion.v','control/policy_action_map.v','control/oled.v','control/ir_remote_control.v'):
  assert run.same_bytes(SOURCE/n,run.PREP/n),n
 t=(run.PREP/'run.tcl').read_text();selected=re.findall(r'^add_file -type verilog \{(.+)\}$',t,re.M)
 assert len(selected)==66 and{Path(p).resolve()for p in selected}=={p.resolve()for p in new.values()}
 assert all(run.PREP.resolve()in Path(p).resolve().parents for p in selected)
 assert all(str(p)not in t and p.as_posix()not in t for p in(SOURCE,PROVED))
 assert[x.strip()for x in t.splitlines()if x.strip().startswith('run ')]==['run syn']
 assert load(run.PREP/'manifest.json')['actual_clock_plan_mhz']==dict(NAV_CPU_peripherals=50,model_fusion_RL_action=25,radar_constraint=100)
 real=load(REAL/'result.json');job=load(REAL/'job.json');assert real['passed']and job['passed']and job['real4500']and not job['inference_substitute']and job['PERIOD_CLKS']==200000 and job['clock_MHz']==25
 assert len(real['numeric_frames'])==20 and'wraps=2'in real['summary']
 assert all(run.metadata(Path(row['path']))==row for row in real['spec']['actual_source_bindings'])
 model=load(BASE/'observation_ring_model_proof1/result.json');protocol=load(BASE/'observation_ring_protocol_probe2/result.json');assert model['passed']and protocol['passed']
 for rel,f in zip(FILES,(derive_observation,derive_tick,derive_policy,derive_wrapper)):assert f((SOURCE/rel).read_text())==(run.PREP/rel).read_text()
 if bind:
  for row in load(run.PREP/'prepared_source_review.json')['bindings']:assert run.metadata(Path(row['path']))==row
 return dict(prepared=True,recorded_at=datetime.now().astimezone().isoformat(),reference=str(SOURCE),prepared_directory=str(run.PREP),changed_RTL=changed,other62_RTL_identical=True,hardware_inputs=66,assets=69,headers=31,original_FW_core_input_mem_weights_math_three_clocks_CST_SDC_OLED_IR_preserved=True,all66_Tcl_inputs_rebound_only_new_prepared=True,real4500_finite_numerical_proof=str(REAL/'result.json'),minimal_source_model_and_load_protocol_passed=True,default_modules_legacy0_candidate_explicit_ring1=True,RAM_initialization_assumed=False,actual_new_RAM_mapping_FF_gain_and_timing_pending=True,automatic_PNR=False,bindings=[run.metadata(p)for p in new.values()])
def prepare():
 assert not run.PREP.exists()and not run.OUTPUT.exists()and not run.CONTROL.exists()
 for p in psutil.process_iter(['pid','cmdline']):
  if p.pid==os.getpid():continue
  cmd=p.info.get('cmdline')or[]
  assert not any(run.OUTPUT.name in str(x)or run.CONTROL.name in str(x)for x in cmd),(p.pid,cmd)
 assert load(REAL/'result.json')['passed']
 for row in load(PROVED/'prepared_source_review.json')['bindings']:assert run.metadata(Path(row['path']))==row
 shutil.copytree(PROVED,run.PREP,ignore=shutil.ignore_patterns('__pycache__','impl','job.json','tool_runtime.json'))
 m=rewrite(load(PROVED/'manifest.json'),PROVED,run.PREP);m.update(stage='syn',synthesis_complete=False,source_preparation_only=True,automatic_PNR=False,automatic_pnr=False,real4500_ring_proof=str(REAL/'result.json'));save(run.PREP/'manifest.json',m)
 t=(PROVED/'run.tcl.disabled_pending_ring_protocol').read_text().replace(str(PROVED),str(run.PREP)).replace(PROVED.as_posix(),run.PREP.as_posix());(run.PREP/'run.tcl').write_text(t)
 proof=source_check(bind=False);save(run.PREP/'prepared_source_review.json',proof)
run.source_check=source_check
if __name__=='__main__':
 prepare();raise SystemExit(run.main())
