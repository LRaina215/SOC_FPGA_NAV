"""Unique complete SYN36 for proved private source-data reset removal.
No automatic PNR. Preserve original full-SYN runtime/memory protection.
"""
from pathlib import Path
import json,os,re,shutil,psutil
import run_model25_joint_syn as run
BASE=Path('F:/nav_integration_g2_20261023')
REFERENCE=BASE/'baseline_fusion_model25_NAV50_syn34'
PROVED=BASE/'baseline_fusion_model25_private_source_noreset_prepared1'
VALIDATION=BASE/'model25_private_source_noreset_validation1'
run.PREP=BASE/'baseline_fusion_model25_private_source_noreset_prepared2'
run.OUTPUT=BASE/'baseline_fusion_model25_private_source_noreset_syn36'
run.CONTROL=BASE/'baseline_fusion_model25_private_source_noreset_one_shot'
run.BUILD_LABEL='SYN36'

def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
def rewrite(v,old,new):
 if isinstance(v,str):return v.replace(str(old),str(new)).replace(old.as_posix(),new.as_posix())
 if isinstance(v,list):return[rewrite(x,old,new)for x in v]
 if isinstance(v,dict):return{k:rewrite(x,old,new)for k,x in v.items()}
 return v

def prepare_inputs():
 assert not run.CONTROL.exists()and not run.OUTPUT.exists()and not run.PREP.exists()
 # Reject duplicate identities before preparing/claiming this unique run.
 for p in psutil.process_iter(['pid','cmdline']):
  if p.pid==os.getpid():continue
  cmd=p.info.get('cmdline')or[]
  assert not any(run.OUTPUT.name in str(x)or run.CONTROL.name in str(x)for x in cmd),(p.pid,cmd)
 review=load(VALIDATION/'validation_review.json');formal=load(VALIDATION/'formal_result.json');rtl=load(VALIDATION/'xsim_result.json')
 assert review['passed_minimal_private_reset_validation']and formal['passed']and formal['total_unsatisfiable_obligations']==25 and rtl['passed']
 assert not rtl['private_initial_values_forced_or_initialized']and review['actual_counts']['phase_resets']==10 and review['actual_counts']['stops25']==10
 from prepare_model25_private_source_noreset import derive
 assert derive((REFERENCE/'model25_related_mailbox.v').read_text())==(PROVED/'model25_related_mailbox.v').read_text()
 assert formal['source_binding']['candidate']==str(PROVED/'model25_related_mailbox.v')
 shutil.copytree(PROVED,run.PREP,ignore=shutil.ignore_patterns('__pycache__','impl','job.json','tool_runtime.json'))
 m=rewrite(load(PROVED/'manifest.json'),PROVED,run.PREP)
 m.update(stage='syn',synthesis_complete=False,source_preparation_only=True,automatic_pnr=False,automatic_PNR=False,private_source_noreset_validation=str(VALIDATION/'validation_review.json'))
 save(run.PREP/'manifest.json',m)
 t=(PROVED/'run.tcl.disabled_pending_proof').read_text()
 t=t.replace(str(PROVED),str(run.PREP)).replace(PROVED.as_posix(),run.PREP.as_posix())
 (run.PREP/'run.tcl').write_text(t,encoding='utf8')
 proof=source_check(bind=False)
 proof.update(prepared=True,unique_SYN36_only=True,automatic_PNR=False)
 save(run.PREP/'prepared_source_review.json',proof)

def source_check(bind=True):
 a,b=run.hardware(REFERENCE),run.hardware(run.PREP)
 assert a.keys()==b.keys()and len(b)==66
 assert sorted(n for n in a if not run.same_bytes(a[n],b[n]))==['model25_related_mailbox.v']
 assert run.same_bytes(run.PREP/'model25_related_mailbox.v',PROVED/'model25_related_mailbox.v')
 for suffixes,count in[({'.hex','.mem','.bin','.mif'},69),({'.h','.vh'},31)]:
  x,y=run.inputs(REFERENCE,suffixes),run.inputs(run.PREP,suffixes)
  assert x.keys()==y.keys()and len(y)==count
  assert all(run.same_bytes(x[n],y[n])for n in x)
 for d in('fw','firmware'):
  x={p.relative_to(REFERENCE/d):p for p in(REFERENCE/d).rglob('*')if p.is_file()and'__pycache__'not in p.parts}
  y={p.relative_to(run.PREP/d):p for p in(run.PREP/d).rglob('*')if p.is_file()and'__pycache__'not in p.parts}
  assert x.keys()==y.keys()and all(run.same_bytes(x[n],y[n])for n in x)
 assert all(run.same_bytes(REFERENCE/n,run.PREP/n)for n in('g2.cst','g2.sdc'))
 t=(run.PREP/'run.tcl').read_text();selected=re.findall(r'^add_file -type verilog \{(.+)\}$',t,re.M)
 assert len(selected)==66 and {Path(n).resolve()for n in selected}=={p.resolve()for p in b.values()}
 assert all(run.PREP.resolve()in Path(n).resolve().parents for n in selected)
 assert [x.strip()for x in t.splitlines()if x.strip().startswith('run ')]==['run syn']
 m=load(run.PREP/'manifest.json');assert m['actual_clock_plan_mhz']==dict(NAV_CPU_peripherals=50,model_fusion_RL_action=25,radar_constraint=100)
 if bind:
  for row in load(run.PREP/'prepared_source_review.json')['bindings']:assert run.metadata(Path(row['path']))==row
 return dict(reference=str(REFERENCE),prepared=str(run.PREP),changed_RTL=['model25_related_mailbox.v'],identical_RTL=65,hardware_inputs=66,assets=69,headers=31,complete_original_FW_unchanged=True,CST_SDC_and_three_clock_plan_unchanged=True,selected66_Tcl_inputs_only_this_prepared_directory=True,formal_and_RTL_validation=str(VALIDATION/'validation_review.json'),private_initial_value_assumption='arbitrary not zero',actual_mapping_benefit_proved=False,bindings=[run.metadata(p)for p in b.values()])
run.source_check=source_check
if __name__=='__main__':
 prepare_inputs()
 raise SystemExit(run.main())
