"""Prepare SYN38 source: independently proved explicitMAX plus private reset.

This only freezes/rebinds the two-file composition.  It launches no EDA tool,
does not modify PNR50, and does not claim combined mapping or timing benefit.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,re,shutil
from run_model25_private_source_syn import rewrite
from prepare_baseline_fusion_retime import metadata,same_bytes
BASE=Path('F:/nav_integration_g2_20261023')
REFERENCE=BASE/'baseline_fusion_model25_NAV50_syn34'
LEASE=BASE/'baseline_fusion_model25_bounded_MAX_syn37'
PRIVATE=BASE/'baseline_fusion_model25_private_source_noreset_syn36'
PREP=BASE/'baseline_fusion_model25_MAX_private_noreset_prepared1'
OUTPUT=BASE/'baseline_fusion_model25_MAX_private_noreset_syn38'
CONTROL=BASE/'baseline_fusion_model25_MAX_private_noreset_one_shot'
LEASE_REVIEW=BASE/'baseline_fusion_model25_bounded_MAX_actual_review_one_shot'
PRIVATE_REVIEW=BASE/'private_source_SYN36_actual_review3'
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
def names(root):return{Path(n).relative_to(root).as_posix()for n in load(root/'manifest.json')['source_files']}
def ordinary(root,suffix):return{p.relative_to(root).as_posix():p for p in root.rglob('*')if p.is_file()and'impl'not in p.relative_to(root).parts and'__pycache__'not in p.parts and p.suffix.lower()in suffix}

def check(bind=True):
 a,b,c,d=map(names,(REFERENCE,LEASE,PRIVATE,PREP));assert a==b==c==d and len(a)==66
 changes=lambda old:sorted(n for n in a if not same_bytes(old/n,PREP/n))
 assert changes(REFERENCE)==['command_lease_guard.v','model25_related_mailbox.v']
 assert changes(LEASE)==['model25_related_mailbox.v']
 assert changes(PRIVATE)==['command_lease_guard.v']
 assert same_bytes(PREP/'command_lease_guard.v',LEASE/'command_lease_guard.v')
 assert same_bytes(PREP/'model25_related_mailbox.v',PRIVATE/'model25_related_mailbox.v')
 for suffix,count in[({'.hex','.mem','.bin','.mif'},69),({'.h','.vh'},31)]:
  lists=[ordinary(root,suffix)for root in(REFERENCE,LEASE,PRIVATE,PREP)]
  assert all(set(row)==set(lists[0])and len(row)==count for row in lists)
  assert all(all(same_bytes(row[n],lists[0][n])for n in row)for row in lists[1:])
 fw={}
 for folder in('fw','firmware'):
  sets=[{p.relative_to(root).as_posix():p for p in(root/folder).rglob('*')if p.is_file()and'__pycache__'not in p.parts}for root in(REFERENCE,LEASE,PRIVATE,PREP)]
  assert all(set(row)==set(sets[0])for row in sets)
  assert all(all(same_bytes(row[n],sets[0][n])for n in row)for row in sets[1:]);fw[folder]=len(sets[0])
 for rel in('g2.cst','g2.sdc'):
  assert all(same_bytes(root/rel,REFERENCE/rel)for root in(LEASE,PRIVATE,PREP))
 t=(PREP/'run.tcl').read_text();selected=re.findall(r'^add_file -type verilog \{(.+)\}$',t,re.M)
 assert len(selected)==66 and{Path(p).resolve()for p in selected}=={(PREP/n).resolve()for n in a}
 assert all(PREP.resolve()in Path(p).resolve().parents for p in selected)
 assert all(str(old)not in t and old.as_posix()not in t for old in(REFERENCE,LEASE,PRIVATE))
 assert[x.strip()for x in t.splitlines()if x.strip().startswith('run ')]==['run syn']
 assert load(PREP/'manifest.json')['actual_clock_plan_mhz']==dict(NAV_CPU_peripherals=50,model_fusion_RL_action=25,radar_constraint=100)
 lease=load(LEASE_REVIEW/'result.json');mapping=load(LEASE_REVIEW/'actual_mapping_retry2/final_review.json')
 assert lease['passed']and mapping['passed']
 private=load(PRIVATE_REVIEW/'final_review.json');assert private['passed']
 formal=load(BASE/'model25_private_source_noreset_validation1/formal_result.json');rtl=load(BASE/'model25_private_source_noreset_validation1/xsim_result.json')
 assert formal['passed']and rtl['passed']and not rtl['private_initial_values_forced_or_initialized']
 math=load(BASE/'lease_deadline_bounded_tree_candidate1/math_proof.json');cycle=load(BASE/'lease_deadline_bounded_tree_candidate1/focused_cycle_result.json')
 assert math['passed']and cycle['passed']
 from prepare_model25_private_source_noreset import derive
 assert derive((REFERENCE/'model25_related_mailbox.v').read_text())==(PREP/'model25_related_mailbox.v').read_text()
 from prepare_lease_bounded_MAX import rewrite as lease_derive
 assert lease_derive((REFERENCE/'command_lease_guard.v').read_text())==(PREP/'command_lease_guard.v').read_text()
 if bind:
  for row in load(PREP/'prepared_source_review.json')['bindings']:assert metadata(Path(row['path']))==row
 return dict(prepared=True,recorded_at=datetime.now().astimezone().isoformat(),prepared_source=str(PREP),SYN38_output=str(OUTPUT),selected_hardware_inputs66=True,
  from_SYN37_changed_RTL=['model25_related_mailbox.v'],from_SYN37_other65_identical=True,
  from_SYN34_changed_RTL=['command_lease_guard.v','model25_related_mailbox.v'],from_SYN34_other64_identical=True,
  from_SYN36_changed_RTL=['command_lease_guard.v'],identical_assets=69,identical_headers=31,complete_original_FW_files=fw,
  original_weights_capacities_CST_SDC_and_three_clock_plan_preserved=True,all66_Tcl_paths_bound_only_new_candidate=True,
  independent_proof_binding=dict(lease_math=str(BASE/'lease_deadline_bounded_tree_candidate1/math_proof.json'),lease_cycle=str(BASE/'lease_deadline_bounded_tree_candidate1/focused_cycle_result.json'),lease_actual200FF=str(LEASE_REVIEW/'actual_mapping_retry2/actual_lease_next_state_proof.json'),lease_fullG2_mapping=str(LEASE_REVIEW/'result.json'),mailbox_minimal_formal=str(BASE/'model25_private_source_noreset_validation1/formal_result.json'),mailbox_fourstate=str(BASE/'model25_private_source_noreset_validation1/xsim_result.json'),mailbox_actual_mapping_and_full_word=str(PRIVATE_REVIEW/'final_review.json')),
  proof_scope='The exact two component RTL files and untouched interfaces/inputs are bound. New combined real netlist, physical reset allocation, resource use and setup/hold remain unverified.',
  actual_combined_SYN_started=False,automatic_PNR=False,PNR50_modified=False,programmed=False,deployment_accepted=False,
  bindings=[metadata(PREP/n)for n in sorted(a)]+[metadata(PREP/n)for n in('manifest.json','run.tcl','g2.cst','g2.sdc')])

def prepare():
 assert not PREP.exists()and not OUTPUT.exists()and not CONTROL.exists()
 assert load(LEASE/'job.json')['phase']=='synthesis_complete'and load(PRIVATE/'job.json')['phase']=='synthesis_complete'
 # Derive from the complete frozen37 inputs, not a live repository checkout.
 shutil.copytree(LEASE,PREP,ignore=shutil.ignore_patterns('impl','__pycache__','job.json','tool_runtime.json','*.log'))
 shutil.copy2(PRIVATE/'model25_related_mailbox.v',PREP/'model25_related_mailbox.v')
 m=rewrite(load(LEASE/'manifest.json'),LEASE,PREP)
 m.update(stage='syn',synthesis_complete=False,source_preparation_only=True,automatic_pnr=False,automatic_PNR=False,candidate='model25_explicitMAX_and_private_source_reset',component_reference_SYN37=str(LEASE),component_reference_SYN36=str(PRIVATE),additional_modified_rtl_inputs=['command_lease_guard.v','model25_related_mailbox.v'])
 save(PREP/'manifest.json',m)
 text=(LEASE/'run.tcl').read_text().replace(str(LEASE),str(PREP)).replace(LEASE.as_posix(),PREP.as_posix())
 (PREP/'run.tcl').write_text(text,encoding='utf8')
 proof=check(bind=False);save(PREP/'prepared_source_review.json',proof);return proof

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--prepare',action='store_true');ap.add_argument('--check',action='store_true');a=ap.parse_args();assert a.prepare!=a.check
 print(json.dumps(prepare()if a.prepare else check(),ensure_ascii=False,indent=2))
