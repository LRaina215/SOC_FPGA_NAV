"""Prepare only the proved explicitMAX tree on frozen SYN34 inputs; no build."""
from pathlib import Path
from datetime import datetime
import argparse,difflib,json,shutil
from prepare_baseline_fusion_retime import metadata,save,same_bytes
from prepare_lease_deadline_compare import OLD
from probe_lease_bounded_tree_mapping import TREE
BASE=Path('F:/nav_integration_g2_20261023')
DEFAULT_SOURCE=BASE/'baseline_fusion_model25_NAV50_prepared4'
LOCAL_PROOF=BASE/'lease_deadline_bounded_tree_candidate1'
LOCAL_CANDIDATE=LOCAL_PROOF/'command_lease_guard.v'
def rewrite(text):
 assert text.count(OLD)==1
 new=text.replace(OLD,TREE);assert new.replace(TREE,OLD)==text
 assert json.loads((LOCAL_PROOF/'math_proof.json').read_text())['passed']
 assert json.loads((LOCAL_PROOF/'focused_cycle_result.json').read_text())['passed']
 return new

def prepare(source,out):
 source=source.resolve();out=out.resolve()
 assert source.is_dir()and not out.exists()
 assert out.parent==BASE.resolve()and out.name.startswith('baseline_fusion_model25_bounded_MAX_')
 review=json.loads((source/'prepared_source_review.json').read_text())
 assert review['prepared']and review['out']==str(source)
 for row in review['bindings']:assert metadata(Path(row['path']))==row
 original=(source/'command_lease_guard.v').read_bytes().decode('utf-8')
 assert '\r'not in original,'Preserve source newlines explicitly rather than normalize them'
 changed=rewrite(original)
 assert changed==LOCAL_CANDIDATE.read_text(),'reuse exact already proved explicit tree'
 shutil.copytree(source,out)
 (out/'command_lease_guard.v').write_text(changed,encoding='utf-8',newline='')
 # Bind selected inputs to the new candidate, rather than its unchanged parent.
 run=(out/'run.tcl').read_text(encoding='utf-8')
 run=run.replace(source.as_posix(),out.as_posix()).replace(str(source),str(out))
 (out/'run.tcl').write_text(run,encoding='utf-8',newline='\n')
 # Preserve inherited model25 review as provenance, then issue a review for
 # this new one-file source candidate. Do not present the inherited proof as
 # proof of the new RTL or physical timing.
 (out/'prepared_source_review.json').rename(out/'inherited_model25_prepared_source_review.json')
 (out/'lease_bounded_MAX_delta.patch').write_text(''.join(difflib.unified_diff(
  original.splitlines(True),changed.splitlines(True),fromfile='prepared4/command_lease_guard.v',
  tofile='bounded_MAX/command_lease_guard.v')),encoding='utf-8',newline='')
 manifest=json.loads((source/'manifest.json').read_text())
 manifest['source_files']=[str(out/Path(p).relative_to(source))for p in manifest['source_files']]
 for p in manifest['source_files']:
  assert '{'+Path(p).as_posix()+'}' in run,p
 assert source.as_posix()not in run and str(source)not in run
 manifest.update(candidate='model25_NAV50_explicit_bounded_MAX_tree',frozen_source_build=str(source),
  source_preparation_only=True,automatic_pnr=False,deployment_accepted=False,synthesis_complete=False,
  additional_modified_rtl_inputs=['command_lease_guard.v'])
 save(out/'manifest.json',manifest)
 # The ModGen directory also carries an unselected model25_pll_tmp.v template.
 # Hardware inputs come from the actual manifest, not an rglob of auxiliaries.
 rtl=[Path(p).relative_to(out)for p in manifest['source_files']]
 assert len(rtl)==66,len(rtl)
 actual=BASE/'baseline_fusion_model25_NAV50_syn34'
 actual_names={Path(p).relative_to(actual).as_posix()for p in json.loads((actual/'manifest.json').read_text())['source_files']}
 assert actual_names=={n.as_posix()for n in rtl}
 assert all(same_bytes(source/n,actual/n)for n in rtl)
 assert all(same_bytes(source/n,actual/n)for n in ('g2.cst','g2.sdc'))
 changed_rtl=[str(n).replace('\\','/')for n in rtl if not same_bytes(source/n,out/n)]
 assert changed_rtl==['command_lease_guard.v'],changed_rtl
 assets=[p.relative_to(source)for p in source.rglob('*')if p.is_file()and p.suffix.lower()in{'.hex','.mem','.mif','.bin'}]
 assert len(assets)==69 and all(same_bytes(source/n,out/n)and same_bytes(source/n,actual/n)for n in assets)
 headers=[p.relative_to(source)for p in source.rglob('*')if p.is_file()and p.suffix.lower()in{'.h','.vh'}]
 assert len(headers)==31 and all(same_bytes(source/n,out/n)and same_bytes(source/n,actual/n)for n in headers)
 for folder in ('fw','firmware'):
  for p in (source/folder).rglob('*'):
   if p.is_file()and'__pycache__'not in p.parts:
    n=p.relative_to(source);assert same_bytes(p,out/n)and same_bytes(p,actual/n)
 assert same_bytes(source/'g2.sdc',out/'g2.sdc')and same_bytes(source/'g2.cst',out/'g2.cst')
 result=dict(prepared=True,recorded_at=datetime.now().astimezone().isoformat(),source=str(source),out=str(out),
  only_changed_RTL=changed_rtl,unchanged_selected_RTL=65,selected_RTL=66,
  unselected_modgen_template_byte_identical=same_bytes(source/'model25_pll/model25_pll_tmp.v',out/'model25_pll/model25_pll_tmp.v'),
  identical_assets=69,identical_headers=31,
  original_firmware_and_all_weights_byte_identical=True,clock_CST_SDC_capacities_unchanged=True,
  selected_run_Tcl_paths_bound_to_new_candidate=True,
  original_counting_saturation_ramp_output_cycles_and_priority_unchanged=True,
  production_timeout_ms=200,age_unsigned_bits=32,timeout_integer_bits=32,
  arithmetic_domain='Exact already proved explicit11LUT NAND32 tree, normal200ms; all32bit age and timeout known bit patterns; timeout0 retains original arithmetic/XZ behavior.',
  full_state_math_proof_pending=False,targeted_cycle_comparison_pending=False,
  exact_math_proof=str(LOCAL_PROOF/'math_proof.json'),exact_finite_cycle_proof=str(LOCAL_PROOF/'focused_cycle_result.json'),
  local_mapping_topology_proof=str(LOCAL_PROOF/'actual_tree_topology.json'),
  local_consumption_context=str(BASE/'lease_consumption_context_candidate1/result.json'),
  local_reference_did_not_reproduce_fullG2_prefix=True,
  new_mapping_or_timing_benefit_verified=False,fullG2_explicit_tree_mapping_not_verified=True,
  new_SYN_PNR_started=False,programmed=False,deployment_accepted=False,
  bindings=[metadata(out/'command_lease_guard.v'),metadata(out/'manifest.json'),metadata(out/'g2.sdc'),metadata(out/'g2.cst'),metadata(out/'run.tcl')])
 save(out/'prepared_source_review.json',result)
 return result

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=DEFAULT_SOURCE);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
 print(json.dumps(prepare(a.source,a.out),indent=2))
