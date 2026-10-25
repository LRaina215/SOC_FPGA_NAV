"""Freeze the proved cycle-preserving fusion square pipeline on completeSYN23.

Source preparation only; no tool execution or hardware actions.
"""
import argparse,copy,difflib,json,re,shutil
from pathlib import Path
from datetime import datetime
from prepare_baseline_fusion_retime import hardware_inputs,metadata,relocate,same_bytes,save
from prepare_fusion_square_pipeline import generate
BASE=Path('F:/nav_integration_g2_20261023');SOURCE=BASE/'baseline_fusion_only_syn23';ROUTED=BASE/'baseline_fusion_only_pnr32'
PROOF=BASE/'fusion_square_pipeline2_candidate1';MATH=BASE/'fusion_square_late2_math_proof1/result.json';BOUNDARY=MATH.parent/'boundary_review.json';OUT=BASE/'baseline_fusion_square2_prepared1'
REVIEW=BASE/'baseline_single_factor_parallel_syn1/review_only_recovery_20261025_235536_156419/fusion_actual_netlist_review.json'
ACTION='control/imu_fusion.v'
load=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
def prepare(out=OUT):
 out=Path(out).resolve();assert not out.exists()
 sjob=load(SOURCE/'job.json');assert sjob['phase']=='synthesis_complete'and sjob['tool_exit']==0
 gate=load(ROUTED/'g2_gate.json');assert gate['routing_passed']and same_bytes(SOURCE/'impl/gwsynthesis/g2_top.vg',ROUTED/'g2_top.vg')
 prior=load(REVIEW);assert prior['passed']and prior['actual_RAM_INIT_clock_26CDC_preflight_passed']and same_bytes(SOURCE/'impl/gwsynthesis/g2_top.vg',Path(prior['reviewed_netlist_copy']))
 manifest=load(SOURCE/'manifest.json');assert manifest['stage']=='syn'and manifest['fusion_norm_pipeline']and not manifest.get('pose_predecode')and not manifest.get('action_map_exact_widths')
 assert manifest['clock_core_mhz']==50 and manifest['wide_split']==1 and not manifest['programmed']and not manifest['control_weights_changed']
 proof=load(PROOF/'result.json');math=load(MATH);boundary=load(BOUNDARY)
 assert proof['passed']and proof['clock_cycles']==200000 and proof['pipeline_reset_phases']==4 and proof['external_outputs_flags_valid_and_engine_publish_cycle_exact']
 assert math['passed']and len(math['proofs'])==5 and all(x['status']=='unsat'for x in math['proofs'])and math['fusion_sum_strictly_below_2_pow48']
 assert boundary['passed']and boundary['exact_inverse_transform_recovers_source']and boundary['helper_exact_inverse_transform_recovers_original_isqrt']and boundary['actual_helper_only_one_private_fusion_call']
 assert math['reset_interruption_phases_checked']==36 and math['reset_start_control_edges_checked']==3060
 for binding in math['source_bindings']+[math['candidate_metadata_observed_not_RTL_proof']]:
  assert metadata(Path(binding['path']))==binding, ('proof input metadata changed',binding['path'])
 assert Path(boundary['candidate']).resolve()==(PROOF/'imu_fusion.v').resolve()and Path(boundary['origin']).resolve()==(SOURCE/ACTION).resolve()
 original=(SOURCE/ACTION).read_text();candidate=(PROOF/'imu_fusion.v').read_text();sqrt=(SOURCE/'control/imu_isqrt.v').read_text()
 assert original==(PROOF/'source_imu_fusion.v').read_text()and candidate==generate(original,sqrt)
 assert same_bytes(SOURCE/ACTION,Path('F:/nav_integration_g2_20261022/fusion_pipeline_equivalence_v4/fusion_candidate.v'))
 def header(s):return s[s.index('module imu_fusion'):s.index(');',s.index('module imu_fusion'))+2]
 assert header(original)==header(candidate)
 assert re.findall(r'\bst\s*<=([^;]+);',original)==re.findall(r'\bst\s*<=([^;]+);',candidate)
 hardware=hardware_inputs(SOURCE,manifest);assert len(hardware)==63
 assets={p.relative_to(SOURCE).as_posix():p for p in SOURCE.rglob('*')if p.is_file()and'impl'not in p.relative_to(SOURCE).parts and p.suffix.lower()in{'.hex','.mem','.bin','.mif'}}
 headers={p.relative_to(SOURCE).as_posix():p for p in SOURCE.rglob('*')if p.is_file()and'impl'not in p.relative_to(SOURCE).parts and p.suffix.lower()in{'.vh','.h'}}
 assert len(assets)==69 and len(headers)==31
 out.mkdir(parents=True)
 for relative,src in {**hardware,**assets,**headers}.items():
  dst=out/relative;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(src,dst)
 fwcounts={}
 for directory in ('fw','firmware'):
  shutil.copytree(SOURCE/directory,out/directory,dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
  files=[p for p in (SOURCE/directory).rglob('*')if p.is_file()and'__pycache__'not in p.relative_to(SOURCE).parts]
  assert all(same_bytes(p,out/p.relative_to(SOURCE))for p in files);fwcounts[directory]=len(files)
 for n in ['g2.cst','g2.sdc','storage_layout.json']:shutil.copyfile(SOURCE/n,out/n)
 shutil.copyfile(PROOF/'imu_fusion.v',out/ACTION)
 changed=sorted(n for n,p in hardware.items()if not same_bytes(p,out/n));assert changed==[ACTION]
 assert all(same_bytes(src,out/n)for n,src in {**assets,**headers}.items())
 run=(SOURCE/'run.tcl').read_text();assert [s.strip()for s in run.splitlines()if s.strip().startswith('run ')]==['run syn']
 (out/'run.tcl').write_text(relocate(run,SOURCE,out),encoding='utf-8',newline='\n')
 m=relocate(copy.deepcopy(manifest),SOURCE,out);m.pop('effective_options',None)
 m.update(source_files=[str(out/n)for n in hardware],frozen_source_build=str(SOURCE),fusion_square_pipeline2=True,pose_predecode=False,action_map_exact_widths=False,source_preparation_only=True,synthesis_complete=False,automatic_pnr=False,programmed=False,deployment_accepted=False,candidate='routed_fusion_with_cycle_exact_two_stage_square_pipeline_only',modified_rtl_inputs=changed,fusion_square2_proof=str(PROOF/'result.json'),fusion_square2_math_proof=str(MATH),fusion_square2_boundary_review=str(BOUNDARY))
 save(out/'manifest.json',m)
 patch=''.join(difflib.unified_diff(original.splitlines(True),candidate.splitlines(True),fromfile='a/'+ACTION,tofile='b/'+ACTION));(out/'fusion_square2_only.patch').write_text(patch,encoding='utf-8')
 result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),source=str(SOURCE),routed_reference=str(ROUTED),prepared=str(out),modified_rtl_inputs=changed,hardware_inputs=63,unchanged_hardware_inputs=62,identical_assets=69,identical_headers=31,firmware_files=fwcounts,all_FW_and_boot_timestamp_bytes_identical=True,firmware_not_recompiled=True,no_failed_action_width_or_POSE_adopted=True,original_4500_weights125Hz_CPU_NAV_capacity_and_stop_sources_unchanged=True,external_FSM_start_and_publication_cycles_preserved=True,source_template_generator_exact=True,proof=str(PROOF/'result.json'),math_proof=str(MATH),independent_boundary_review=str(BOUNDARY),finite_RTL_proof=proof,independent_math_proof=math,independent_candidate_boundary_review=boundary,logical_added_private_register_bits=290,private_square_and_sqrt_rem_retimed_not_full_state_equivalence=True,IO_CST_and20ns10ns_SDC_byte_identical=True,new_joint_RAM_INIT_clock_and26CDC_review_required=True,source_actual_RAM=dict(source_BSRAM=prior['source_BSRAM'],wide_leaves=prior['wide_leaves'],CPU_SP_count=prior['CPU_SP_count']),new_actual_RAM_or_FF_DSP_resources_unknown=True,synthesis_or_PNR_started=False,programmed=False,deployment_accepted=False,bindings=[metadata(p)for p in [SOURCE/ACTION,PROOF/'imu_fusion.v',PROOF/'result.json',MATH,BOUNDARY,out/ACTION,SOURCE/'firmware/nav_fw.bin',out/'firmware/nav_fw.bin']])
 save(out/'prepared_source_review.json',result);return result
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=OUT);args=ap.parse_args();r=prepare(args.out);print(json.dumps({k:r[k]for k in ['passed','source','prepared','modified_rtl_inputs','identical_assets','identical_headers']},ensure_ascii=False))
