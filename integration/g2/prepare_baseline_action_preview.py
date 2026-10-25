"""Freeze the proved private-preview product stage on routed fusion SYN23."""
import argparse
import copy
from datetime import datetime
import json
from pathlib import Path
import shutil

from prepare_baseline_fusion_retime import hardware_inputs,metadata,relocate,same_bytes,save

BASE=Path('F:/nav_integration_g2_20261023')
ORIGIN=BASE/'baseline_fusion_only_syn23'
CANDIDATE=BASE/'action_preview_product_candidate1'

def prepare(out):
    assert not out.exists()
    load=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
    source_proof=load(CANDIDATE/'prepared_review.json')
    arithmetic=load(CANDIDATE/'arithmetic_result.json');policy=load(CANDIDATE/'policy_result.json')
    assert source_proof['prepared']and arithmetic['passed']and policy['passed']
    assert policy['unchanged_actual_scheduler']and policy['source_frame_ownership_relation_checked']
    changed=source_proof['changed_RTL'];assert len(changed)==3
    for row in source_proof['source_bindings']+source_proof['candidate_bindings']:
        assert metadata(Path(row['path']))==row
    manifest=load(ORIGIN/'manifest.json');assert manifest['stage']=='syn'and manifest['fusion_norm_pipeline']
    assert not any(manifest.get(k)for k in('pose_predecode','fusion_square_pipeline2','action_map_exact_widths','action_map_constant_CSA','control_weights_changed','programmed'))
    hw=hardware_inputs(ORIGIN,manifest);assert len(hw)==63
    allfiles=[p for p in ORIGIN.rglob('*')if p.is_file()and'impl'not in p.relative_to(ORIGIN).parts]
    assets={p.relative_to(ORIGIN).as_posix():p for p in allfiles if p.suffix.lower()in{'.hex','.mem','.bin','.mif'}}
    headers={p.relative_to(ORIGIN).as_posix():p for p in allfiles if p.suffix.lower()in{'.h','.vh'}}
    assert len(assets)==69 and len(headers)==31
    out.mkdir(parents=True)
    for name,path in {**hw,**assets,**headers}.items():
        dst=out/name;dst.parent.mkdir(exist_ok=True,parents=True);shutil.copyfile(path,dst)
    firmware={}
    for directory in('fw','firmware'):
        shutil.copytree(ORIGIN/directory,out/directory,dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
        files=[p for p in(ORIGIN/directory).rglob('*')if p.is_file()and'__pycache__'not in p.parts]
        assert all(same_bytes(p,out/p.relative_to(ORIGIN))for p in files);firmware[directory]=len(files)
    assert firmware==dict(fw=61,firmware=92)
    for name in('g2.cst','g2.sdc','storage_layout.json'):shutil.copyfile(ORIGIN/name,out/name)
    for name in changed:shutil.copyfile(CANDIDATE/name,out/name)
    assert sorted(name for name in hw if not same_bytes(ORIGIN/name,out/name))==sorted(changed)
    assert all(same_bytes(path,out/name)for name,path in{**assets,**headers}.items())
    run=(ORIGIN/'run.tcl').read_text();assert [s.strip()for s in run.splitlines()if s.strip().startswith('run ')]==['run syn']
    (out/'run.tcl').write_text(relocate(run,ORIGIN,out),newline='\n')
    frozen=relocate(copy.deepcopy(manifest),ORIGIN,out);frozen.pop('effective_options',None)
    frozen.update(source_files=[str(out/n)for n in hw],frozen_source_build=str(ORIGIN),action_preview_product=True,
        source_preparation_only=True,synthesis_complete=False,automatic_pnr=False,programmed=False,deployment_accepted=False,
        modified_rtl_inputs=changed,preview_source_review=str(CANDIDATE/'prepared_review.json'),preview_arithmetic_review=str(CANDIDATE/'arithmetic_result.json'),preview_scheduler_review=str(CANDIDATE/'policy_result.json'))
    save(out/'manifest.json',frozen)
    proof=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),source=str(ORIGIN),candidate=str(CANDIDATE),prepared=str(out),
        modified_rtl_inputs=changed,hardware_inputs=63,unchanged_hardware_inputs=60,identical_assets=69,identical_headers=31,
        firmware_files=firmware,firmware_not_recompiled=True,clock_CST_and_SDC_unchanged=True,
        inference_core_algorithm_all_weights_125Hz_and_external_actuation_edges_unchanged=True,
        CPU128KiB_NAV_capacities_and_stop_contract_preserved=True,new_SYN_or_PNR_started=False,
        private_preview_owned_by_same_original_scheduler_publication=True,global_clock_requests_or_new_timing_exceptions=0,
        actual_new_RAM_INIT_clocks_CDC_product_mapping_and_full_timing_review_pending=True,
        bindings=[metadata(p)for p in [CANDIDATE/n for n in changed]+[CANDIDATE/n for n in('prepared_review.json','arithmetic_result.json','policy_result.json')]+[out/n for n in changed]+[out/'g2.cst',out/'g2.sdc',out/'firmware/nav_fw.bin']])
    save(out/'prepared_source_review.json',proof)
    shutil.copyfile(CANDIDATE/'preview_product.patch',out/'preview_product.patch')
    return proof

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True)
    result=prepare(p.parse_args().out);print(json.dumps(result,indent=2))
