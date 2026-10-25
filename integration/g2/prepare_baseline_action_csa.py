"""Freeze the exact constant-CSA action module onto routed SYN23, no build."""
import argparse
import copy
from datetime import datetime
import difflib
import json
from pathlib import Path
import re
import shutil

from prepare_action_constant_csa import SCALE
from prepare_baseline_fusion_retime import hardware_inputs,metadata,relocate,same_bytes,save


ROOT=Path('F:/nav_integration_g2_20261023')
ORIGIN=ROOT/'baseline_fusion_only_syn23'
BASELINE=ROOT/'baseline_fusion_only_pnr32'
CANDIDATE=ROOT/'action_constant_csa_candidate1'
ACTION='control/policy_action_map.v'


def load(path):return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def bind_contract(source,prepared,proof):
    """No trust in a summary alone: bind actual original/tested/module text."""
    assert Path(proof['source']).resolve()==ORIGIN.resolve()
    assert Path(proof['routed_reference']).resolve()==BASELINE.resolve()
    assert proof['modified_rtl_inputs']==[ACTION]
    for row in proof['bindings']:
        assert metadata(Path(row['path']))==row,('proof input changed',row['path'])
    numeric=load(CANDIDATE/'result.json')
    module=load(CANDIDATE/'module_syn1/result.json')
    assert numeric['passed'] and numeric['frames']==8770 and numeric['exhaustive_scale_values_checked']==32772
    assert numeric['clip_domain_values_per_gain']==8193 and numeric['parameter_sets']==2
    assert numeric['outputs_last_actions_and_validity_cycle_exact']
    assert numeric['original64bit_product_and_rounding_preserved'] and numeric['all_other_integer_gains_literal_original_fallback']
    assert numeric['invalid_cycles']==numeric['run_stop_cycles']==64 and numeric['reset_scenarios']==1
    for row in numeric['source_bindings']:
        assert metadata(Path(row['path']))==row
    assert module['passed'] and module['runtime']['tool_exit']==0 and not module['runtime']['cancelled']
    assert module['target_action_DSP_removed_in_isolated_mapping'] is True and module['DSP_primitives']=={}
    assert (CANDIDATE/'module_syn1/action.v').read_text(encoding='utf-8')==(CANDIDATE/'policy_action_map.v').read_text(encoding='utf-8').replace('module policy_action_map #(','module g2_top #(',1)
    old=(ORIGIN/ACTION).read_text(encoding='utf-8')
    original_test=(CANDIDATE/'action_original.v').read_text(encoding='utf-8').replace('module action_map_original #(','module policy_action_map #(',1)
    assert old==original_test
    pattern=re.compile(r'    function signed \[31:0\] scale;.*?    endfunction',re.S)
    assert len(pattern.findall(old))==1
    expected=pattern.sub(lambda match:SCALE,old)
    assert (CANDIDATE/'policy_action_map.v').read_text(encoding='utf-8')==expected
    assert pattern.sub('SCALE',expected)==pattern.sub('SCALE',old)
    assert re.findall(r'scale\(actions\[[^\]]+\],(-?\d+)\)',expected)==['-600','600','19','-19','19','19']
    assert 'TEST PASSED frames=8770' in (CANDIDATE/'xsim.log').read_text(encoding='utf-8',errors='replace')
    for root in (prepared,source):
        assert same_bytes(CANDIDATE/'policy_action_map.v',root/ACTION)
        manifest=load(root/'manifest.json')
        assert manifest['action_map_constant_CSA'] and manifest['fusion_norm_pipeline']
        assert not any(manifest.get(k) for k in ('pose_predecode','action_map_exact_widths','fusion_square_pipeline2','control_weights_changed','programmed'))
    assert same_bytes(ORIGIN/'impl/gwsynthesis/g2_top.vg',BASELINE/'g2_top.vg')


def prepare(out):
    out=Path(out).resolve();assert not out.exists()
    numeric=load(CANDIDATE/'result.json');module=load(CANDIDATE/'module_syn1/result.json')
    assert numeric['passed'] and module['passed'] and module['target_action_DSP_removed_in_isolated_mapping']
    assert module['DSP_primitives']=={}
    manifest=load(ORIGIN/'manifest.json')
    assert manifest['stage']=='syn' and manifest['fusion_norm_pipeline']
    assert not any(manifest.get(k) for k in ('pose_predecode','action_map_exact_widths','fusion_square_pipeline2','control_weights_changed','programmed'))
    assert load(ORIGIN/'job.json')['phase']=='synthesis_complete'
    assert load(BASELINE/'g2_gate.json')['routing_passed']
    hw=hardware_inputs(ORIGIN,manifest);assert len(hw)==63
    assets={p.relative_to(ORIGIN).as_posix():p for p in ORIGIN.rglob('*') if p.is_file() and 'impl' not in p.relative_to(ORIGIN).parts and p.suffix.lower() in {'.hex','.mem','.bin','.mif'}}
    headers={p.relative_to(ORIGIN).as_posix():p for p in ORIGIN.rglob('*') if p.is_file() and 'impl' not in p.relative_to(ORIGIN).parts and p.suffix.lower() in {'.h','.vh'}}
    assert len(assets)==69 and len(headers)==31
    out.mkdir(parents=True)
    for name,path in {**hw,**assets,**headers}.items():
        destination=out/name;destination.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(path,destination)
    firmware={}
    for directory in ('fw','firmware'):
        shutil.copytree(ORIGIN/directory,out/directory,dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
        files=[p for p in (ORIGIN/directory).rglob('*') if p.is_file() and '__pycache__' not in p.parts]
        assert all(same_bytes(p,out/p.relative_to(ORIGIN)) for p in files)
        firmware[directory]=len(files)
    for name in ('g2.cst','g2.sdc','storage_layout.json'):
        shutil.copyfile(ORIGIN/name,out/name)
    shutil.copyfile(CANDIDATE/'policy_action_map.v',out/ACTION)
    changed=sorted(name for name in hw if not same_bytes(ORIGIN/name,out/name));assert changed==[ACTION]
    assert all(same_bytes(path,out/name) for name,path in {**assets,**headers}.items())
    assert all(same_bytes(BASELINE/name,out/name) for name in ('g2.cst','g2.sdc'))
    run=(ORIGIN/'run.tcl').read_text(encoding='utf-8')
    assert [line.strip() for line in run.splitlines() if line.strip().startswith('run ')]==['run syn']
    (out/'run.tcl').write_text(relocate(run,ORIGIN,out),encoding='utf-8',newline='\n')
    frozen=relocate(copy.deepcopy(manifest),ORIGIN,out);frozen.pop('effective_options',None)
    frozen.update(source_files=[str(out/name) for name in hw],frozen_source_build=str(ORIGIN),
        action_map_constant_CSA=True,action_map_exact_widths=False,pose_predecode=False,fusion_square_pipeline2=False,
        source_preparation_only=True,synthesis_complete=False,automatic_pnr=False,programmed=False,deployment_accepted=False,
        modified_rtl_inputs=changed,action_constant_CSA_proof=str(CANDIDATE/'result.json'),
        action_constant_CSA_module_proof=str(CANDIDATE/'module_syn1/result.json'))
    save(out/'manifest.json',frozen)
    result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),source=str(ORIGIN),
        routed_reference=str(BASELINE),prepared=str(out),modified_rtl_inputs=changed,
        hardware_inputs=63,unchanged_hardware_inputs=62,identical_assets=69,identical_headers=31,
        firmware_files=firmware,all_original_firmware_and_build_timestamp_bytes_identical=True,
        firmware_not_recompiled=True,only_private_scale_function_changed=True,
        no_POSE_square2_failed_width_or_other_donor_change_adopted=True,
        external_registers_validity_stop_and_reset_cycles_preserved=True,
        original_4500_all_weights125Hz_CPU128KiB_NAV_capacities_and_stop_sources_unchanged=True,
        original_IO_CST_and20ns10ns_SDC_byte_identical=True,
        numeric_proof=str(CANDIDATE/'result.json'),module_proof=str(CANDIDATE/'module_syn1/result.json'),
        bindings=[metadata(p) for p in [ORIGIN/ACTION,CANDIDATE/'action_original.v',CANDIDATE/'policy_action_map.v',
            CANDIDATE/'result.json',CANDIDATE/'xsim.log',CANDIDATE/'module_syn1/action.v',
            CANDIDATE/'module_syn1/result.json',CANDIDATE/'module_syn1/impl/gwsynthesis/g2_top.vg',out/ACTION,
            ORIGIN/'firmware/nav_fw.bin',out/'firmware/nav_fw.bin',BASELINE/'g2_top.vg']],
        new_joint_SYN_PNR_started=False,programmed=False,deployment_accepted=False)
    save(out/'prepared_source_review.json',result)
    bind_contract(out,out,result)
    (out/'action_constant_CSA.patch').write_text(''.join(difflib.unified_diff((ORIGIN/ACTION).read_text().splitlines(True),(out/ACTION).read_text().splitlines(True),fromfile='a/'+ACTION,tofile='b/'+ACTION)),encoding='utf-8')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--out',type=Path,required=True)
    result=prepare(parser.parse_args().out);print(json.dumps({k:result[k] for k in ('passed','source','prepared','modified_rtl_inputs')}))
