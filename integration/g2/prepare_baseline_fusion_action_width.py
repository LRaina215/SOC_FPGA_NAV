"""Add only the proved action-map widths to the routed fusion-only source.

No firmware build, simulation, synthesis or implementation is run here.
The donor's POSE change and timestamp assets are deliberately not adopted.
"""
import argparse
import copy
from datetime import datetime
import difflib
import json
from pathlib import Path
import re
import shutil

from prepare_baseline_fusion_retime import hardware_inputs, metadata, relocate, same_bytes, save

BASE = Path('F:/nav_integration_g2_20261023')
SOURCE = BASE/'baseline_fusion_only_syn23'
ROUTED = BASE/'baseline_fusion_only_pnr32'
DONOR = BASE/'directed_action_coverage50'
PROOF = Path('F:/nav_integration_g2_20261022/action_map_exact_widths_equivalence')
SOURCE_REVIEW = BASE/'baseline_single_factor_parallel_syn1/review_only_recovery_20261025_235536_156419/fusion_actual_netlist_review.json'
OUT = BASE/'baseline_fusion_action_width_prepared1'
ACTION = 'control/policy_action_map.v'


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def prepare(out=OUT):
    source, routed, donor, proof, out = [p.resolve() for p in (SOURCE, ROUTED, DONOR, PROOF, Path(out))]
    assert not out.exists()
    job = load(source/'job.json'); gate = load(routed/'g2_gate.json')
    assert job['phase'] == 'synthesis_complete' and job['tool_exit'] == 0
    assert gate['routing_passed']
    assert same_bytes(source/'impl/gwsynthesis/g2_top.vg', routed/'g2_top.vg')
    reviewed = load(SOURCE_REVIEW)
    assert reviewed['passed'] and reviewed['actual_RAM_INIT_clock_26CDC_preflight_passed']
    assert Path(reviewed['source']).resolve() == source
    assert same_bytes(source/'impl/gwsynthesis/g2_top.vg', Path(reviewed['reviewed_netlist_copy']))
    manifest = load(source/'manifest.json')
    assert manifest['stage'] == 'syn' and manifest['fusion_norm_pipeline']
    assert not manifest.get('pose_predecode') and not manifest.get('action_map_exact_widths')
    assert manifest['wide_split'] == 1 and manifest['clock_core_mhz'] == 50
    assert not manifest['control_weights_changed'] and not manifest['programmed']
    original = (source/ACTION).read_text(encoding='utf-8')
    reference = (proof/'action_original.v').read_text(encoding='utf-8')
    reference = reference.replace('module action_map_original #(', 'module policy_action_map #(', 1)
    assert reference == original
    assert same_bytes(donor/ACTION, proof/'action_candidate.v')
    candidate = (donor/ACTION).read_text(encoding='utf-8')
    function = re.compile(r'    function signed \[31:0\] scale;.*?    endfunction', re.S)
    assert len(function.findall(original)) == len(function.findall(candidate)) == 1
    assert function.sub('ACTION_SCALE_FUNCTION', original) == function.sub('ACTION_SCALE_FUNCTION', candidate)
    gains = re.findall(r'scale\(actions\[[^\]]+\],(-?\d+)\)', candidate)
    assert gains == ['-600','600','19','-19','19','19']
    evidence = load(proof/'result.json')
    assert evidence['pass_'] and evidence['frames'] == 8770
    assert evidence['clip_domain_values_per_gain'] == 8193
    assert evidence['invalid_cycles'] == evidence['run_stop_cycles'] == 64
    assert evidence['reset_scenarios'] == 1 and evidence['parameter_sets'] == 2
    assert evidence['outputs_last_actions_and_validity_cycle_exact']
    log = (proof/'xsim.log').read_text(encoding='utf-8', errors='replace')
    assert 'TEST PASSED' in log and 'Fatal:' not in log
    fallback = load(BASE/'baseline_action_fallback_review_20261025/reuse_review.json')
    assert fallback['passed'] and fallback['numeric_cycle_proof']['baseline_matches_proof_reference_except_test_module_name']
    assert fallback['numeric_cycle_proof']['standby_matches_proof_candidate_exact_bytes']
    hardware = hardware_inputs(source, manifest)
    assert len(hardware) == 63
    assets = {p.relative_to(source).as_posix():p for p in source.rglob('*') if p.is_file()
              and 'impl' not in p.relative_to(source).parts and p.suffix.lower() in {'.hex','.mem','.bin','.mif'}}
    headers = {p.relative_to(source).as_posix():p for p in source.rglob('*') if p.is_file()
               and 'impl' not in p.relative_to(source).parts and p.suffix.lower() in {'.vh','.h'}}
    assert len(assets) == 69 and len(headers) == 31
    out.mkdir(parents=True)
    for name, origin in {**hardware, **assets, **headers}.items():
        destination = out/name; destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(origin, destination)
    firmware_counts = {}
    for directory in ('fw','firmware'):
        shutil.copytree(source/directory, out/directory, dirs_exist_ok=True,
                        ignore=shutil.ignore_patterns('__pycache__'))
        files = [p for p in (source/directory).rglob('*') if p.is_file() and '__pycache__' not in p.relative_to(source).parts]
        assert all(same_bytes(p, out/p.relative_to(source)) for p in files)
        firmware_counts[directory] = len(files)
    for name in ('g2.cst','g2.sdc','storage_layout.json'):
        shutil.copyfile(source/name,out/name)
    shutil.copyfile(proof/'action_candidate.v',out/ACTION)
    changed = sorted(name for name in hardware if not same_bytes(source/name,out/name))
    assert changed == [ACTION]
    assert all(same_bytes(p,out/name) for name,p in {**assets,**headers}.items())
    assert all(same_bytes(source/name,out/name) and same_bytes(source/name,routed/name) for name in ('g2.cst','g2.sdc'))
    run = (source/'run.tcl').read_text(encoding='utf-8')
    assert [line.strip() for line in run.splitlines() if line.strip().startswith('run ')] == ['run syn']
    (out/'run.tcl').write_text(relocate(run, source, out),encoding='utf-8',newline='\n')
    prepared = relocate(copy.deepcopy(manifest), source, out)
    prepared.pop('effective_options', None)
    prepared.update(source_files=[str(out/name) for name in hardware], frozen_source_build=str(source),
        action_map_exact_widths=True, pose_predecode=False, source_preparation_only=True,
        synthesis_complete=False, automatic_pnr=False, programmed=False, deployment_accepted=False,
        candidate='routed_fusion_plus_cycle_exact_action_widths_only', modified_rtl_inputs=changed,
        action_width_proof=str(proof/'result.json'))
    save(out/'manifest.json', prepared)
    patch = ''.join(difflib.unified_diff(original.splitlines(keepends=True),candidate.splitlines(keepends=True),
        fromfile='a/'+ACTION,tofile='b/'+ACTION))
    (out/'action_width_only.patch').write_text(patch,encoding='utf-8',newline='\n')
    result = dict(passed=True,review_kind='strict frozen routed-fusion source and reused finite exact-action proof preparation',
        recorded_at=datetime.now().astimezone().isoformat(),source=str(source),routed_reference=str(routed),
        prepared=str(out),donor_action_source=str(donor/ACTION),modified_rtl_inputs=changed,
        hardware_inputs=63,unchanged_hardware_inputs=62,identical_assets=69,identical_headers=31,
        firmware_files=firmware_counts,all_original_firmware_and_build_timestamp_bytes_identical=True,
        firmware_not_recompiled=True,only_private_scale_function_changed=True,no_POSE_weight9_or_other_donor_change_adopted=True,
        prior_fusion_RTL_byte_identical=True,original_NAV_CPU_capacity_and_stop_RTL_byte_identical=True,
        original_4500_all_weights_and125Hz_sources_byte_identical=True,external_registers_validity_stop_and_reset_cycles_preserved=True,
        original_IO_CST_and20ns10ns_SDC_byte_identical=True,
        source_actual_RAM=dict(source_BSRAM=reviewed['source_BSRAM'],wide_leaves=reviewed['wide_leaves'],
            CPU_SP_count=reviewed['CPU_SP_count'],source_actual_INIT_and_CPU128KiB_proved=True,
            source_preflight=str(SOURCE_REVIEW),new_joint_RAM_INIT_clock_CDC_pending=True),
        reused_proof=evidence,contract=dict(clipped_action_domain=[-4096,4096],gain_callset=list(map(int,gains)),
            maximum_absolute_product=2457600,maximum_rounding_sum=2459648,product_bits=23,
            output_ports_and_register_edges_unchanged=True,quantization=False,additional_cycles=0,
            scope='Stored finite8770-frame full-module comparison and exhaustive clipped-domain scale tests; not all crossproducts or physical/vehicle acceptance'),
        bindings=[metadata(p) for p in (source/ACTION,proof/'action_original.v',proof/'action_candidate.v',proof/'result.json',
                  donor/ACTION,out/ACTION,source/'firmware/nav_fw.bin',out/'firmware/nav_fw.bin',routed/'g2_top.vg')],
        new_SYN_or_PNR_started=False,programmed=False,deployment_accepted=False)
    save(out/'prepared_source_review.json',result)
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,default=OUT);args=parser.parse_args()
    result=prepare(args.out);print(json.dumps({k:v for k,v in result.items() if k in ('passed','source','prepared','modified_rtl_inputs','identical_assets','identical_headers')},ensure_ascii=False))


if __name__=='__main__':main()
