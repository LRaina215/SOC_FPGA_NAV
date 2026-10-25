"""Prepare only the proved norm-register rebalance on the routed old baseline.

This command copies frozen inputs and writes a patch/provenance review. It never
invokes synthesis, implementation, simulation, firmware builds or programming.
The retimed payload must match the preserved v4 behavioral proof and the actual
successful isolated synthesis inputs; no current working-tree RTL is adopted.
"""
import argparse
import copy
from datetime import datetime
import difflib
import json
from pathlib import Path
import re
import shutil

ASSET_SUFFIXES = {'.hex', '.mem', '.bin', '.mif'}


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def save(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n',
                          encoding='utf-8', newline='\n')


def metadata(path):
    path = Path(path)
    stat = path.stat()
    return dict(path=str(path), bytes=stat.st_size, mtime_ns=stat.st_mtime_ns)


def same_bytes(a, b):
    if a.stat().st_size != b.stat().st_size:
        return False
    with a.open('rb') as first, b.open('rb') as second:
        while True:
            left, right = first.read(65536), second.read(65536)
            if left != right:
                return False
            if not left:
                return True


def relocate(value, source, out):
    if isinstance(value, str):
        return value.replace(source.as_posix(), out.as_posix()).replace(str(source), str(out))
    if isinstance(value, list):
        return [relocate(x, source, out) for x in value]
    if isinstance(value, dict):
        return {k: relocate(v, source, out) for k, v in value.items()}
    return value


def retime(original):
    """The three exact text changes used by the preserved unsigned v4 proof."""
    changes = [
        ('    reg [63:0] sqrt_value = 0;', '''    // Preserve the original absolute-value/square spelling and numeric range.
    // These registers, like the old sqrt_value, retain their value on reset;
    // sqrt_start and the engine reset still prevent stale values being used.
    reg [63:0] norm_square_x=0,norm_square_y=0,norm_square_z=0;
    // The maximum sum is 3*2^46, so 64 bits retain the original exact range.
    // The signed-square candidates triggered SP00019 both with and without
    // carry-save; the exact tool-internal cause remains unverified.
    wire [63:0] sqrt_value=norm_square_x+norm_square_y+norm_square_z;'''),
        ('''                sqrt_value <= {32'd0, abs16(accel_x)} * abs16(accel_x)
                            + {32'd0, abs16(accel_y)} * abs16(accel_y)
                            + {32'd0, abs16(accel_z)} * abs16(accel_z);''', '''                norm_square_x<={32'd0, abs16(accel_x)} * abs16(accel_x);
                norm_square_y<={32'd0, abs16(accel_y)} * abs16(accel_y);
                norm_square_z<={32'd0, abs16(accel_z)} * abs16(accel_z);'''),
        ('''                sqrt_value <= abs24(egx) * abs24(egx)
                            + abs24(egy) * abs24(egy)
                            + abs24(egz) * abs24(egz);''', '''                norm_square_x<=abs24(egx) * abs24(egx);
                norm_square_y<=abs24(egy) * abs24(egy);
                norm_square_z<=abs24(egz) * abs24(egz);''')]
    for before, after in changes:
        assert original.count(before) == 1, ('unexpected frozen fusion input', before)
        original = original.replace(before, after, 1)
    return original


def hardware_inputs(source, manifest):
    paths = [Path(p).resolve() for p in manifest['source_files']]
    result = {p.relative_to(source).as_posix(): p for p in paths}
    assert len(result) == len(paths)
    return result


def historical_reviews(source, history_root):
    baseline = hardware_inputs(source, load(source/'manifest.json'))
    rows = []
    for name in ('directed_fusion_coverage_effective50',
                 'directed_fusion_pose_coverage_effective50',
                 'directed_fusion_pose_unsigned_coverage_effective50'):
        build = history_root/name
        inputs = hardware_inputs(build, load(build/'manifest.json'))
        assert set(inputs) == set(baseline)
        changed = sorted(k for k in baseline if not same_bytes(baseline[k], inputs[k]))
        log = (build/'gw_sh.log').read_text(encoding='utf-8', errors='replace')
        rows.append(dict(build=str(build), modified_rtl_inputs=changed,
            errors=re.findall(r'^.*ERROR\s*\(.*$', log, re.M),
            emitted_netlist=(build/'impl/gwsynthesis/g2_top.vg').exists(),
            generated_report_100_percent='[100%] Generate report file ' in log,
            PNR_directory_present=(build/'impl/pnr').exists(),
            scope='Only this actual frozen directory; no physical timing claim.'))
    assert rows[0]['modified_rtl_inputs'] == ['control/imu_fusion.v']
    assert all('SP00019' in row['errors'][0] for row in rows[:2])
    assert rows[2]['modified_rtl_inputs'] == ['control/imu_fusion.v', 'nav/rtl/top/nav_subsystem.v']
    assert rows[2]['generated_report_100_percent'] and not rows[2]['errors']
    return rows


def prepare(source, routed_reference, proof, isolated_synthesis, out):
    source, routed_reference, proof, isolated_synthesis, out = [
        Path(p).resolve() for p in (source, routed_reference, proof, isolated_synthesis, out)]
    assert not out.exists() and source not in out.parents and out not in source.parents
    manifest = load(source/'manifest.json')
    assert source.name == 'directed_static_coverage_effective50'
    assert manifest['stage'] == 'syn' and manifest['clock_core_mhz'] == 50
    assert not manifest['control_weights_changed'] and not manifest['programmed']
    assert not manifest.get('fusion_norm_pipeline') and not manifest.get('pose_predecode')
    gate = load(routed_reference/'g2_gate.json')
    assert gate['routing_passed'] and not gate['g2_resource_core_gate_pass']
    assert same_bytes(source/'impl/gwsynthesis/g2_top.vg', routed_reference/'g2_top.vg')
    assert all(same_bytes(source/name, routed_reference/name) for name in ('g2.cst', 'g2.sdc'))
    assert (source/'g2.sdc').read_text().splitlines() == [
        'create_clock -name clk50 -period 20 [get_ports {clk50}]',
        'create_clock -name rgmii_rxc -period 10 [get_ports {rgmii_rxc}]']
    assert 'CLOCK_LOC ' not in (source/'g2.cst').read_text(encoding='utf-8')
    original = (source/'control/imu_fusion.v').read_text(encoding='utf-8')
    candidate = retime(original)
    proof_result = load(proof/'result.json')
    assert proof_result['pass_'] and proof_result['external_outputs_state_start_sqrt_input_cycle_exact']
    assert proof_result['clock_cycles'] == 200000 and proof_result['norm_extreme_random_vectors'] == 64
    assert 'TEST PASSED' in (proof/'xsim.log').read_text(encoding='utf-8')
    assert 'Fatal:' not in (proof/'xsim.log').read_text(encoding='utf-8')
    canonical_original = (proof/'fusion_original.v').read_text(encoding='utf-8')
    canonical_original = canonical_original.replace('ORIGINAL_IMU_FUSION_V_INCLUDED', 'IMU_FUSION_V_INCLUDED')
    canonical_original = re.sub(r'\bfusion_original\b', 'imu_fusion', canonical_original)
    canonical_original = re.sub(r'\boriginal_divider3\b', 'imu_div100_3', canonical_original)
    assert canonical_original == original, 'frozen behavioral reference differs beyond harness names/newlines'
    assert candidate == (proof/'fusion_candidate.v').read_text(encoding='utf-8')
    assert same_bytes(source/'control/imu_fusion.v', isolated_synthesis/'original/fusion.v')
    assert same_bytes(proof/'fusion_candidate.v', isolated_synthesis/'retimed/fusion.v')
    isolated = load(isolated_synthesis/'result.json')
    assert all(row['synthesis_succeeded'] and row['tool_exit'] == 0 and not row['errors']
               for row in isolated['runs'])
    history = historical_reviews(source, source.parent)
    hardware = hardware_inputs(source, manifest)
    assert len(hardware) == 63
    assets = {p.relative_to(source).as_posix(): p for p in source.rglob('*')
        if p.is_file() and 'impl' not in p.relative_to(source).parts and p.suffix.lower() in ASSET_SUFFIXES}
    headers = {p.relative_to(source).as_posix(): p for p in source.rglob('*')
        if p.is_file() and 'impl' not in p.relative_to(source).parts and p.suffix.lower() in {'.vh', '.h'}}
    assert len(assets) == 69 and len(headers) == 31
    out.mkdir(parents=True)
    for name, origin in {**hardware, **assets, **headers}.items():
        target = out/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(origin, target)
    for name in ('fw', 'firmware'):
        if (source/name).is_dir():
            shutil.copytree(source/name, out/name, dirs_exist_ok=True)
    for name in ('g2.cst', 'g2.sdc', 'storage_layout.json'):
        shutil.copyfile(source/name, out/name)
    shutil.copyfile(proof/'fusion_candidate.v', out/'control/imu_fusion.v')
    modified = sorted(name for name in hardware if not same_bytes(source/name, out/name))
    assert modified == ['control/imu_fusion.v']
    assert all(same_bytes(p, out/name) for name, p in {**assets, **headers}.items())
    run = (source/'run.tcl').read_text(encoding='utf-8')
    assert [x.strip() for x in run.splitlines() if x.strip().startswith('run ')] == ['run syn']
    (out/'run.tcl').write_text(relocate(run, source, out), encoding='utf-8', newline='\n')
    prepared_manifest = relocate(copy.deepcopy(manifest), source, out)
    prepared_manifest.pop('effective_options', None)
    prepared_manifest.update(source_files=[str(out/name) for name in hardware],
        frozen_source_build=str(source), fusion_norm_pipeline=True,
        source_preparation_only=True, automatic_pnr=False, programmed=False,
        deployment_accepted=False, candidate='baseline_fusion_only_original_square_retime',
        modified_rtl_inputs=modified, synthesis_complete=False)
    save(out/'manifest.json', prepared_manifest)
    patch = ''.join(difflib.unified_diff(original.splitlines(keepends=True),
        candidate.splitlines(keepends=True), fromfile='a/control/imu_fusion.v',
        tofile='b/control/imu_fusion.v'))
    (out/'fusion_only.patch').write_text(patch, encoding='utf-8', newline='\n')
    original_row, retimed_row = isolated['runs']
    def total(row, key):
        return int(re.match(r'\d+', row['synthesis_resources']['T_'+key])[0])
    resource_keys = ('Register', 'Lut', 'Alu', 'MULTALU27X18', 'MULT27X36')
    resource_evidence = dict(scope='Historical isolated SYN totals T_*, not fresh joint or physical values.',
        original={k: total(original_row, k) for k in resource_keys},
        retimed={k: total(retimed_row, k) for k in resource_keys},
        delta={k: total(retimed_row, k)-total(original_row, k) for k in resource_keys},
        estimated_internal_fmax_mhz=[original_row['estimated_fmax_mhz'], retimed_row['estimated_fmax_mhz']],
        source_binding_checked=True, joint_RAM_and_timing_unmeasured=True)
    contract = dict(FSM_states_unchanged=True, sqrt_start_cycles_unchanged=True,
        external_output_cycles_unchanged=True, arithmetic_widths_and_abs_square_expressions_unchanged=True,
        previous_sum_register_replaced_by_three_64bit_square_registers=True,
        square_register_capture_states=[4, 9], sqrt_engine_captures_sum_on_original_next_edge=True,
        no_new_FSM_state_or_extra_sample_latency=True, reset_behavior='Both old sum and new squares retain values on reset; engine/start reset prevents stale use.',
        initial_values_all_zero=True, norm_sum_maximum='3*2^46; original64-bit sum preserved',
        proof_scope='Reused finite 200000-cycle full RTL equivalence, not new formal or physical acceptance.',
        proof_result=proof_result, isolated_resources=resource_evidence)
    result = dict(passed=True, review_kind='strict_source_preparation_and_frozen_proof_binding_only',
        created_at=datetime.now().astimezone().isoformat(), source=str(source),
        routed_reference=str(routed_reference), prepared=str(out),
        modified_rtl_inputs=modified, hardware_inputs=63, unchanged_hardware_inputs=62,
        unchanged_assets=69, unchanged_headers=31,
        complete_original4500_weights_125Hz_CPU_NAV_and_stop_inputs_byte_identical=True,
        original_IO_and_20ns10ns_SDC_byte_identical=True, no_pose_or_other_optimization_adopted=True,
        candidate_bytes_equal_preserved_behavioral_and_isolated_SYN_input=True,
        original_behavioral_reference_equal_after_harness_names_and_newline_normalization=True,
        existing_proof_reused_without_new_simulation=True, contract=contract,
        historical_variants=history, bindings=[metadata(p) for p in (
            source/'control/imu_fusion.v', routed_reference/'g2_top.vg',
            proof/'fusion_original.v', proof/'fusion_candidate.v', proof/'result.json',
            proof/'xsim.log', isolated_synthesis/'original/fusion.v',
            isolated_synthesis/'retimed/fusion.v', isolated_synthesis/'result.json',
            out/'control/imu_fusion.v', out/'fusion_only.patch')],
        actual_joint_SYN_RAM_INIT_clock_CDC_and_timing_pending=True,
        no_synthesis_or_PNR_started=True, ready_for_PNR=False, programmed=False,
        deployment_accepted=False)
    save(out/'prepared_source_review.json', result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('source', 'routed-reference', 'proof', 'isolated-synthesis', 'out'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    result = prepare(args.source, args.routed_reference, args.proof, args.isolated_synthesis, args.out)
    print(json.dumps({k: v for k, v in result.items() if k not in ('contract', 'historical_variants', 'bindings')},
                     ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
