"""Bind the final SLF128/storage/safety source and completed consumer proofs.

This is the source admission for one joint synthesis. It does not accept a
newly mapped primitive, routing, timing, or vehicle behavior.
"""
from pathlib import Path
from datetime import datetime
import argparse
import re

from prepare_baseline_fusion_retime import hardware_inputs, load, metadata, same_bytes, save

BASE = Path('F:/nav_integration_g2_20261023')
REFERENCE = BASE/'baseline_fusion50_ring_MAX_prepared1'
CANDIDATE = BASE/'stage3_SLF128_storage_RTL_prepared5'
EXPECTED_CHANGED = {
    'nav/rtl/common/nav_ram_sdp.v', 'nav/rtl/common/nav_async_fifo.v',
    'nav/rtl/lidar/xt_frame_rx.v', 'nav/rtl/map/trav_v3.v',
    'nav/rtl/match/slf_lf.v', 'nav/rtl/match/slf_match.v',
    'nav/rtl/plan/traj_eval.v', 'nav/rtl/net/host_mbx.v',
    'nav/rtl/regs/nav_regfile.v', 'nav/rtl/top/nav_subsystem.v',
    'nav/rtl/soc/nav_soc.v', 'nav/rtl/net/rgmii_mac100.v', 'g2_top.v',
    'fc_v4_top.v', 'fixed4500_weight_rom.v'}


def review(combined_result, output, candidate=CANDIDATE, fifo_mapping=None,
           lf_mapping=None, lf_validation=None, weight_mapping=None,
           weight_source_proof=None, weight_actual_proof=None):
    global CANDIDATE
    CANDIDATE = candidate.resolve()
    assert not output.exists(), 'Keep prior review, never overwrite it'
    firmware_binding = load(CANDIDATE/'firmware_binding_review.json')
    for row in firmware_binding['complete_frozen_input_bindings']:
        assert metadata(row['path']) == row, row
    if fifo_mapping is None:
        assert firmware_binding['source63_and_all_FW_unchanged']
    assert firmware_binding['all11_CPU_assets_consistent']
    assert firmware_binding['firmware_BIN_bytes'] == 53572
    compiled = Path(firmware_binding['firmware_package'])
    release = load(compiled/'result.json')
    assert release['passed'] and release['no_recompile']
    assert release['all_four_lanes_and_word_image_reconstruct_BIN_and_zero_padding']
    stack = load(Path(release['strict_review']))
    assert stack['complete'] and stack['passed']
    assert stack['actual_static_bound_bytes'] == 2576
    assert release['remaining_after_6KiB_stack_bytes'] == 1348
    image = (CANDIDATE/'firmware/nav_fw.bin').read_bytes()
    assert same_bytes(CANDIDATE/'firmware/nav_fw.bin', compiled/'firmware/nav_fw.bin')
    assert same_bytes(CANDIDATE/'firmware/nav_fw.elf', compiled/'firmware/nav_fw.elf')
    full = image + bytes(131072-len(image))
    assert len(image) == release['image_bytes'] and len(full) == 131072
    for prefix in ('', 'firmware/'):
        lanes = [bytes(int(x, 16) for x in (CANDIDATE/f'{prefix}ram_b{i}.hex').read_text().split()) for i in range(4)]
        assert all(len(x) == 32768 for x in lanes)
        assert bytes(v for row in zip(*lanes) for v in row) == full
        words = [int(x, 16) for x in (CANDIDATE/f'{prefix}ram_w.hex').read_text().split()]
        assert b''.join(v.to_bytes(4, 'little') for v in words) == full
    m = load(CANDIDATE/'manifest.json')
    a = hardware_inputs(REFERENCE, load(REFERENCE/'manifest.json'))
    b = hardware_inputs(CANDIDATE, m)
    assert set(a) == set(b) and len(b) == 63
    changed = {name for name in a if not same_bytes(a[name], b[name])}
    assert changed == EXPECTED_CHANGED, changed
    assert m['SLF_LOG2N'] == 7 and m['NP_MAP_LOG2N_unchanged'] == 7
    assert m['actual_clock_plan_mhz'] == dict(all_NAV_CPU_fusion_RL_peripherals=50, radar_constraint=100)
    assert m['fusion_norm_pipeline'] and m['OBS_RING_STORAGE'] == 1
    assert not m['control_weights_changed'] and m['period_clocks'] == 400000
    safety = load(BASE/'stage2_SLF128_forward_safeFW_prepared8/safety_rebase_review.json')
    assert safety['FIFO_text_equals_proved_candidate3'] and safety['MAC_text_equals_proved_candidate2']
    for rel in ('nav/rtl/net/rgmii_mac100.v', 'g2_top.v'):
        assert same_bytes(CANDIDATE/rel, BASE/'stage2_SLF128_forward_safeFW_prepared8'/rel)
    assert same_bytes(CANDIDATE/'nav/rtl/common/nav_fifo_ob.v', REFERENCE/'nav/rtl/common/nav_fifo_ob.v')
    ports = (CANDIDATE/'nav/rtl/top/nav_subsystem.v').read_text()
    assert "iu_valid?{1'b1,22'd0,iu_first,iu_data}:32'd0" in re.sub(r'\s+', '', ports)
    targeted_path = BASE/'stage2_SLF128_targeted_validation2/job.json'
    targeted = load(targeted_path)
    assert targeted['passed'] and targeted['frozen_inputs_unchanged']
    combined = load(combined_result)
    assert combined['passed'], 'Actual combined consumer regression must finish'
    # The combined collector must bind the actual selected candidate, not only
    # a historical independent component or the validation-only 256 reference.
    combined_candidate = Path(combined['candidate']).resolve()
    fifo_dependencies = []
    if fifo_mapping is None:
        assert combined_candidate == CANDIDATE.resolve()
        assert same_bytes(CANDIDATE/'nav/rtl/common/nav_async_fifo.v', BASE/'stage2_SLF128_forward_safeFW_prepared8/nav/rtl/common/nav_async_fifo.v')
    else:
        if combined_candidate != CANDIDATE.resolve():
            assert combined_candidate == (BASE/'stage3_SLF128_storage_RTL_prepared5').resolve()
            old_inputs = hardware_inputs(combined_candidate, load(combined_candidate/'manifest.json'))
            delta = [n for n in b if not same_bytes(b[n], old_inputs[n])]
            expected_delta = {'nav/rtl/common/nav_async_fifo.v'}
            if lf_mapping is not None:
                expected_delta.add('nav/rtl/match/slf_lf.v')
            if weight_mapping is not None:
                expected_delta.add('fixed4500_weight_rom.v')
            assert set(delta) == expected_delta, delta
        old_fw = BASE/'stage3_SLF128_storage_RTL_prepared5/fw'
        for path in old_fw.rglob('*'):
            if path.is_file():
                assert same_bytes(path, CANDIDATE/'fw'/path.relative_to(old_fw))
        canonical = BASE/'stage3_FIFO_async_reset_syntax_candidate1/nav_async_fifo.v'
        assert b['nav/rtl/common/nav_async_fifo.v'].read_text(encoding='utf8') == canonical.read_text(encoding='utf8')
        finite = BASE/'stage3_FIFO_async_reset_syntax_RTL_probe1/result.json'
        finite_result = load(finite)
        assert finite_result['passed'] and finite_result['complete_source_transfer_expression_identity']
        assert load(fifo_mapping)['passed'], 'Actual same C small-netlist review must pass'
        fifo_dependencies += [canonical, finite, fifo_mapping]
    assert combined['eval_cycles_equal_original128']
    assert combined['RAM_cases'] == 9
    assert combined['original_five_frame_245_candidate_all_structure_clear_prior_checks']
    lf_dependencies = []
    assert (lf_mapping is None) == (lf_validation is None)
    if lf_mapping is not None:
        # Only the LF array-read schedule differs from the already admitted
        # canonical-FIFO source. The original full consumer test is reused for
        # unchanged logic; these two proofs cover the new actual LF source.
        previous = BASE/'stage3_SLF128_storage_RTL_prepared6'
        previous_inputs = hardware_inputs(previous, load(previous/'manifest.json'))
        delta = {n for n in b if not same_bytes(b[n], previous_inputs[n])}
        expected_delta = {'nav/rtl/match/slf_lf.v'}
        if weight_mapping is not None:
            expected_delta.add('fixed4500_weight_rom.v')
        assert delta == expected_delta, delta
        validation = load(lf_validation)
        assert validation['passed'] and Path(validation['source']).resolve() == previous.resolve()
        repaired = Path(validation['candidate'])/'slf_lf.v'
        assert same_bytes(b['nav/rtl/match/slf_lf.v'], repaired)
        profiles = {r['profile']: r for r in validation['profiles']}
        assert set(profiles) == {'old_new_miter', 'five_frame_model'}
        assert all(r['passed'] for r in profiles.values())
        assert any('full_maps=10 resets=5 natural_private_X=1' in s
                   for s in profiles['old_new_miter']['markers'])
        assert 'TEST PASSED LF128_ORIGINAL_5FRAME_MODEL' in profiles['five_frame_model']['markers']
        mapped = load(lf_mapping)
        assert mapped['passed'] and mapped['actual_device_version'] == 'C'
        assert mapped['tool_exit_code'] == 0 and mapped['row_RAM_count'] == 3
        assert mapped['data_concatenation_and_padding_checked']
        assert mapped['array_read_equivalent_not_rst_and_run']
        assert mapped['array_write_equivalent_not_rst_and_v1_and_not_c1bit8']
        assert mapped['BLKSEL_included_OCE_not_used_as_array_enable']
        assert mapped['all_queries_UNSAT'] and not mapped['physical_chunk_collision_events']
        assert mapped['actual_scan_run_edges'] == 16900 and mapped['trace_edges'] >= 16908
        assert all(r['array_write'] and not r['array_read'] for r in mapped['final_drain_write_edge'])
        assert mapped['all_frozen_build_inputs_unchanged']
        small_source = Path(mapped['build'])/'slf_lf.v'
        assert same_bytes(repaired, small_source)
        lf_dependencies += [lf_mapping, lf_validation, repaired, small_source,
                            Path(mapped['actual_VG']), Path(mapped['details'])]
    weight_dependencies = []
    assert len([x for x in (weight_mapping, weight_source_proof, weight_actual_proof) if x is not None]) in (0, 3)
    if weight_mapping is not None:
        assert lf_mapping is not None, 'This combined revision includes the closed LF repair'
        source_proof = load(weight_source_proof)
        actual_proof = load(weight_actual_proof)
        mapping = load(weight_mapping)
        assert source_proof['passed'] and actual_proof['passed'] and mapping['passed']
        weight_source = Path(source_proof['candidate'])/'fixed4500_weight_rom.v'
        assert same_bytes(b['fixed4500_weight_rom.v'], weight_source)
        assert any('all65536=1' in s and 'fourstate=640' in s for s in source_proof['markers'])
        assert mapping['synthesis_exit0'] and mapping['one_small_C_SYN_only']
        assert mapping['actual_ROMs'] == 35 and mapping['high_pROM8'] == 15 and mapping['low_pROMX9_9'] == 20
        for key in ('all35_payloads_unique_exact', 'high15_actual_CE_common_en',
                    'all_low_CE_false_when_en0', 'all_actual_ROM_address_bits_correct',
                    'all8_selector_and_flip_capture_functions_checked',
                    'selector_and_ROM_CE_hold_on_en0', 'public_output_only_ROM_and_FF_state',
                    'all_queries_UNSAT', 'all_frozen_inputs_unchanged',
                    'all_recorded_original_identities_exited'):
            assert mapping[key], key
        assert mapping['READ_MODE'] == 0 and mapping['physical_selector_correction_FFs'] == 8
        actual_module = Path(mapping['actual_module'])
        assert metadata(actual_module) == actual_proof['actual_module']
        assert metadata(actual_proof['official_library_unchanged']['path']) == actual_proof['official_library_unchanged']
        assert any('all40704=1' in s and 'idle_hold=81409' in s and 'official_unmodified_prim_sim=1' in s
                   for s in actual_proof['markers'])
        small_source = Path(mapping['actual_build'])/'fixed4500_weight_rom.v'
        assert same_bytes(weight_source, small_source)
        weight_dependencies += [weight_mapping, weight_source_proof, weight_actual_proof,
            weight_source, small_source, Path(mapping['actual_VG']), actual_module,
            Path(mapping['details']), Path(actual_proof['official_library_unchanged']['path'])]
    composition = load(CANDIDATE/'composition_review.json')
    budget = composition['source_budget']['totals']
    assert budget == dict(source=270, wide=12, physical=282)
    dependencies = [Path(__file__).resolve(), CANDIDATE/'firmware_binding_review.json', CANDIDATE/'composition_review.json',
                    compiled/'result.json', Path(release['strict_review']), targeted_path,
                    combined_result, BASE/'stage2_SLF128_forward_safeFW_prepared8/safety_rebase_review.json']
    dependencies += fifo_dependencies
    dependencies += lf_dependencies
    dependencies += weight_dependencies
    for suffix in ('source_contracts.json', 'manifest.json', 'run.tcl'):
        dependencies.append(CANDIDATE/suffix)
    paths = {row['path']: Path(row['path']) for row in firmware_binding['complete_frozen_input_bindings']}
    for path in dependencies:
        paths[str(path)] = path
    result = dict(passed=True, recorded_at=datetime.now().astimezone().isoformat(),
                  candidate=str(CANDIDATE), production_SLF_LOG2N=7, CPU_RAM_bytes=131072,
                  combined_RTL_contracts_passed=True, combined_result=str(combined_result),
                  new_ELF_BIN_128KiB_INIT_bound=True, firmware_BIN_bytes=len(image),
                  BSS_bytes=70004, static_stack_bytes=2576, reserve_stack_bytes=6144,
                  spare_after_reserved_stack_bytes=1348, changed_15_RTL=sorted(changed),
                  source_RTL_files=63, source_BSRAM_prediction=270, wide_split_prediction=12,
                  physical_BSRAM_prediction=282, resource_predictions_not_measured=True,
                  injection_remains_distributed=True, public_nav_fifo_ob_unchanged=True,
                  FIFO_reset_syntax_small_mapping_review=str(fifo_mapping) if fifo_mapping else None,
                  LF_run_read_small_mapping_review=str(lf_mapping) if lf_mapping else None,
                  LF_changed_source_RTL_validation=str(lf_validation) if lf_validation else None,
                  weight_high_common_en_small_mapping_review=str(weight_mapping) if weight_mapping else None,
                  weight_source_all65536_and_postmap_all40704_bound=bool(weight_mapping),
                  combined_regression_source=str(combined_candidate),
                  keep4500_all_weights125Hz_IR_OLED=True,
                  original_dualUART_wiring_preserved=True,
                  P11_39_control_command_chain_implemented=False,
                  joint_SYN_authorized=True, PNR_authorized=False,
                  bindings=[metadata(paths[k]) for k in sorted(paths)],
                  remaining=['actual new VG CEB/BLKSEL/INIT and all memory geometry',
                             'actual CPU image and new FIFO reset/26 CDC connections',
                             'physical <=300 BSRAM, LUT<45825 and FF<31120',
                             'two place options unrouted0 and all 50/100MHz timing/reset/IO',
                             'model multi-seed P95/no-loss and long patrol',
                             'P11-39 command input and safe ownership/expiry arbitration',
                             'new image actual Pico busy host-loss and physical stop'])
    save(output, result)
    return result


if __name__ == '__main__':
    import json
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--combined-result', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--candidate', type=Path, default=CANDIDATE)
    p.add_argument('--fifo-mapping', type=Path)
    p.add_argument('--lf-mapping', type=Path)
    p.add_argument('--lf-validation', type=Path)
    p.add_argument('--weight-mapping', type=Path)
    p.add_argument('--weight-source-proof', type=Path)
    p.add_argument('--weight-actual-proof', type=Path)
    a = p.parse_args()
    r = review(a.combined_result, a.out, a.candidate, a.fifo_mapping,
               a.lf_mapping, a.lf_validation, a.weight_mapping,
               a.weight_source_proof, a.weight_actual_proof)
    print(json.dumps(dict(passed=r['passed'], candidate=r['candidate'], bindings=len(r['bindings'])), ensure_ascii=False))
