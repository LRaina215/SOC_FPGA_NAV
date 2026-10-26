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


def review(combined_result, output):
    assert not output.exists(), 'Keep prior review, never overwrite it'
    firmware_binding = load(CANDIDATE/'firmware_binding_review.json')
    for row in firmware_binding['complete_frozen_input_bindings']:
        assert metadata(row['path']) == row, row
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
    for rel in ('nav/rtl/common/nav_async_fifo.v', 'nav/rtl/net/rgmii_mac100.v', 'g2_top.v'):
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
    assert Path(combined['candidate']).resolve() == CANDIDATE.resolve()
    assert combined['eval_cycles_equal_original128']
    assert combined['RAM_cases'] == 9
    assert combined['original_five_frame_245_candidate_all_structure_clear_prior_checks']
    composition = load(CANDIDATE/'composition_review.json')
    budget = composition['source_budget']['totals']
    assert budget == dict(source=270, wide=12, physical=282)
    dependencies = [CANDIDATE/'firmware_binding_review.json', CANDIDATE/'composition_review.json',
                    compiled/'result.json', Path(release['strict_review']), targeted_path,
                    combined_result, BASE/'stage2_SLF128_forward_safeFW_prepared8/safety_rebase_review.json']
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
                  keep4500_all_weights125Hz_IR_OLED_dualUART=True,
                  joint_SYN_authorized=True, PNR_authorized=False,
                  bindings=[metadata(paths[k]) for k in sorted(paths)],
                  remaining=['actual new VG CEB/BLKSEL/INIT and all memory geometry',
                             'actual CPU image and new FIFO reset/26 CDC connections',
                             'physical <=300 BSRAM, LUT<45825 and FF<31120',
                             'two place options unrouted0 and all 50/100MHz timing/reset/IO',
                             'model multi-seed P95/no-loss and long patrol',
                             'new image actual Pico busy host-loss and physical stop'])
    save(output, result)
    return result


if __name__ == '__main__':
    import json
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--combined-result', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    r = review(a.combined_result, a.out)
    print(json.dumps(dict(passed=r['passed'], candidate=r['candidate'], bindings=len(r['bindings'])), ensure_ascii=False))
