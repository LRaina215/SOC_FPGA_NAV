"""Move exact rounding before the existing private capture edge, not an output.

This is an independent boundary contrast against completed SYN30. The original
64-bit CSA and rounding functions stay literal. E2 captures the exact rounded
value, E3 still applies latest feed-forward, limits, reset and stop priority.
The earlier data path may become slower: actual timing must be measured.
"""
import argparse
from datetime import datetime
import difflib
import json
from pathlib import Path
import shutil
import psutil
import prepare_action_preview_product as preview
import prepare_baseline_action_preview as joint
import prepare_registered_action_storage as storage
from prepare_action_constant_csa import rounded
from prepare_baseline_fusion_retime import metadata, same_bytes, save

BASE = storage.BASE
SOURCE = BASE/'baseline_fusion_registered_CSA_storage_syn30'
ACTION = storage.ACTION
WIDTHS = dict(lh=6, lk=6, lw=11, rh=6, rk=6, rw=11)
CALLS = dict(lh=('0', '19'), lk=('24', '(-19)'), lw=('48', '(-600)'),
             rh=('72', '19'), rk=('96', '19'), rw=('120', '600'))


def transform(original):
    pairs = []
    for group, width in WIDTHS.items():
        old_width = storage.WIDTHS[group]
        pairs.append((f'reg signed [{old_width-1}:0] product_{group};',
                      f'reg signed [{width-1}:0] product_{group};'))
        index, gain = CALLS[group]
        old = f'product_{group} <= constant_product(preview_actions[{index}+:24],{gain});'
        new = f'product_{group} <= rounded_product(constant_product(preview_actions[{index}+:24],{gain}));'
        pairs.append((old, new))
        old = f'rounded_product({{{{{64-old_width}{{product_{group}[{old_width-1}]}}}},product_{group}}})'
        pairs.append((old, f'$signed(product_{group})'))
    text = original
    for old, new in pairs:
        assert text.count(old) == 1, old
        text = text.replace(old, new, 1)
    inverse = text
    for old, new in reversed(pairs):
        assert inverse.count(new) == 1, new
        inverse = inverse.replace(new, old, 1)
    assert inverse == original
    return text


def range_proof():
    rows = []
    for gain, width in ((19, 6), (-19, 6), (600, 11), (-600, 11)):
        values = [rounded(value*gain) for value in range(-4096, 4097)]
        for value in values:
            low = value & ((1 << width)-1)
            restored = low-(1 << width) if low & (1 << (width-1)) else low
            assert value == restored
        rows.append(dict(gain=gain, bits=width, minimum=min(values), maximum=max(values), values=len(values)))
    return dict(passed=True, combinations=32772, ranges=rows,
                proof='The unchanged clip covers exactly [-4096,4096]; the literal original signed64 CSA and rounding are pure. Capturing their composed value at the same E2 edge gives the original rounding of the captured product at E3. All 8193 clipped values for each gain fit losslessly in the indicated signed width.',
                precision_or_external_output_cycle_change=False,
                all_private_values_must_be_written_before_original_valid_consumption=True)


def prepare(out, prepared):
    assert not out.exists() and not prepared.exists()
    review = json.loads((BASE/'baseline_fusion_registered_CSA_storage_one_shot/actual_SYN30_review.json').read_text())
    assert review['passed'] and review['private_storage_contrast']['actual_product_FF'] == 112
    for row in review['reviewed_input_metadata']:
        assert metadata(Path(row['path'])) == row
    proof = preview.prepare(out)
    for name in proof['changed_RTL']:
        shutil.copyfile(SOURCE/name, out/name)
    old = (out/ACTION).read_text()
    new = transform(old)
    (out/ACTION).write_text(new, newline='\n')
    rp = range_proof()
    rp.update(source_binding=metadata(SOURCE/ACTION), candidate_binding=metadata(out/ACTION))
    save(out/'range_proof.json', rp)
    (out/'rounding_boundary.patch').write_text(''.join(difflib.unified_diff(old.splitlines(True), new.splitlines(True), fromfile='SYN30/'+ACTION, tofile='private_rounded/'+ACTION)))
    proof.update(recorded_at=datetime.now().astimezone().isoformat(),
                 source_single_delta_vs_SYN30=ACTION,
                 candidate_bindings=[metadata(out/n) for n in proof['changed_RTL']],
                 private_product_bits_before_mapping=sum(WIDTHS.values()), private_rounded_product_widths=WIDTHS,
                 original64bit_CSA_and_rounding_functions_unchanged=True,
                 private_capture_edge_and_external_output_edges_unchanged=True,
                 private_registered_quantity='exact rounded product, not the unrounded product',
                 pre_capture_path_may_be_slower_and_requires_actual_timing=True,
                 range_proof=str(out/'range_proof.json'), actual_mapping_and_timing_pending=True)
    save(out/'prepared_review.json', proof)
    preview.simulate(out)
    preview.simulate_policy(out)
    joint.CANDIDATE = out
    result = joint.prepare(prepared)
    manifest = json.loads((prepared/'manifest.json').read_text())
    manifest.update(preview_constant_CSA=True, private_rounded_product_capture=True,
                    private_rounding_relocation_only_vs_SYN30=True, automatic_pnr=False)
    save(prepared/'manifest.json', manifest)
    result.update(private_rounding_relocation_only_vs_SYN30=True,
                  range_proof=str(out/'range_proof.json'),
                  actual_joint_mapping_or_timing_passed=False)
    save(prepared/'prepared_source_review.json', result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--prepared', type=Path, required=True)
    args = parser.parse_args()
    psutil.Process().nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    result = prepare(args.out, args.prepared)
    print(json.dumps(dict(passed=result['passed'], prepared=str(args.prepared), rounded_source_bits=46, actual_benefit_not_measured=True)))
