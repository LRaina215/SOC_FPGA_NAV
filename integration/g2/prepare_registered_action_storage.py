"""A storage-only contrast against SYN29, retaining its 64-bit CSA arithmetic.

The clipped signed product fits exactly in 18 bits for +/-19 and 23 bits for
+/-600. Only the private publication register is narrowed. Sign-extension
restores the original 64-bit rounding input; no arithmetic/output edge changes.
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
from prepare_baseline_fusion_retime import metadata, same_bytes, save

BASE = Path('F:/nav_integration_g2_20261023')
SOURCE = BASE/'baseline_fusion_registered_CSA_syn29'
CANDIDATE29 = BASE/'action_registered_CSA_candidate1'
ACTION = 'control/policy_action_map.v'
WIDTHS = dict(lh=18, lk=18, lw=23, rh=18, rk=18, rw=23)
GAINS = dict(lh=19, lk=-19, lw=-600, rh=19, rk=19, rw=600)
OLD_DECL = ('    reg signed [63:0] product_lh,product_lk,product_lw;\n'
            '    reg signed [63:0] product_rh,product_rk,product_rw;')
NEW_DECL = '\n'.join(f'    reg signed [{w-1}:0] product_{g};' for g, w in WIDTHS.items())


def transform(text):
    assert text.count(OLD_DECL) == 1
    result = text.replace(OLD_DECL, NEW_DECL, 1)
    for group, width in WIDTHS.items():
        old = f'rounded_product(product_{group})'
        new = f'rounded_product({{{{{64-width}{{product_{group}[{width-1}]}}}},product_{group}}})'
        assert result.count(old) == 1
        result = result.replace(old, new, 1)
    inverse = result.replace(NEW_DECL, OLD_DECL, 1)
    for group, width in WIDTHS.items():
        new = f'rounded_product({{{{{64-width}{{product_{group}[{width-1}]}}}},product_{group}}})'
        inverse = inverse.replace(new, f'rounded_product(product_{group})', 1)
    assert inverse == text
    return result


def range_proof():
    rows = []
    for gain, width in [(19, 18), (-19, 18), (600, 23), (-600, 23)]:
        products = [value*gain for value in range(-4096, 4097)]
        for product in products:
            low = product & ((1 << width)-1)
            restored = low-(1 << width) if low & (1 << (width-1)) else low
            assert restored == product
        assert min(products) >= -(1 << (width-1))
        assert max(products) < (1 << (width-1))
        rows.append(dict(gain=gain, bits=width, minimum=min(products), maximum=max(products), checked_values=len(products)))
    return dict(passed=True, proof='For every signed24 input the unchanged clip returns a signed integer in [-4096,4096]. Exhaust all 8193 results for each call-site gain; truncation followed by sign extension is identity.',
                combinations=32772, ranges=rows, rounding_64bit_function_unchanged=True,
                precision_loss_or_saturation_change=False)


def prepare(out, prepared):
    assert not out.exists() and not prepared.exists()
    old = json.loads((CANDIDATE29/'prepared_review.json').read_text())
    assert old['source_SINGLE_delta_vs_SYN28'] == ACTION
    for row in old['source_bindings']+old['candidate_bindings']:
        assert metadata(Path(row['path'])) == row
    for name in old['changed_RTL']:
        assert same_bytes(CANDIDATE29/name, SOURCE/name)
    # Generate the original independently routed-source comparison benches,
    # then adopt exactly the already measured SYN29 arithmetic and scheduler.
    proof = preview.prepare(out)
    for name in proof['changed_RTL']:
        shutil.copyfile(SOURCE/name, out/name)
    original = (out/ACTION).read_text()
    changed = transform(original)
    (out/ACTION).write_text(changed, newline='\n')
    assert original.split('    function signed [63:0] constant_product;', 1)[1].split(OLD_DECL, 1)[0] == changed.split('    function signed [63:0] constant_product;', 1)[1].split(NEW_DECL, 1)[0]
    rp = range_proof()
    rp.update(recorded_at=datetime.now().astimezone().isoformat(), source_binding=metadata(SOURCE/ACTION), candidate_binding=metadata(out/ACTION))
    save(out/'range_proof.json', rp)
    (out/'storage_only.patch').write_text(''.join(difflib.unified_diff(original.splitlines(True), changed.splitlines(True), fromfile='SYN29/'+ACTION, tofile='storage_only/'+ACTION)))
    proof.update(recorded_at=datetime.now().astimezone().isoformat(), candidate_bindings=[metadata(out/n) for n in proof['changed_RTL']],
                 source_single_delta_vs_SYN29=ACTION, private_product_bits_before_mapping=sum(WIDTHS.values()),
                 product_storage_widths=WIDTHS, constant_CSA_combinational_function_64bit_unchanged=True,
                 exact_sign_extension_before_original_rounding=True, original_capture_and_publication_edges_unchanged=True,
                 range_proof=str(out/'range_proof.json'), actual_mapping_and_timing_pending=True)
    save(out/'prepared_review.json', proof)
    preview.simulate(out)
    preview.simulate_policy(out)
    joint.CANDIDATE = out
    result = joint.prepare(prepared)
    manifest = json.loads((prepared/'manifest.json').read_text())
    manifest.update(preview_constant_CSA=True, private_product_storage_exact=True,
                    registered_product_storage_only_vs_SYN29=True, automatic_pnr=False)
    save(prepared/'manifest.json', manifest)
    result.update(registered_product_storage_only_vs_SYN29=True, range_proof=str(out/'range_proof.json'),
                  private_product_bits_before_mapping=sum(WIDTHS.values()), actual_joint_mapping_or_timing_passed=False)
    save(prepared/'prepared_source_review.json', result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--prepared', type=Path, required=True)
    args = parser.parse_args()
    psutil.Process().nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    result = prepare(args.out, args.prepared)
    print(json.dumps(dict(passed=result['passed'], prepared=str(args.prepared), arithmetic_range_values=32772, private_storage_bits=118)))
