"""Exact rounding bias before the original private DSP output capture.

For integral p, symmetric nearest/away-at-ties rounding equals
(p + 2048 - (p < 0)) arithmetic-shifted by12. Compute the sign from the
clipped input and constant gain so no multiplier-output feedback is needed.
The intended multiplier-plus-bias DSP mapping must be measured, not assumed.
"""
import argparse
from datetime import datetime
import difflib
import json
from pathlib import Path
import psutil
import prepare_action_preview_product as preview
import prepare_baseline_action_preview as joint
from prepare_action_constant_csa import rounded
from prepare_baseline_fusion_retime import metadata, same_bytes, save

BASE = Path('F:/nav_integration_g2_20261023')
SOURCE = BASE/'baseline_fusion_action_preview_syn28'
ACTION = 'control/policy_action_map.v'
FUNCTION = '''    // Keep signed64 arithmetic. Offset selects nearest rounding away at
    // ties, including negative values. Its sign comes from the stable input,
    // not from feedback around the multiply/accumulate output.
    function signed [63:0] biased_product;
        input signed [23:0] a;
        input integer gain;
        reg signed [63:0] bounded64,offset64;
        begin
            bounded64=$signed(clip(a));
            offset64=(((bounded64<0)^(gain<0)) && bounded64!=0)
                     ? 64'sd2047 : 64'sd2048;
            biased_product=bounded64*gain+offset64;
        end
    endfunction
'''
OLD_ROUND = 'rounded_product=product<0 ? -((-product+2048)>>>12) : ((product+2048)>>>12);'
NEW_ROUND = 'rounded_product=product>>>12; // private product already contains the exact sign-dependent bias'
CALLS = [('0', '19'), ('24', '(-19)'), ('48', '(-600)'),
         ('72', '19'), ('96', '19'), ('120', '600')]


def transform(original):
    marker = '    reg signed [63:0] product_lh,product_lk,product_lw;'
    assert original.count(marker) == original.count(OLD_ROUND) == 1
    text = original.replace(marker, FUNCTION+marker, 1).replace(OLD_ROUND, NEW_ROUND, 1)
    for index, gain in CALLS:
        old = f'$signed(clip(preview_actions[{index}+:24]))*{gain}'
        assert text.count(old) == 1
        text = text.replace(old, f'biased_product(preview_actions[{index}+:24],{gain})', 1)
    inverse = text.replace(FUNCTION, '', 1).replace(NEW_ROUND, OLD_ROUND, 1)
    for index, gain in CALLS:
        inverse = inverse.replace(f'biased_product(preview_actions[{index}+:24],{gain})',
                                  f'$signed(clip(preview_actions[{index}+:24]))*{gain}', 1)
    assert inverse == original
    return text


def math_proof():
    rows = []
    for gain in (19, -19, 600, -600):
        for clipped in range(-4096, 4097):
            p = clipped*gain
            sign_from_input = ((clipped < 0) ^ (gain < 0)) and clipped != 0
            assert sign_from_input == (p < 0)
            biased = p+(2047 if sign_from_input else 2048)
            assert -(1 << 63) <= biased < (1 << 63)
            assert (biased >> 12) == rounded(p)
        rows.append(dict(gain=gain, values=8193, minimum_product=-4096*abs(gain), maximum_product=4096*abs(gain)))
    return dict(passed=True, combinations=32772, cases=rows,
                theorem='Euclidean quotient/remainder of p by4096: adding2048 implements positive half-up; subtracting1 when p<0 gives the original symmetric away-at-ties result. The unchanged clip exhausts exactly8193 integer values.',
                signed64_no_overflow=True, sign_derived_without_DSP_output_feedback=True,
                quantization_or_external_cycle_change=False)


def prepare(out, prepared):
    assert not out.exists() and not prepared.exists()
    proof = preview.prepare(out)
    assert all(same_bytes(out/name, SOURCE/name) for name in proof['changed_RTL'])
    old = (out/ACTION).read_text()
    new = transform(old)
    (out/ACTION).write_text(new, newline='\n')
    math = math_proof()
    math.update(source_binding=metadata(SOURCE/ACTION), candidate_binding=metadata(out/ACTION))
    save(out/'math_proof.json', math)
    (out/'dsp_round_bias.patch').write_text(''.join(difflib.unified_diff(old.splitlines(True), new.splitlines(True), fromfile='SYN28/'+ACTION, tofile='DSP_round_bias/'+ACTION)))
    proof.update(recorded_at=datetime.now().astimezone().isoformat(),
                 candidate_bindings=[metadata(out/n) for n in proof['changed_RTL']],
                 source_single_delta_vs_SYN28=ACTION, exact_private_round_bias_capture=True,
                 original_signed64_product_domain_and_E2_E3_edges_preserved=True,
                 no_new_global_requests_or_timing_exceptions=True,
                 intended_DSP_multiply_plus_bias_output_register_not_yet_verified=True,
                 math_proof=str(out/'math_proof.json'))
    save(out/'prepared_review.json', proof)
    preview.simulate(out)
    preview.simulate_policy(out)
    joint.CANDIDATE = out
    result = joint.prepare(prepared)
    manifest = json.loads((prepared/'manifest.json').read_text())
    manifest.update(private_DSP_round_bias_capture=True,
                    round_bias_only_vs_SYN28=True, automatic_pnr=False)
    save(prepared/'manifest.json', manifest)
    result.update(round_bias_only_vs_SYN28=True, math_proof=str(out/'math_proof.json'),
                  actual_joint_DSP_mapping_or_timing_passed=False)
    save(prepared/'prepared_source_review.json', result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--prepared', type=Path, required=True)
    args = parser.parse_args()
    psutil.Process().nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    result = prepare(args.out, args.prepared)
    print(json.dumps(dict(passed=result['passed'], prepared=str(args.prepared), exact_values=32772, intended_mapping_unmeasured=True)))
