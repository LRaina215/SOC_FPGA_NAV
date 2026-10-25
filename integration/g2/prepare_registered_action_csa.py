"""Single arithmetic implementation contrast against the failed SYN28 product.

Keep the already proved private publication register edge, signed64 domain,
rounding, outputs and scheduler. Replace only its constant product expressions
with exact carry-save constant sums; no global requests, UART or reset changes.
"""
import argparse
from datetime import datetime
import difflib
import json
from pathlib import Path
import re
import psutil
import prepare_action_preview_product as preview
from prepare_action_constant_csa import SCALE
from prepare_baseline_fusion_retime import metadata,save,same_bytes
import prepare_baseline_action_preview as joint

BASE=Path('F:/nav_integration_g2_20261023')
SYN28=BASE/'baseline_fusion_action_preview_syn28'
FORMER=BASE/'action_constant_csa_candidate1'
ACTION='control/policy_action_map.v'

def prepare(out,prepared):
    old_proof=json.loads((FORMER/'result.json').read_text());assert old_proof['passed']
    assert old_proof['original64bit_product_and_rounding_preserved']
    assert old_proof['exhaustive_scale_values_checked']==32772
    for row in old_proof['source_bindings']:assert metadata(Path(row['path']))==row
    # Derive from the independently routed source, then verify it equals the
    # exact former SYN28 module before this single new expression change.
    p=preview.prepare(out)
    assert all(same_bytes(SYN28/n,out/n)for n in p['changed_RTL'])
    text=(out/ACTION).read_text();original=text
    function=SCALE.replace('function signed [31:0] scale;','function signed [63:0] constant_product;',1)
    final='            scale=product<0 ? -((-product+2048)>>>12) : ((product+2048)>>>12);'
    assert function.count(final)==1
    function=function.replace(final,'            constant_product=product;',1)+'\n'
    # Require the body of the proven exact CSA, excluding only its unchanged
    # rounding, to be reproduced literally. There is no narrower arithmetic.
    assert function.replace('function signed [63:0] constant_product;','function signed [31:0] scale;',1).replace('            constant_product=product;',final,1).rstrip()==SCALE
    marker='    reg signed [63:0] product_lh,product_lk,product_lw;'
    assert text.count(marker)==1;text=text.replace(marker,function+marker,1)
    pairs=[('0','19'),('24','(-19)'),('48','(-600)'),('72','19'),('96','19'),('120','600')]
    for index,gain in pairs:
        old=f'$signed(clip(preview_actions[{index}+:24]))*{gain}'
        assert text.count(old)==1
        new=f'constant_product(preview_actions[{index}+:24],{gain})'
        text=text.replace(old,new,1)
    inverse=text.replace(function,'',1)
    for index,gain in pairs:inverse=inverse.replace(f'constant_product(preview_actions[{index}+:24],{gain})',f'$signed(clip(preview_actions[{index}+:24]))*{gain}',1)
    assert inverse==original
    (out/ACTION).write_text(text,newline='\n')
    (out/'registered_constant_CSA.patch').write_text(''.join(difflib.unified_diff(original.splitlines(True),text.splitlines(True),fromfile='SYN28/'+ACTION,tofile='registered_CSA/'+ACTION)))
    p.update(recorded_at=datetime.now().astimezone().isoformat(),candidate_bindings=[metadata(out/n)for n in p['changed_RTL']],
        source_SINGLE_delta_vs_SYN28=ACTION,private_register_capture_edges_and_64bit_products_unchanged=True,
        exact_CSA_function_bound_to_prior_proof=str(FORMER/'result.json'),not_unregistered_PNR37_input=True,
        no_fusion_square_UART_reset_or_global_allocation_change=True,actual_mapping_and_timing_pending=True)
    save(out/'prepared_review.json',p)
    preview.simulate(out);preview.simulate_policy(out)
    joint.CANDIDATE=out
    result=joint.prepare(prepared)
    manifest=json.loads((prepared/'manifest.json').read_text())
    manifest.update(preview_constant_CSA=True,action_map_constant_CSA=False,
                    registered_product_arithmetic_only_vs_SYN28=True,automatic_pnr=False)
    save(prepared/'manifest.json',manifest)
    result.update(registered_product_arithmetic_only_vs_SYN28=True,private_product_64bit_and_original_edges=True,
                  expected_mapping_change='replace six registered action DSP products with ordinary registered exact constant sums; must be measured',
                  actual_joint_DSP_FF_LUT_timing_not_measured=True,proof_of_single_delta=str(out/'prepared_review.json'))
    save(prepared/'prepared_source_review.json',result)
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--prepared',type=Path,required=True);a=p.parse_args()
    me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    r=prepare(a.out,a.prepared);print(json.dumps(dict(passed=r['passed'],prepared=str(a.prepared),modified_rtl_inputs=r['modified_rtl_inputs'])))
