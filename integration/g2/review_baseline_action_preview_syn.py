"""Reuse full actual RAM/INIT/clock/CDC review with a distinct three-RTL contract.

No numeric alias or actual physical/global allocation is accepted by this
source contract. Product mapping and real timing must still be examined.
"""
from pathlib import Path
import json
import review_baseline_fusion_action_width_syn as engine

BASE=Path('F:/nav_integration_g2_20261023')
CANDIDATE=BASE/'action_preview_product_candidate1'
CHANGED=sorted(['control/policy_action_map.v','control/rl_policy_tick_v5.v','control/mem_policy.v'])

def bind(source,prepared,proof):
    assert proof['passed']and sorted(proof['modified_rtl_inputs'])==CHANGED
    for row in proof['bindings']:assert engine.metadata(Path(row['path']))==row
    for name in CHANGED:
        assert engine.same(CANDIDATE/name,prepared/name)and engine.same(prepared/name,source/name)
    for name in('prepared_review.json','arithmetic_result.json','policy_result.json'):
        evidence=engine.load(CANDIDATE/name)
        assert evidence.get('passed',evidence.get('prepared'))
    for root in(prepared,source):
        manifest=engine.load(root/'manifest.json');assert manifest['action_preview_product']
        assert manifest['fusion_norm_pipeline']
        assert not any(manifest.get(k)for k in('action_map_constant_CSA','action_map_exact_widths','fusion_square_pipeline2','pose_predecode','control_weights_changed','programmed'))
    a=(source/'control/rl_policy_tick_v5.v').read_text()
    assert a.count('assign preview_actions={a5,a4,a3,a2,a1,a0};')==1
    old=(engine.ORIGINAL/'control/rl_policy_tick_v5.v').read_text()
    a=a.replace('    output wire [143:0] preview_actions,\n','',1).replace('    assign preview_actions={a5,a4,a3,a2,a1,a0};\n','',1)
    assert a==old,'original scheduler logic changed outside private wire'

engine.EXPECTED_MODIFIED_RTL=CHANGED
engine.bind_action_contract=bind
if __name__=='__main__':raise SystemExit(engine.main())
