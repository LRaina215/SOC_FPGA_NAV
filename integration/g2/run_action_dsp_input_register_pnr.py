"""One PNR47 after complete SYN33 and the independent actual AREG proof."""
from pathlib import Path
import run_action_preview_pnr as run
from review_action_dsp_input_register_syn import bind

BASE = Path('F:/nav_integration_g2_20261023')
run.SOURCE = BASE/'baseline_fusion_DSP_AREG_syn33'
run.PREPARED = BASE/'baseline_fusion_DSP_AREG_prepared1'
run.REVIEW = BASE/'baseline_fusion_DSP_AREG_one_shot/actual_SYN33_review.json'
run.MAPPING = BASE/'baseline_fusion_DSP_AREG_one_shot/root_actual_DSP_AREG_mode_review.json'
run.OUTPUT = BASE/'baseline_fusion_DSP_AREG_pnr47'
run.CONTROL = BASE/'baseline_fusion_DSP_AREG_pnr_one_shot'
run.PNR_LABEL = 'PNR47'
run.SYN_LABEL = 'SYN33'
run.CHANGE_CONCEPT = 'Private E2 capture at six hard DSP A inputs, original E3 rounding/latest feed-forward/stop; use routed35 split1 and no new global requests'
run.PROFILE = dict(source=run.SOURCE, prepared=run.PREPARED, output=run.OUTPUT, interval=13)
run.bind = bind


def mapping_check(mapped):
    assert mapped['passed'] and mapped['actual_signed_multiply_AREG_mode_proved']
    assert mapped['original_E2_input_capture_and_E3_output_contract_preserved']
    assert mapped['separate_product_FF_count'] == mapped['new_global_requests'] == 0
    assert len(mapped['checks']) == 18 and len(mapped['DSPs']) == 6
    assert mapped['all24bit_action_states_per_group'] == 1 << 24
    assert Path(mapped['full_review']).resolve() == run.REVIEW.resolve()
    return mapped['evidence_bindings']


run.mapping_check = mapping_check
if __name__ == '__main__':
    raise SystemExit(run.main())
