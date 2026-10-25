"""One PNR44 from completed SYN32 and the actual six-DSP mode proof."""
from pathlib import Path
import run_action_preview_pnr as run
from review_registered_dsp_round_bias_syn import bind

BASE = Path('F:/nav_integration_g2_20261023')
run.SOURCE = BASE/'baseline_fusion_DSP_round_bias_syn32'
run.PREPARED = BASE/'baseline_fusion_DSP_round_bias_prepared1'
run.REVIEW = BASE/'baseline_fusion_DSP_round_bias_one_shot/actual_SYN32_review.json'
run.MAPPING = BASE/'baseline_fusion_DSP_round_bias_one_shot/root_actual_DSP_mode_review.json'
run.OUTPUT = BASE/'baseline_fusion_DSP_round_bias_pnr44'
run.CONTROL = BASE/'baseline_fusion_DSP_round_bias_pnr_one_shot'
run.PNR_LABEL = 'PNR44'
run.SYN_LABEL = 'SYN32'
run.CHANGE_CONCEPT = 'Exact sign-dependent rounding bias absorbed into the original six private DSP output registers; preserve E2/E3 and all external behavior'
run.PROFILE = dict(source=run.SOURCE, prepared=run.PREPARED, output=run.OUTPUT, interval=13)
run.bind = bind


def mapping_check(mapped):
    assert mapped['passed'] and mapped['actual_signed_multiply_add_bias_mode_proved']
    assert mapped['original_E2_capture_and_E3_output_contract_preserved']
    assert mapped['separate_product_FF_count'] == mapped['new_global_requests'] == 0
    assert len(mapped['checks']) == 18 and len(mapped['DSPs']) == 6
    assert mapped['all24bit_action_states_per_group'] == 1 << 24
    assert Path(mapped['full_review']).resolve() == run.REVIEW.resolve()
    return mapped['evidence_bindings']


run.mapping_check = mapping_check
if __name__ == '__main__':
    raise SystemExit(run.main())
