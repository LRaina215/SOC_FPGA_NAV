"""Unused PNR43 entry; an independent completed SYN31 review is required."""
from pathlib import Path
import run_action_preview_pnr as run
from review_registered_action_rounding_syn import bind

BASE = Path('F:/nav_integration_g2_20261023')
run.SOURCE = BASE/'baseline_fusion_registered_rounding_syn31'
run.PREPARED = BASE/'baseline_fusion_registered_rounding_prepared1'
run.REVIEW = BASE/'baseline_fusion_registered_rounding_one_shot/actual_SYN31_review.json'
run.MAPPING = run.REVIEW
run.OUTPUT = BASE/'baseline_fusion_registered_rounding_pnr43'
run.CONTROL = BASE/'baseline_fusion_registered_rounding_pnr_one_shot'
run.PNR_LABEL = 'PNR43'
run.SYN_LABEL = 'SYN31'
run.CHANGE_CONCEPT = 'Move the original exact rounding before the same private E2 register; preserve E3 latest feed-forward/output/stop; no new global requests or clock/CDC exceptions'
run.PROFILE = dict(source=run.SOURCE, prepared=run.PREPARED, output=run.OUTPUT, interval=13)
run.bind = bind


def mapping_check(mapped):
    assert mapped['passed']
    actual = mapped['registered_constant_CSA_actual_mapping']
    assert actual['actual_action_DSP_cells'] == {}
    assert actual['actual_product_FF_count'] > 0 and len(actual['actual_product_groups']) == 6
    assert actual['private_products_clocked_on_original_core_and_come_from_same_core_actions']
    assert mapped['private_rounded_capture_actual_mapping']['original_external_cycle_and_latest_feedforward_preserved']
    return mapped['reviewed_input_metadata']


run.mapping_check = mapping_check
if __name__ == '__main__':
    raise SystemExit(run.main())
