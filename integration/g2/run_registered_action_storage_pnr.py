"""An unused unique PNR42 entry, requiring the actual completed SYN30 review."""
from pathlib import Path
import run_action_preview_pnr as run
from review_registered_action_storage_syn import bind

BASE = Path('F:/nav_integration_g2_20261023')
run.SOURCE = BASE/'baseline_fusion_registered_CSA_storage_syn30'
run.PREPARED = BASE/'baseline_fusion_registered_CSA_storage_prepared1'
run.REVIEW = BASE/'baseline_fusion_registered_CSA_storage_one_shot/actual_SYN30_review.json'
run.MAPPING = run.REVIEW
run.OUTPUT = BASE/'baseline_fusion_registered_CSA_storage_pnr42'
run.CONTROL = BASE/'baseline_fusion_registered_CSA_storage_pnr_one_shot'
run.PNR_LABEL = 'PNR42'
run.SYN_LABEL = 'SYN30'
run.CHANGE_CONCEPT = 'Exact private product storage only vs SYN29, retaining 64-bit CSA arithmetic and original publication/output edges; no new clock/global/timing requests'
run.PROFILE = dict(source=run.SOURCE, prepared=run.PREPARED, output=run.OUTPUT, interval=13)
run.bind = bind


def mapping_check(mapped):
    assert mapped['passed']
    actual = mapped['registered_constant_CSA_actual_mapping']
    assert actual['actual_action_DSP_cells'] == {}
    assert actual['actual_product_FF_count'] > 0 and len(actual['actual_product_groups']) == 6
    assert actual['private_products_clocked_on_original_core_and_come_from_same_core_actions']
    assert mapped['private_storage_contrast']['arithmetic_and_output_edges_preserved']
    return mapped['reviewed_input_metadata']


run.mapping_check = mapping_check
if __name__ == '__main__':
    raise SystemExit(run.main())
