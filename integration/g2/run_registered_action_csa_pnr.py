"""PNR41 only after actual SYN29 RAM/INIT/clocks/26CDC/product review passes."""
from pathlib import Path
import run_action_preview_pnr as run
from review_registered_action_csa_syn import bind

BASE=Path('F:/nav_integration_g2_20261023')
run.SOURCE=BASE/'baseline_fusion_registered_CSA_syn29'
run.PREPARED=BASE/'baseline_fusion_registered_CSA_prepared1'
run.REVIEW=BASE/'baseline_fusion_registered_CSA_one_shot/actual_SYN29_review_retry1.json'
run.MAPPING=run.REVIEW
run.OUTPUT=BASE/'baseline_fusion_registered_CSA_pnr41'
run.CONTROL=BASE/'baseline_fusion_registered_CSA_pnr_one_shot'
run.PNR_LABEL='PNR41'
run.SYN_LABEL='SYN29'
run.CHANGE_CONCEPT='Registered exact constant CSA products on the same already-proved private publication edge; one arithmetic delta vs SYN28, no global request change'
run.PROFILE=dict(source=run.SOURCE,prepared=run.PREPARED,output=run.OUTPUT,interval=13)
run.bind=bind

def mapping_check(mapped):
    assert mapped['passed']
    actual=mapped['registered_constant_CSA_actual_mapping']
    assert actual['actual_action_DSP_cells']=={}
    assert actual['actual_product_FF_count']>0 and len(actual['actual_product_groups'])==6
    assert actual['private_products_clocked_on_original_core_and_come_from_same_core_actions']
    return mapped['reviewed_input_metadata']

run.mapping_check=mapping_check
if __name__=='__main__':raise SystemExit(run.main())
