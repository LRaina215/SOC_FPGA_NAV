"""One independently gated full SYN29, followed by its strict actual review."""
from pathlib import Path
import run_action_preview_syn as runner

BASE=Path('F:/nav_integration_g2_20261023')
runner.PREP=BASE/'baseline_fusion_registered_CSA_prepared1'
runner.OUTPUT=BASE/'baseline_fusion_registered_CSA_syn29'
runner.CONTROL=BASE/'baseline_fusion_registered_CSA_one_shot'
runner.BUILD_LABEL='SYN29'
runner.REVIEW_SCRIPT=Path(__file__).with_name('review_registered_action_csa_syn.py')

if __name__=='__main__':raise SystemExit(runner.main())
