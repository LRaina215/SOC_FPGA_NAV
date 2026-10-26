"""One independent memory-gated SYN33; no current PNR exit dependency."""
from pathlib import Path
import run_action_preview_syn as runner

BASE=Path('F:/nav_integration_g2_20261023')
runner.PREP=BASE/'baseline_fusion_DSP_AREG_prepared1'
runner.OUTPUT=BASE/'baseline_fusion_DSP_AREG_syn33'
runner.CONTROL=BASE/'baseline_fusion_DSP_AREG_one_shot'
runner.BUILD_LABEL='SYN33'
runner.REVIEW_SCRIPT=Path(__file__).with_name('review_action_dsp_input_register_syn.py')

if __name__=='__main__':
    raise SystemExit(runner.main())
