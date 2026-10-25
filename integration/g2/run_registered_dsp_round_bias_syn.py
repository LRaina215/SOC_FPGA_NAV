"""One independent memory-gated SYN32; no current PNR exit dependency."""
from pathlib import Path
import run_action_preview_syn as runner

BASE=Path('F:/nav_integration_g2_20261023')
runner.PREP=BASE/'baseline_fusion_DSP_round_bias_prepared1'
runner.OUTPUT=BASE/'baseline_fusion_DSP_round_bias_syn32'
runner.CONTROL=BASE/'baseline_fusion_DSP_round_bias_one_shot'
runner.BUILD_LABEL='SYN32'
runner.REVIEW_SCRIPT=Path(__file__).with_name('review_registered_dsp_round_bias_syn.py')

if __name__=='__main__':
    raise SystemExit(runner.main())
