"""One independent memory-gated SYN31, no active PNR exit dependency."""
from pathlib import Path
import run_action_preview_syn as runner

BASE = Path('F:/nav_integration_g2_20261023')
runner.PREP = BASE/'baseline_fusion_registered_rounding_prepared1'
runner.OUTPUT = BASE/'baseline_fusion_registered_rounding_syn31'
runner.CONTROL = BASE/'baseline_fusion_registered_rounding_one_shot'
runner.BUILD_LABEL = 'SYN31'
runner.REVIEW_SCRIPT = Path(__file__).with_name('review_registered_action_rounding_syn.py')

if __name__ == '__main__':
    raise SystemExit(runner.main())
