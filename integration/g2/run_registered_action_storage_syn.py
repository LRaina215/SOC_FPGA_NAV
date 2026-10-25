"""One independent memory-gated SYN30, followed by actual netlist review."""
from pathlib import Path
import run_action_preview_syn as runner

BASE = Path('F:/nav_integration_g2_20261023')
runner.PREP = BASE/'baseline_fusion_registered_CSA_storage_prepared1'
runner.OUTPUT = BASE/'baseline_fusion_registered_CSA_storage_syn30'
runner.CONTROL = BASE/'baseline_fusion_registered_CSA_storage_one_shot'
runner.BUILD_LABEL = 'SYN30'
runner.REVIEW_SCRIPT = Path(__file__).with_name('review_registered_action_storage_syn.py')

if __name__ == '__main__':
    raise SystemExit(runner.main())
