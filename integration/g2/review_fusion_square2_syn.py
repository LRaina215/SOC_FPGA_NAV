"""Use the shared actual-VG audit with the routed SYN23/PNR32 reference.

Only reference constants in this new process change. Historical reviewers and
results stay intact; both full old/new VGs are still independently parsed.
"""
from pathlib import Path
import review_baseline_single_factor_syn as engine

engine.ORIGINAL=Path('F:/nav_integration_g2_20261023/baseline_fusion_only_syn23')
engine.BASELINE=Path('F:/nav_integration_g2_20261023/baseline_fusion_only_pnr32')

if __name__=='__main__':raise SystemExit(engine.main())
