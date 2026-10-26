"""Actual two-clock SYN40 ring partition review; never starts an EDA tool."""
from pathlib import Path
import argparse,traceback
import review_model25_ring_semantics_syn39 as core

core.OLD=core.BASE/'baseline_fusion_only_syn23'
core.NEW=core.BASE/'baseline_fusion50_ring_MAX_syn40'
core.PREP=core.BASE/'baseline_fusion50_ring_MAX_prepared1'
core.MODEL='u_control/model_thread'
core.OBS=core.MODEL+'/observation'
core.POL=core.MODEL+'/policy'
core.CLOCK='clk50_d'
core.PERIOD=400000
core.PROFILE='fusion50'
OUT=core.BASE/'fusion50_ring_SYN40_actual_semantics_review1'

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--review',action='store_true');p.add_argument('--out',type=Path,default=OUT);a=p.parse_args()
 if not a.review:print('Prepared only. After SYN40 completes: --review; output '+str(a.out/'final_review.json'))
 else:
  try:
   assert core.load(core.NEW/'job.json')['phase']=='synthesis_complete','SYN40 has not completed'
   assert not a.out.exists(),'Preserve previous review outputs; select a fresh --out'
   core.run(a.out)
  except Exception:
   a.out.mkdir(parents=True,exist_ok=True)
   if not(a.out/'final_review.json').exists():core.save(a.out/'final_review.json',dict(passed=False,error=traceback.format_exc(),hardware_inputs_unchanged=True))
   raise
