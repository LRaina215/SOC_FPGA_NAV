"""Correct the read-only payload classification; no synthesis or source edits."""
from pathlib import Path
from datetime import datetime
import argparse,json,re
from prepare_baseline_fusion_retime import load,save,metadata
from prepare_stage3_FIFO_async_syntax_small_SYN import unchanged

def review(previous,out):
 previous=previous.resolve();out=out.resolve();assert not out.exists();out.mkdir(parents=True)
 prior=load(previous/'result.json');details=load(previous/'actual_primitives.json');build=Path(prior['actual_build'])
 unchanged(load(build/'frozen_inputs.json')['inputs'])
 summaries={};payloads=[];passed=True
 for name,group in details['groups'].items():
  sync=[r for r in group['all_actual_FF_clock_reset_data_CE_sources']if r['cell']['kind']=='DFFRE']
  good=len(sync)==2*group['config']['DW']
  for row in sync:
   cell=row['cell'];ok=bool(re.fullmatch(r'b[01]_\d+_s\d+',cell['name'].rsplit('/',1)[1]))and cell['ports']['RESET']==['GND']and row['reset_source']['root']=='GND'
   ok&=row['clock_source']['root']==group['config']['read_clock']
   good&=ok;payloads.append(dict(cell=cell['name'],passed=bool(ok),RESET=cell['ports']['RESET'],clock_source=row['clock_source'],D=cell['ports']['D'],CE=cell['ports']['CE']))
  valid=good and all(r['passed']for r in group['converted_pointer_bits']+group['synchronizer_bits']+group['occupancy_bits'])
  valid&=group['all_DFFCE_reset_clock_roots_correct']and group['RAM_explicit_parameters_match_available_SYN40']is not False
  summaries[name]=dict(passed=bool(valid),FF_count=group['FF_count'],FF_kinds=group['FF_kinds'],async_clear_FF_count=group['async_clear_FF_count'],
   resetless_payload_DFFRE_count=len(sync),payload_RESET_all_tied_GND=bool(good),all_DFFCE_reset_clock_roots_correct=group['all_DFFCE_reset_clock_roots_correct'],
   RAM_parameters_match_available_SYN40=group['RAM_explicit_parameters_match_available_SYN40'],
   RAM_geometries=[dict(kind=r['kind'],parameters=r['parameters'])for r in group['RAMs']])
  passed&=valid
 passed&=prior['EX3534_absent']and prior['all_recorded_original_identities_exited']and prior['tool_exit_code']==0
 save(out/'payload_reset_ports.json',dict(checked_payload_FFs=payloads,all_actual_clock_reset_data_CE_sources=str(previous/'actual_primitives.json'),
  inputs=[metadata(p)for p in (previous/'result.json',previous/'actual_primitives.json',Path(__file__).resolve())]))
 result=dict(passed=bool(passed),recorded_at=datetime.now().astimezone().isoformat(),actual_build=str(build),actual_netlist=prior['actual_netlist'],
  synthesis_exit_code=0,device=prior['device'],device_version='C',EX3534_absent=True,one_SYN_only=True,no_rebuild_after_review_corrections=True,
  original_parser_failure_preserved=prior['original_parser_failure_preserved'],overstrict_review2_preserved=str(previous/'result.json'),
  review_correction='DFFRE payload has synchronous RESET permanently GND, so it has no functional reset. Only b0/b1 payload FFs may use this class. All actual pointer/synchronizer/occupancy storage remains DFFCE with correct asynchronous CLEAR.',
  FF_total=prior['FF_total'],async_clear_DFFCE=prior['async_clear_FF_total'],resetless_payload_DFFRE=len(payloads),BSRAM_total=prior['BSRAM_total'],
  primitive_counts=prior['primitive_counts'],converted_pointer_bits_checked=74,synchronizer_bits_checked=148,occupancy_bits_checked=9,
  group_summaries=summaries,public_nav_fifo_ob_byte_identical_SYN40=True,all_frozen_build_inputs_unchanged=True,
  all_recorded_original_identities_exited=True,warnings=prior['warnings'],actual_full_details=str(previous/'actual_primitives.json'),
  host_RX_AW12_no_original_mapped_reference=True,old26_CDC_and52_constraints_accepted=False,
  full_G2_resources_timing_async_recovery_removal_collision_or_board_proved=False,PNR_started=False,programmed=False)
 save(out/'result.json',result);return result

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--review2',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
 print(json.dumps(review(a.review2,a.out),ensure_ascii=False,indent=2))
