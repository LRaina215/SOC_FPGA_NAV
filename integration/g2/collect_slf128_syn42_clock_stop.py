"""Collect completed read-only SYN42 clock/stop evidence; no graph or EDA.

The initializer supplement reads the exact primitives already saved by the
completed graph audit and binds their INIT defaults to the installed, unmodified
vendor library.  A passing clock partition does not clear the independent LF
memory collision or accept the image for deployment.
"""
from pathlib import Path
from collections import Counter
from datetime import datetime
import argparse,json,re
from prepare_baseline_fusion_retime import metadata,save

BASE=Path('F:/nav_integration_g2_20261023')
DEFAULT=BASE/'SLF128_storage_safeFW_syn42_clock_CDC_stop_review4'
REF=BASE/'baseline_fusion32_CDC52_pnr35/cdc_reference_audit.json'
LIB=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')

def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))

def collect(review,out):
 assert not out.exists(),out
 result=load(review/'result.json');assert result['passed']and result['phase']=='review_complete'
 assert result['input_bindings']==[metadata(Path(x['path']))for x in result['input_bindings']]
 clocks=load(review/'actual_two_clocks.json');cdc=load(review/'actual_original26_and_CDC52.json')
 fifo=load(review/'actual_FIFO_state_reset_classes.json');ir=load(review/'actual_IR_stop_partition.json');lease=load(review/'actual_lease_stop_partition.json')
 reset=load(review/'actual_reset_wrapper_roles.json');timing=load(review/'actual_synthesis_estimated_paths.json')
 reference=load(REF);library=LIB.read_text(encoding='utf8',errors='strict');checks=[]
 for row in cdc['rows']:
  for role,refrole in [('source','source_register'),('first','first_register'),('second','second_register')]:
   c=row[role];q=c['ports']['Q']
   candidates=[x['actual_source_first_second'][refrole]for x in reference['paths']if x['actual_source_first_second'][refrole]['primitive']['ports']['Q']==q]
   assert len(candidates)==1,(role,q,candidates)
   r=candidates[0];matches=re.findall(r'module\s+'+re.escape(c['kind'])+r'\b[\s\S]*?parameter\s+INIT\s*=\s*([^;]+)',library)
   assert len(matches)==1,(c['kind'],matches)
   default=matches[0].strip();actual=row[role+'_reset']['parameters'].get('INIT',default)
   assert actual==r['effective_INIT'],(role,c,actual,r['effective_INIT'])
   checks.append(dict(role=role,actual_name=c['name'],actual_Q=q,kind=c['kind'],explicit_INIT=row[role+'_reset']['parameters'].get('INIT'),published_default=default,effective_INIT=actual,reference_effective_INIT=r['effective_INIT'],passed=True))
 assert len(checks)==78
 kinds=Counter(x['kind']for g in fifo['groups'].values()for x in g['live_rows'])
 changes=Counter((x['old_kind'],x['new_kind'])for x in cdc['intended_reset_type_changes_from_SYN40'])
 preserved=[]
 for number in(1,2,3):
  p=BASE/f'SLF128_storage_safeFW_syn42_clock_CDC_stop_review{number}'
  if p.exists():
   entry=dict(path=str(p),preserved=True)
   if(p/'result.json').exists():entry.update(error=load(p/'result.json').get('error'))
   if number==3:entry['notes']='Sampling source Q was promoted to its actual register-file public l_rx_neg port; the first reader assumed a local Q name. Controller metadata finalization also shadowed its state dictionary. Both are reviewer defects, not hardware counterexamples; all section artifacts remain.'
   preserved.append(entry)
 fragment=(review/'actual_SYN42_CDC52_fragment.sdc').read_text()
 assert fragment.count('set_false_path')==26 and fragment.count('set_max_delay')==26 and fragment.count(' -hold')==26
 lf=BASE/'SLF128_LF_actual_control_trace1/result.json';assert lf.exists()
 evidence=dict(at=datetime.now().astimezone().isoformat(),scope='Completed SYN42 mapped clock/known26/FIFO-reset and IR/lease partitions only',passed=True,
  actual_review=str(review/'result.json'),actual_review_input_bindings=result['input_bindings'],new_EDA_started=False,programmed=False,
  clock_pins_checked=clocks['checked_clock_pins'],constant_clock_pins=clocks['constant_clock_pins'],actual_clock_roots_and_phases=clocks['root_pin_counts'],no_PLL_or_model25_bridges=True,
  original26=dict(passed=True,paths=26,groups=dict(Counter(x['group']for x in cdc['rows'])),first_Q_only_second_D=True,first_and_second_CE1=True,
    actual_endpoint_source=str(review/'actual_original26_and_CDC52.json'),INIT_roles_checked=len(checks),INIT_checks=checks,
    intended_reset_kind_changes=[dict(old=a,new=b,roles=n)for(a,b),n in sorted(changes.items())],
    SDC_fragment=str(review/'actual_SYN42_CDC52_fragment.sdc'),commands=52,sourceQ_to_firstD_only=True,stage1_to_stage2_remains_timed=True,
    PNR_Actived_status_physical_delays_setup_hold_MTBF_accepted=False),
  reset_wrappers=dict(proof=str(review/'actual_reset_wrapper_roles.json'),live_wrappers=len(reset['actual_roles']),FFs=2*len(reset['actual_roles']),kind='DFFPE',core_assertion_source='!key_rst_n',release='two local-domain stages',pruned_host_RX_write_half_proof=str(review/'actual_pruned_reset_scopes.json')),
  FIFO=dict(total_FFs=fifo['actual_FIFO_FFs'],kinds=dict(kinds),by_scope={k:v['actual_FFs']for k,v in fifo['groups'].items()},public_nav_fifo_ob_source_unchanged=fifo['public_nav_fifo_ob_source_unchanged'],proof=str(review/'actual_FIFO_state_reset_classes.json')),
  stop=dict(IR_actual_FFs=ir['actual_FFs'],IR_next_and_output_checks=len(ir['all_actual_next_states_and_outputs_equal']),lease_actual_FFs=lease['actual_FFs'],lease_next_and_output_checks=len(lease['all_actual_next_states_and_outputs_equal']),all_binary_partition_checks_passed=True,
    original4500_stop_enable_feedback_source_contract=str(review/'source_stop_contract.json'),whole4500_CPU_RK_runtime_or_four_state_equivalence_claimed=False),
  synthesis_timing=dict(listed_paths=len(timing['actual_listed_paths']),first_path_summary=timing['actual_listed_paths'][0]['summary'],report=str(review/'actual_synthesis_estimated_paths.json'),estimated_not_routed=True,complete_negative_endpoint_coverage=False,timing_passed=False),
  preserved_reader_failures=preserved,independent_blocker=dict(path=str(lf),description='Actual mapped LF terminal drain cycle has simultaneous read/write at a conflicting physical address. Clock audit passing does not clear this memory hazard.'),
  physical_unknown=result['physical_unknown'],PNR42_authorized_by_this_review=False,deployment_accepted=False)
 save(out,evidence);print(json.dumps(dict(passed=True,out=str(out),clock_pins=evidence['clock_pins_checked'],original26=26,FIFO_FFs=fifo['actual_FIFO_FFs'],IR_FFs=ir['actual_FFs'],lease_FFs=lease['actual_FFs'],independent_LF_blocker_preserved=True),ensure_ascii=False))

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--review',type=Path,default=DEFAULT);p.add_argument('--out',type=Path,required=True);a=p.parse_args();collect(a.review,a.out)
