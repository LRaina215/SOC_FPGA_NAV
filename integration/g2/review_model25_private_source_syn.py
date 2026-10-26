"""Actual SYN36 inventory/protocol review; no build or automatic PNR."""
from pathlib import Path
from collections import Counter,defaultdict
from datetime import datetime
import argparse,json,traceback
import review_model25_joint_syn as base
import prove_model25_private_source_transport as transport
SOURCE=base.BASE/'baseline_fusion_model25_private_source_noreset_syn36'
PREP=base.BASE/'baseline_fusion_model25_private_source_noreset_prepared2'
BASELINE=base.BASE/'baseline_fusion_model25_NAV50_syn34'
def audit(out):
 base.NEW=SOURCE;base.PREP=PREP
 ctx=base.run(out)
 protocol=transport.complete(ctx)
 old=base.read(BASELINE/'impl/gwsynthesis/g2_top.vg');new=ctx['after']
 counts=lambda cells:dict(Counter(c['kind']for c in cells.values()))
 before,after=counts(old),counts(new)
 summary=lambda c:dict(FF=sum(v for k,v in c.items()if k.startswith('DFF')),LUT=sum(v for k,v in c.items()if __import__('re').fullmatch(r'LUT[1-6]',k)),ALU=c.get('ALU',0),**{k:v for k,v in c.items()if k.startswith('MULT')or k in('SP','SDP','DP','SDPX9B','DPB','ROM16','RAM16SDP4')})
 a,b=summary(before),summary(after)
 # Cold-private INIT may change; public control and all memory INIT are checked
 # independently and are not silently allowed to change with it.
 result=dict(passed=False,inventory_and_public_private_protocol_passed=True,independent_final_word_binding_reset_load_and_resource_verdict_pending=True,recorded_at=datetime.now().astimezone().isoformat(),source=str(SOURCE),reference=str(BASELINE),actual_private_FFs=len(protocol['all_private']),actual_Boolean_checks=len(protocol['checks']),resources=dict(before=a,after=b,delta={k:b.get(k,0)-a.get(k,0)for k in set(a)|set(b)}),automatic_PNR=False,programmed=False,deployment_accepted=False)
 base.save(out/'actual_private_mapping_inventory.json',result)
 print(json.dumps(result,ensure_ascii=False))
 return ctx,protocol,result
def finalize(out,payload_review,reset_review):
 """Aggregate independent real-netlist proofs; never overwrite first rejection."""
 basic=base.load(out/'base_inventory.json')
 protocol=base.load(out/'private_transport_stage_result.json')
 inventory=base.load(out/'actual_private_mapping_inventory.json')
 payload=base.load(payload_review/'final_review.json')
 reset=base.load(reset_review/'actual_private_reset_loads.json')
 resource=base.load(reset_review/'actual_vendor_synthesis_resources.json')
 assert basic['passed']and protocol['passed']and payload['passed']and reset['passed']and resource['passed']
 assert payload['actual_source']==str(SOURCE)
 assert payload['actual_VG_metadata']==base.meta(SOURCE/'impl/gwsynthesis/g2_top.vg')
 assert all(base.meta(Path(x['path']))==x for x in basic['original_hardware_bound_before'])
 files=[Path(p).relative_to(SOURCE).as_posix()for p in base.load(SOURCE/'manifest.json')['source_files']]
 changed=[n for n in files if not base.same(SOURCE/n,BASELINE/n)]
 assert len(files)==66 and changed==['model25_related_mailbox.v'],changed
 for name in('actual_guard_cold_start_and_INIT1_proof.json','actual_original_radar26.json'):
  assert base.load(out/name)['passed']
 assert protocol['actual_private_FFs']==551 and reset['actual_async_reset_load_reduction']==551
 assert resource['delta']['T_Register']==0 and resource['delta']['T_Alu']==139 and resource['delta']['T_Lut']==-118
 result=dict(passed=True,scope='Actual completed SYN36 single-factor private source reset removal mapping, Boolean/full-word/consumer proof. No physical timing or deployment acceptance.',recorded_at=datetime.now().astimezone().isoformat(),
  source=str(SOURCE),reference=str(BASELINE),changed_RTL=changed,other65_RTL_unchanged=True,
  all_RAM_geometry_INIT_CPU128KiB_original_FW_assets69_headers31_preserved=True,
  PLL50to25_and_three_real_clock_roots_preserved=True,original_radar26_and_guard_INIT1_cold_clear_proved=True,
  public_req_ACK_valid_overflow_sent_destination_guard_stop_reset_contract_preserved=True,
  private551_capture_function_same_source_word_proved=True,all_destination_words_DSP_AREG24_and_final272_consumers_proved=True,
  latest_capture_CE_full_word_change_no_added_reset_gate_proved=True,
  configuration48_synchronous_RESET_pins_are_capture_only_data_zero=True,
  actual_async_reset_pin_reduction=551,reset_root_changes=reset['reset_root_changes'],
  vendor_resource_delta=resource['delta'],physical_BSRAM_prediction=332,actual_physical_BSRAM=None,
  protocol_Boolean_checks=protocol['Boolean_checks'],payload_Boolean_checks=payload['Boolean_checks'],unproved_payload_bits=[],
  configuration_shared_sent_split_into_private139_plus_public_sent1_total_FF_unchanged=True,
  reports=dict(base=str(out/'base_inventory.json'),protocol=str(out/'actual_new_private_source_transport.json'),payload=str(payload_review/'final_review.json'),reset=str(reset_review/'actual_private_reset_loads.json'),resources=str(reset_review/'actual_vendor_synthesis_resources.json')),
  actual_VG_metadata=payload['actual_VG_metadata'],whole_system_sequential_equivalence_claimed=False,
  unknowns=['Physical PNR resource/global allocation/setup/hold and bundled-data delay','Physical reset pulse/recovery/removal/IO and complete vehicle stopping'],automatic_PNR=False,PNR_started=False,programmed=False,deployment_accepted=False)
 base.save(out/'final_review.json',result);print(json.dumps(result,ensure_ascii=False));return result
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--finalize-existing',action='store_true');ap.add_argument('--payload-review',type=Path);ap.add_argument('--reset-review',type=Path);args=ap.parse_args()
 if not args.finalize_existing:args.out.mkdir(exist_ok=False)
 try:
  if args.finalize_existing:
   assert args.payload_review and args.reset_review
   finalize(args.out,args.payload_review,args.reset_review)
  else:audit(args.out)
 except BaseException:base.save(args.out/'review_failed.json',dict(passed=False,traceback=traceback.format_exc()));raise
