"""Finish SYN40 lease input-role review with selected actual guard/LUT cones.

This does not reparse the complete G2 graph, run EDA, alter a frozen input or
claim whole-control sequential equivalence. The added three factors must be
the actual LUT3 inputs of the same externally bound feedback-valid output.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,re,traceback
from revalidate_lidar_cdc import definitions
from synthesis_connectivity import CELL,PORT
from audit_netlist_memories import PARAM
from audit_routing_topology import primitive_directions
from prepare_baseline_fusion_retime import metadata,same_bytes
from review_model25_joint_syn import module_bindings
from review_model50_observation_ring_syn40 import actual_lease_equivalence,OLD,NEW,PREP,LIB

def save(p,v):
 assert not p.exists(),('Evidence already exists',str(p))
 p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf8')

def run(out):
 assert not out.exists();out.mkdir(parents=True)
 job=json.loads((NEW/'job.json').read_text());assert job['phase']=='synthesis_complete'and job['tool_exit']==0
 watched=[root/'impl/gwsynthesis/g2_top.vg'for root in(OLD,NEW)]+[LIB]
 before=[metadata(p)for p in watched]
 om,nm=[definitions(p)for p in watched[:2]]
 source=OLD/'control4500_core.v';candidate=NEW/'control4500_core.v'
 assert same_bytes(source,candidate)
 assert 'assign feedback_valid=left_speed_valid && right_speed_valid && model_ready;'in source.read_text()
 control=module_bindings(nm,['u_control'])['u_control']
 for port in('odo_valid','left_speed_valid','right_speed_valid','model_ready'):
  assert control['internal_declarations'][port][0]=='output'
  assert control['ports'][port]==[port],('Producer output not the actual top net',port,control['ports'][port])
 body=nm[control['module_kind']]
 parameters={}
 for raw,key,value in PARAM.findall(body):parameters.setdefault(raw.strip().lstrip('\\'),{})[key]=value.strip()
 candidates=[]
 for kind,raw,args in CELL.findall(body):
  ports={pin:expr.strip().lstrip('\\')for pin,expr in PORT.findall(args)}
  if ports.get('F')=='odo_valid':
   assert kind=='LUT3',('Expected exact feedback conjunction LUT3',kind,raw)
   assert set(ports)=={'I0','I1','I2','F'}
   assert {ports[k]for k in('I0','I1','I2')}=={'left_speed_valid','right_speed_valid','model_ready'}
   local=raw.lstrip('\\');name='u_control/'+local
   candidates.append((dict(name=name,kind=kind,ports={p:[v]for p,v in ports.items()}),parameters[local]))
 assert len(candidates)==1,('Actual complete feedback output driver not unique',len(candidates))
 cell,params=candidates[0];top={cell['name']:cell}
 proof=actual_lease_equivalence(om,nm,out,primitive_directions(LIB),top,lambda c:params)
 result=dict(passed=proof['passed'],recorded_at=datetime.now().astimezone().isoformat(),
   source=str(OLD),candidate=str(NEW),actual_completed_SYN40_exit=0,
   actual_existing_guard_common_ports_top_bindings_equal=True,
   actual_three_new_ports_are_exported_existing_feedback_conjunction_factors=True,
   actual_feedback_LUT3=cell,actual_feedback_LUT3_parameters=params,
   actual_full_200FF_next_state_and48_output_proof=proof,
   inherited_source_control_feedback_contract_byte_identical=True,
   partial_main_review='F:/nav_integration_g2_20261023/SYN40_ring_resources_memories_clocks_review2',
   input_bindings=before,input_metadata_unchanged=before==[metadata(p)for p in watched],
   whole_upstream_control_sequential_equivalence_claimed=False,
   current_partial_storage_clocks_original26_IR_proofs_not_overwritten=True,
   new_EDA_started=False,source_modified=False,arbitrary_XZ_semantics_claimed=False,
   physical_timing_or_deployment_accepted=False)
 assert result['input_metadata_unchanged'];save(out/'result.json',result);return result

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
 try:
  r=run(a.out);print(json.dumps(dict(passed=r['passed'],result=str(a.out/'result.json'),actual_FFs=200),ensure_ascii=False))
 except BaseException as e:
  if a.out.exists():save(a.out/'review_failed.json',dict(passed=False,error=str(e),traceback=traceback.format_exc(),recorded_at=datetime.now().astimezone().isoformat()))
  raise
