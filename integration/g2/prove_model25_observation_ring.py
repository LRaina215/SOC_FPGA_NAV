"""Small ring metadata/word model; not an RTL-frontend/full4500 proof."""
from pathlib import Path
from datetime import datetime
import argparse,json,re
from prove_soc_console_remap import z3
from prepare_model25_observation_ring import SOURCE,PREP,FILES,derive_observation,derive_tick,derive_policy,derive_wrapper
def run(out):
 out.mkdir(exist_ok=False)
 for rel,f in zip(FILES,(derive_observation,derive_tick,derive_policy,derive_wrapper)):
  assert f((SOURCE/rel).read_text())==(PREP/rel).read_text(),rel
 # This graph is closed under both legal commit and common observation reset.
 # It therefore covers arbitrarily many commits, not just two sampled wraps.
 initial=(False,0,(0,0,0,0,0));state=initial;states=[]
 while state not in states:
  states.append(state);initialized,work,hist=state
  state=(True,(work+1)%6,hist[1:]+(work,)if initialized else(work,)*5)
 assert state in states and len(states)==11
 checks=[]
 def unsat(label,bad):
  s=z3.Solver();s.set(timeout=10000);s.add(bad);r=s.check();assert r==z3.unsat,(label,str(r),str(s.model())if r==z3.sat else s.reason_unknown());checks.append(dict(name=label,passed=True,result='UNSAT'))
 rows=[]
 for i,(initialized,work,hist)in enumerate(states):
  assert 0<=work<6 and all(0<=slot<6 for slot in hist)
  if initialized:assert work not in hist
  next_hist=hist[1:]+(work,)if initialized else(work,)*5;next_work=(work+1)%6
  assert next_work not in next_hist
  memory=z3.Array('arbitrary_slot_word'+str(i),z3.BitVecSort(3),z3.BitVecSort(24));fresh=z3.BitVec('fresh_raw_word'+str(i),24)
  golden=[z3.BitVec('golden'+str(i)+'_'+str(j),24)for j in range(5)]
  assumptions=z3.And(*[z3.Select(memory,z3.BitVecVal(hist[j],3))==golden[j]for j in range(5)])if initialized else z3.BoolVal(True)
  changed=z3.Store(memory,z3.BitVecVal(work,3),fresh)
  expected=golden[1:]+[fresh]if initialized else[fresh]*5
  unsat('atomic_history_commit_state'+str(i),z3.And(assumptions,z3.Or(*[z3.Select(changed,z3.BitVecVal(next_hist[j],3))!=expected[j]for j in range(5)])))
  if initialized:unsat('staging_does_not_modify_committed_slots_state'+str(i),z3.Or(*[z3.Select(changed,z3.BitVecVal(k,3))!=z3.Select(memory,z3.BitVecVal(k,3))for k in hist]))
  rows.append(dict(initialized=initialized,work=work,history=list(hist),next_work=next_work,next_history=list(next_hist),work_and_live_disjoint=True))
 a,b=z3.BitVecs('slot_a slot_b',3);x,y=z3.BitVecs('lane_a lane_b',5)
 addr=lambda slot,lane:z3.ZeroExt(5,slot)*z3.BitVecVal(28,8)+z3.ZeroExt(3,lane)
 domain=z3.And(z3.ULT(a,6),z3.ULT(b,6),z3.ULT(x,28),z3.ULT(y,28))
 unsat('packed168_address_injective',z3.And(domain,addr(a,x)==addr(b,y),z3.Or(a!=b,x!=y)))
 unsat('packed168_address_no8bit_wrap',z3.And(z3.ULT(a,6),z3.ULT(x,28),z3.UGE(addr(a,x),168)))
 # Original clip_current function is byte-for-byte kept; all24bit raw values
 # map to the exact required signed current range without changing history.
 q=z3.BitVec('arbitrary_signed24_raw',24)
 clipped=z3.If(q>z3.BitVecVal(409600,24),z3.BitVecVal(409600,24),z3.If(q<z3.BitVecVal(-409600,24),z3.BitVecVal(-409600,24),q))
 unsat('clip_all24bit_range',z3.Or(clipped>409600,clipped< -409600))
 stream=[]
 for index in range(168):
  segment=0 if index<28 else 1 if index<56 else 2 if index<84 else 3 if index<112 else 4 if index<140 else 5
  lane=index-segment*28;assert 0<=lane<28
  stream.append(dict(index=index,current=segment==0,history_frame=None if segment==0 else segment-1,lane=lane,core_history=index>=28,core_address=index-28 if index>=28 else index))
 result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),scope='Closed reachable pointer graph plus arbitrary24bit-per-lane array relation. Exact source transformation is bound, but this is not an RTL frontend sequential proof.',source=str(PREP),closed_metadata_states=len(states),metadata_graph=rows,checks=checks,total_UNSAT=len(checks),reset_transition_to_initial_from_every_state=True,unknown_RAM_initial_values=True,single_writer_complete28word_commit_assumed=True,no_new_capture_during168_LOAD_contract_required=True,stream_order=stream,full4500_production_or_physical_timing_not_accepted=True)
 (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items()if k not in('checks','metadata_graph','stream_order')},indent=2));return result
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();run(a.out)
