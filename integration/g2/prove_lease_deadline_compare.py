"""Exact32bit arithmetic miter, bound to the frozen one-expression source diff."""
from pathlib import Path
from datetime import datetime
import argparse,json,time
from prepare_lease_deadline_compare import OLD,NEW,rewrite
from prepare_baseline_fusion_retime import metadata,save
from prove_soc_console_remap import z3

def prove(source,out):
 started=time.monotonic();out.mkdir(parents=True,exist_ok=False)
 review=json.loads((source/'prepared_source_review.json').read_text())
 original=Path(review['source'])/'command_lease_guard.v';candidate=source/'command_lease_guard.v'
 a=original.read_text();b=candidate.read_text()
 assert rewrite(a)==b and b.replace(NEW,OLD)==a
 age=z3.BitVec('age_unsigned32',32);timeout=z3.BitVec('integer_timeout_bit_pattern32',32)
 tick,have=z3.Bools('tick have_lease')
 old=z3.UGE(age+z3.BitVecVal(1,32),timeout)
 new=z3.If(timeout==0,old,z3.And(z3.UGE(age,timeout-z3.BitVecVal(1,32)),age!=z3.BitVecVal(0xffffffff,32)))
 checks=[]
 for label,miter in [('all_age32_timeout32_arithmetic',z3.Xor(old,new)),
   ('all_age32_timeout32_tick_lease_boolean_gate',z3.Xor(z3.And(tick,have,old),z3.And(tick,have,new)))]:
  solver=z3.Solver();solver.set(timeout=30000);solver.add(miter)
  savepath=out/(label+'.smt2');savepath.write_text(solver.to_smt2(),encoding='utf-8')
  status=solver.check()
  assert status==z3.unsat,(label,str(status),str(solver.model())if status==z3.sat else solver.reason_unknown())
  checks.append(dict(label=label,status=str(status),smt2=str(savepath),statistics=str(solver.statistics())))
 # A meaningful negative control: dropping the maximum-age exclusion changes
 # the default200ms expression at age0xffffffff. This is not a passing test.
 bad=z3.Solver();bad.add(z3.Xor(z3.UGE(age+1,z3.BitVecVal(200,32)),z3.UGE(age,z3.BitVecVal(199,32))))
 assert bad.check()==z3.sat
 badage=bad.model().eval(age).as_long();assert badage==0xffffffff
 result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),solver='Z3 '+z3.get_version_string(),
  elapsed_s=time.monotonic()-started,proofs=checks,known_age_state_count=2**32,known_timeout_bit_pattern_count=2**32,
  no_reachability_assumption=True,unchanged_remainder_of_module_text=True,
  implications='For every same prestate and input, identical deadline and all other statements yield identical registered next state/output; no pipeline, priority or period change.',
  four_state_scope='All-state solver is2state. Timeout0 explicitly retains original arithmetic; directed SV X/Z semantics checked separately in finite fixture.',
  bad_naive_rewrite_counterexample=dict(age_hex=hex(badage),timeout=200,original=False,naive_without_MAX_exclusion=True),
  input_bindings=[metadata(original),metadata(candidate)],new_SYN_PNR_started=False,physical_mapping_or_timing_accepted=False)
 save(out/'result.json',result);return result

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--source',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
 print(json.dumps(prove(a.source,a.out),indent=2))
