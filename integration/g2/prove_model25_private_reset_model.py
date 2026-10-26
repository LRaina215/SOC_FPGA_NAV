"""Lightweight induction for the exact private-data reset transformation.
Manual transition model, bound to source block literals and independent4state RTL.
No EDA build, ModelSim or FPGA operations.
"""
import sys,json,time,importlib.util,argparse
from pathlib import Path
sys.path.insert(0,'D:/Desktop/SOC FPGA/nav/integration/g2')
from prove_soc_console_remap import z3
ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);args=ap.parse_args();OUT=args.out;OUT.mkdir(exist_ok=False)
BOOLS=('request','ack1','ack2','req1','req2','acknowledge','overflow','valid')

def prove(width,latest):
 def state(tag):
  d={k:z3.Bool(tag+':'+k)for k in BOOLS}
  d.update(holding=z3.BitVec(tag+':holding',width),data=z3.BitVec(tag+':data',width))
  if latest:d.update(sent=z3.Bool(tag+':sent'),last=z3.BitVec(tag+':last_sent',width))
  return d
 o,n=state('old'),state('new');sa,da,written=z3.Bools('source_released destination_released written_in_epoch')
 se,de,reset,rs,rd,valid=z3.Bools('source_edge destination_edge common_reset release_source release_destination external_valid')
 data=z3.BitVec('arbitrary_word',width);zero=z3.BitVecVal(0,width)
 def invariants(o,n,sa,da,w):
  inv=[o[k]==n[k]for k in BOOLS]+[o['data']==n['data']]
  inv+=[z3.Implies(w,z3.And(sa,o['holding']==n['holding']))]
  handshake=('request','ack1','ack2','req1','req2','acknowledge')
  inv+=[z3.Implies(z3.Not(w),z3.And(*[z3.Not(o[k])for k in handshake],o['data']==zero))]
  inv+=[z3.Implies(z3.Not(sa),z3.And(*[z3.Not(o[k])for k in('request','ack1','ack2','overflow')],o['holding']==zero))]
  inv+=[z3.Implies(z3.Not(da),z3.And(*[z3.Not(o[k])for k in('req1','req2','acknowledge','valid')],o['data']==zero))]
  if latest:
   inv+=[o['sent']==n['sent'],z3.Implies(o['sent'],z3.And(w,o['last']==n['last'])),z3.Implies(z3.Not(sa),z3.And(z3.Not(o['sent']),o['last']==zero))]
  return z3.And(*inv)
 def step(s,private_unreset):
  ready=s['request']==s['ack2']
  send=z3.And(ready,z3.Or(z3.Not(s['sent']),data!=s['last']))if latest else valid
  accept=z3.And(send,ready);commit=z3.Xor(s['req2'],s['acknowledge'])
  sr=z3.Or(reset,z3.Not(sa));dr=z3.Or(reset,z3.Not(da))
  t={}
  for k,source in [('ack1','acknowledge'),('ack2','ack1')]:t[k]=z3.If(sr,False,z3.If(se,s[source],s[k]))
  for k,source in [('req1','request'),('req2','req1')]:t[k]=z3.If(dr,False,z3.If(de,s[source],s[k]))
  t['request']=z3.If(sr,False,z3.If(z3.And(se,accept),z3.Not(s['request']),s['request']))
  t['overflow']=z3.If(sr,False,z3.If(se,z3.Or(s['overflow'],z3.And(send,z3.Not(ready))),s['overflow']))
  t['acknowledge']=z3.If(dr,False,z3.If(z3.And(de,commit),s['req2'],s['acknowledge']))
  t['valid']=z3.If(dr,False,z3.If(de,commit,s['valid']))
  t['data']=z3.If(dr,zero,z3.If(z3.And(de,commit),s['holding'],s['data']))
  captured=z3.If(z3.And(se,accept),data,s['holding'])
  t['holding']=captured if private_unreset else z3.If(sr,zero,captured)
  if latest:
   t['sent']=z3.If(sr,False,z3.If(z3.And(se,send),True,s['sent']))
   cap=z3.If(z3.And(se,send),data,s['last'])
   t['last']=cap if private_unreset else z3.If(sr,zero,cap)
  return t,accept
 # Base: public/control and old reset data zero; new private words unconstrained.
 base=[]
 for s in(o,n):base +=[z3.Not(s[k])for k in BOOLS]+[s['data']==zero]
 base +=[o['holding']==zero,z3.Not(sa),z3.Not(da),z3.Not(written)]
 if latest:base +=[z3.Not(o['sent']),z3.Not(n['sent']),o['last']==zero]
 results=[]
 def unsat(label,constraints):
  solver=z3.Solver();solver.set(timeout=15000);solver.add(*constraints);start=time.time();r=solver.check()
  if r!=z3.unsat:raise AssertionError((label,str(r),str(solver.model())if r==z3.sat else solver.reason_unknown()))
  results.append(dict(label=label,status='UNSAT',seconds=time.time()-start))
 inv=invariants(o,n,sa,da,written)
 unsat('base_arbitrary_new_private_data',base+[z3.Not(inv)])
 old,accept=step(o,False);new,naccept=step(n,True)
 nsa=z3.If(reset,False,z3.Or(sa,rs));nda=z3.If(reset,False,z3.Or(da,rd))
 nw=z3.If(z3.Or(reset,z3.Not(sa)),False,z3.Or(written,z3.And(se,accept)))
 unsat('one_step_induction_all_edges_data_reset_and_monotonic_domain_release',[inv,z3.Not(invariants(old,new,nsa,nda,nw))])
 unsat('source_accept_and_ready_equivalence',[inv,z3.Xor(accept,naccept)])
 unsat('unwritten_epoch_cannot_commit',[inv,z3.Not(written),z3.Xor(o['req2'],o['acknowledge'])])
 unsat('private_word_holds_when_busy_without_reset',[inv,written,z3.Not(reset),sa,z3.Not(o['request']==o['ack2']),new['holding']!=n['holding']])
 return dict(logical_width=width,latest=latest,passed=True,checks=results,arbitrary_private_initial_words=True,arbitrary_full_word_inputs=True,arbitrary_source_and_destination_edges_including_indefinite_halt=True,common_reset_assertion_only=True,release_may_differ_between_domains=True,unilateral_reset_reassertion_not_supported=True)
BASE=Path('F:/nav_integration_g2_20261023')
reference=BASE/'baseline_fusion_model25_NAV50_prepared4/model25_related_mailbox.v'
candidate=BASE/'baseline_fusion_model25_private_source_noreset_prepared1/model25_related_mailbox.v'
prep=Path(__file__).with_name('prepare_model25_private_source_noreset.py')
spec=importlib.util.spec_from_file_location('private_reset_source_transform',prep);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
original=reference.read_text(encoding='utf8');actual=candidate.read_text(encoding='utf8')
assert module.derive(original)==actual
assert original.split('module g2_model25_guard',1)[1]==actual.split('module g2_model25_guard',1)[1]
source_binding=dict(reference=str(reference),candidate=str(candidate),exact_private_only_transformation=actual==module.derive(original),guard_byte_identical=True,reference_bytes=reference.stat().st_size,candidate_bytes=candidate.stat().st_size)
rows=[prove(w,l)for w,l in[(128,False),(140,True),(336,False),(145,False),(15,True)]]
r=dict(passed=True,kind='inductive relational transition-model proof',source_bound_to_private_only_transformation=True,source_binding=source_binding,source_rtl_frontend_formal_equivalence_claimed=False,rows=rows,total_unsatisfiable_obligations=sum(len(x['checks'])for x in rows),full_system_or_physical_signoff=False)
(OUT/'formal_result.json').write_text(json.dumps(r,indent=2)+'\n',encoding='utf8');print(json.dumps(dict(passed=True,obligations=r['total_unsatisfiable_obligations'],widths=[x['logical_width']for x in rows])))
