"""Review completed sequential single-guard mappings; never launch builds."""
from pathlib import Path
from datetime import datetime
import argparse,collections,json,re,time,traceback
from synthesis_connectivity import read,DECL
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from audit_routing_topology import primitive_directions
from prove_soc_console_remap import Logic,z3
from prepare_baseline_fusion_retime import save,metadata,same_bytes
from review_lease_deadline_syn_mapping import local_guard,hierarchy_resources,report_paths,LIB

def run(base,out):
 out.mkdir(parents=True,exist_ok=False);started=time.monotonic()
 try:
  job=json.loads((base/'job.json').read_text());assert job['phase']=='small_pair_complete_actual_topology_and_equivalence_review_pending'
  dirs=[base/'reference',base/'bounded_tree'];vg=[d/'impl/gwsynthesis/g2_top.vg'for d in dirs]
  for d in dirs:
   rt=json.loads((d/'small_SYN_runtime.json').read_text());assert rt['exit_code']==0 and rt['PNR']is False
   tcl=(d/'run.tcl').read_text();selected=re.findall(r'^add_file -type verilog \{(.+)\}$',tcl,re.M)
   assert selected==[(d/'guard.v').as_posix(),(d/'top.v').as_posix()]
   assert set(d.glob('*.v'))=={d/'guard.v',d/'top.v'}
   assert not any(d.rglob('*.hex'))and not any(d.rglob('*.mem'))
  assert same_bytes(dirs[0]/'top.v',dirs[1]/'top.v')and same_bytes(dirs[0]/'clock.sdc',dirs[1]/'clock.sdc')
  assert (base/'command_lease_guard.v').read_text()==(dirs[1]/'guard.v').read_text() # Windows text-mode copy changes only CRLF
  body0,c0,p0,q0=local_guard(definitions(vg[0]),out,'reference')
  body1,c1,p1,q1=local_guard(definitions(vg[1]),out,'bounded')
  assert q0.keys()==q1.keys(),dict(only_before=sorted(q0.keys()-q1.keys()),only_after=sorted(q1.keys()-q0.keys()))
  directions=primitive_directions(LIB);shared={q:z3.Bool('shared_actual_FF:'+q)for q in q0}
  logic=[Logic(c0,p0,directions,shared,{}),Logic(c1,p1,directions,shared,{})]
  checks=[];sat_calls=0
  def equal(label,x,y):
   nonlocal sat_calls
   delta=z3.simplify(z3.Xor(x,y))
   if z3.is_false(delta):checks.append(dict(label=label,method='canonical_boolean_identity'));return
   solver=z3.Solver();solver.set(timeout=30000);solver.add(delta);sat_calls+=1;res=solver.check()
   assert res==z3.unsat,(label,str(res),str(solver.model())if res==z3.sat else solver.reason_unknown())
   checks.append(dict(label=label,method='actual_mapped_miter',result=str(res)))
  for q in sorted(q0):
   x,y=q0[q],q1[q];assert x['kind']==y['kind']and p0(x)==p1(y),(q,x,y)
   assert x['ports']['CLK']==y['ports']['CLK']==['clk_d']
   equal(q+'/next',logic[0].next_state(x),logic[1].next_state(y))
  ports=lambda body:{n.lstrip('\\'):(hi,lo)for d,hi,lo,n in DECL.findall(body)if d=='output'}
  outputs=ports(body0);assert outputs==ports(body1)
  bits=[]
  for n,(hi,lo)in outputs.items():
   for net in([n]if not hi else[n+'['+str(i)+']'for i in range(int(hi),int(lo)-1,-1)]):
    equal(net+'/public_output',logic[0].expr(net),logic[1].expr(net));bits.append(net)
  # The wrapper is common; verify actual100 launch FFs and input buffer bindings,
  # not just its source text. Avoid treating an OBUF as a logic gate.
  full=[read(v)for v in vg];params=[parameter_reader(definitions(v))for v in vg]
  shells=[]
  for idx,cells in enumerate(full):
   rows={n:dict(c,params=params[idx](c))for n,c in cells.items()if not n.startswith('dut/')and not re.fullmatch(r'LUT[1-6]',c['kind'])}
   shells.append(rows)
  assert shells[0]==shells[1],dict(wrapper_only_before=sorted(shells[0].keys()-shells[1].keys()),wrapper_only_after=sorted(shells[1].keys()-shells[0].keys()))
  launch_FFs=sum(c['kind'].startswith('DFF')for c in shells[0].values());assert launch_FFs==100
  resources=[hierarchy_resources(d/'impl/gwsynthesis/g2_top_syn_rsc.xml')for d in dirs]
  result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),elapsed_s=time.monotonic()-started,
   small_only_no_G2_or_assets=True,reference=str(dirs[0]),candidate=str(dirs[1]),
   actual_guard_FFs=len(q0),actual_launch_FFs=launch_FFs,actual_total_FFs=len(q0)+launch_FFs,
   actual_FF_kinds_all_INIT_clocks_equal=True,actual_wrapper_nonLUT_cells_and_ports_equal=True,
   public_output_bits=len(bits),sat_calls=sat_calls,checks=checks,
   actual_resources_before=resources[0],actual_resources_after=resources[1],
   reference_estimated_paths=report_paths(dirs[0]),candidate_estimated_paths=report_paths(dirs[1]),
   focused_cycle=json.loads((base/'focused_cycle_result.json').read_text()),math=json.loads((base/'math_proof.json').read_text()),
   state_scope='Every mapped lease FF prestate and external input in this complete guard wrapper; no fullG2 physical or numerical claim.',
   mapping_topology_review='separate actual_tree_topology.json; source LUT count and preservation attributes are not acceptance',
   estimated_single_guard_not_joint_timing=True,new_joint_SYN_PNR=False,programmed=False,
   source_bindings=[metadata(d/'guard.v')for d in dirs],actual_VG_bindings=[metadata(v)for v in vg])
  save(out/'result.json',result);return result
 except Exception as exc:
  save(out/'failed.json',dict(passed=False,reason=str(exc),traceback=traceback.format_exc()));raise

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--base',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
 r=run(a.base,a.out);print(json.dumps({k:r[k]for k in('passed','actual_guard_FFs','actual_launch_FFs','sat_calls','elapsed_s')},indent=2))
