"""Inspect actual small-SYN lease MAX-tree topology; never starts synthesis.
Replay: D:/Anaconda/python.exe -B -X utf8 review_lease_bounded_tree_topology.py
"""
from pathlib import Path
from datetime import datetime
from functools import lru_cache
import argparse,collections,json,re,traceback
from synthesis_connectivity import read
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from audit_routing_topology import primitive_directions
from prove_soc_console_remap import Logic,literal,z3

DEFAULT=Path('F:/nav_integration_g2_20261023/lease_deadline_bounded_tree_candidate1')
LIB=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
def meta(p):
 p=Path(p);s=p.stat()
 return dict(path=str(p),bytes=s.st_size,mtime_ns=s.st_mtime_ns)
def save(p,v):
 p=Path(p)
 assert not p.exists(),('existing evidence is not overwritten',str(p))
 p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf8')

def snapshot(vg):
 modules=definitions(vg);cells=read(vg);params=parameter_reader(modules)
 directions=primitive_directions(LIB)
 drivers=collections.defaultdict(list);loads=collections.defaultdict(list)
 for n,c in cells.items():
  for pin,nets in c['ports'].items():
   for i,net in enumerate(nets):
    if net in ('GND','VCC'):continue
    if directions[c['kind']][pin]=='output':drivers[net].append((n,pin,i))
    elif directions[c['kind']][pin] in ('input','inout'):loads[net].append((n,pin,i))
 ffs={n:c for n,c in cells.items()if c['kind'].startswith('DFF')}
 age={}
 for n,c in ffs.items():
  m=re.fullmatch(r'dut/age_ms_(\d+)_s\d+',n)
  if m:
   bit=int(m[1]);assert bit not in age
   q,=c['ports']['Q'];assert drivers[q]==[(n,'Q',0)]
   age[bit]=dict(bit=bit,register=c,parameters=params(c),actual_Q=q)
 assert set(age)==set(range(32))
 outputs={}
 for n,c in ffs.items():
  m=re.fullmatch(r'dut/out_(forward|yaw)_q12_(\d+)_s\d+',n)
  if m:
   key=m[1],int(m[2]);assert key not in outputs;outputs[key]=c
 assert set(outputs)=={(axis,i)for axis in ('forward','yaw')for i in range(24)}
 return dict(modules=modules,cells=cells,params=params,directions=directions,drivers=drivers,loads=loads,ffs=ffs,age=age,outputs=outputs)

def analysis(s,root=None):
 cells,params,directions,drivers=s['cells'],s['params'],s['directions'],s['drivers']
 age_Q={v['actual_Q']:i for i,v in s['age'].items()}
 @lru_cache(None)
 def cone(net):
  ds=drivers.get(net,[])
  if not ds:return frozenset(),frozenset([net]),0
  assert len(ds)==1,('nonunique used net driver',net,ds)
  name,pin,index=ds[0];c=cells[name]
  if c['kind'].startswith('DFF'):return frozenset(),frozenset([net]),0
  children=[cone(v)for p,ns in c['ports'].items()if directions[c['kind']][p] in ('input','inout')for v in ns if v not in ('GND','VCC')]
  return frozenset([name]).union(*(x[0]for x in children)),frozenset().union(*(x[1]for x in children)),1+max((x[2]for x in children),default=0)
 @lru_cache(None)
 def pure_and(net):
  if net in age_Q:return frozenset([age_Q[net]]),0,frozenset()
  if net=='VCC':return frozenset(),0,frozenset()
  ds=drivers.get(net,[])
  if len(ds)!=1:return None
  name,pin,index=ds[0];c=cells[name]
  if not re.fullmatch(r'LUT[1-6]',c['kind'])or pin!='F':return None
  width=int(c['kind'][3:])
  if literal(params(c).get('INIT','0'))!=1<<((1<<width)-1):return None
  values=[pure_and(c['ports']['I'+str(i)][0])for i in range(width)]
  if any(x is None for x in values):return None
  return frozenset().union(*(x[0]for x in values)),1+max(x[1]for x in values),frozenset([name]).union(*(x[2]for x in values))
 @lru_cache(None)
 def root_paths(net):
  if root is not None and net==root:return dict(minimum=0,maximum=0,path=[])
  ds=drivers.get(net,[])
  if len(ds)!=1:return None
  name,pin,index=ds[0];c=cells[name]
  if c['kind'].startswith('DFF'):return None
  paths=[(p,i,v,root_paths(v))for p,ns in c['ports'].items()if directions[c['kind']][p]in('input','inout')for i,v in enumerate(ns)]
  found=[v for v in paths if v[3]is not None]
  if not found:return None
  chosen=max(found,key=lambda v:v[3]['maximum'])
  return dict(minimum=1+min(v[3]['minimum']for v in found),maximum=1+chosen[3]['maximum'],path=chosen[3]['path']+[dict(instance=name,kind=c['kind'],output_pin=pin,input_pin=chosen[0],input_index=chosen[1],parameters=params(c))])
 output_cone=set();endpoints=[];pure_rows=[]
 for(axis,bit),c in sorted(s['outputs'].items()):
  for pin in ('D','CE'):
   if pin not in c['ports']:continue
   net,=c['ports'][pin];cn,leaves,levels=cone(net);output_cone.update(cn)
   path=root_paths(net)if root is not None else None
   endpoints.append(dict(axis=axis,bit=bit,register=c['name'],pin=pin,net=net,combinational_primitive_depth=levels,root_reachable=path is not None,root_to_endpoint=path))
 for name,c in cells.items():
  if not re.fullmatch(r'LUT[1-6]',c['kind']):continue
  result=pure_and(c['ports']['F'][0])
  if result is not None and result[0]:
   pure_rows.append(dict(instance=name,kind=c['kind'],actual_ports=c['ports'],parameters=params(c),age_bit_support=sorted(result[0]),LUT_levels=result[1],in_output_D_or_CE_cones=name in output_cone))
 in_outputs=[r for r in pure_rows if r['in_output_D_or_CE_cones']]
 return dict(endpoints=endpoints,pure_age_all1_AND_nodes=pure_rows,pure_age_all1_AND_nodes_in_output_cones=in_outputs,maximum_pure_age_AND_depth_in_output_cones=max((r['LUT_levels']for r in in_outputs),default=0),maximum_support_pure_age_AND_nodes_in_output_cones=[r for r in in_outputs if len(r['age_bit_support'])==max((len(x['age_bit_support'])for x in in_outputs),default=0)],output_combinational_instances=sorted(output_cone)),cone,pure_and

def run(base,out):
 vg=base/'bounded_tree/impl/gwsynthesis/g2_top.vg'
 refvg=base/'reference/impl/gwsynthesis/g2_top.vg'
 job=base/'job.json';jobstate=json.loads(job.read_text(encoding='utf8'))
 assert jobstate['phase']=='small_pair_complete_actual_topology_and_equivalence_review_pending'and jobstate['tool_runtime']['exit_code']==0
 bindings=[meta(p)for p in(vg,refvg,base/'bounded_tree/guard.v',base/'bounded_tree/top.v',LIB)]
 s=snapshot(vg);ref=snapshot(refvg)
 cells,params=s['cells'],s['params']
 prefix='dut/g_deadline_nonzero.'
 expected=[prefix+f'g_max4[{i}].u_and'for i in range(8)]+[prefix+'u_low16',prefix+'u_high16',prefix+'u_nonmax32']
 assert all(n in cells for n in expected),('explicit LUT missing',sorted(set(expected)-cells.keys()))
 assert len([n for n in cells if n.startswith(prefix)])==11
 tree=[];groups=[]
 for i in range(8):
  c=cells[expected[i]]
  assert c['kind']=='LUT4'and literal(params(c)['INIT'])==0x8000
  want=[s['age'][i*4+j]['actual_Q']for j in range(4)]
  assert [c['ports']['I'+str(j)][0]for j in range(4)]==want,(c,want)
  q,=c['ports']['F'];groups.append(q)
  tree.append(dict(level=1,age_bit_support=list(range(i*4,i*4+4)),actual_cell=c,parameters=params(c),direct_actual_age_FF_Q_inputs=True))
 for j,stem in enumerate(('u_low16','u_high16')):
  c=cells[prefix+stem]
  assert c['kind']=='LUT4'and literal(params(c)['INIT'])==0x8000
  assert [c['ports']['I'+str(i)][0]for i in range(4)]==groups[j*4:j*4+4]
  tree.append(dict(level=2,age_bit_support=list(range(j*16,j*16+16)),actual_cell=c,parameters=params(c)))
 c=cells[prefix+'u_nonmax32'];root,=c['ports']['F']
 assert c['kind']=='LUT2'and literal(params(c)['INIT'])==7
 assert c['ports']['I0']==cells[prefix+'u_low16']['ports']['F']and c['ports']['I1']==cells[prefix+'u_high16']['ports']['F']
 tree.append(dict(level=3,age_bit_support=list(range(32)),actual_cell=c,parameters=params(c),function='NOT(AND(age_ms[31:0]))'))
 topology,cone,pure=analysis(s,root)
 reference_topology,_,_=analysis(ref)
 states={net:z3.Bool('actual_state:'+n)for n,ff in s['ffs'].items()for net in ff['ports']['Q']}
 logic=Logic(cells,params,s['directions'],states,{})
 want=z3.Not(z3.And(*[states[s['age'][i]['actual_Q']]for i in range(32)]))
 delta=z3.simplify(z3.Xor(logic.expr(root),want))
 root_identity_method='canonical_Boolean_identity'
 if not z3.is_false(delta):
  solver=z3.Solver();solver.set(timeout=10000);solver.add(delta)
  status=solver.check()
  assert status==z3.unsat,('actual root NAND32 not Boolean-identical',str(status))
  root_identity_method='SAT_unsatisfiable_actual_root_NAND32_miter'
 cut=Logic(cells,params,s['directions'],states,{})
 cutroot=z3.Bool('actual_nonmax_root_cut');cut.memo[root]=cutroot
 sensitive={};checks=[]
 for row in topology['endpoints']:
  if not row['root_reachable']:continue
  net=row['net']
  if net not in sensitive:
   expr=cut.expr(net)
   difference=z3.simplify(z3.Xor(z3.substitute(expr,(cutroot,z3.BoolVal(False))),z3.substitute(expr,(cutroot,z3.BoolVal(True)))))
   solver=z3.Solver();solver.set(timeout=10000);solver.add(difference);result=solver.check()
   assert result!=z3.unknown,('root sensitization timeout',net,solver.reason_unknown())
   sensitive[net]=dict(sensitizable=result==z3.sat,method='Boolean derivative with actual root wire cut; all actual FF Qs and primary inputs free',status=str(result))
   checks.append(dict(net=net,**sensitive[net]))
  row['root_functionally_sensitizable']=sensitive[net]['sensitizable']
 counts={axis:{pin:sum(r['root_reachable']for r in topology['endpoints']if r['axis']==axis and r['pin']==pin)for pin in ('D','CE')}for axis in ('forward','yaw')}
 functional_counts={axis:{pin:sum(r.get('root_functionally_sensitizable',False)for r in topology['endpoints']if r['axis']==axis and r['pin']==pin)for pin in ('D','CE')}for axis in ('forward','yaw')}
 assert all(sum(counts[a].values())>0 and sum(functional_counts[a].values())>0 for a in ('forward','yaw'))
 assert all(meta(Path(r['path']))==r for r in bindings)
 result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),scope='Actual small guard structural topology and root-wire Boolean influence only; not full G2 mapping, routed timing or whole sequential equivalence',candidate_VG=str(vg),reference_VG=str(refvg),exact_explicit_LUT_count=11,actual_age_FFs=s['age'],tree=tree,all_eight_4bit_groups_directly_connected_to_real_age_FF_Qs=True,exact_MAX_tree_LUT_depth_from_every_age_FF_to_nonmax_root=3,root_net=root,actual_root_NAND32_Boolean_identity=True,actual_root_identity_method=root_identity_method,root_immediate_actual_loads=s['loads'][root],root_output_D_CE_reachability_counts=counts,root_output_D_CE_functional_sensitivity_counts=functional_counts,root_cut_sensitivity_checks=checks,candidate_topology=topology,reference_topology=reference_topology,old_12level_pure_age_AND_prefix_in_output_decision_cones=any(r['LUT_levels']>=12 and len(r['age_bit_support'])>=31 for r in topology['pure_age_all1_AND_nodes_in_output_cones']),attributes_used_as_success_evidence=False,source_or_tool_mutation=False,physical_benefit_claimed=False,metadata_bindings=bindings)
 save(out,result)
 return result

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--base',type=Path,default=DEFAULT);p.add_argument('--out',type=Path,default=DEFAULT/'actual_tree_topology.json');a=p.parse_args()
 try:
  r=run(a.base,a.out)
  print(json.dumps(dict(passed=True,result=str(a.out),tree_LUTs=r['exact_explicit_LUT_count'],tree_depth=r['exact_MAX_tree_LUT_depth_from_every_age_FF_to_nonmax_root'],root=r['root_net'],root_output_reachable=r['root_output_D_CE_reachability_counts'],root_output_sensitive=r['root_output_D_CE_functional_sensitivity_counts'],candidate_max_pure_age_AND_in_output=r['candidate_topology']['maximum_pure_age_AND_depth_in_output_cones'],reference_max_pure_age_AND_in_output=r['reference_topology']['maximum_pure_age_AND_depth_in_output_cones'],long_old_prefix_still_present=r['old_12level_pure_age_AND_prefix_in_output_decision_cones']),indent=2))
 except Exception as e:
  save(a.out.with_name(a.out.stem+'_failed.json'),dict(passed=False,error=str(e),traceback=traceback.format_exc(),recorded_at=datetime.now().astimezone().isoformat()))
  raise
