"""Audit completed explicitMAX SYN37 against SYN34, including actual tree survival, consumption and mapped lease.

No synthesis/PnR, waveform simulation, source mutation or board action. Local
lease next-state miters cut at the same200 actual FF Qs, not source predictors.
"""
from pathlib import Path
from datetime import datetime
import argparse,collections,json,re,time,traceback,xml.etree.ElementTree as ET
from bs4 import BeautifulSoup
from synthesis_connectivity import read,CELL,PORT,DECL
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from audit_routing_topology import primitive_directions
from prove_soc_console_remap import Logic,literal,z3
from prepare_baseline_fusion_retime import metadata,save,same_bytes
from review_lease_bounded_tree_topology import snapshot,analysis
from audit_routing_topology import primitive_directions

LIB=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')

def report_paths(build):
 text=(build/'impl/gwsynthesis/g2_top_syn.rpt.html').read_text();result=[]
 for part in text.split('Path Summary:')[1:]:
  fields=[x.get_text(' ',strip=True)for x in BeautifulSoup(part.split('</table>',1)[0],'html.parser').find_all('td')]
  summary=dict(zip(fields[::2],fields[1::2]));rows=[]
  for tr in BeautifulSoup(part.split('Data Arrival Path:',1)[1].split('Data Required Path:',1)[0],'html.parser').find_all('tr'):
   row=[x.get_text(' ',strip=True)for x in tr.find_all('td')]
   if len(row)==6:rows.append(dict(zip(('arrival_ns','delay_ns','type','edge','fanout','node'),row)))
  result.append(dict(summary=summary,arrival_path=rows))
 assert result
 return result

def local_guard(modules,out,name):
 body=modules['command_lease_guard'];assert not re.search(r'^\s*assign\s',body,re.M)
 file=out/(name+'_actual_guard.v');file.write_text('module g2_top ('+body+'endmodule\n',encoding='utf-8')
 cells=read(file);params=parameter_reader({'g2_top':body})
 ffs={c['ports']['Q'][0]:c for c in cells.values()if c['kind'].startswith('DFF')}
 assert len(ffs)==sum(c['kind'].startswith('DFF')for c in cells.values())
 return body,cells,params,ffs

def pure_and(cells,params,root):
 drivers={c['ports']['F'][0]:c for c in cells.values()if re.fullmatch(r'LUT[1-6]',c['kind'])}
 used={};memo={}
 def walk(net):
  if re.fullmatch(r'age_ms\[\d+\]',net):return {net},0
  if net in memo:return memo[net]
  c=drivers.get(net)
  if c is None:return None
  width=int(c['kind'][3:])
  if literal(params(c)['INIT'])!=1<<((1<<width)-1):return None
  leaves=set();levels=0
  for i in range(width):
   value=walk(c['ports']['I'+str(i)][0])
   if value is None:return None
   leaves|=value[0];levels=max(levels,value[1])
  used[c['name']]=dict(**c,params=params(c));memo[net]=(leaves,levels+1);return memo[net]
 result=walk(root);assert result is not None,root
 return dict(root=root,leaves=sorted(result[0]),levels=result[1],actual_primitives=list(used.values()))

def hierarchy_resources(path):
 def walk(node,parent=''):
  name=(parent+'/'+node.get('name')).strip('/');yield name,node.attrib
  for child in node:yield from walk(child,name)
 return dict(walk(ET.parse(path).getroot()))

def actual_guard_wrapper(modules,out,label):
 body=modules['command_lease_guard']
 declarations=[m[0]for m in re.finditer(r'^\s*(?:input|output)\s+[^;]+;',body,re.M)]
 ports=[n.lstrip('\\')for d,h,l,n in DECL.findall(body)if d in ('input','output')]
 assert len(ports)==len(set(ports))and declarations
 connections=',\n'.join('.'+n+'('+n+')'for n in ports)
 text='module g2_top ('+','.join(ports)+');\n'+'\n'.join(declarations)+'\n  command_lease_guard dut (\n'+connections+'\n);\nendmodule\nmodule command_lease_guard ('+body+'endmodule\n'
 file=out/(label+'_actual_guard_wrapper.v');file.write_text(text,encoding='utf-8')
 return file


def explicit_tree(modules,oldmodules,out):
 file=actual_guard_wrapper(modules,out,'SYN37');reference=actual_guard_wrapper(oldmodules,out,'SYN34')
 s=snapshot(file);rs=snapshot(reference);cells,params=s['cells'],s['params'];prefix='dut/g_deadline_nonzero.'
 expected=[prefix+f'g_max4[{i}].u_and'for i in range(8)]+[prefix+'u_low16',prefix+'u_high16',prefix+'u_nonmax32']
 assert all(n in cells for n in expected),('explicit_tree_missing',sorted(set(expected)-cells.keys()))
 assert len([n for n in cells if n.startswith(prefix)])==11
 groups=[];tree=[]
 for i in range(8):
  c=cells[expected[i]];assert c['kind']=='LUT4'and literal(params(c)['INIT'])==0x8000
  want=[s['age'][i*4+j]['actual_Q']for j in range(4)]
  assert [c['ports']['I'+str(j)][0]for j in range(4)]==want
  groups.append(c['ports']['F'][0]);tree.append(dict(level=1,cell=c,parameters=params(c),age_bit_support=list(range(i*4,i*4+4))))
 for j,n in enumerate(('u_low16','u_high16')):
  c=cells[prefix+n];assert c['kind']=='LUT4'and literal(params(c)['INIT'])==0x8000
  assert [c['ports']['I'+str(i)][0]for i in range(4)]==groups[j*4:j*4+4]
  tree.append(dict(level=2,cell=c,parameters=params(c),age_bit_support=list(range(j*16,j*16+16))))
 c=cells[prefix+'u_nonmax32'];assert c['kind']=='LUT2'and literal(params(c)['INIT'])==7
 assert c['ports']['I0']==cells[prefix+'u_low16']['ports']['F']and c['ports']['I1']==cells[prefix+'u_high16']['ports']['F']
 root=c['ports']['F'][0];tree.append(dict(level=3,cell=c,parameters=params(c),age_bit_support=list(range(32))))
 nt,_,_=analysis(s,root);ot,_,_=analysis(rs)
 states={q:z3.Bool('actual_tree_FF:'+n)for n,ff in s['ffs'].items()for q in ff['ports']['Q']}
 logic=Logic(cells,params,s['directions'],states,{})
 solver=z3.Solver();solver.set(timeout=10000);solver.add(z3.Xor(logic.expr(root),z3.Not(z3.And(*[states[s['age'][i]['actual_Q']]for i in range(32)]))))
 assert solver.check()==z3.unsat
 cut=Logic(cells,params,s['directions'],states,{});rootvar=z3.Bool('actual_tree_root_cut');cut.memo[root]=rootvar
 sensitive={};counts={a:{p:0 for p in('D','CE')}for a in('forward','yaw')}
 for row in nt['endpoints']:
  if not row['root_reachable']:continue
  net=row['net']
  if net not in sensitive:
   expr=cut.expr(net);solver=z3.Solver();solver.set(timeout=10000)
   solver.add(z3.Xor(z3.substitute(expr,(rootvar,z3.BoolVal(False))),z3.substitute(expr,(rootvar,z3.BoolVal(True)))))
   result=solver.check();assert result!=z3.unknown;sensitive[net]=result==z3.sat
  row['root_functionally_sensitizable']=sensitive[net]
  counts[row['axis']][row['pin']]+=int(sensitive[net])
 assert all(sum(counts[a].values())>0 for a in('forward','yaw'))
 assert not any(r['LUT_levels']>3 and len(r['age_bit_support'])>=24 for r in nt['pure_age_all1_AND_nodes_in_output_cones'])
 result=dict(passed=True,actual_explicit_LUT_count=11,actual_age_Q_to_nonmax_LUT_levels=3,actual_tree=tree,actual_NAND32_miter_unsat=True,
  old_fullG2_topology=ot,new_fullG2_topology=nt,actual_root=root,actual_root_loads=s['loads'][root],output_D_CE_functional_sensitivity=counts,
  old_prefix_removed_from_output_decision_cones=True,source_attributes_not_used_as_success_evidence=True,physical_timing_claimed=False)
 save(out/'actual_fullG2_tree_topology.json',result);return result


def run(old,new,prepared,out):
 started=time.monotonic();out.mkdir(parents=True,exist_ok=False)
 def stage(label,**kw):save(out/'progress.json',dict(phase=label,updated_at=datetime.now().astimezone().isoformat(),**kw))
 try:
  stage('bind_completed_SYN35_and_actual_selected_source')
  job=json.loads((new/'job.json').read_text());assert job['phase']=='synthesis_complete'and job['tool_exit']==0
  h=lambda root:{Path(p).relative_to(root).as_posix():Path(p)for p in json.loads((root/'manifest.json').read_text())['source_files']}
  a,b,p=h(old),h(new),h(prepared);assert a.keys()==b.keys()==p.keys()and len(b)==66
  changed=[n for n in a if not same_bytes(a[n],b[n])];assert changed==['command_lease_guard.v']
  assert all(same_bytes(b[n],p[n])for n in b)
  assert same_bytes(new/'command_lease_guard.v',Path('F:/nav_integration_g2_20261023/lease_deadline_bounded_tree_candidate1/command_lease_guard.v'))
  source_bindings=[metadata(q)for q in b.values()]
  frozen_sets={}
  for suffixes,label,count in[({'.hex','.mem','.bin','.mif'},'assets',69),({'.h','.vh'},'headers',31)]:
   orig={q.relative_to(old):q for q in old.rglob('*')if q.is_file()and'impl'not in q.relative_to(old).parts and'__pycache__'not in q.parts and q.suffix.lower()in suffixes}
   assert len(orig)==count and all(same_bytes(q,new/n)for n,q in orig.items());frozen_sets[label]=count
  fw={}
  for folder in('fw','firmware'):
   orig={q.relative_to(old):q for q in(old/folder).rglob('*')if q.is_file()and'__pycache__'not in q.parts}
   assert all(same_bytes(q,new/n)for n,q in orig.items());fw[folder]=len(orig)
  for name in('g2.cst','g2.sdc'):assert same_bytes(old/name,new/name)
  new_tcl=(new/'run.tcl').read_text();selected=re.findall(r'^add_file -type verilog \{(.+)\}$',new_tcl,re.M)
  assert len(selected)==66 and {Path(q).resolve()for q in selected}=={q.resolve()for q in b.values()}
  assert 'baseline_fusion_model25_NAV50_prepared4'not in new_tcl and 'baseline_fusion_model25_bounded_MAX_prepared1'not in new_tcl
  stage('parse_actual_module_resource_and_lease_boundaries')
  av=old/'impl/gwsynthesis/g2_top.vg';bv=new/'impl/gwsynthesis/g2_top.vg';am,bm=definitions(av),definitions(bv)
  assert am.keys()==bm.keys();module_changes=sorted(n for n in am if am[n]!=bm[n])
  ab,ac,ap,aq=local_guard(am,out,'SYN34');bb,bc,bp,bq=local_guard(bm,out,'SYN37')
  assert aq.keys()==bq.keys()and len(aq)==200
  directions=primitive_directions(LIB);state={net:z3.Bool('actual_lease_state:'+net)for net in aq}
  la=Logic(ac,ap,directions,state,{});lb=Logic(bc,bp,directions,state,{})
  checks=[];sat_calls=0
  def equal(label,x,y):
   nonlocal sat_calls
   delta=z3.simplify(z3.Xor(x,y))
   if z3.is_false(delta):checks.append(dict(label=label,method='canonical_Boolean_identity'));return
   solver=z3.Solver();solver.set(timeout=30000);solver.add(delta);sat_calls+=1;result=solver.check()
   assert result==z3.unsat,(label,str(result),str(solver.model())if result==z3.sat else solver.reason_unknown())
   checks.append(dict(label=label,method='actual_mapped_next_state_miter',status=str(result)))
  for i,net in enumerate(sorted(aq)):
   x,y=aq[net],bq[net];assert x['kind']==y['kind']and ap(x)==bp(y),(net,x['kind'],y['kind'])
   assert x['ports']['CLK']==y['ports']['CLK']==['clk50_d']
   equal(net+'/next',la.next_state(x),lb.next_state(y))
   if i%25==0:stage('prove_actual_lease_FF_next_states',proved=i+1,total=200)
  # Public combinational outputs may include balance/constant and merged FF
  # aliases. Port directions and exact bit names must agree before comparison.
  outputs=lambda body:{n.lstrip('\\'):(hi,lo)for d,hi,lo,n in DECL.findall(body)if d=='output'}
  ao,bo=outputs(ab),outputs(bb);assert ao==bo,(ao,bo)
  output_bits=[]
  for name,(hi,lo)in ao.items():
   bits=[name]if not hi else[name+'['+str(i)+']'for i in range(int(hi),int(lo)-1,-1)]
   for net in bits:equal(net+'/public_output',la.expr(net),lb.expr(net));output_bits+=bits
  save(out/'actual_lease_next_state_proof.json',dict(passed=True,actual_shared_FFs=200,actual_clocks_same50MHz=True,
   actual_FF_kinds_all_INIT_and_other_parameters_same=True,public_output_bit_checks=len(output_bits),sat_calls=sat_calls,
   checks=checks,scope='Actual mapped lease combinational next-state/public output equivalence at every shared FF prestate and every external input; not fullG2 numeric or physical timing proof.'))
  # Recover the actual reduction structure, instead of assuming the source
  # comparison was mapped as a balanced tree.
  old_prefix=pure_and(ac,ap,'age_ms_31_13');assert old_prefix['levels']==12 and set(old_prefix['leaves'])=={'age_ms['+str(i)+']'for i in range(32)}
  actual_tree=explicit_tree(bm,am,out)
  old_paths,new_paths=report_paths(old),report_paths(new)
  old_r,new_r=hierarchy_resources(old/'impl/gwsynthesis/g2_top_syn_rsc.xml'),hierarchy_resources(new/'impl/gwsynthesis/g2_top_syn_rsc.xml')
  assert old_r.keys()==new_r.keys()
  resource_changes=[dict(hierarchy=n,changed_fields={k:[old_r[n].get(k),new_r[n].get(k)]for k in old_r[n].keys()|new_r[n].keys()if old_r[n].get(k)!=new_r[n].get(k)})for n in old_r if old_r[n]!=new_r[n]]
  result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),elapsed_s=time.monotonic()-started,
   actual_old=str(old),actual_new=str(new),actual_prepared_input=str(prepared),prior_small_tree_used_only_for_byte_identical_semantic_proof=True,
   selected_RTL_delta=changed,identical_other_selected_RTL=65,identical_assets_headers=frozen_sets,identical_complete_original_FW=fw,
   actual_completed_SYN_exit=job['tool_exit'],actual_VG_modules_before=len(am),actual_VG_modules_after=len(bm),
   actual_definition_text_changes=module_changes,all_other_module_netlists_byte_identical=False,
   module_change_scope='Global synthesis renamed some ports with_Z and remapped local equivalent logic, including ROM decoding; source identity/resource similarity is not full mapped equivalence proof outside lease.',
   global_resources_before=old_r['g2_top'],global_resources_after=new_r['g2_top'],
   lease_resources_before=old_r['g2_top/u_command_lease'],lease_resources_after=new_r['g2_top/u_command_lease'],
   all_hierarchy_resource_changes=resource_changes,
   old12level_all32_AND=old_prefix,actual_explicit_tree=actual_tree,
   old_reported_paths=old_paths,new_reported_paths=new_paths,
   actual_lease_next_state_proof=str(out/'actual_lease_next_state_proof.json'),
   RAM_clock_CDC_review='Separate actual memory/PLL/three-clock/radar26 reviewer; no same-module-text shortcut is used.',
   mapping_goal_achieved=True,actual_new_reported_negative_paths=[r for r in new_paths if float(r['summary']['Slack'])<0],complete_negative_path_coverage_verified=False,
   recommendation='Compare every actually listed new negative estimate and complete memory/clock/CDC review before deciding any PNR. Explicit3level mapping alone does not prove overall timing benefit or identify any previous routefailure cause.',
   actual_routed_setup_hold_or_benefit_accepted=False,new_PNR_started=False,programmed=False,
   source_bindings_unchanged_at_end=all(metadata(Path(row['path']))==row for row in source_bindings),
   bindings=[metadata(av),metadata(bv),metadata(new/'impl/gwsynthesis/g2_top_syn.rpt.html')])
  save(out/'final_review.json',result);stage('actual_fullG2_lease_tree_mapping_and_equivalence_review_complete_not_timing_accepted',passed=True,mapping_goal_achieved=True);return result
 except Exception as exc:
  save(out/'review_failed.json',dict(passed=False,reason=str(exc),traceback=traceback.format_exc(),recorded_at=datetime.now().astimezone().isoformat()));raise

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--old',type=Path,required=True);p.add_argument('--new',type=Path,required=True);p.add_argument('--prepared',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
 result=run(a.old,a.new,a.prepared,a.out);print(json.dumps(dict(passed=result['passed'],final_review=str(a.out/'final_review.json'),mapping_goal_achieved=result['mapping_goal_achieved'],old_first_path=result['old_reported_paths'][0]['summary'],new_first_path=result['new_reported_paths'][0]['summary']),indent=2))
