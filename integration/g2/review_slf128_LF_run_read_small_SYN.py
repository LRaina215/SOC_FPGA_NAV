"""Actual C LF row packing, array enables and finite scan/drain control trace."""
from pathlib import Path
from datetime import datetime
from collections import Counter
import argparse,json,re,traceback
import psutil
from prepare_baseline_fusion_retime import load,save,metadata
from prepare_slf128_LF_run_read_small_SYN import unchanged
from synthesis_connectivity import MODULE,read
from audit_netlist_memories import BSRAM
from audit_routing_topology import primitive_directions
from review_stage3_slf128_memories import Tree,LIBRARY
from prove_soc_console_remap import Logic,literal,z3

def sequence(expressions,transitions,initial_values,input_values):
 nodes=[];ids={}
 def add(e):
  key=e.get_id()
  if key in ids:return ids[key]
  child=[add(x)for x in e.children()];kind=e.decl().kind()
  if z3.is_true(e):op=('const',True)
  elif z3.is_false(e):op=('const',False)
  elif not child:op=('var',str(e))
  else:
   kinds={z3.Z3_OP_NOT:'not',z3.Z3_OP_AND:'and',z3.Z3_OP_OR:'or',z3.Z3_OP_XOR:'xor',z3.Z3_OP_ITE:'if',z3.Z3_OP_EQ:'eq',z3.Z3_OP_DISTINCT:'neq'}
   assert kind in kinds,('unsupported Boolean node',str(e));op=(kinds[kind],child)
  ids[key]=len(nodes);nodes.append(op);return ids[key]
 ei={n:add(e)for n,e in expressions.items()};ti={q:add(e)for q,e in transitions.items()};state=dict(initial_values)
 for edge in range(18000):
  state.update(input_values(edge));values=[]
  for op,arg in nodes:
   if op=='const':v=arg
   elif op=='var':v=state[arg]
   elif op=='not':v=not values[arg[0]]
   elif op=='and':v=all(values[i]for i in arg)
   elif op=='or':v=any(values[i]for i in arg)
   elif op=='xor':v=sum(values[i]for i in arg)%2==1
   elif op=='if':v=values[arg[1]]if values[arg[0]]else values[arg[2]]
   elif op=='eq':v=all(values[i]==values[arg[0]]for i in arg[1:])
   elif op=='neq':v=len(set(values[i]for i in arg))==len(arg)
   else:raise AssertionError(op)
   values.append(v)
  yield edge,{n:values[i]for n,i in ei.items()}
  state.update({f'actual_FF:{q}':values[i]for q,i in ti.items()})

def review(build,out):
 build=build.resolve();out=out.resolve();assert not out.exists();out.mkdir(parents=True)
 result=dict(passed=False,build=str(build),scope='Actual mapped port Boolean proof and finite complete scan/drain; no RAM-data simulation or physical signoff',PNR_started=False)
 try:
  tool=load(build/'tool_result.json');assert tool['tool_passed'];unchanged(load(build/'frozen_inputs.json')['inputs'])
  vg=build/'impl/gwsynthesis/g2_top.vg';text=vg.read_text(encoding='utf8');assert 'Device Version: C'in text[:600]
  fullmods=dict(MODULE.findall(text));assert 'slf_lf'in fullmods
  raw='module g2_top ('+text.split('module slf_lf (',1)[1].split('endmodule',1)[0]+'endmodule\n'
  local=out/'verbatim_LF_parser_view.v';local.write_text(raw,encoding='utf8')
  tree=Tree(local);cells=read(local);directions=primitive_directions(LIBRARY)
  qcells={q:c for c in cells.values()if c['kind'].startswith('DFF')for q in c['ports']['Q']};qs={q:z3.Bool('actual_FF:'+q)for q in qcells}
  supported=lambda k:bool(re.fullmatch(r'LUT[1-6]',k))or k.startswith('MUX2_LUT')or k in('ALU','INV','IBUF','BUFG','VCC','GND')or k.startswith('DFF')
  opaque={(n,p,i):z3.Bool(f'actual_opaque:{n}:{p}:{i}')for n,c in cells.items()if not supported(c['kind'])for p,ns in c['ports'].items()if directions[c['kind']].get(p)=='output'for i in range(len(ns))}
  logic=Logic(cells,lambda c:tree.params[c['name']],directions,qs,opaque)
  rows=sorted((c for c in cells.values()if c['kind']in BSRAM),key=lambda c:c['name']);assert len(rows)==3 and all(c['kind']=='SDPB'for c in rows)
  assert [c['name']for c in rows]==['lb1_lb1_0_0_s','lb1_lb1_0_1_s','lb1_lb1_0_2_s']
  bits=lambda n,h,l:[f'{n}[{i}]'for i in range(h,l-1,-1)]
  expected_di=[bits('l1',12,0)+['q_occ']+bits('sm_q_d',13,11)+['VCC']+bits('sm_q_d',10,8)+['VCC']+bits('sm_q_d',17,8),
   bits('l3',6,0)+bits('l2',18,0)+bits('l1',18,13),['GND']*20+bits('l3',18,7)]
  assert all(c['ports']['DI']==expected_di[i]for i,c in enumerate(rows)),'actual input concatenation differs'
  assert rows[0]['ports']['DO']==bits('l2',12,0)+bits('l1',18,0)
  assert rows[1]['ports']['DO'][7:]==bits('l3',18,0)+bits('l2',18,13)
  assert rows[2]['ports']['DO'][20:29]==bits('l4',18,10)
  sinks={}
  for c in cells.values():
   for pin,ns in c['ports'].items():
    if directions[c['kind']][pin]=='input':
     for n in ns:sinks.setdefault(n,[]).append((c['name'],pin))
  padding=rows[1]['ports']['DO'][:7]+rows[2]['ports']['DO'][:20]+rows[2]['ports']['DO'][29:]
  assert not any(sinks.get(n)for n in padding),'unexpected live padding output'
  expressions={};gate_proofs=[]
  for c in rows:
   p=tree.params[c['name']];assert literal(p['BIT_WIDTH_0'])==literal(p['BIT_WIDTH_1'])==32
   assert literal(p['READ_MODE'])==0 and p['RESET_MODE']=='"SYNC"'
   assert c['ports']['CLKA']==c['ports']['CLKB']==['clk50_d']
   assert c['ports']['ADA']==['GND']+bits('c1',7,0)+['GND']+['VCC']*4
   assert c['ports']['ADB']==['GND']+bits('c',7,0)+['GND']*5
   def active(port,parameter,enable):
    select=literal(p[parameter]);ns=c['ports'][port]
    return z3.And(logic.expr(c['ports'][enable][0]),*[logic.expr(n)==z3.BoolVal(bool(select>>(2-i)&1))for i,n in enumerate(ns)])
   wa=active('BLKSELA','BLK_SEL_0','CEA');ra=active('BLKSELB','BLK_SEL_1','CEB')
   for label,actual,expected in [('array_read',ra,z3.And(z3.Not(logic.expr('rst_d')),logic.expr('run'))),
    ('array_write',wa,z3.And(z3.Not(logic.expr('rst_d')),logic.expr('v1'),z3.Not(logic.expr('c1[8]'))))]:
    s=z3.Solver();s.set(timeout=10000);s.add(z3.Xor(actual,expected));status=str(s.check());assert status=='unsat',(c['name'],label,status)
    gate_proofs.append(dict(primitive=c['name'],port=label,all_state_boolean_equivalence=status,expression=str(z3.simplify(actual))))
   expressions[c['name']+':write']=wa;expressions[c['name']+':read']=ra
   for port in ('ADA','ADB'):
    for i,n in enumerate(c['ports'][port][:9]):expressions[c['name']+':'+port+':'+str(13-i)]=logic.expr(n)
  for name in('run','v1','busy_d','done_d'):expressions[name]=logic.expr(name)
  for name in('c','c1','r'):
   for i in range(9):expressions[f'{name}[{i}]']=logic.expr(f'{name}[{i}]')
  variables=lambda e:{str(v)for v in z3.z3util.get_vars(e)}
  needed=set().union(*(variables(e)for e in expressions.values()));transitions={};done=set()
  while True:
   todo={n[len('actual_FF:'):]for n in needed if n.startswith('actual_FF:')}-done
   if not todo:break
   for q in sorted(todo):e=logic.next_state(qcells[q]);transitions[q]=e;needed.update(variables(e));done.add(q)
  extra=needed-{f'actual_FF:{q}'for q in done};assert extra<={'external:rst_d','external:start_d'},('control is not closed',sorted(extra))
  assert all(qcells[q]['ports']['CLK']==['clk50_d']for q in done)
  initial={f'actual_FF:{q}':bool(literal(tree.params[qcells[q]['name']].get('INIT',"1'b0")))for q in done}
  counts={c['name']:dict(read=0,write=0)for c in rows};collisions=[];tail=[];run_edges=0;final_write_reads=[]
  for edge,obs in sequence(expressions,transitions,initial,lambda edge:{'external:rst_d':edge<3,'external:start_d':edge==6}):
   word=lambda name,width:sum(int(obs[f'{name}[{i}]'])<<i for i in range(width))
   row=dict(edge=edge,run=obs['run'],v1=obs['v1'],busy=obs['busy_d'],done=obs['done_d'],r=word('r',9),c=word('c',9),c1=word('c1',9))
   run_edges+=int(row['run'])
   for c in rows:
    n=c['name'];we=obs[n+':write'];re_=obs[n+':read'];counts[n]['read']+=int(re_);counts[n]['write']+=int(we)
    address=lambda port:sum(int(obs[n+':'+port+':'+str(i)])<<(i-5)for i in range(5,14))
    wa,ra=address('ADA'),address('ADB')
    if we and re_ and wa==ra:collisions.append(dict(row,primitive=n,write_word=wa,read_word=ra,physical_18bit_chunks=[2*wa,2*wa+1]))
    if we:assert row['c1']<=129 and wa==row['c1']
    if re_:assert row['c']<=129 and ra==row['c']
    if not row['run']and row['v1']:final_write_reads.append(dict(row,primitive=n,array_write=we,array_read=re_,write_word=wa,read_word=ra))
   if edge>16895:tail.append(row)
   if edge>10 and row['done']:break
  assert run_edges==16900 and all(c==dict(read=16900,write=16900)for c in counts.values()),('incomplete scan',run_edges,counts)
  assert len(final_write_reads)==3 and all(r['array_write']and not r['array_read']for r in final_write_reads)
  assert not collisions and row['done']
  runtime=load(build/'small_SYN_runtime.json');ids=[dict(pid=tool['controller_pid'],created_epoch=tool['controller_created_epoch'],name='controller')]+runtime['processes']
  for identity in ids:
   try:identity['original_identity_alive']=abs(psutil.Process(identity['pid']).create_time()-identity['created_epoch'])<.01
   except psutil.NoSuchProcess:identity['original_identity_alive']=False
  assert not any(r['original_identity_alive']for r in ids)
  save(out/'actual_ports_and_control.json',dict(RAMs=[dict(c,parameters=tree.params[c['name']])for c in rows],gate_proofs=gate_proofs,
   control_FFs=[dict(q=q,cell=qcells[q],parameters=tree.params[qcells[q]['name']],next_state=str(transitions[q]))for q in sorted(done)],last_control_edges=tail,
   final_write_reads=final_write_reads,process_identities=ids,actual_data_packing_proved=True,unused_output_padding_bits=padding,
   actual_cells_with_parameters=[dict(c,parameters=tree.params[c['name']])for c in cells.values()],
   inputs=[metadata(p)for p in(vg,LIBRARY,Path(__file__).resolve(),build/'frozen_inputs.json')]))
  result.update(passed=True,recorded_at=datetime.now().astimezone().isoformat(),actual_VG=str(vg),actual_device_version='C',tool_exit_code=0,
   primitive_counts=dict(Counter(c['kind']for c in cells.values())),row_RAM_count=3,row_RAM_mode='SDPB32/32 READ_MODE0 RESET_MODE SYNC',
   data_concatenation_and_padding_checked=True,array_read_equivalent_not_rst_and_run=True,array_write_equivalent_not_rst_and_v1_and_not_c1bit8=True,
   BLKSEL_included_OCE_not_used_as_array_enable=True,gate_boolean_queries=6,all_queries_UNSAT=True,
   closed_actual_control_FFs=len(done),actual_scan_run_edges=run_edges,trace_edges=edge+1,row_access_counts=counts,
   final_drain_write_edge=final_write_reads,physical_chunk_collision_events=collisions,
   physical_chunk_rule='32bit word covers both18bit chunks {2*AD[13:5],2*AD[13:5]+1}; AD[4:1] are byte-write masks.',
   all_frozen_build_inputs_unchanged=True,all_recorded_original_identities_exited=True,details=str(out/'actual_ports_and_control.json'),
   RAM_data_simulated=False,physical_mapping_timing_electrical_or_full_G2_accepted=False)
 except Exception as error:result.update(error=str(error),traceback=traceback.format_exc())
 save(out/'result.json',result);return result

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--build',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
 result=review(a.build,a.out);print(json.dumps(result,ensure_ascii=False,indent=2));raise SystemExit(0 if result['passed']else 1)
