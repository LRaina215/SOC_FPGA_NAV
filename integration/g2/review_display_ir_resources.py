"""Read-only display/IR inventory for frozen SYN37/PNR35/PNR49.
Does not launch EDA tools, alter sources or touch SYN38/PNR50.
"""
from pathlib import Path
from collections import Counter,defaultdict,deque
from datetime import datetime
import argparse,json,re,traceback,gc,xml.etree.ElementTree as ET
from synthesis_connectivity import read,PORT,CELL,DECL
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from review_model25_joint_syn import module_bindings
from audit_netlist_memories import BSRAM
from audit_routing_topology import primitive_directions

BASE=Path('F:/nav_integration_g2_20261023')
LIB=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
BUILDS=[('SYN37','baseline_fusion_model25_bounded_MAX_syn37'),('PNR35','baseline_fusion32_CDC52_pnr35'),('PNR49','baseline_fusion_model25_NAV50_pnr49')]
SCOPES={'OLED':'u_control/display','IR':'u_control/remote_thread'}
VO_CELL=re.compile(r'^(\w+)\s+(\\\S+|[^\s(]+)\s*\(\s*\n(.*?)^\);',re.M|re.S)
RESET_PINS={'RESET','CLEAR','SET','PRESET'}
def load(p):return json.loads(Path(p).read_text(encoding='utf8'))
def meta(p):
 p=Path(p);s=p.stat();return dict(path=str(p),bytes=s.st_size,mtime_ns=s.st_mtime_ns)
def save(p,v):
 p=Path(p);assert not p.exists(),str(p)
 p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
def same(a,b):
 if a.stat().st_size!=b.stat().st_size:return False
 with a.open('rb')as x,b.open('rb')as y:
  while True:
   aa=x.read(1024*1024);bb=y.read(1024*1024)
   if aa!=bb:return False
   if not aa:return True
def under(n,p):return n==p or n.startswith(p+'/')
def classify(cells):
 counts=Counter(c['kind']for c in cells.values())
 return dict(leaf_cells=len(cells),FF=sum(v for k,v in counts.items()if k.startswith('DFF')),
  LUT=sum(v for k,v in counts.items()if re.fullmatch(r'LUT[1-6]',k)),
  MUX2_LUT=sum(v for k,v in counts.items()if k.startswith('MUX2_LUT')),
  ALU=counts['ALU'],BSRAM=sum(counts[k]for k in BSRAM),
  SSRAM_RAM16=sum(v for k,v in counts.items()if k.startswith('RAM16')),
  distributed_ROM16=sum(v for k,v in counts.items()if k.startswith('ROM16')),
  primitive_types=dict(counts))
def resources(path):
 def walk(node,parent=''):
  name=(parent+'/'+node.get('name')).strip('/');yield name,node.attrib
  for child in node:yield from walk(child,name)
 return dict(walk(ET.parse(path).getroot()))
def controls(cells,global_pins):
 bykind={}
 for label,pins in [('reset_set_clear',RESET_PINS),('CE',{'CE'})]:
  groups=defaultdict(list)
  for n,c in cells.items():
   if not c['kind'].startswith('DFF'):continue
   for pin in pins:
    if pin in c['ports']:
     vals=c['ports'][pin];assert len(vals)==1,(n,pin,vals)
     groups[vals[0]].append((n,pin))
  rows=[dict(net=net,constant=net in('VCC','GND'),scope_FF_pin_count=len(set(items)),
   scope_unique_FFs=len({n for n,p in items}),global_same_net_reset_or_CE_FF_pin_count=len(set(global_pins.get(net,[]))),
   scoped_FF_pins=[dict(instance=n,pin=p)for n,p in sorted(set(items))])for net,items in sorted(groups.items())]
  bykind[label]=dict(including_constants_unique_nets=len(rows),nonconstant_unique_nets=sum(not r['constant']for r in rows),
   total_scope_FF_pin_count=sum(r['scope_FF_pin_count']for r in rows),
   nonconstant_scope_FF_pin_count=sum(r['scope_FF_pin_count']for r in rows if not r['constant']),nets=rows)
 return bykind
def primitive_indexes(cells,directions):
 drivers=defaultdict(list);loads=defaultdict(list);gp=defaultdict(list)
 for n,c in cells.items():
  for pin,ns in c['ports'].items():
   for i,net in enumerate(ns):
    if c['kind'].startswith('DFF')and pin in RESET_PINS|{'CE'}:gp[net].append((n,pin))
    if net in('VCC','GND'):continue
    if directions[c['kind']][pin]=='output':drivers[net].append((n,pin,i))
    if directions[c['kind']][pin]in('input','inout'):loads[net].append((n,pin,i))
 return drivers,loads,gp
def first_state(net,cells,loads,directions,scope):
 queue=deque([(net,[])]);visited=set();endpoints={};terminals={}
 while queue:
  signal,path=queue.popleft()
  if signal in visited:continue
  visited.add(signal)
  for n,p,i in loads.get(signal,[]):
   c=cells[n]
   if under(n,scope):continue
   edge=dict(instance=n,kind=c['kind'],pin=p,index=i,net=signal)
   if c['kind'].startswith('DFF')or c['kind']in BSRAM or c['kind'].startswith(('RAM16','ROM16','MULT','DP')):
    endpoints[(n,p,i)]=dict(**edge,path=path+[edge]);continue
   outs=[v for pp,vs in c['ports'].items()if directions[c['kind']][pp]=='output'for v in vs if v not in('VCC','GND')]
   if not outs:terminals[(n,p,i)]=dict(**edge,path=path+[edge])
   for out in outs:
    if not loads.get(out):terminals[(n,p,i)]=dict(**edge,output_net=out,path=path+[edge])
    else:queue.append((out,path+[edge]))
 return dict(external_first_state_or_memory_pin_count=len(endpoints),endpoints=list(endpoints.values()),terminal_connections=list(terminals.values()),combinational_reachable_nets=len(visited))
def vo_selected(path,directions):
 selected={label:{}for label in SCOPES};gp=defaultdict(list)
 text=path.read_text(encoding='utf8')
 total=0
 for m in VO_CELL.finditer(text):
  kind,name,body=m.groups()
  if kind=='module':continue
  assert kind in directions,('unclassified post-PNR primitive',kind,name)
  name=name.lstrip('\\');ports={p.lstrip('\\'):v.strip().lstrip('\\').strip()for p,v in PORT.findall(body)}
  total+=1
  if kind.startswith('DFF'):
   for pin in RESET_PINS|{'CE'}:
    if pin in ports:
     assert not re.search(r'[,{}]',ports[pin]),('non-scalar FF control',name,pin,ports[pin])
     gp[ports[pin]].append((name,pin))
  for label,scope in SCOPES.items():
   if under(name,scope):selected[label][name]=dict(name=name,kind=kind,ports={p:[v]for p,v in ports.items()})
 return selected,gp,total

def source_evidence(root):
 files=['control4500_core.v','g2_top.v','control/ir_remote_control.v','control/oled.v','control/oled_driver.v','control/oled_front.v','control/motor520_closed_loop.v']
 result={}
 patterns=['parameter IR_ENABLE','parameter NAV_INPUT_ENABLE','parameter OLED_MODEL_DEBUG','wire command_emergency_stop','wire control_run','wire wheel_control_enable','wire signed [23:0] command_height_q12','.external_fault(command_emergency_stop','.enable(MODEL_ENABLE && command_run)','.ready(),.error()','latched until hardware reset','if(completed_frame[23:16]==8\'h1c)','if(!reset_n || !enable','fault_code<=0']
 for rel in files:
  p=root/rel
  if not p.exists():continue
  lines=p.read_text(encoding='utf-8-sig').splitlines()
  hits=[dict(line=i,text=t.strip())for i,t in enumerate(lines,1)if any(q in t for q in patterns)]
  result[rel]=dict(metadata=meta(p),source_lines=hits)
 return result
def run(out):
 out.mkdir(exist_ok=False)
 directions=primitive_directions(LIB);all_results={}
 for label,folder in BUILDS:
  print('inspect '+label,flush=True)
  root=BASE/folder;m=load(root/'manifest.json');cur=m;chain=[]
  while 'source_files'not in cur:
   if 'source'in cur:chain.append(cur['source'])
   cur=cur['source_manifest']
  source_files=[Path(n)for n in cur['source_files']]
  source_root=next(p.parent for p in source_files if p.name=='g2_top.v')
  vg=root/'impl/gwsynthesis/g2_top.vg'if label=='SYN37'else root/'g2_top.vg'
  source_vg=source_root/'impl/gwsynthesis/g2_top.vg'
  assert same(vg,source_vg),('PNR input VG differs frozen source',label)
  xml=source_root/'impl/gwsynthesis/g2_top_syn_rsc.xml'
  watch=[vg,xml]+[source_root/n for n in ('g2_top.v','control4500_core.v','control/ir_remote_control.v','control/oled.v','control/oled_driver.v')]
  vo=root/'impl/pnr/g2_top.vo'
  if label!='SYN37':watch+=[vo,root/'impl/pnr/g2_top.rpt.txt']
  bound=[meta(p)for p in watch]
  modules=definitions(vg);cells=read(vg);params=parameter_reader(modules)
  drivers,loads,gp=primitive_indexes(cells,directions)
  bindings=module_bindings(modules,list(SCOPES.values()))
  hr=resources(xml);rows={}
  for what,scope in SCOPES.items():
   selected={n:c for n,c in cells.items()if under(n,scope)}
   row=dict(actual_synthesis_leaf_counts=classify(selected),hierarchy_report=hr['g2_top/'+scope],
    hierarchy_report_file=str(xml),actual_control_pin_loads=controls(selected,gp),actual_module_binding=bindings[scope])
   decl=bindings[scope]['internal_declarations'];boundaries=[]
   for port,nets in bindings[scope]['ports'].items():
    direction=decl[port][0]
    if direction not in('output','inout'):continue
    for i,net in enumerate(nets):
     inside_drivers=[dict(instance=n,pin=p,index=j,cell=cells[n],parameters=params(cells[n]))for n,p,j in drivers.get(net,[])if under(n,scope)]
     if not inside_drivers:continue
     direct=[dict(instance=n,kind=cells[n]['kind'],pin=p,index=j,cell=cells[n],parameters=params(cells[n]))for n,p,j in loads.get(net,[])if not under(n,scope)]
     boundaries.append(dict(port=port,direction=direction,vector_index=i,net=net,actual_inside_drivers=inside_drivers,
      external_direct_input_loads=direct,forward_to_first_external_state=first_state(net,cells,loads,directions,scope)))
   row['actual_output_boundary_consumers']=boundaries
   row['actual_memory_leaves']=[dict(**c,parameters=params(c))for c in selected.values()if c['kind']in BSRAM or c['kind'].startswith(('RAM16','ROM16'))]
   # No reset estimate is taken from RTL variable widths.
   rows[what]=row
  if label!='SYN37':
   print('inspect post-PNR VO '+label,flush=True)
   sc,vgp,total=vo_selected(vo,directions)
   for what in SCOPES:
    rows[what]['actual_post_PNR_VO_leaf_counts']=classify(sc[what])
    rows[what]['actual_post_PNR_VO_control_pin_loads']=controls(sc[what],vgp)
   post=dict(file=str(vo),all_explicit_leaf_instances_parsed=total,scope='emitted post-PNR simulation netlist, including an unrouted implementation; not route/timing acceptance')
  else:post=None
  assert all(meta(Path(r['path']))==r for r in bound)
  report=dict(label=label,build=str(root),frozen_source_root=str(source_root),source_chain=chain,
   frozen_source_VG_byte_identical_to_reviewed_input=True,reviewed_VG=str(vg),modules=rows,
   source_binding=source_evidence(source_root),post_PNR_netlist=post,bindings_unchanged_at_end=True,metadata_bindings=bound)
  save(out/(label+'_actual_OLED_IR.json'),report)
  all_results[label]=dict(report_file=str(out/(label+'_actual_OLED_IR.json')),
   modules={w:dict(synthesis_leaf_counts=v['actual_synthesis_leaf_counts'],hierarchy=v['hierarchy_report'],
    post_PNR_leaf_counts=v.get('actual_post_PNR_VO_leaf_counts'),
    synthesis_control_pin_summary={k:{q:x[q]for q in ('including_constants_unique_nets','nonconstant_unique_nets','total_scope_FF_pin_count','nonconstant_scope_FF_pin_count')}for k,x in v['actual_control_pin_loads'].items()},
    boundary_outputs=len(v['actual_output_boundary_consumers']),
    first_external_state_pin_count=sum(b['forward_to_first_external_state']['external_first_state_or_memory_pin_count']for b in v['actual_output_boundary_consumers']))for w,v in rows.items()})
  del modules,cells,drivers,loads,gp,params,bindings,rows,report;gc.collect()
 summary=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),
  scope='Frozen source, synthesis hierarchy, actual synthesis/PNR leaves and structural I/O/control dependency review. No source/tool changes, SYN/PNR/simulation or timing acceptance.',
  reports=all_results,global_report_FF_savings_is_not_sum_of_source_array_bits=True,
  reset_CE_count_contract='Unique actual nets and unique (FF instance,pin) pairs; constants shown separately. Global net counts combine RESET/CLEAR/SET/PRESET/CE and do not infer synchronous reset logic hidden in D cones.',
  source_or_design_tool_mutated=False,PNR50_or_SYN38_accessed=False)
 save(out/'result.json',summary)
 print(json.dumps(summary,ensure_ascii=False),flush=True)
 return summary
def interpret(out):
 rows=[];control_rows=[];dependency={}
 for label,_ in BUILDS:
  data=load(out/(label+'_actual_OLED_IR.json'));dependency[label]={}
  for scope,item in data['modules'].items():
   pairs=[('SYN input VG',item['actual_synthesis_leaf_counts'],item['actual_control_pin_loads'])]
   if 'actual_post_PNR_VO_leaf_counts'in item:pairs.append(('post-PNR VO',item['actual_post_PNR_VO_leaf_counts'],item['actual_post_PNR_VO_control_pin_loads']))
   for stage,c,pins in pairs:
    inv=c['primitive_types'].get('INV',0)
    assert int(item['hierarchy_report']['T_Lut'].split('(')[0])==c['LUT']+inv
    rows.append(dict(build=label,scope=scope,stage=stage,FF=c['FF'],explicit_LUT=c['LUT'],INV=inv,LUT_including_INV=c['LUT']+inv,MUX2_LUT=c['MUX2_LUT'],ALU=c['ALU'],BSRAM=c['BSRAM'],SSRAM_RAM16=c['SSRAM_RAM16'],DSP={k:v for k,v in c['primitive_types'].items()if k.startswith('MULT')}))
    control_rows.append(dict(build=label,scope=scope,stage=stage,counts={k:{q:x[q]for q in ('including_constants_unique_nets','nonconstant_unique_nets','total_scope_FF_pin_count','nonconstant_scope_FF_pin_count')}for k,x in pins.items()},reset_nets=[dict(net=n['net'],scope_FF_pins=n['scope_FF_pin_count'])for n in pins['reset_set_clear']['nets']]))
   union={(e['instance'],e['pin'],e['index'])for b in item['actual_output_boundary_consumers']for e in b['forward_to_first_external_state']['endpoints']}
   dependency[label][scope]=dict(unique_external_first_state_or_memory_pins=len(union),count_scope='Structural graph reachability only; merged cones are deduplicated. Not functional sensitization or removable-resource count.',actual_boundary_outputs=[dict(port=b['port'],net=b['net'],direct_external_loads=[dict(instance=x['instance'],kind=x['kind'],pin=x['pin'])for x in b['external_direct_input_loads']])for b in item['actual_output_boundary_consumers']])
 save(out/'interpretation.json',dict(passed=True,resource_rows=rows,actual_unique_control_net_and_FF_pin_rows=control_rows,dependencies=dependency,
  LUT_accounting='For these exact frozen rows, report T_Lut equals explicit LUT1..6 plus INV; MUX2_LUT5/6 leaves are listed separately. No generic LUT-equivalence estimate is substituted.',
  OLED=dict(source_ready_error_unconnected=True,actual_outputs_only_two_IIC_IOBUF_OEN_inputs=True,control_state_feedback_found=False,framebuffer='Both actual SDPB leaves are gram_gram_0_0_s / _s0, width8; the 1024x8 source array is BSRAM, not8192FF.'),
  IR=dict(pure_display=False,latched_emergency_stop_key='NEC key1c sets until device reset; repeats do not toggle or clear.',command_mask='Actual ir_emergency_stop drives all48 forward/yaw mask LUT2 inputs; source command_emergency_stop.',model_external_fault_and_height=True,wheel_control_enable_and_fault_clear=True,lease_rearm='Indirect via external_fault -> model_ready -> odo_valid/fresh feedback. Actual have_lease CE cone is reachable; IR is not the SAFE ch_stop input or a NAV_MODE enum.',direction_key_caveat='NAV_INPUT_ENABLE selects NAV forward/yaw, but IR keys and hold remain physically in wheel-scaling cones. Source gain1 may make that arithmetic branch numerically identical; structural reachability is not functional sensitization.'),
  savings='Direct SYN37 scope contains634+154=788FF,2BSRAM,112ALU. Removing source features may further eliminate upstream-only logic, but no resynthesis was performed. No8000FF saving or route improvement is established.'))
 print('interpretation '+str(out/'interpretation.json'),flush=True)

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=BASE/'display_IR_resource_control_review1');p.add_argument('--summarize-existing',action='store_true');a=p.parse_args()
 try:
  if not a.summarize_existing:run(a.out)
  interpret(a.out)
 except Exception as e:
  if a.out.exists():save(a.out/'review_failed.json',dict(passed=False,error=str(e),traceback=traceback.format_exc()))
  raise
