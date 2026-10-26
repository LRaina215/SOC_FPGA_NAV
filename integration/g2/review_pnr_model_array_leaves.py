"""Inventory four real model-storage groups in PNR35/PNR49 emitted VO, serially.
No source/constraint/tool/process mutations and no EDA tool invocation.
"""
from pathlib import Path
from collections import defaultdict,Counter
from datetime import datetime
import argparse,json,re,gc,traceback
from review_display_ir_resources import PORT,RESET_PINS,LIB,BASE,meta,save
from audit_netlist_memories import PARAM
from audit_routing_topology import primitive_directions
from prove_soc_console_remap import literal

BUILDS=[('PNR35','baseline_fusion32_CDC52_pnr35','u_control/model_thread'),('PNR49','baseline_fusion_model25_NAV50_pnr49','u_control/model_thread/model')]
REL={'history_flat':'observation/history_flat','current_flat':'observation/current_flat','snapshot':'policy/snapshot','input_mem':'policy/core/input_mem'}
# RAM16 leaves close as .DO(...)); on one line; do not require a new-line );.
VO_CELL=re.compile(r'^(\w+)\s+(\\\S+|[^\s(]+)\s*\(\s*\n(.*?)\)\s*;',re.M|re.S)
def control_table(cells,global_pins,pins):
 groups=defaultdict(set)
 for n,c in cells.items():
  for pin in pins:
   if pin not in c['ports']:continue
   net=c['ports'][pin];assert not re.search(r'[,{}]',net),(n,pin,net)
   groups[net].add((n,pin))
 rows=[]
 for net,items in sorted(groups.items()):
  global_same_class={(n,p)for n,p in global_pins.get(net,set())if p in pins}
  rows.append(dict(net=net,constant=net in('GND','VCC'),scope_unique_FF_pins=len(items),scope_unique_FFs=len({n for n,p in items}),
   global_same_control_class_FF_pins=len(global_same_class),global_same_control_class_unique_FFs=len({n for n,p in global_same_class}),
   global_all_reset_CE_FF_pins=len(global_pins.get(net,set())),actual_scope_pins=[dict(instance=n,pin=p)for n,p in sorted(items)]))
 return dict(unique_nets_including_constants=len(rows),nonconstant_unique_nets=sum(not r['constant']for r in rows),
  unique_FF_pins_including_constants=sum(r['scope_unique_FF_pins']for r in rows),
  nonconstant_unique_FF_pins=sum(r['scope_unique_FF_pins']for r in rows if not r['constant']),nets=rows)
def analyze(root,model,defaults,global_directions):
 vo=root/'impl/pnr/g2_top.vo';before=meta(vo)
 selected={k:{}for k in REL};gp=defaultdict(set);total=0;global_ff_count=0
 text=vo.read_text(encoding='utf8')
 for m in VO_CELL.finditer(text):
  kind,name,body=m.groups()
  if kind=='module':continue
  assert kind in global_directions,(kind,name)
  assert not re.search(r'^defparam\s|^(?:DFF\w*|RAM16\w*)\s',body,re.M),('primitive block absorbed following cells',kind,name)
  total+=1;name=name.lstrip('\\')
  if kind.startswith('DFF'):global_ff_count+=1
  match=[g for g,p in REL.items()if name.startswith(model+'/'+p)]
  if not kind.startswith('DFF')and not match:continue
  ports={p.lstrip('\\'):v.strip().lstrip('\\').strip()for p,v in PORT.findall(body)}
  if kind.startswith('DFF'):
   for pin in RESET_PINS|{'CE'}:
    if pin in ports:gp[ports[pin]].add((name,pin))
  for g in match:selected[g][name]=dict(name=name,kind=kind,ports=ports)
 params=defaultdict(dict);wanted={n for cs in selected.values()for n,c in cs.items()if c['kind'].startswith('DFF')or c['kind']=='RAM16SDP4'}
 for m in PARAM.finditer(text):
  inst,key,value=m.groups();inst=inst.strip().lstrip('\\')
  if inst in wanted:
   assert key not in params[inst]
   params[inst][key]=value.strip()
 groups={}
 for group,allcells in selected.items():
  prefix=model+'/'+REL[group];ffs={n:c for n,c in allcells.items()if c['kind'].startswith('DFF')}
  memories={n:c for n,c in allcells.items()if c['kind'].startswith('RAM16')}
  logical=defaultdict(list);unmatched=[]
  for n,c in ffs.items():
   stem=re.fullmatch(re.escape(prefix)+r'_(\d+)_s(\d+)(.*)',n)
   record=dict(**c,parameters=params[n])
   if stem:
    record['source_bit_index_from_instance']=int(stem[1]);record['synthesis_suffix']=stem[2]+stem[3]
    q=c['ports'].get('Q')
    expected=prefix+'['+stem[1]+']'
    record['actual_Q_matches_plain_logical_bit_name']=re.sub(r'\s+(?=\[)','',q or '')==expected
    logical[int(stem[1])].append(record)
   else:unmatched.append(record)
  duplicates=[]
  for index,items in sorted(logical.items()):
   if len(items)<2:continue
   baseline=items[0]
   same_inputs=all(x['kind']==baseline['kind']and x['parameters']==baseline['parameters']and {p:v for p,v in x['ports'].items()if p!='Q'}=={p:v for p,v in baseline['ports'].items()if p!='Q'}for x in items[1:])
   duplicates.append(dict(source_bit_index=index,actual_FF_instances=len(items),actual_Q_nets=[x['ports'].get('Q')for x in items],
    identical_kind_INIT_and_all_non_Q_pins=same_inputs,interpretation='Exact primitive-copy transition inputs'if same_inputs else'Name-index duplicate only; no equivalence assumed',cells=items))
  ram_rows=[]
  for n,c in memories.items():
   assert c['kind']=='RAM16SDP4',(group,n,c['kind'])
   effective=dict(defaults);effective.update(params[n])
   assert set(effective)=={'INIT_0','INIT_1','INIT_2','INIT_3'},(n,effective)
   assert all(0<=literal(v)<65536 for v in effective.values())
   ram_rows.append(dict(**c,explicit_parameters=params[n],effective_INIT_parameters=effective,address_bits=4,data_bits=4,words=16,capacity_bits=64))
  ram_controls={}
  for pin in ('CLK','WRE'):
   nets=defaultdict(list)
   for n,c in memories.items():
    net=c['ports'][pin];assert not re.search(r'[,{}]',net),(n,pin,net)
    nets[net].append(n)
   ram_controls[pin]=dict(unique_nets=len(nets),RAM_control_pins=sum(len(v)for v in nets.values()),nets=[dict(net=n,RAM_pins=len(ns),RAM_instances=ns,global_same_net_FF_reset_CE_pins=len(gp.get(n,set())))for n,ns in sorted(nets.items())])
  groups[group]=dict(prefix=prefix,actual_prefix_primitive_counts=dict(Counter(c['kind']for c in allcells.values())),actual_FF_count=len(ffs),
   logical_bit_indices_from_names=len(logical),minimum_named_bit=min(logical)if logical else None,maximum_named_bit=max(logical)if logical else None,
   duplicate_logical_bit_indices=len(duplicates),excess_FF_instances_above_named_unique_bits=sum(len(v)-1 for v in logical.values()),
   plain_logical_Q_name_matches=sum(r['actual_Q_matches_plain_logical_bit_name']for rows in logical.values()for r in rows),
   logical_index_mapping_scope='Instance-index grouping, with actual Q and other primitive pins retained; this is not a proof of source-bit/state equivalence.',
   duplicates=duplicates,unmatched_FF_names=unmatched,actual_FF_cells=[dict(**c,parameters=params[n])for n,c in ffs.items()],
   RESET_CLEAR_SET_PRESET=control_table(ffs,gp,RESET_PINS),CE=control_table(ffs,gp,{'CE'}),
   actual_RAM16SDP4_count=len(memories),RAM_capacity_bits=sum(r['capacity_bits']for r in ram_rows),
   RAM_capacity_bytes=sum(r['capacity_bits']for r in ram_rows)//8,
   RAM_capacity_scope='Sum of real primitive storage capacity, including address padding and replicas; not guaranteed distinct source payload bits.',
   RAM_parameters_and_connections=ram_rows,RAM_control_pins=ram_controls)
 physical_report=(root/'impl/pnr/g2_top.rpt.txt').read_text(encoding='utf8')
 report_ff=int(re.search(r'--Logic Register as FF\s*\|\s*(\d+)/',physical_report)[1])
 lexical_FF_headers=len(re.findall(r'^DFF\w*\s+(?:\\\S+|[^\s(]+)\s*\(',text,re.M))
 assert global_ff_count==lexical_FF_headers,('primitive parser missed actual FF headers',global_ff_count,lexical_FF_headers)
 assert meta(vo)==before
 return dict(passed=True,build=str(root),reviewed_post_PNR_VO=str(vo),actual_leaf_instances_parsed=total,global_actual_FF_count=global_ff_count,all_explicit_DFF_headers_accounted=True,global_PNR_report_FF_count=report_ff,global_report_minus_explicit_DFF_count=report_ff-global_ff_count,report_count_difference_explained=False,parser_handles_inline_RAM_DO_closing=True,model_hierarchy=model,groups=groups,metadata_unchanged_at_end=True,metadata=before)
def run(out):
 out.mkdir(exist_ok=False)
 lib=LIB.read_text(encoding='utf8')
 body=re.search(r'\bmodule\s+RAM16SDP4\b(.*?)\bendmodule',lib,re.S)[1]
 defaults=dict(re.findall(r'parameter\s+(INIT_[0-3])\s*=\s*([^;]+);',body))
 assert len(defaults)==4
 assert re.search(r'input\s+\[3:0\]\s+WAD',body)and re.search(r'input\s+\[3:0\]\s+DI',body)
 directions=primitive_directions(LIB);summary={}
 for label,folder,model in BUILDS:
  print('streaming '+label,flush=True)
  r=analyze(BASE/folder,model,defaults,directions)
  save(out/(label+'_four_storage_groups.json'),r)
  summary[label]=dict(report=str(out/(label+'_four_storage_groups.json')),groups={g:dict(FF=x['actual_FF_count'],named_unique_bits=x['logical_bit_indices_from_names'],duplicated_bit_indices=x['duplicate_logical_bit_indices'],excess_named_FF_copies=x['excess_FF_instances_above_named_unique_bits'],unmatched_FF_names=len(x['unmatched_FF_names']),RAM16SDP4=x['actual_RAM16SDP4_count'],RAM_bits=x['RAM_capacity_bits'],RESET_nets=x['RESET_CLEAR_SET_PRESET']['nonconstant_unique_nets'],RESET_FF_pins=x['RESET_CLEAR_SET_PRESET']['nonconstant_unique_FF_pins'],CE_nets=x['CE']['nonconstant_unique_nets'],CE_FF_pins=x['CE']['nonconstant_unique_FF_pins'])for g,x in r['groups'].items()})
  del r;gc.collect()
 result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),scope='Actual emitted PNR35/PNR49 primitive instances, source-index name repetitions, real controls and primitive RAM capacity only. No synthesis/simulation or timing/functional-change acceptance.',reports=summary,
  vendor_RAM16SDP4_definition=str(LIB),RAM16SDP4_words=16,RAM16SDP4_width=4,RAM16SDP4_bits=64,
  input_mem_counted_as_FFs=False,whole_designs_parsed_serially=True,source_or_existing_evidence_or_design_tools_modified=False)
 save(out/'result.json',result);print(json.dumps(result,ensure_ascii=False),flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=BASE/'PNR35_49_model_storage_physical_review1');a=p.parse_args()
 try:run(a.out)
 except Exception as e:
  if a.out.exists():save(a.out/'review_failed.json',dict(passed=False,error=str(e),traceback=traceback.format_exc()))
  raise
