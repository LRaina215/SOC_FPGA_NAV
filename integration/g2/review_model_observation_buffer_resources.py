"""Read-only actualSYN37 buffer resource/driver attribution; no EDA/build."""
from pathlib import Path
from datetime import datetime
from functools import lru_cache
import argparse,collections,json,re,traceback
from synthesis_connectivity import read,DECL
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from audit_routing_topology import primitive_directions
from audit_netlist_memories import BSRAM
from review_lease_deadline_syn_mapping import hierarchy_resources
from prepare_baseline_fusion_retime import metadata
from prove_soc_console_remap import z3
BASE=Path('F:/nav_integration_g2_20261023')
SYN=BASE/'baseline_fusion_model25_bounded_MAX_syn37'
LIB=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')

def save(p,r):assert not p.exists();p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def resource(cells):
 count=collections.Counter(c['kind']for c in cells.values())
 return dict(FF=sum(v for k,v in count.items()if k.startswith('DFF')),LUT_explicit=sum(v for k,v in count.items()if re.fullmatch(r'LUT[1-6]',k)),INV=count['INV'],ALU=count['ALU'],BSRAM=sum(v for k,v in count.items()if k in BSRAM),SSRAM=sum(v for k,v in count.items()if k.startswith('RAM16')),ROM16=count['ROM16'],primitive_counts=dict(count))

def inspect_module(mods,kind,out):
 body=mods[kind];f=out/(kind+'_actual_module.v');f.write_text('module g2_top ('+body+'endmodule\n')
 cells=read(f);params=parameter_reader({'g2_top':body});directions=primitive_directions(LIB)
 # Child-module cells are retained as boundary consumers, never flattened or
 # mistaken for a LUT/FF attributable to this storage expression.
 for child,childbody in mods.items():
  if child not in directions:directions[child]={n.lstrip('\\'):d for d,h,l,n in DECL.findall(childbody)if d in('input','output','inout')}
 drivers=collections.defaultdict(list);loads=collections.defaultdict(list)
 for n,c in cells.items():
  for pin,nets in c['ports'].items():
   direction=directions[c['kind']][pin]
   for i,net in enumerate(nets):
    if net in('GND','VCC'):continue
    (drivers if direction=='output'else loads)[net].append((n,pin,i))
 external_outputs=set()
 for d,h,l,n in DECL.findall(body):
  if d!='output':continue
  n=n.lstrip('\\');external_outputs.update([n]if not h else[n+'['+str(i)+']'for i in range(int(h),int(l)-1,-1)])
 @lru_cache(None)
 def cone(net):
  ds=drivers.get(net,[])
  if not ds:return frozenset()
  assert len(ds)==1,(kind,net,ds)
  n,pin,i=ds[0];c=cells[n]
  if c['kind'].startswith(('DFF','RAM16','ROM16'))or c['kind']in BSRAM or c['kind']in mods:return frozenset()
  # No inferred submodule or sequential DSP is crossed by these data mux cones.
  assert c['kind']in('VCC','GND','INV')or re.fullmatch(r'LUT[1-6]|MUX2_LUT[5-8]|ALU',c['kind']),(kind,net,c['kind'])
  return frozenset([n]).union(*(cone(v)for p,ns in c['ports'].items()if directions[c['kind']][p]in('input','inout')for v in ns if v not in('GND','VCC')))
 def group(stem):
  regs={n:c for n,c in cells.items()if c['kind'].startswith('DFF')and re.fullmatch(re.escape(stem)+r'_(\d+)_s\d+(?:.*)?',n)}
  memory={n:c for n,c in cells.items()if c['kind'].startswith('RAM16')and n.startswith(stem+'_')}
  storage={**regs,**memory};bits=[int(re.match(re.escape(stem)+r'_(\d+)_',n)[1])for n in regs]
  direct={};allcone=set()
  for n,c in regs.items():
   for net in c['ports'].get('D',[]):
    ds=drivers.get(net,[])
    if ds and not cells[ds[0][0]]['kind'].startswith('DFF'):direct[ds[0][0]]=cells[ds[0][0]]
    allcone.update(cone(net))
  controls={}
  for pin in('CE','CLEAR','RESET','PRESET','SET'):
   pinload=collections.defaultdict(list)
   for n,c in regs.items():
    for net in c['ports'].get(pin,[]):pinload[net].append(n)
   rows=[]
   for net,regnames in sorted(pinload.items()):
    other=[x for x in loads[net]if cells[x[0]]['kind'].startswith('DFF')and x[1]==pin]
    rows.append(dict(actual_net=net,constant=net in('GND','VCC'),group_unique_FF_pins=len(regnames),all_same_module_FF_pins=len(other),register_names=regnames))
   controls[pin]=rows
  # Literal-deletion upper bound only. Values still have to be computed and
  # delivered by any replacement RAM/stream design, so this is not net QoR.
  gone=set(regs);allowed={n for n in allcone if re.fullmatch(r'LUT[1-6]|MUX2_LUT[5-8]|ALU|INV',cells[n]['kind'])}
  changed=True
  while changed:
   changed=False
   for n in sorted(allowed-gone):
    outputs=[v for p,ns in cells[n]['ports'].items()if directions[cells[n]['kind']][p]=='output'for v in ns]
    if any(v in external_outputs for v in outputs):continue
    if all(all(load[0]in gone for load in loads[v])for v in outputs):gone.add(n);changed=True
  removable={n:cells[n]for n in gone if n not in regs}
  result=dict(storage=resource(storage),logical_bit_indices=sorted(bits),duplicate_logical_bit_labels=len(bits)-len(set(bits)),
   direct_D_driver_resources=resource(direct),direct_D_driver_instances=sorted(direct),
   all_combinational_D_fanin_resources=resource({n:cells[n]for n in allcone}),
   literal_deletion_exclusive_upstream_upper_bound=resource(removable),literal_deletion_upper_bound_not_function_preserving_optimization=True,
   reset_CE_module_scoped_loads=controls,storage_cells=[dict(**c,parameters=params(c))for c in storage.values()])
  return result,dict(cells=cells,params=params,directions=directions,direct=direct,allcone=allcone)
 return group,cells,params

def run(out):
 out.mkdir(parents=True,exist_ok=False);mods=definitions(SYN/'impl/gwsynthesis/g2_top.vg')
 results={};analysis={}
 for kind,stems in [('rl_observation_v5',['history_flat','current_flat']),('rl_policy_tick_v5',['snapshot']),('fc_v4_top',['input_mem'])]:
  g,c,p=inspect_module(mods,kind,out)
  for stem in stems:results[stem],analysis[stem]=g(stem)
 # Decode sharing from actual missing FF labels and the source's exact clip.
 hist=set(results['history_flat']['logical_bit_indices']);cur=set(results['current_flat']['logical_bit_indices'])
 assert set(range(3360))-hist=={(112+j)*24+23 for j in range(28)}
 assert set(range(672))-cur=={j*24+k for j in range(28)for k in range(19,23)}
 w=z3.BitVec('arbitrary_signed_word24',24);v=lambda n:z3.BitVecVal(n,24)
 clip=z3.If(w>v(409600),v(409600),z3.If(w<v(-409600),v(-409600),w))
 solver=z3.Solver();solver.add(z3.Or(z3.Extract(23,23,clip)!=z3.Extract(23,23,w),*[z3.Extract(i,i,clip)!=z3.Extract(23,23,clip)for i in range(19,23)]));assert solver.check()==z3.unsat
 obs=(SYN/'control/rl_observation_v5.v').read_text();assert 'current_flat[j*24+:24]<=clip_current(words[j])'in obs and 'history_flat[(112+j)*24+:24]<=words[j]'in obs
 # Identical write ports+primitive overrides prove two read copies, rather than
 # guessing from a count or lexical suffix. No memory contents are inferred.
 rams=results['input_mem']['storage_cells'];writes=collections.defaultdict(list)
 for c in rams:
  assert c['kind']=='RAM16SDP4'and len(c['ports']['DO'])==4 and len(c['ports']['WAD'])==len(c['ports']['RAD'])==4
  key=json.dumps(dict(kind=c['kind'],parameters=c['parameters'],ports={p:c['ports'][p]for p in('CLK','WRE','WAD','DI')}),sort_keys=True)
  writes[key].append(c)
 assert len(rams)==132 and len(writes)==66 and all(len(a)==2 and a[0]['ports']['RAD']!=a[1]['ports']['RAD']for a in writes.values())
 replication=[dict(identical_write_and_INIT_pair=[x['name']for x in pair],write_ports={p:pair[0]['ports'][p]for p in('CLK','WRE','WAD','DI')},read_ports=[x['ports']['RAD']for x in pair])for pair in writes.values()]
 for stem,r in results.items():save(out/(stem+'_actual.json'),r)
 rsc=hierarchy_resources(SYN/'impl/gwsynthesis/g2_top_syn_rsc.xml')
 levels={n:rsc['g2_top/'+n]for n in('u_control','u_nav','u_soc')}
 direct_union={}
 for stem in('history_flat','current_flat','snapshot'):
  module='observation'if stem!='snapshot'else'policy'
  direct_union.update({module+'/'+n:c for n,c in analysis[stem]['direct'].items()})
 summary=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),scope='Actual completeSYN37 resource andD-input-cone attribution; not a new optimized implementation, timing proof or CLS placement estimate.',
  source_bits=dict(history_flat=3360,current_flat=672,snapshot=4032,input_mem=4032),source_three_FF_vectors_bits=8064,
  actual_storage_resources={s:r['storage']for s,r in results.items()},actual_named_three_vector_unique_FFs=7924,
  actual_history_current_sharing=dict(history_name_FF3332=True,current_name_FF560=True,newest_history_sign_bits_missing=28,current_sign_extended_duplicate_bits_eliminated=112,
   source_all_signed24_clip_sign_and_high19_to23_alias_miter='unsat',actual_missing_register_labels_exactly_match_source_aliases=True,
   source_sign_alias_maps_history_newest_sign_to_current_sign_register=True,physical_FF_union=3892,semantic_history_FFs=3360,semantic_current_FFs=560,shared_semantic_FFs=28,
   removing_current_alone_cannot_count_shared_history_sign28=True),
  input_mem_actual_replication=dict(logical_words=168,word_bits=24,logical_bits=4032,actual_RAM16SDP4=132,actual_allocated_bits=132*16*4,read_copies=2,identical_write_groups=66,padded_words_per_copy=176,
   paired_same_primitive_INIT_CLK_WRE_WAD_DI_different_RAD=replication),
  direct_D_LUT_driver_resources_per_group={s:r['direct_D_driver_resources']for s,r in results.items()},
  direct_D_LUT_driver_union_three_FF_vectors=resource(direct_union),
  literal_deletion_upper_bounds={s:r['literal_deletion_exclusive_upstream_upper_bound']for s,r in results.items()},
  hierarchy_same_SYN37_totals=levels,CPU_is_separate_u_soc_not_inside_u_nav=True,
  interpretation=['7924FF is the existing unique physical storage label count; a function-preserving replacement must add back its own storage/control/stream registers, so8000netFFsaving is not measured.',
   'input_mem is alreadydistributedRAM withtwo readcopies, not4032additionalFF. Merely adding its sourcebits toFFsaving doublecounts a different resourceclass.',
   'D-driver LUT counts andliteral deletion upperbounds are attributable oldimplementationlogic, not guaranteed removable logic: clipping, protocol, address/read mux andnewRAM control must still implementoriginalbehavior.',
   'No SYN37 physicalCLS occupancy exists. FF/LUT savings do not translate toCLS percentages without packing/placement of an actual replacement.6–9percentagepoints on69120CLS means4148–6221CLS, currently unmeasured.',
   'SYN37 control hasmoreFFthanu_nav alone butfewerLUT/ALU/RAM; u_soc is a separate navigationCPU hierarchy.40kweightcount does not bound buffering, sensor, arithmetic ordisplay hardware.'],
  frozen_design_modified=False,new_EDA_or_PNR_started=False,bindings=[metadata(SYN/'impl/gwsynthesis/g2_top.vg'),metadata(SYN/'impl/gwsynthesis/g2_top_syn_rsc.xml'),metadata(SYN/'control/rl_observation_v5.v'),metadata(SYN/'control/rl_policy_tick_v5.v'),metadata(SYN/'fc_v4_top.v')])
 save(out/'result.json',summary);return summary
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args()
 try:r=run(a.out);print(json.dumps({k:r[k]for k in('actual_storage_resources','direct_D_LUT_driver_resources_per_group','direct_D_LUT_driver_union_three_FF_vectors','literal_deletion_upper_bounds')},indent=2))
 except Exception as exc:
  save(a.out/'failed.json',dict(passed=False,reason=str(exc),traceback=traceback.format_exc()));raise
