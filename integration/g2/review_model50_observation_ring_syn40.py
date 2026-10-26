"""Read completed SYN40 storage/resources/two-clock/radar26 and MAX structure.

No EDA or build mutation. Ring capture/address Boolean semantics are reviewed
independently; SYN estimates and source/actual leaf counts are not routed STA.
"""
from pathlib import Path
from datetime import datetime
from collections import Counter
import argparse,json,re,traceback
from bs4 import BeautifulSoup
from synthesis_connectivity import read,DECL
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from audit_netlist_memories import BSRAM
from audit_routing_topology import primitive_directions
from prepare_baseline_fusion_retime import metadata,same_bytes,hardware_inputs
from review_baseline_single_factor_syn import cpu_image
from review_model25_observation_ring_syn39 import index,clock_reader,clock_inventory,resources,memories,ring_structure
from review_lease_bounded_MAX_joint_mapping import hierarchy_resources,report_paths,explicit_tree,local_guard
from review_model25_joint_syn import module_bindings
from prove_soc_console_remap import Logic,z3

BASE=Path('F:/nav_integration_g2_20261023')
OLD=BASE/'baseline_fusion_only_syn23'
NEW=BASE/'baseline_fusion50_ring_MAX_syn40'
PREP=BASE/'baseline_fusion50_ring_MAX_prepared1'
MAX_REFERENCE=BASE/'baseline_fusion_model25_bounded_MAX_syn37'
REF35=BASE/'baseline_fusion32_CDC52_pnr35'
LIB=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
PREFIX='u_control/model_thread/'
OBS=PREFIX+'observation/'
POLICY=PREFIX+'policy/'
DELTA={'command_lease_guard.v','control/rl_observation_v5.v','control/rl_policy_tick_v5.v','control/mem_policy.v'}

def load(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def save(p,v):
 assert not p.exists(),('do not overwrite evidence',str(p))
 p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf8')

def source_review():
 a,b,p=[hardware_inputs(root,load(root/'manifest.json'))for root in(OLD,NEW,PREP)]
 assert len(a)==len(b)==len(p)==63 and a.keys()==b.keys()==p.keys()
 changed={n for n in a if not same_bytes(a[n],b[n])};assert changed==DELTA,sorted(changed)
 assert all(same_bytes(b[n],p[n])for n in b)
 sets={}
 for suffix,label,count in[({'.hex','.mem','.bin','.mif'},'assets',69),({'.h','.vh'},'headers',31)]:
  before={q.relative_to(OLD):q for q in OLD.rglob('*')if q.is_file()and'impl'not in q.relative_to(OLD).parts and'__pycache__'not in q.parts and q.suffix.lower()in suffix}
  after={q.relative_to(NEW):q for q in NEW.rglob('*')if q.is_file()and'impl'not in q.relative_to(NEW).parts and'__pycache__'not in q.parts and q.suffix.lower()in suffix}
  assert before.keys()==after.keys()and len(before)==count and all(same_bytes(before[n],after[n])for n in before)
  sets[label]=count
 fw={}
 for folder in('fw','firmware'):
  before={q.relative_to(OLD):q for q in(OLD/folder).rglob('*')if q.is_file()and'__pycache__'not in q.parts}
  after={q.relative_to(NEW):q for q in(NEW/folder).rglob('*')if q.is_file()and'__pycache__'not in q.parts}
  assert before.keys()==after.keys()and all(same_bytes(before[n],after[n])for n in before);fw[folder]=len(before)
 assert same_bytes(OLD/'g2.cst',NEW/'g2.cst')and same_bytes(REF35/'g2.sdc',NEW/'g2.sdc')
 t=(NEW/'run.tcl').read_text();selected=re.findall(r'^add_file -type verilog \{(.+)\}$',t,re.M)
 assert len(selected)==63 and{Path(x).resolve()for x in selected}=={q.resolve()for q in b.values()}
 assert all(NEW.resolve()in Path(x).resolve().parents for x in selected)
 assert [x.strip()for x in t.splitlines()if x.strip().startswith('run ')]==['run syn']
 return dict(passed=True,changed_RTL=sorted(changed),other59_RTL_byte_identical=True,
   all63_actual_frozen_inputs_bound=True,identical_assets_headers=sets,complete_original_FW=fw,
   original_CST_and_35_single20_10ns52command_SDC_preserved=True,
   source_bindings=[metadata(q)for q in b.values()],new_EDA_started=False,automatic_PNR=False)

def old_groups(cells):
 stems=dict(history_flat=OBS+'history_flat_',current_flat=OBS+'current_flat_',words=OBS+'words_',snapshot=POLICY+'snapshot_')
 return {key:{n:c for n,c in cells.items()if c['kind'].startswith('DFF')and n.startswith(stem)
          and(key=='words'or re.match(r'\d+_s',n[len(stem):]))}for key,stem in stems.items()}

def timing_summary(paths):
 text=(NEW/'impl/gwsynthesis/g2_top_syn.rpt.html').read_text();soup=BeautifulSoup(text,'html.parser');tables={}
 for anchor in('clock','performance'):
  tag=soup.find('a',attrs={'name':anchor});assert tag
  table=tag.find_parent().find_next('table');headers=[x.get_text(' ',strip=True)for x in table.find_all('th')]
  rows=[[x.get_text(' ',strip=True)for x in tr.find_all('td')]for tr in table.find_all('tr')if tr.find_all('td')]
  assert rows and all(len(row)==len(headers)for row in rows);tables[anchor]=[dict(zip(headers,row))for row in rows]
 return dict(actual_synthesis_clock_summary=tables['clock'],actual_synthesis_Fmax_summary=tables['performance'],
   actual_detailed_listed_paths=paths,only_available_estimates_not_global_routed_STA=True,
   actual_SDC_targets_MHz=dict(clk50=50,rgmii_rxc=100),
   complete_negative_endpoint_coverage_verified=False,all_target_timing_accepted=False)

def actual_lease_equivalence(om,nm,out,directions,top_after,top_parameters):
 ab,ac,ap,aq=local_guard(om,out,'SYN23');bb,bc,bp,bq=local_guard(nm,out,'SYN40')
 assert aq.keys()==bq.keys()and len(aq)==200,('actual lease state boundary',len(aq),len(bq))
 def ports(body,kind):return {n.lstrip('\\'):(hi,lo)for d,hi,lo,n in DECL.findall(body)if d==kind}
 old_inputs,new_inputs=ports(ab,'input'),ports(bb,'input')
 added=set(new_inputs)-set(old_inputs)
 assert set(old_inputs)<=set(new_inputs)and added=={'left_speed_valid','right_speed_valid','model_ready'},('Unreviewed actual lease input role change',added,set(old_inputs)-set(new_inputs))
 assert all(old_inputs[n]==new_inputs[n]for n in old_inputs)
 assert all(not any(new_inputs[n])for n in added)
 assert ports(ab,'output')==ports(bb,'output')
 old_binding=module_bindings(om,['u_command_lease'])['u_command_lease']['ports']
 new_binding=module_bindings(nm,['u_command_lease'])['u_command_lease']['ports']
 assert all(old_binding[p]==new_binding[p]for p in old_binding),'Common actual guard external nets changed'
 assert all(new_binding[p]==[p]for p in added),'New exported role bound to an unexpected net'
 assert old_binding['odo_valid']==new_binding['odo_valid']==['odo_valid']
 # SYN23 exposes the complete feedback_valid conjunction as odo_valid;
 # SYN40 additionally exports its three factors. Prove that exact relation
 # in the actual new full-design netlist before using it in guard miters.
 # These are explicit combinational cut points at the three producer outputs,
 # not assumed aliases or forced hardware/simulation states.
 factors={n:z3.Bool('actual_feedback_factor:'+n)for n in sorted(added)}
 conjunction=z3.And(*factors.values())
 context=Logic(top_after,top_parameters,directions,{},{});context.memo.update(factors)
 actual_feedback=context.expr('odo_valid')
 solver=z3.Solver();solver.set(timeout=30000);solver.add(z3.Xor(actual_feedback,conjunction))
 assert solver.check()==z3.unsat,'Actual odo_valid does not equal the three exported factors'
 used={name:dict(**top_after[name],parameters=top_parameters(top_after[name]))
       for net in context.memo if net not in factors and net not in('GND','VCC')
       for name,pin,bit in context.drivers.get(net,[])}
 assert used and not any(c['kind'].startswith('DFF')for c in used.values())
 state={q:z3.Bool('actual_50MHz_lease_state:'+q)for q in aq}
 a,b=Logic(ac,ap,directions,state,{}),Logic(bc,bp,directions,state,{})
 a.memo['odo_valid']=conjunction
 b.memo.update(factors);b.memo['odo_valid']=conjunction
 checks=[]
 def equivalent(label,x,y):
  delta=z3.simplify(z3.Xor(x,y))
  if z3.is_false(delta):checks.append(dict(label=label,result='canonical_Boolean_identity'));return
  solver=z3.Solver();solver.set(timeout=30000);solver.add(delta);answer=solver.check()
  assert answer==z3.unsat,(label,str(answer),str(solver.model())if answer==z3.sat else solver.reason_unknown())
  checks.append(dict(label=label,result='UNSAT'))
 for q in sorted(aq):
  x,y=aq[q],bq[q]
  assert x['kind']==y['kind']and ap(x)==bp(y),(q,'FF kind/INIT/parameters changed')
  assert x['ports']['CLK']==y['ports']['CLK']==['clk50_d']
  equivalent(q+'/next',a.next_state(x),b.next_state(y))
 count=0
 for name,(hi,lo)in ports(ab,'output').items():
  bits=[name]if not hi else[name+'['+str(i)+']'for i in range(int(hi),int(lo)-1,-1)]
  for q in bits:equivalent(q+'/public_output',a.expr(q),b.expr(q));count+=1
 result=dict(passed=True,reference='actual_SYN23_50MHz_guard',candidate='actual_SYN40_50MHz_guard',
   actual_shared_FFs=200,actual_FF_kinds_and_explicit_parameters_INIT_equal=True,
   common_actual_input_roles_widths_and_top_nets_identical=True,
   new_exported_factor_roles=sorted(added),actual_public_output_bits=count,checks=checks,
   actual_feedback_conjunction_context=dict(passed=True,miter='UNSAT',
      relation='odo_valid == left_speed_valid && right_speed_valid && model_ready',
      actual_combinational_cells=used,old_actual_guard_top_ports=old_binding,new_actual_guard_top_ports=new_binding,
      original_source_feedback_assign='control4500_core.v: feedback_valid=left_speed_valid && right_speed_valid && model_ready',
      source_producer_RTL_preserved=True,whole_upstream_control_sequential_equivalence_claimed=False),
   all_actual_next_state_and_public_output_miters_UNSAT_or_identical=True,
   scope='Every actual FF prestate and external binary input; no arbitrary XZ, routed timing or wholeG2 claim')
 save(out/'actual_MAX_next_state_proof.json',result);return result

def run(out):
 job=load(NEW/'job.json');assert job['phase']=='synthesis_complete'and job['tool_exit']==0,('SYN40 incomplete',job)
 assert not out.exists();out.mkdir(parents=True)
 source=source_review();save(out/'source_binding.json',source)
 watched=[OLD/'impl/gwsynthesis/g2_top.vg',NEW/'impl/gwsynthesis/g2_top.vg',
          NEW/'impl/gwsynthesis/g2_top_syn_rsc.xml',NEW/'impl/gwsynthesis/g2_top_syn.rpt.html',LIB]
 bindings=[metadata(p)for p in watched]
 print('parse completed original50 SYN23 and ring SYN40',flush=True)
 om,nm=definitions(watched[0]),definitions(watched[1]);before,after=read(watched[0]),read(watched[1])
 op,np=parameter_reader(om),parameter_reader(nm);directions=primitive_directions(LIB)
 od,ol=index(before,directions);nd,nl=index(after,directions)
 oc,nc=clock_reader(before,od),clock_reader(after,nd)
 oi,ocounts=clock_inventory(before,oc);ni,ncounts=clock_inventory(after,nc)
 assert {r['root']for r in ncounts}=={'clk50','rgmii_rxc'}
 assert not any(c['kind']=='PLL'for c in after.values())
 assert not any('model_clock_guard/'in n or any('/'+b+'/'in n for b in('bmi_bridge','config_bridge','action_bridge','fusion_bridge','status_bridge'))for n in after)
 assert all(oi[k]==ni[k]for k in oi.keys()&ni.keys())
 save(out/'actual_two_clocks.json',dict(passed=True,before_counts=ocounts,after_counts=ncounts,
   all_common_actual_clock_pin_roots_and_phase_equal=True,actual_after_clock_pins=len(ni),
   no_PLL_or_model25_guard_or_five_related_bridges=True,all_actual_target_physical_timing_pending=True))
 print('all nonring RAM geometry INIT and original CPU',flush=True)
 am,bm=memories(before,op),memories(after,np);ringnames={n for n in bm if n.startswith(OBS+'g_ring.')}
 assert am.keys()==bm.keys()-ringnames,('nonring memory names changed',sorted(am.keys()-bm.keys()),sorted(bm.keys()-am.keys()-ringnames))
 portchanges=[]
 for n,a in am.items():
  b=bm[n];assert a['kind']==b['kind']and a['parameters']==b['parameters'],('nonring memory kind/parameters/INIT',n)
  assert a['ports'].keys()==b['ports'].keys()
  for pin in a['ports']:
   assert len(a['ports'][pin])==len(b['ports'][pin])
   if 'CLK'in pin:assert [oc(v)for v in a['ports'][pin]]==[nc(v)for v in b['ports'][pin]]
  if a['ports']!=b['ports']:portchanges.append(dict(name=n,ports={pin:dict(before=a['ports'][pin],after=b['ports'][pin])for pin in a['ports']if a['ports'][pin]!=b['ports'][pin]}))
 lib=LIB.read_text();body=re.search(r'\bmodule\s+SP\b(.*?)\bendmodule',lib,re.S)[1]
 defaults=dict(re.findall(r'\bparameter\s+(INIT_RAM_\w+)\s*=\s*([^;]+);',body));assert len(defaults)==64
 oldimage,newimage=cpu_image(am,defaults),cpu_image(bm,defaults);binary=(NEW/'firmware/nav_fw.bin').read_bytes()
 word=b''.join(int(x,16).to_bytes(4,'little')for x in(NEW/'ram_w.hex').read_text().split())
 assert len(binary)==51712 and len(newimage)==131072 and newimage==oldimage==word==binary+bytes(131072-len(binary))
 for lane in range(4):assert bytes(int(x,16)for x in(NEW/f'ram_b{lane}.hex').read_text().split())==newimage[lane::4]
 bsram=sum(c['kind']in BSRAM for c in after.values());assert bsram==314
 memory=dict(passed=True,unchanged_nonring_memory_leaves=len(am),new_ring_RAM_leaves=len(ringnames),
   all_nonring_kinds_all_explicit_parameters_and_INIT_identical=True,BSRAM=bsram,
   wide32_36_leaves=sum(c['kind']in BSRAM and any(np(c).get(k)in('32','36')for k in('BIT_WIDTH','BIT_WIDTH_0','BIT_WIDTH_1'))for c in after.values()),
   CPU_SP_bitplanes=64,CPU_reconstructed_bytes=131072,original_firmware_bytes=51712,
   CPU_BIN_zero_tail_wordHEX_four_lanes_equal=True,memory_port_changes=portchanges,
   all_changed_memory_data_port_logic_equivalence_proved=not bool(portchanges),
   physical_BSRAM_split_count_CLS_and_route_pending=True)
 save(out/'actual_nonring_memory_INIT_CPU.json',memory)
 print('old wide FFs and actual ring topology',flush=True)
 old,new=old_groups(before),old_groups(after)
 assert {k:len(v)for k,v in old.items()}==dict(history_flat=3332,current_flat=560,words=672,snapshot=4032)
 assert not any(new.values()),('wide storage survived',{k:len(v)for k,v in new.items()})
 ring=ring_structure(after,np,directions,nd,nl,nc,observation_prefix=OBS,policy_prefix=POLICY,expected_clock=('clk50',0))
 for row in ring['ring_related_nonclock_fanout']:
  row['same_named_net_before_nonclock_primitive_input_pins']=sum(not re.fullmatch(r'CLK\d*|CLKA|CLKB|CLOCK|WCLK|RCLK',pin)for n,pin,i in ol.get(row['net'],[]))
  row['explicit_generated_ring_net_name']=row['net'].startswith(OBS+'g_ring.')
 ring['maximum_explicit_generated_ring_net_nonclock_fanout']=max((x['nonclock_primitive_input_pins']for x in ring['ring_related_nonclock_fanout']if x['explicit_generated_ring_net_name']),default=0)
 save(out/'new_ring_actual_structure.json',ring)
 print('actual original26, stopping chain and original two-domain interfaces',flush=True)
 from review_model50_syn40_clock_stop import complete
 ctx=dict(before=before,after=after,om=om,nm=nm,op=op,np=np,directions=directions,oc=oc,nc=nc,old=OLD,new=NEW,prepared=PREP)
 interfaces=complete(ctx,out/'clock_stop')
 assert interfaces['passed']
 print('MAX11LUT tree and actual prior proof boundary',flush=True)
 mm=definitions(MAX_REFERENCE/'impl/gwsynthesis/g2_top.vg')
 tree=explicit_tree(nm,mm,out)
 same_guard=mm['command_lease_guard']==nm['command_lease_guard']
 lease_proof=actual_lease_equivalence(om,nm,out,directions,after,np)
 lease_ffs={n:c for n,c in after.items()if n.startswith('u_command_lease/')and c['kind'].startswith('DFF')}
 assert len(lease_ffs)==200,('actual MAX guard FF count',len(lease_ffs))
 guard_ports_old=module_bindings(om,['u_command_lease']);guard_ports_new=module_bindings(nm,['u_command_lease'])
 assert all(v==guard_ports_new['u_command_lease']['ports'][p]for p,v in guard_ports_old['u_command_lease']['ports'].items()),'actual common lease external binding changed'
 lease=dict(passed=True,actual_MAX_guard_definition_byte_identical_proved_SYN37=same_guard,
    literal_definition_difference_not_used_as_equivalence_proof=True,
    actual_original50_200FF_next_state_and_public_outputs_equal=lease_proof,
    actual_original50_common_lease_external_instance_ports_identical=True,
    three_new_factor_ports_proved_redundant_to_existing_odo_valid=True,actual_FF_count=200,
    actual_explicit_tree_LUTs=tree['actual_explicit_LUT_count'],actual_tree_levels=tree['actual_age_Q_to_nonmax_LUT_levels'],
    physical_critical_path_or_route_benefit_claimed=False)
 save(out/'actual_MAX_stop_inheritance.json',lease)
 oldr,newr=[hierarchy_resources(root/'impl/gwsynthesis/g2_top_syn_rsc.xml')['g2_top']for root in(OLD,NEW)]
 timing=timing_summary(report_paths(NEW));save(out/'actual_synthesis_timing_estimates.json',timing)
 result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),source=source,
    actual_complete_SYN40_exit=0,source_SYN23=str(OLD),actual_SYN40=str(NEW),
    actual_resources_before=resources(before),actual_resources_after=resources(after),
    global_resource_report_before=oldr,global_resource_report_after=newr,actual_old_named_FFs={k:len(v)for k,v in old.items()},
    actual_old8596FFs_removed=True,actual_new_ring_resources=ring['actual_ring_resources'],
    actual_ring_RAM_bits=ring['actual_allocated_RAM_bits'],actual_prefetch_FFs=24,
    actual_slot_pointer_FFs=ring['slot_pointer_total_FFs'],actual_memory_CPU=memory,
    actual_two_clock_roots_only=True,actual_clock_counts=ncounts,actual_interfaces_and_stopping=interfaces,actual_MAX_stop_inheritance=lease,
    actual_ring_value_address_prefetch_semantics='Separate region review; counts/topology do not replace it',
    actual_synthesis_timing_estimates=timing,complete_negative_endpoint_coverage_verified=False,
    actual_physical_CLS_and_routing_or_target_timing_unknown=True,automatic_PNR=False,new_EDA_started=False,
    metadata_bindings=bindings,all_bound_inputs_unchanged=all(metadata(Path(r['path']))==r for r in bindings),
    physical_CDC_reset_pulse_IO_whole_vehicle_acceptance=False)
 assert result['all_bound_inputs_unchanged'];save(out/'result.json',result);return result

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
 try:
  r=run(a.out);print(json.dumps(dict(passed=r['passed'],result=str(a.out/'result.json'),resources=r['global_resource_report_after']),ensure_ascii=False))
 except BaseException as exc:
  if a.out.exists():save(a.out/'review_failed.json',dict(passed=False,error=str(exc),traceback=traceback.format_exc(),recorded_at=datetime.now().astimezone().isoformat()))
  raise
