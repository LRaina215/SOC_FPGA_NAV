"""Read-only actual failed model25/NAV50 physical result audit; starts no tools."""
from pathlib import Path
from collections import Counter,defaultdict
from datetime import datetime
import argparse,json,re,sys,traceback,psutil
sys.path.insert(0,'D:/Desktop/SOC FPGA/nav/integration/g2')
from audit_pin_loads import PORTS
from audit_global_muxes import bits
from audit_routing_topology import primitive_directions
from audit_netlist_memories import BSRAM,PARAM
from review_baseline_single_factor_syn import cpu_image
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from synthesis_connectivity import read
from review_model25_joint_syn import same,meta,load,save
BASE=Path('F:/nav_integration_g2_20261023')
LIB=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
SOURCE=BASE/'baseline_fusion_model25_NAV50_syn34'
REFERENCE=BASE/'baseline_fusion32_CDC52_pnr35'
REVIEW=BASE/'model25_SYN34_actual_review1/actual_review4'
PAYLOAD=BASE/'model25_SYN34_actual_review1/payload_semantics_review1'
CACHE=BASE/'pnr34_vs_fusion32_physical_review1/pnr32_actual_storage_loads_and_DSP.json'
def norm(s):return re.sub(r'\s+(?=\[)','',s.replace('\\','').strip())
def report(build):
 t=(build/'impl/pnr/g2_top.rpt.txt').read_text();g=[];resource={}
 for line in t.split('6. Global Clock Signals',1)[1].split('7. Pinout',1)[0].splitlines():
  if '| PRIMARY'in line or'| LW 'in line:
   n,k,l=[p.strip()for p in line.split('|')];g.append(dict(net=n,resource=k,location=l))
 for key in('Logic','Register','CLS','BSRAM','DSP'):
  m=re.search(r'^\s*'+key+r'\s*\|\s*([\d.]+)/',t,re.M);resource[key]=float(m[1])if'.'in m[1]else int(m[1])
 m=re.search(r'(\d+) LUT, (\d+) ALU, (\d+) ROM16',t);resource.update(dict(zip(('LUT','ALU','ROM16'),map(int,m.groups()))))
 resource['RAM16']=int(re.search(r'--SSRAM\(RAM16\)\s*\|\s*(\d+)',t)[1]);return dict(resources=resource,globals=g)
def reset_load_inventory(build,selected):
 directions=primitive_directions(LIB);inverters={};buffers={};reset_rows=[];loads=defaultdict(Counter)
 begin=re.compile(r'^(?!module\b)(\w+)\s+(\\\S+|\w+)\s*\(\s*$')
 def finish(kind,raw,lines):
  name=norm(raw);ports={p:bits(v)for p,v in PORTS.findall(''.join(lines))}
  if kind=='INV':inverters[ports['O'][0]]=ports['I'][0]
  if kind in('BUF','BUFG'):buffers[ports['O'][0]]=ports['I'][0]
  for pin,nets in ports.items():
   if directions[kind][pin]in('input','inout'):
    for net in nets:
     if net in selected:loads[net][pin]+=1
  if kind.startswith('DFF'):
   for pin in('CLEAR','PRESET','RESET','SET'):
    if pin in ports and ports[pin]!=['GND']:
     assert len(ports[pin])==1
     reset_rows.append(dict(cell=name,kind=kind,pin=pin,net=ports[pin][0]))
 cur=None;lines=[]
 with(build/'impl/pnr/g2_top.vo').open()as f:
  for line in f:
   if cur:
    lines.append(line)
    if line.rstrip().endswith(');'):finish(*cur,lines);cur=None;lines=[]
    continue
   m=begin.match(line)
   if m:cur=m.groups();lines=[line]
 def root(net):
  original=net;inv=0;seen=set()
  while net in inverters or net in buffers:
   assert net not in seen;seen.add(net)
   if net in inverters:net=inverters[net];inv^=1
   else:net=buffers[net]
  return net,inv
 def group(name):
  pre='u_control/model_thread/'
  if name.startswith(pre):
   x=name[len(pre):]
   if x.startswith('model/'):x=x[len('model/'):]
   if x.startswith('policy/core/'):return'control/model/policy/core'
   return'control/model_or_transport/'+(x.split('/')[0]if '/'in x else'wrapper_private')
  p=name.split('/')
  if len(p)==2:return p[0]+'/top_io_state'
  return'/'.join(p[:2])if len(p)>1 else p[0]
 groups=defaultdict(Counter);by_pin=defaultdict(Counter)
 for row in reset_rows:
  net,inv=root(row['net']);row.update(root=net,inversions=inv,module_group=group(row['cell']))
  key=(net,inv);groups[key][row['module_group']]+=1;by_pin[key][row['pin']]+=1
 return dict(scope='Actual logical reset pin connections only. Collapse only actual INV/BUF/BUFG; compound reset LUT roots remain distinct. Not failed nets, routing status or reset electrical timing.',
  build=str(build),direct_selected_net_loads=[dict(net=n,total_input_pins=sum(loads[n].values()),pin_counts=dict(loads[n]))for n in sorted(selected)],
  reset_roots=[dict(root=n,inversions=i,register_reset_pins=sum(c.values()),by_module=dict(c),by_pin=dict(by_pin[(n,i)]))for(n,i),c in sorted(groups.items(),key=lambda x:(-sum(x[1].values()),x[0]))],reset_pin_rows=reset_rows)

def audit(build,out):
 assert load(REVIEW/'final_review.json')['passed']and load(PAYLOAD/'final_review.json')['passed']
 rt=load(build/'tool_runtime.json');gate=load(build/'g2_gate.json');a,b=report(REFERENCE),report(build)
 assert rt['tool_exit']==1 and not rt['cancelled']and rt['max_runtime_s']==rt['max_no_observable_progress_s']==0
 assert gate['unrouted_nets']==427 and not gate['routing_passed']and not(build/'impl/pnr/g2_top.tr').exists()
 proc=[]
 for pid in(36628,10168):
  proc.append(dict(pid=pid,present=psutil.pid_exists(pid)))
 wanted=set();targetnets={r['net']for v in(a,b)for r in v['globals']}
 channels=load(REVIEW/'actual_five_related_channels.json')['channels'];radar=load(REVIEW/'actual_original_radar26.json')['paths']
 expected={}
 def add(c):
  assert c['name']not in expected or expected[c['name']]==c
  expected[c['name']]=c;wanted.add(c['name'])
  for ns in c['ports'].values():targetnets.update(n for n in ns if n not in('GND','VCC'))
 for ch in channels:
  for c in ch['actual_control_registers'].values():add(c)
  for r in ch['source_payload']:add(r['cell'])
  for r in ch['destination_source_bit_connections']:add(r['actual_destination'])
 for row in radar:
  for k in('actual_source','actual_first','actual_second'):add(row[k])
 for row in load(PAYLOAD/'actual_final_consumers_and_clock_roles.json')['consumers']:add(row['actual_consumer'])
 configDSP=load(PAYLOAD/'actual_configuration_DSP_AREG24.json');add(configDSP['actual_DSP'])
 pll=load(REVIEW/'actual_clock_PLL_inventory.json')['PLL'];add(pll)
 guard=load(REVIEW/'actual_guard_cold_start_and_INIT1_proof.json');add(guard['actual_fault_primitive'])
 for c in guard['original_startup_registers']:add(c)
 # Also bind all small guard FF, event-valid/overflow FF and final wrapper flags.
 extra=read(SOURCE/'impl/gwsynthesis/g2_top.vg',lambda n:n=='u_control'or n=='u_control/model_clock_guard'or n.startswith('u_control/model_clock_guard/')or n=='u_control/model_thread'or(n.startswith('u_control/model_thread/')and'/model/'not in n))
 for n,c in extra.items():
  if c['kind'].startswith('DFF')and ((n.startswith('u_control/model_clock_guard/'))or(n.startswith('u_control/model_thread/')and not any(q in n for q in('/model/','holding_','dst_data_','last_sent_','action_hold50_','fusion_hold50_')))):add(c)
 params=parameter_reader(definitions(SOURCE/'impl/gwsynthesis/g2_top.vg'))
 directions=primitive_directions(LIB);memory={};actual={};clockcells={};clocks=set();ffclocks=[];loadcounts=Counter();pinloads=defaultdict(Counter);globaldrivers=defaultdict(list);pk=Counter();positionkeys=set();allparams={}
 begin=re.compile(r'^(?!module\b)(\w+)\s+(\\\S+|\w+)\s*\(\s*$')
 def finish(kind,raw,lines):
  name=norm(raw);ports=dict(PORTS.findall(''.join(lines)));pk[kind]+=1
  c=dict(name=name,kind=kind,ports={p:bits(v)for p,v in ports.items()},parameters={})
  keep=name in wanted or kind.startswith('DFF')or kind in BSRAM or kind in('IBUF','INV','BUF','BUFG','PLL')
  if kind in BSRAM:memory[name]=c
  if kind in('IBUF','INV','BUF','BUFG','PLL'):clockcells[name]=c
  if kind.startswith('DFF'):ffclocks.append((name,c['ports']['CLK'][0]))
  for pin,nets in c['ports'].items():
   for net in nets:
    if net in('GND','VCC'):continue
    if directions[kind][pin]in('input','inout'):
     if re.fullmatch(r'CLK\d*|CLKA|CLKB|CLOCK|WCLK|RCLK|CLKIN',pin):clocks.add(net)
     elif net in targetnets:loadcounts[net]+=1;pinloads[net][pin]+=1
    if directions[kind][pin]in('output','inout')and net in targetnets:globaldrivers[net].append(dict(name=name,kind=kind,pin=pin,ports=c['ports']));keep=True
  if keep:actual[name]=c;allparams[name]=c['parameters']
 cur=None;lines=[]
 with(build/'impl/pnr/g2_top.vo').open()as f:
  for line in f:
   if cur:
    lines.append(line)
    if line.rstrip().endswith(');'):finish(*cur,lines);cur=None;lines=[]
    continue
   m=begin.match(line)
   if m:cur=m.groups();lines=[line];continue
   if line.startswith('defparam '):
    for raw,key,value in PARAM.findall(line):
     name=norm(raw)
     if key.upper()in('LOC','RLOC','BEL','SITE','PLACEMENT','X','Y'):positionkeys.add(key)
     if name in allparams:allparams[name][key]=value.strip()
 assert cur is None
 save(out/'actual_physical_memory_clock_target_cells.json',dict(memory=memory,selected={n:actual[n]for n in wanted if n in actual},clockcells=clockcells))
 cache=load(CACHE)
 def mapped(n):
  p='u_control/model_thread/'
  return p+'model/'+n[len(p):]if n.startswith(p)else n
 geometry={n:dict(kind=c['kind'],parameters=c['parameters'])for n,c in memory.items()}
 wanted_geometry={mapped(n):v for n,v in cache['physical_memory_geometry'].items()}
 assert set(geometry)==set(wanted_geometry),(sorted(set(geometry)-set(wanted_geometry)),sorted(set(wanted_geometry)-set(geometry)))
 diff=[dict(name=n,actual=geometry[n],expected=wanted_geometry[n])for n in geometry if geometry[n]!=wanted_geometry[n]]
 save(out/'actual_memory_geometry_INIT_comparison.json',dict(passed=not diff,count=len(geometry),explicit_model_path_relocation='u_control/model_thread/* -> u_control/model_thread/model/*',all332_geometry_and_explicit_INIT_identical=not diff,differences=diff))
 assert not diff and len(geometry)==b['resources']['BSRAM']==332
 assert len(cache['wide_pairs'])==18
 for pair in cache['wide_pairs']:
  assert mapped(pair['source'])in geometry and mapped(pair['added_after_pnr']['name'])in geometry
 spmacro=re.search(r'\bmodule SP\b(.*?)\bendmodule',LIB.read_text(),re.S)[1]
 initdefault=dict(re.findall(r'\bparameter\s+(INIT_RAM_\w+)\s*=\s*([^;]+);',spmacro))
 image=cpu_image(geometry,initdefault);binary=(SOURCE/'firmware/nav_fw.bin').read_bytes()
 assert image==binary+bytes(131072-len(binary))and len(binary)==51712
 pub=LIB.read_text();defaults={}
 def effective(kind,p):
  if kind not in defaults:
   macro=re.search(r'\bmodule '+re.escape(kind)+r'\b(.*?)\bendmodule',pub,re.S)[1];macro=re.sub(r'/\*.*?\*/|//[^\n]*','',macro,flags=re.S)
   defaults[kind]={k:v.strip()for k,v in re.findall(r'\bparameter\s+(\w+)\s*=\s*([^;]+);',macro)}
  assert set(p)<=set(defaults[kind]),(kind,set(p)-set(defaults[kind]))
  return defaults[kind]|p
 source_drivers={};physical_drivers={}
 for sourcecells,dtable in[(extra,source_drivers),(actual,physical_drivers)]:
  for c in sourcecells.values():
   for pin,nets in c['ports'].items():
    if directions[c['kind']][pin]=='output':
     for n in nets:
      if n not in('GND','VCC'):dtable[n]=c
 aliases=[]
 def actual_alias(source_net,physical_net):
  if source_net==physical_net:return True
  a=source_drivers.get(source_net);b=physical_drivers.get(physical_net)
  if a and b and a['kind']==b['kind']=='INV'and a['ports']['I']==b['ports']['I']:
   aliases.append(dict(SYN_net=source_net,physical_net=physical_net,method='actual_same_input_INV_copy',source_actual=a,physical_actual=b));return True
  a=physical_drivers.get(source_net)
  if a and b and a['kind']==b['kind']and a['kind'].startswith('DFF'):
   pa={p:n for p,n in a['ports'].items()if p!='Q'};pb={p:n for p,n in b['ports'].items()if p!='Q'}
   if pa==pb and effective(a['kind'],a['parameters'])==effective(b['kind'],b['parameters']):
    aliases.append(dict(SYN_net=source_net,physical_net=physical_net,method='actual_copy_FF_same_kind_all_nonQ_inputs_effective_INIT_and_clock',source_actual=a,physical_actual=b));return True
  return False
 differences=[];targetrows=[];inactive_PLL_changes=[];proved_port_changes=[]
 for name,c in expected.items():
  p=actual.get(name);assert p is not None,('missing target',name)
  ps=params(c);pp=p['parameters']
  ports={k:dict(SYN=c['ports'].get(k),physical=p['ports'].get(k))for k in set(c['ports'])|set(p['ports'])if c['ports'].get(k)!=p['ports'].get(k)}
  ep,eq=effective(c['kind'],ps),effective(p['kind'],pp)
  pd={k:dict(SYN=ep.get(k),physical=eq.get(k))for k in set(ep)|set(eq)if ep.get(k)!=eq.get(k)}
  unresolved_ports={}
  for pin,row in ports.items():
   if len(row['SYN'])==len(row['physical'])and all(actual_alias(sn,pn)for sn,pn in zip(row['SYN'],row['physical'])):
    proved_port_changes.append(dict(cell=name,pin=pin,**row))
   else:unresolved_ports[pin]=row
  unresolved_parameters={}
  for key,row in pd.items():
   m=re.fullmatch(r'ODIV([1-6])_SEL',key)
   if c['kind']=='PLL'and m:
    index=m[1];enable='CLKOUT'+index+'_EN';pin='CLKOUT'+index
    assert ep[enable]==eq[enable]=='"FALSE"'
    assert all(loadcounts[n]==0 and n not in clocks for n in p['ports'][pin])
    inactive_PLL_changes.append(dict(parameter=key,**row,disabled_output=pin,enabled_in_both=False,observed_loads=0));continue
   unresolved_parameters[key]=row
  if unresolved_ports or unresolved_parameters or c['kind']!=p['kind']:differences.append(dict(name=name,SYNkind=c['kind'],physicalkind=p['kind'],ports=unresolved_ports,parameters=unresolved_parameters))
  targetrows.append(dict(name=name,kind=c['kind'],exact_all_ports_effective_parameters_and_INIT_preserved=not ports and not pd,actual_duplicated_ports_proved=bool(ports)and not unresolved_ports,effective_parameters_equivalent_for_all_enabled_outputs=not unresolved_parameters))
 save(out/'actual_three_clock_five_channels_26_and_guard_preservation.json',dict(passed=not differences,targeted_actual_cells=len(targetrows),rows=targetrows,proved_implementation_copies=proved_port_changes,copy_driver_proofs=aliases,inactive_PLL_divider_normalizations=inactive_PLL_changes,differences=differences))
 assert not differences
 clockdrivers={}
 for c in clockcells.values():
  for pin,nets in c['ports'].items():
   if directions[c['kind']][pin]=='output':
    for net in nets:clockdrivers[net]=(c,pin)
 def root(net,seen=()):
  assert net not in seen
  c,pin=clockdrivers[net]
  if c['kind']=='IBUF':return c['ports']['I'][0]
  if c['kind']=='PLL':assert c['name']==pll['name']and pin=='CLKOUT0';return'model25'
  assert c['kind']in('INV','BUF','BUFG');return root(c['ports']['I'][0],seen+(net,))
 roots={n:root(n)for n in clocks};assert set(roots.values())=={'clk50','model25','rgmii_rxc'}
 modelclk=Counter();allff=Counter()
 for name,net in ffclocks:
  clk=roots[net];allff[clk]+=1
  if name.startswith('u_control/model_thread/model/'):
   assert clk=='model25';modelclk[clk]+=1
 save(out/'actual_three_clock_roots_and_roles.json',dict(passed=True,actual_clock_nets=roots,actual_FF_by_clock=dict(allff),explicit_DFF_count=sum(allff.values()),vendor_Register_count=b['resources']['Register'],vendor_Register_minus_explicit_DFF=sum([b['resources']['Register'],-sum(allff.values())]),Register_count_gap_explanation_verified=False,actual_inner_model_FF_by_clock=dict(modelclk),actual_PLL=actual[pll['name']],clock_targets_mhz=dict(clk50=50,model25=25,rgmii_rxc=100),physical_skew_setup_hold_and_clock_signoff=False))
 normalized=lambda p,r:p.read_text().replace(str(r),'<BUILD>').replace(r.as_posix(),'<BUILD>').split()
 sdc=[x.strip()for x in(build/'g2.sdc').read_text().splitlines()if x.strip()and not x.strip().startswith('#')]
 old_sdc=[x.strip()for x in(REFERENCE/'g2.sdc').read_text().splitlines()if x.strip()and not x.strip().startswith('#')]
 timing=lambda rows:[x for x in rows if x.startswith(('set_false_path','set_max_delay'))]
 assert timing(sdc)==timing(old_sdc)and len(timing(sdc))==52
 assert sum(x.startswith('create_clock ')for x in sdc)==2
 assert sdc.count('create_generated_clock -name model25 -source [get_ports {clk50}] -divide_by 2 [get_pins {u_model_pll/PLL_inst/CLKOUT0}]')==1
 assert sdc.count('report_timing -setup -max_paths 5000 -max_common_paths 1')==1
 assert not any(x.startswith(('set_clock_groups','set_multicycle_path'))for x in sdc)
 inputs=dict(VG_same_as_completed_SYN34=same(build/'g2_top.vg',SOURCE/'impl/gwsynthesis/g2_top.vg'),CST_same_as_routed35=same(build/'g2.cst',REFERENCE/'g2.cst'),device_same_as_routed35=same(build/'impl/pnr/device.cfg',REFERENCE/'impl/pnr/device.cfg'),cmd_options_same_as_routed35=normalized(build/'impl/pnr/cmd.do',build)==normalized(REFERENCE/'impl/pnr/cmd.do',REFERENCE),SDC_original26_hold_plus26_max_delay_unchanged=True,SDC_all3_targets_and_same5000_report=True)
 assert all(inputs.values())
 effectivegate=gate['cdc_first_stage_candidate'];assert effectivegate['physical_audit']['passed']and len(effectivegate['physical_audit']['paths'])==26
 original={k:actual[k]for r in radar for k in(r['source_register'],r['first_register'],r['second_register'])}
 save(out/'actual_global_allocations_and_loads.json',dict(baseline35=a['globals'],PNR49=b['globals'],nets=[dict(net=n,loads=loadcounts[n],pins=dict(pinloads[n]),actual_drivers=globaldrivers[n],failure_net_identity_verified=False)for n in sorted({r['net']for v in(a,b)for r in v['globals']})],placement_related_VO_parameter_keys=sorted(positionkeys),reported_resource_locations_are_global_tracks_not_cell_or_congestion_coordinates=True))
 reset_targets={r['net']for v in(a,b)for r in v['globals']}|{'u_control/model_rst_n','u_control/model_transport_rst_n','u_control/model_thread/n1113_6','u_control/model_thread/gowin_split_inv_from_n1113_s2_onet','u_control/n2375_7','u_control/n2371_7','r2'}
 reset_before=reset_load_inventory(REFERENCE,reset_targets);reset_after=reset_load_inventory(build,reset_targets)
 reset_alias_rows=[r for r in proved_port_changes if r['pin']=='CLEAR'and r['SYN']==['u_control/model_thread/n1113_6']]
 assert len(reset_alias_rows)==548
 module_counts=Counter()
 for r in reset_alias_rows:
  short=r['cell'].removeprefix('u_control/model_thread/');module_counts[short.split('/')[0]]+=1
 roots_proved={row['physical_net']:row['source_actual']['ports']['I'][0]for row in aliases if row['method']=='actual_same_input_INV_copy'}
 assert roots_proved['u_control/model_thread/gowin_split_inv_from_n1113_s2_onet']=='u_control/model_rst_n'
 reset_comparison=dict(passed=True,port_copy_differences=len(proved_port_changes),reset_copy_input_differences=548,reset_copy_root='u_control/model_rst_n',reset_copy_polarity='all copied INV active-high CLEAR when model_rst_n low',reset548_by_actual_module=dict(module_counts),reset548_actual_rows=reset_alias_rows,before35=reset_before,after49=reset_after,scope='Verified logical reset and global load connections only, not identities of427 failed nets or unique failure causality')
 save(out/'actual_reset548_modules_and35_load_comparison.json',reset_comparison)
 result=dict(passed=True,scope='Read-only failedPNR49 actual resource/RAM INIT/CPU/3clock/five related channel target connections/original26/guard preservation. No physical timing or deployment acceptance.',recorded_at=datetime.now().astimezone().isoformat(),build=str(build),runtime=rt,original_process_snapshot=proc,unrouted=427,actual_resources=b['resources'],resource_delta_vs_routed35={k:b['resources'][k]-a['resources'][k]for k in a['resources']},physical_RAM_count=332,all332_geometry_and_explicit_INIT_preserved=True,physical18_wide_split_pairs_retained=True,actual_CPU_image_bytes=131072,FW_bin_bytes=len(binary),all_targeted_register_PLL_DSP_AREG_control_data_and_clock_connections_preserved=True,targeted_actual_cells=len(targetrows),original_radar26_actual_connections_preserved=True,new_five_channel_actual_connections_preserved=True,all12639_or_more_actual_inner_model_FFs_still25MHz=dict(modelclk),actual_three_clock_roots=roots,global_assignment=dict(before35=a['globals'],after49=b['globals']),inputs=inputs,TR_available=False,setup_hold_recovery_pulse_IO_timing=None,effective52_constraint_status=effectivegate.get('effective_constraint_status'),actual_setup_report5000_coverage_verified=False,failed_net_identities_verified=False,failed_net_names=[],congestion_coordinates=None,unrouted_count_not_a_unique_root_cause=True,no_new_SYN_or_PNR_started=True,no_running_inputs_modified=True,programmed=False,deployment_accepted=False,metadata=[meta(p)for p in(build/'tool_runtime.json',build/'impl/pnr/g2_top.vo',build/'impl/pnr/g2_top.rpt.txt',build/'impl/pnr/cmd.do',build/'impl/pnr/device.cfg',CACHE,REVIEW/'final_review.json',PAYLOAD/'final_review.json')])
 save(out/'final_review.json',result)
 print(json.dumps({k:result[k]for k in('passed','unrouted','actual_resources','resource_delta_vs_routed35','targeted_actual_cells','actual_three_clock_roots','TR_available')},ensure_ascii=False))
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--build',type=Path,default=BASE/'baseline_fusion_model25_NAV50_pnr49');ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(exist_ok=False)
 psutil.Process().nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
 try:audit(a.build,a.out)
 except BaseException:save(a.out/'review_failed.json',dict(passed=False,traceback=traceback.format_exc()));raise
