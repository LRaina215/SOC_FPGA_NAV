"""Actual routed PNR32 26-path first-stage hold/max review and source preparation. Never launches a build."""
from pathlib import Path
from collections import defaultdict,Counter
from functools import lru_cache
import sys,re,json,datetime,psutil
psutil.Process().nice(psutil.IDLE_PRIORITY_CLASS);psutil.Process().cpu_affinity([16])
sys.path.insert(0,'D:/Desktop/SOC FPGA/nav/integration/g2')
from audit_pin_loads import CELLS,PORTS
from audit_cdc_first_stages import bits,sdc_fragment
from audit_routing_topology import primitive_directions
from synthesis_connectivity import read,MODULE,CELL
from audit_netlist_memories import PARAM
ROOT=Path('F:/nav_integration_g2_20261023/fusion32_CDC52_preflight1')
PNR=Path('F:/nav_integration_g2_20261023/baseline_fusion_only_pnr32')
OLD=Path('F:/nav_integration_g2_20261023/baseline_fusion_only_syn23')
REF=Path('F:/nav_integration_g2_20261023/pnr27_lidar_cdc_hold_candidate1/review_result.json')
LIB=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
def load(p):return json.loads(p.read_text(encoding='utf-8'))
def save(p,r):p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def meta(p):s=p.stat();return dict(path=str(p),bytes=s.st_size,mtime_ns=s.st_mtime_ns)
initial=load(PNR/'pure_CDC_physical_connection_review.json');reference=load(REF)
assert initial['passed'] and reference['passed'] and len(initial['paths'])==len(reference['paths'])==26
key=lambda r:(r['source_register'],r['first_register'],r['second_register'],r['source_clock'],r['target_clock'],r['route_max_delay_ns'])
assert sorted(map(key,initial['paths']))==sorted(map(key,reference['paths'])), 'previous reviewed26 identity/budgets differ'
metadata={str(p):meta(p) for p in [PNR/'g2_top.vg',PNR/'g2.sdc',PNR/'g2.cst',PNR/'impl/pnr/g2_top.vo',PNR/'impl/pnr/g2_top.tr',OLD/'impl/gwsynthesis/g2_top.vg',REF,LIB]}
assert (PNR/'g2_top.vg').read_bytes()==(OLD/'impl/gwsynthesis/g2_top.vg').read_bytes()
assert (PNR/'g2.sdc').read_bytes()==(OLD/'g2.sdc').read_bytes()
base=(PNR/'g2.sdc').read_text();assert re.findall(r'^create_clock .*$',base,re.M)==['create_clock -name clk50 -period 20 [get_ports {clk50}]','create_clock -name rgmii_rxc -period 10 [get_ports {rgmii_rxc}]']
assert not re.search(r'^set_',base,re.M)
wanted={r[k] for r in initial['paths'] for k in ('source_register','first_register','second_register')}
physical_text=(PNR/'impl/pnr/g2_top.vo').read_text(encoding='utf-8');cells={};directions=primitive_directions(LIB)
# Actual first-stage load counts include clock/reset/CE/RAM/DSP inputs, using vendor directions.
first_qs=set();source_nets=set()
for kind,name,body in CELLS.findall(physical_text):
 name=name.lstrip('\\')
 if name in wanted:
  assert name not in cells;cells[name]=dict(name=name,kind=kind,ports={pin:bits(e) for pin,e in PORTS.findall(body)})
assert cells.keys()==wanted
first_qs={cells[r['first_register']]['ports']['Q'][0] for r in initial['paths']}
source_nets={cells[r['first_register']]['ports']['D'][0] for r in initial['paths']}
loads=defaultdict(list);drivers=defaultdict(list)
for kind,name,body in CELLS.findall(physical_text):
 name=name.lstrip('\\')
 for pin,e in PORTS.findall(body):
  for net in bits(e):
   if net not in first_qs|source_nets:continue
   assert pin in directions.get(kind,{}),('unclassified queried pin',kind,pin)
   if directions[kind][pin] in ('input','inout') and net in first_qs:loads[net].append((name,pin))
   if directions[kind][pin] in ('output','inout') and net in source_nets:drivers[net].append((name,pin))
pp={}
for name,k,v in PARAM.findall(physical_text):
 name=name.strip().lstrip('\\')
 if name in wanted:
  assert k not in pp.setdefault(name,{});pp[name][k]=v.strip()
ancestors=set()
for name in wanted:
 parts=name.split('/')
 ancestors.update('/'.join(parts[:i]) for i in range(1,len(parts)+1))
syn=read(PNR/'g2_top.vg',lambda n:n in ancestors or n.startswith('u_nav/') and n.count('/')==1)
assert wanted<=syn.keys(),('selected primitive missing',wanted-syn.keys())
mods=dict(MODULE.findall((PNR/'g2_top.vg').read_text(encoding='utf-8')))
@lru_cache(None)
def child(kind):return {n.lstrip('\\'):k for k,n,_ in CELL.findall(mods[kind])}
@lru_cache(None)
def kind_at(h):
 if not h:return 'g2_top'
 par,_,local=h.rpartition('/');return child(kind_at(par))[local]
@lru_cache(None)
def params(kind):
 out={}
 for n,k,v in PARAM.findall(mods[kind]):
  n=n.strip().lstrip('\\');assert k not in out.setdefault(n,{});out[n][k]=v.strip()
 return out
lib_text=LIB.read_text(encoding='utf-8')
def default_init(kind):
 body=re.search(r'\bmodule\s+'+re.escape(kind)+r'\b(.*?)\bendmodule',lib_text,re.S).group(1)
 return re.search(r'\bparameter\s+INIT\s*=\s*([^;]+);',body).group(1).strip()
replicated_states=[]
def physical_driver(net):
 found=[]
 for match in CELLS.finditer(physical_text):
  kind,name,body=match.groups();ports={p:bits(e) for p,e in PORTS.findall(body)}
  if any(net in ns and directions.get(kind,{}).get(pin) in ('output','inout') for pin,ns in ports.items()):
   found.append(dict(name=name.lstrip('\\'),kind=kind,ports=ports))
 assert len(found)==1,('physical D driver not unique',net)
 return found[0]
def physical_params(name):
 out={}
 for inst,key,value in PARAM.findall(physical_text):
  if inst.strip().lstrip('\\')==name:
   assert key not in out;out[key]=value.strip()
 return out
rows=[]
for row in initial['paths']:
 row=dict(row);records={}
 for role in ('source_register','first_register','second_register'):
  name=row[role];c=cells[name];sc=syn[name]
  exact=c==sc;replica=None
  if not exact:
   assert role=='source_register' and c['kind']==sc['kind']
   differing={pin for pin in c['ports'].keys()|sc['ports'].keys() if c['ports'].get(pin)!=sc['ports'].get(pin)}
   assert differing=={'D'} and len(c['ports']['D'])==len(sc['ports']['D'])==1,('unproved CDC register change',name,differing)
   olddrivers=[v for v in syn.values() if v['kind'].startswith('DFF') and v['ports'].get('Q')==sc['ports']['D']]
   assert len(olddrivers)==1;old=olddrivers[0];new=physical_driver(c['ports']['D'][0])
   assert old['kind']==new['kind'] and old['kind'].startswith('DFF')
   assert {pin:nets for pin,nets in old['ports'].items() if pin!='Q'}=={pin:nets for pin,nets in new['ports'].items() if pin!='Q'},('replicated DFF inputs differ',old,new)
   op,_,ol=old['name'].rpartition('/');oldp=params(kind_at(op)).get(ol,{})
   newp=physical_params(new['name']);assert set(oldp)<= {'INIT'} and set(newp)<= {'INIT'}
   assert oldp.get('INIT',default_init(old['kind']))==newp.get('INIT',default_init(new['kind']))
   replica=dict(consumer=name,pin='D',original_state=old,physical_replica=new,original_parameters=oldp,replica_parameters=newp,all_state_inputs_identical=True,effective_INIT_identical=True,proof='Same initial state and identical D/CLK/CE/reset transition; inductive state equality, no generated-name assumption')
   replicated_states.append(replica)
  par,_,local=name.rpartition('/');sp=params(kind_at(par)).get(local,{})
  assert set(sp)<= {'INIT'} and set(pp.get(name,{}))<= {'INIT'},('unexpected register parameters',name,sp,pp.get(name,{}))
  published=default_init(c['kind']);assert published in ("1'b0","1'b1")
  effective_syn=sp.get('INIT',published);effective_physical=pp.get(name,{}).get('INIT',published)
  assert effective_syn==effective_physical,('effective INIT changed',name,sp,pp.get(name,{}))
  if role!='source_register':assert effective_syn=="1'b0"
  records[role]=dict(primitive=c,synthesis_explicit_parameters=sp,physical_explicit_parameters=pp.get(name,{}),published_vendor_INIT_default=published,effective_INIT=effective_syn,frozen_VG_connections_identical=exact,frozen_VG_connections_identical_or_exact_replica_proved=True,replica_transition_proof=replica,effective_initialization_identical=True)
 source,first,second=[cells[row[k]] for k in ('source_register','first_register','second_register')]
 assert source['ports']['Q']==first['ports']['D'] and len(first['ports']['D'])==1
 assert drivers[first['ports']['D'][0]]==[(source['name'],'Q')]
 assert first['ports']['Q']==second['ports']['D'] and len(first['ports']['Q'])==1
 assert loads[first['ports']['Q'][0]]==[(second['name'],'D')]
 assert source['ports']['CLK']==[row['source_clock']]
 for c in (first,second):
  assert c['ports']['CLK']==[row['target_clock']]
  assert c['ports'].get('CE',['VCC'])==['VCC']
 assert source['ports']['CLK']!=first['ports']['CLK']
 reset_pins={'RESET','CLEAR','PRESET','SET'}
 assert {p:v for p,v in first['ports'].items() if p in reset_pins}=={p:v for p,v in second['ports'].items() if p in reset_pins}
 assert first['kind']==second['kind']
 row.update(actual_source_first_second=records,all_vendor_direction_first_Q_loads=loads[first['ports']['Q'][0]],same_first_second_reset_connections=True,source_reset_assertion_release_signoff=False)
 rows.append(row)
# Bind the exact observed 12 negative-hold paths, retaining all 25 vendor sample rows.
tr=(PNR/'impl/pnr/g2_top.tr').read_text(encoding='utf-8')
def table(title,end):
 sec=tr.split('\n'+title+'\n',1)[1].split('\n'+end+'\n',1)[0];out=[]
 for ln in sec.splitlines():
  a=ln.split()
  if len(a)==9 and a[0].isdigit():
   out.append(dict(index=int(a[0]),slack_ns=float(a[1]),source_pin=a[2],target_pin=a[3],source_clock=a[4],target_clock=a[5],relation_ns=float(a[6]),skew_ns=float(a[7]),data_delay_ns=float(a[8])))
 return out
hold=table('3.1.2 Hold Paths Table','3.1.3 Recovery Paths Table');setup=table('3.1.1 Setup Paths Table','3.1.2 Hold Paths Table')
assert len(hold)==len(setup)==25
neg=[r for r in hold if r['slack_ns']<0];assert len(neg)==12
pairs={(r['source_register']+'/Q',r['first_register']+'/D'):r for r in rows}
for r in neg:
 k=r['source_pin'],r['target_pin'];assert k in pairs
 assert pairs[k]['group'] in ('rx_write_gray','rx_clock_monitor') and (r['source_clock'],r['target_clock'])==('rgmii_rxc:[R]','clk50:[R]')
 assert pairs[k]['route_max_delay_ns']==8.0
 r['audited_group']=pairs[k]['group'];r['planned_hold_only_exception_covers_exact_path']=True
assert int(re.search(r'<Numbers of Setup Violated Endpoints>:(\d+)',tr)[1])==3781
assert int(re.search(r'<Numbers of Hold Violated Endpoints>:(\d+)',tr)[1])==12
assert all(r['source_clock']==r['target_clock']=='clk50:[R]' and (r['source_pin'],r['target_pin']) not in pairs for r in setup)
fragment=sdc_fragment(initial);full=base+'\n'+fragment
assert len(re.findall(r'^set_false_path .* -hold$',full,re.M))==26
assert len(re.findall(r'^set_max_delay ',full,re.M))==26
assert not re.search(r'^set_false_path (?![^\n]*-hold$)',full,re.M)
assert not re.search(r'^set_(?:clock_groups|multicycle_path|clock_uncertainty|clock_latency)',full,re.M)
assert re.findall(r'^create_clock .*$',full,re.M)==re.findall(r'^create_clock .*$',base,re.M)
for r in rows:assert r['route_max_delay_ns']<{'rgmii_rxc_d':10.0,'clk50_d':20.0}[r['source_clock']]
# Unchanged after read-only audit; no hashes and no changes to any PNR input.
for path,m in metadata.items():assert meta(Path(path))==m,('input changed during audit',path)
(ROOT/'lidar26_first_stage_only.sdc').write_text(fragment,encoding='utf-8',newline='\n')
(ROOT/'g2_fusion32_CDC52_candidate.sdc').write_bytes((PNR/'g2.sdc').read_bytes()+b'\n'+fragment.encode('utf-8'))
result=dict(schema=1,recorded_at=datetime.datetime.now().astimezone().isoformat(),passed=True,read_only_audit=True,source_frozen_netlist=str(PNR/'g2_top.vg'),actual_physical_netlist=str(PNR/'impl/pnr/g2_top.vo'),actual_timing_report=str(PNR/'impl/pnr/g2_top.tr'),original_26_review_reference=str(REF),frozen_source_VG_and_SDC_bytes_equal_to_original=True,metadata=metadata,reviewed_paths=26,group_counts=dict(Counter(r['group'] for r in rows)),actual_registers_audited=len(wanted),proved_actual_replication_aliases=replicated_states,original_clock_file_bytes_prefix_preserved=True,paths=rows,actual_negative_hold_paths=neg,all_observed_12_negative_hold_are_audited_async_first_stage=True,actual_hold_sample=hold,actual_setup_sample=setup,existing_setup_violated_endpoints=3781,same_domain_setup_constraints_relaxed=False,unchanged_clock_periods_ns={'clk50':20,'rgmii_rxc':10},hold_only_false_paths=26,explicit_max_delays=26,route_budgets_ns={'rx_write_gray':8.0,'core_read_gray':18.0,'rx_clock_monitor':8.0,'rx_sampling_select':18.0},second_stage_paths_not_excepted=True,reset_and_pulse_width_paths_not_excepted=True,physical_constraint_application_or_delay_signoff=False,MTBF_signoff=False,full_CDC_signoff=False,deployment_accepted=False,PNR_inputs_modified=False,new_build_started=False,candidate_full_sdc=str(ROOT/'g2_fusion32_CDC52_candidate.sdc'),candidate_fragment=str(ROOT/'lidar26_first_stage_only.sdc'),execution_policy={'priority':psutil.Process().nice(),'cpu_affinity':psutil.Process().cpu_affinity()})
save(ROOT/'review_result.json',result)
print(json.dumps({k:v for k,v in result.items() if k in ['passed','reviewed_paths','actual_registers_audited','all_observed_12_negative_hold_are_audited_async_first_stage','existing_setup_violated_endpoints','candidate_full_sdc']},ensure_ascii=False,indent=2))
