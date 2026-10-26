"""Read-only final PNR50 implementation/clock/lease/channel review; no EDA."""
from pathlib import Path
from datetime import datetime
from collections import Counter,defaultdict
import argparse,json,re,traceback,psutil
from synthesis_connectivity import read,DECL,TOKEN,PORT
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from audit_netlist_memories import PARAM
from audit_routing_topology import primitive_directions
from prepare_baseline_fusion_retime import metadata,same_bytes
from prove_soc_console_remap import literal
BASE=Path('F:/nav_integration_g2_20261023')
SYN=BASE/'baseline_fusion_model25_bounded_MAX_syn37'
PNR=BASE/'baseline_fusion_model25_bounded_MAX_pnr50'
REF49=BASE/'baseline_fusion_model25_NAV50_pnr49'
REF35=BASE/'baseline_fusion32_CDC52_pnr35'
LIB=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
# PNR53/54 put this standard annotation before exactly3 explicit DFFREs.
# Accept only the observed Gowin IO-register Boolean-string attribute; keep
# the original3 capture groups and all port/primitive semantic checks intact.
IO_ATTRIBUTE=r'(?:\(\*\s*gowin_io_reg\s*=\s*"(?:TRUE|FALSE)"\s*\*\)\s*)?'
CELL=re.compile(r'^'+IO_ATTRIBUTE+r'(\w+)\s+(\\\S+|[^\s(]+)\s*\(\s*\n(.*?)\)\s*;',re.M|re.S)
CLOCK=re.compile(r'CLK\d*|CLKA|CLKB|CLOCK|WCLK|RCLK')
BUFFERS={'IBUF','BUF','BUFG','INV'}
CHANNELS=[('BMI','u_control/model_thread/bmi_bridge',128),('configuration','u_control/model_thread/config_bridge',140),('action','u_control/model_thread/action_bridge',336),('fusion','u_control/model_thread/fusion_bridge',145),('status','u_control/model_thread/status_bridge',15)]
def load(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def save(p,v):assert not p.exists(),p;p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
def selected(n):return n.startswith('u_command_lease/')or n.startswith('u_control/model_clock_guard/')or any(n==p or n.startswith(p+'/')for _,p,w in CHANNELS)
def resource(r):
 c=Counter(x['kind']for x in r.values());return dict(FF=sum(v for k,v in c.items()if k.startswith('DFF')),LUT=sum(v for k,v in c.items()if re.fullmatch(r'LUT[1-6]',k)),ALU=c['ALU'],primitives=dict(c))
def parse_vo(path,extra):
 text=path.read_text(encoding='utf8');assert not re.search(r'^assign\s',text,re.M)
 attribute_heads=[m[0]for m in re.finditer(r'^[ \t]*\(\*.*$',text,re.M)]
 assert all(re.match(r'^'+IO_ATTRIBUTE+r'\w+\s+(?:\\\S+|[^\s(]+)\s*\(',s)for s in attribute_heads),('unsupported physical attribute syntax',attribute_heads)
 declarations={n.lstrip('\\'):(d,int(h)if h else None,int(l)if l else None)for d,h,l,n in DECL.findall(text)}
 def expand(expr):
  ns=[];pos=0
  while pos<len(expr):
   if not expr[pos:].strip():break
   m=TOKEN.match(expr,pos);assert m,('unsupported VO net expression',expr[pos:]);pos=m.end();w,b,v,n,h,l,punct=m.groups()
   if punct:continue
   if w:
    assert not re.search('[xXzZ]',v);value=int(v,{'b':2,'h':16,'d':10}[b]);ns+=['VCC'if value>>i&1 else'GND'for i in range(int(w)-1,-1,-1)]
   else:
    n=n.lstrip('\\')
    if h is not None:ns +=[n+'['+str(i)+']'for i in range(int(h),int(l or h)+(1 if int(l or h)>=int(h)else-1),1 if int(l or h)>=int(h)else-1)]
    elif n in declarations and declarations[n][1]is not None:
     _,h,l=declarations[n];ns +=[n+'['+str(i)+']'for i in range(h,l+(1 if l>=h else-1),1 if l>=h else-1)]
    else:ns.append(n)
  return ns
 cells={};clockpins={};total=0;ff=0
 for m in CELL.finditer(text):
  kind,name,body=m.groups()
  if kind=='module':continue
  assert not re.search(r'^defparam\s|^(?:DFF\w*|RAM16\w*)\s',body,re.M),('absorbed cells',name)
  n=name.lstrip('\\');total+=1;ff+=kind.startswith('DFF')
  raw={p.lstrip('\\'):v for p,v in PORT.findall(body)}
  for pin,v in raw.items():
   if CLOCK.fullmatch(pin):
    for i,net in enumerate(expand(v)):
     if net not in('GND','VCC'):clockpins[(n,pin,i)]=net
  if not(selected(n)or n in extra or kind.startswith('DFF')or kind in BUFFERS or kind=='PLL'):continue
  cells[n]=dict(name=n,kind=kind,ports={p:expand(v)for p,v in raw.items()})
 assert ff==len(re.findall(r'^'+IO_ATTRIBUTE+r'DFF\w*\s+(?:\\\S+|[^\s(]+)\s*\(',text,re.M))
 params={n:{}for n in cells}
 for name,k,v in PARAM.findall(text):
  n=name.strip().lstrip('\\')
  if n in params:assert k not in params[n];params[n][k]=v.strip()
 return cells,params,clockpins,dict(leaf_instances=total,explicit_DFF_heads=ff,parser_handles_inline_RAM_DO_close=True,parser_handles_known_gowin_IO_attribute=True,known_attributed_primitive_heads=len(attribute_heads))
def report(root):
 p=root/'impl/pnr/g2_top.rpt.txt';t=p.read_text();r={}
 for name in('Logic','Register','CLS','BSRAM','DSP'):
  m=re.search(r'^\s*'+name+r'\s*\|\s*([\d.]+)/([\d.]+)',t,re.M);assert m;r[name]=dict(used=float(m[1])if'.'in m[1]else int(m[1]),available=float(m[2])if'.'in m[2]else int(m[2]))
 m=re.search(r'--LUT,ALU,ROM16\s*\|\s*\d+\((\d+) LUT, (\d+) ALU, (\d+) ROM16\)',t);assert m
 r.update(LUT=int(m[1]),ALU=int(m[2]),ROM16=int(m[3]),SSRAM=int(re.search(r'--SSRAM\(RAM16\)\s*\|\s*(\d+)',t)[1]))
 section=re.search(r'6\.[^\n]*\n(.*?)7\. Pinout by Port Name',t,re.S);assert section;part=section[1]
 rows=[]
 for line in part.splitlines():
  cols=[x.strip()for x in line.split('|')]
  if len(cols)==3 and cols[1]in('PRIMARY','LW','HCLK'):rows.append(dict(signal=cols[0],kind=cols[1],locations=cols[2].split()))
 r['global_assignment_rows']=rows;return r
def run(out):
 out.mkdir(exist_ok=False);watched=[SYN/'impl/gwsynthesis/g2_top.vg',PNR/'impl/pnr/g2_top.vo',PNR/'impl/pnr/g2_top.rpt.txt',PNR/'tool_runtime.json'];binding=[metadata(p)for p in watched]
 ref=load(BASE/'model25_SYN34_actual_review1/actual_review4/actual_original_radar26.json');assert ref['passed']and len(ref['paths'])==26
 extra={row[role]for row in ref['paths']for role in('source_register','first_register','second_register')}
 print('parse completed SYN37 and selected actual physical VO',flush=True)
 modules=definitions(watched[0]);source=read(watched[0]);sp=parameter_reader(modules);directions=primitive_directions(LIB)
 libtext=LIB.read_text();defaults={}
 for kind,body in re.findall(r'\bmodule\s+(\w+)\b(.*?)\bendmodule',libtext,re.S):
  defaults[kind]={k:v.strip()for k,v in re.findall(r'\bparameter\s+(\w+)\s*=\s*([^;]+);',body)}
 def effective(kind,overrides):
  result=dict(defaults[kind]);result.update(overrides)
  def norm(v):
   try:return literal(v)
   except ValueError:return v.strip()
  return {k:norm(v)for k,v in result.items()}
 actual,pp,clockpins,parser=parse_vo(watched[1],extra)
 source_params={n:sp(c)for n,c in source.items()}
 def drivers(cells):
  d=defaultdict(list)
  for n,c in cells.items():
   if c['kind']not in BUFFERS|{'PLL'}:continue
   for p,ns in c['ports'].items():
    if directions[c['kind']][p]=='output':
     for net in ns:d[net].append((n,p))
  return d
 def canonical(cells,d,net,seen=()):
  if net in('GND','VCC'):return(net,0)
  assert net not in seen,('buffer cycle',net)
  ds=d.get(net,[])
  if not ds:return(net,0)
  assert len(ds)==1,('clock/buffer driver',net,ds)
  n,p=ds[0];c=cells[n]
  if c['kind']=='PLL':return(n+'/'+p,0)
  if c['kind']=='IBUF':return(c['ports']['I'][0],0)
  assert c['kind']in('BUF','BUFG','INV')
  r,v=canonical(cells,d,c['ports']['I'][0],seen+(net,));return r,v^int(c['kind']=='INV')
 sd,pd=drivers(source),drivers(actual)
 source_Q={c['ports']['Q'][0]:n for n,c in source.items()if c['kind'].startswith('DFF')}
 physical_Q={c['ports']['Q'][0]:n for n,c in actual.items()if c['kind'].startswith('DFF')}
 cloned_aliases={}
 def prove_replica_alias(x,y):
  ca,cb=canonical(source,sd,x),canonical(actual,pd,y)
  if ca==cb:return ca
  assert ca[1]==cb[1] and ca[0]in source_Q and cb[0]in physical_Q,('not a buffer or registered replica alias',x,y,ca,cb)
  original=source_Q[ca[0]];copy=physical_Q[cb[0]];assert original in actual,('replica original FF missing',original,copy)
  a,b=actual[original],actual[copy]
  assert a['kind']==b['kind'] and effective(a['kind'],pp[original])==effective(b['kind'],pp[copy]),('replica kind/INIT',original,copy)
  assert a['ports'].keys()==b['ports'].keys()
  for pin in a['ports']:
   if pin=='Q':continue
   assert [canonical(actual,pd,v)for v in a['ports'][pin]]==[canonical(actual,pd,v)for v in b['ports'][pin]],('replica transition input',original,copy,pin)
  aa=source[original];assert aa['kind']==a['kind'] and effective(aa['kind'],source_params[original])==effective(a['kind'],pp[original])
  assert aa['ports'].keys()==a['ports'].keys()
  for pin in aa['ports']:
   if pin=='Q':continue
   assert [canonical(source,sd,v)for v in aa['ports'][pin]]==[canonical(actual,pd,v)for v in a['ports'][pin]],('replica anchor transition input changed',original,pin)
  cloned_aliases[(x,y)]=dict(source_net=x,physical_net=y,source_original_FF=original,physical_replica_FF=copy,same_kind_all_effective_INIT_parameters_and_all_transition_inputs=True,source_original_physical_anchor_transition_identical=True)
  return ca
 def equal_cell(n):
  a,b=source.get(n),actual.get(n);assert a and b,('missing selected actual cell',n)
  original_kinds=[a['kind'],b['kind']]
  def normalize_buffer(c,p):
   if c['kind']in('INV','BUF'):
    assert set(c['ports'])=={'I','O'}
    return dict(name=c['name'],kind='LUT1',ports={'I0':c['ports']['I'],'F':c['ports']['O']}),{'INIT':"2'h1"if c['kind']=='INV'else"2'h2"}
   return c,p
  a,ap=normalize_buffer(a,source_params[n]);b,bp=normalize_buffer(b,pp[n])
  assert a['kind']==b['kind'],('actual primitive kind differs',n,a['kind'],b['kind'])
  ea,eb=effective(a['kind'],ap),effective(b['kind'],bp);assert ea.keys()==eb.keys()
  parameter_differences={k:dict(source=ea[k],physical=eb[k])for k in ea if ea[k]!=eb[k]}
  if parameter_differences:
   assert a['kind']=='PLL'and set(parameter_differences)<={f'ODIV{i}_SEL'for i in range(1,7)},('actual active primitive parameters differ',n,parameter_differences)
   wholevo=watched[1].read_text()
   for k in parameter_differences:
    i=int(re.fullmatch(r'ODIV([1-6])_SEL',k)[1]);enable=f'CLKOUT{i}_EN';pin=f'CLKOUT{i}'
    assert ea[enable]==eb[enable]=='\"FALSE\"'
    assert len(a['ports'][pin])==len(b['ports'][pin])==1
    net=a['ports'][pin][0];assert net==b['ports'][pin][0]
    assert not[(cn,p)for cn,c in source.items()for p,ns in c['ports'].items()if directions[c['kind']][p]in('input','inout')and net in ns],('disabled PLL source output used',net)
    assert wholevo.count('\\'+net+' ')==2,('disabled PLL physical output not only declaration and output pin',net,wholevo.count('\\'+net+' '))
   del wholevo
  assert a['ports'].keys()==b['ports'].keys(),('actual portset',n)
  diffs=[]
  for pin,ns in a['ports'].items():
   v=b['ports'][pin];assert len(ns)==len(v),(n,pin)
   for i,(x,y)in enumerate(zip(ns,v)):
    if x!=y:
     ca=prove_replica_alias(x,y)
     diffs.append(dict(pin=pin,index=i,source_net=x,physical_net=y,canonical_root=ca))
  return dict(name=n,logical_comparison_kind=a['kind'],source_kind=original_kinds[0],physical_kind=original_kinds[1],buffer_LUT1_reencoding_checked=original_kinds[0]!=original_kinds[1],all_effective_vendor_parameters_INIT_equal=not bool(parameter_differences),disabled_unused_PLL_output_divider_changes=parameter_differences,source_explicit_parameters=source_params[n],physical_explicit_parameters=pp[n],all_pins_equal_or_proved_buffer_inversion_alias=True,proved_pin_aliases=diffs)
 print('check full selected channel/guard/lease primitive mapping',flush=True)
 related={};checks=[]
 for name,prefix,width in CHANNELS:
  names={n for n,c in source.items()if (n==prefix or n.startswith(prefix+'/'))and c['kind']not in('VCC','GND')};physical={n for n,c in actual.items()if (n==prefix or n.startswith(prefix+'/'))and c['kind']not in('VCC','GND')}
  assert names==physical,('related channel cellset',name,sorted(names-physical),sorted(physical-names))
  cs=[equal_cell(n)for n in sorted(names)];checks+=cs;related[name]=dict(logical_width=width,actual_cells=len(cs),source_physical_primitive_ports_and_INIT_preserved=True,resources=resource({n:actual[n]for n in names}),alias_pin_count=sum(len(c['proved_pin_aliases'])for c in cs))
 for n in sorted(extra):checks.append(equal_cell(n))
 original26=[]
 for row in ref['paths']:
  a,b,c=[actual[row[role]]for role in('source_register','first_register','second_register')]
  assert a['ports']['Q']==b['ports']['D']and b['ports']['Q']==c['ports']['D']
  assert canonical(actual,pd,a['ports']['CLK'][0])==canonical(source,sd,source[row['source_register']]['ports']['CLK'][0])
  assert canonical(actual,pd,b['ports']['CLK'][0])==canonical(source,sd,source[row['first_register']]['ports']['CLK'][0])
  assert canonical(actual,pd,c['ports']['CLK'][0])==canonical(source,sd,source[row['second_register']]['ports']['CLK'][0])
  original26.append(dict(source=a['name'],first=b['name'],second=c['name'],source_to_first_to_second_preserved=True))
 # Full selected first-Q fanout is recounted from the unfiltered VO text to avoid scope-only claims.
 first_q={actual[row['first_register']]['ports']['Q'][0]:row['second_register']for row in ref['paths']}
 text=watched[1].read_text();fqloads=defaultdict(list)
 for kind,n,body in CELL.findall(text):
  if kind=='module':continue
  n=n.lstrip('\\')
  for pin,e in PORT.findall(body):
   if directions[kind][pin.lstrip('\\')]=='output':continue
   plain=e.strip().lstrip('\\');plain=re.sub(r'\s+(?=\[)','',plain)
   if plain in first_q:fqloads[plain].append((n,pin))
 for q,n in first_q.items():assert fqloads[q]==[(n,'D')],('physical firststage fanout',q,fqloads[q])
 save(out/'actual_related_channels_original26.json',dict(passed=True,channels=related,primitive_mapping_checks=checks,original26=original26,first_stage_single_fanout_recounted=True,proved_registered_replica_aliases=list(cloned_aliases.values()),physical_payload_delay_reset_release_MTBF_signed_off=False))
 print('trace all actual nonconstant clock pins',flush=True)
 roots=Counter();clockrows=[]
 for key,net in clockpins.items():
  r,phase=canonical(actual,pd,net)
  root={'clk50':'clk50','rgmii_rxc':'rgmii_rxc','u_model_pll/PLL_inst/CLKOUT0':'model25'}.get(r)
  assert root is not None,('unknown physical clock root',key,net,r,phase)
  roots[(root,phase)]+=1
  clockrows.append(dict(instance=key[0],pin=key[1],index=key[2],actual_net=net,root=root,inverted=bool(phase)))
 pll='u_model_pll/PLL_inst';pllcheck=equal_cell(pll)
 save(out/'actual_three_clock_roots_PLL.json',dict(passed=True,all_nonconstant_actual_clock_pins=len(clockpins),root_pin_counts=[dict(root=r,inverted=bool(p),pins=n)for(r,p),n in sorted(roots.items())],clock_pin_traces=clockrows,PLL=actual[pll],PLL_parameters=pp[pll],PLL_active_parameters_ports_equal_unused_disabled_output_dividers_audited=pllcheck,route_delay_and_timing_accepted=False))
 print('check explicit physical lease MAX-tree and all200 actual lease FFs',flush=True)
 lease={n:c for n,c in source.items()if n.startswith('u_command_lease/')and c['kind']not in('VCC','GND')};ple={n:c for n,c in actual.items()if n.startswith('u_command_lease/')and c['kind']not in('VCC','GND')}
 ffn={n for n,c in lease.items()if c['kind'].startswith('DFF')};pffn={n for n,c in ple.items()if c['kind'].startswith('DFF')};assert ffn==pffn and len(ffn)==200
 ffchecks=[equal_cell(n)for n in sorted(ffn)]
 prefix='u_command_lease/g_deadline_nonzero.';expected=[prefix+f'g_max4[{i}].u_and'for i in range(8)]+[prefix+'u_low16',prefix+'u_high16',prefix+'u_nonmax32']
 assert {n for n in ple if n.startswith(prefix)}==set(expected)
 treechecks=[equal_cell(n)for n in expected]
 groups=[];tree=[]
 age={int(re.fullmatch(r'u_command_lease/age_ms_(\d+)_s\d+',n)[1]):c for n,c in ple.items()if re.fullmatch(r'u_command_lease/age_ms_(\d+)_s\d+',n)};assert set(age)==set(range(32))
 for i in range(8):
  c=ple[expected[i]];assert c['kind']=='LUT4'and literal(pp[c['name']]['INIT'])==0x8000
  assert[c['ports']['I'+str(j)][0]for j in range(4)]==[age[i*4+j]['ports']['Q'][0]for j in range(4)]
  groups+=c['ports']['F'];tree.append(dict(instance=c['name'],level=1,age_bits=list(range(i*4,i*4+4))))
 for i,n in enumerate(expected[8:10]):
  c=ple[n];assert c['kind']=='LUT4'and literal(pp[n]['INIT'])==0x8000
  assert[c['ports']['I'+str(j)][0]for j in range(4)]==groups[i*4:i*4+4];tree.append(dict(instance=n,level=2))
 c=ple[expected[-1]];assert c['kind']=='LUT2'and literal(pp[c['name']]['INIT'])==7
 assert c['ports']['I0']==ple[expected[8]]['ports']['F']and c['ports']['I1']==ple[expected[9]]['ports']['F']
 tree.append(dict(instance=c['name'],level=3));root=c['ports']['F'][0]
 # Compare every lease primitive, not only FF pins, so the existing SYN37 all-state next-state proof can carry over despite buffer aliases.
 lease_names=set(lease);actual_names=set(ple);extra_lease=sorted(actual_names-lease_names);missing_lease=sorted(lease_names-actual_names)
 lease_all=[equal_cell(n)for n in sorted(lease_names&actual_names)]
 assert not missing_lease and all(ple[n]['kind']in('BUF','BUFG','INV')for n in extra_lease),('lease cellset',missing_lease,extra_lease)
 save(out/'actual_lease_tree_200FF.json',dict(passed=True,actual_FFs=200,FF_kind_INIT_clock_and_all_control_D_Q_pins_preserved=ffchecks,all_lease_primitive_checks=lease_all,added_buffer_cells=[ple[n]for n in extra_lease],actual_explicit_LUTs=11,actual_age_Q_to_nonmax_levels=3,tree=tree,tree_cell_checks=treechecks,root=root,root_immediate_loads=[dict(instance=n,pin=p,index=i)for n,c in ple.items()for p,ns in c['ports'].items()if directions[c['kind']][p]!='output'for i,v in enumerate(ns)if v==root],no_physical_timing_or_failed_net_identity_claim=True))
 reports={tag:report(folder)for tag,folder in [('35',REF35),('49',REF49),('50',PNR)]}
 runtime=load(PNR/'tool_runtime.json');assert runtime['cancelled']is False and runtime['max_runtime_s']==runtime['max_no_observable_progress_s']==0 and runtime['tool_exit']==1
 process_identities=[]
 for pid,created in [(13292,1792992266.360427),(64260,1792992162.0133598)]:
  exists=psutil.pid_exists(pid);now=psutil.Process(pid).create_time()if exists else None;original_present=exists and abs(now-created)<1
  assert not original_present,('original PNR50 identity still present',pid,created,now)
  process_identities.append(dict(pid=pid,original_creation_epoch=created,current_PID_exists=exists,current_creation_epoch=now,original_identity_present=False))
 assert not(PNR/'impl/pnr/g2_top.tr').exists()
 assert same_bytes(PNR/'g2.cst',REF49/'g2.cst')and same_bytes(PNR/'g2.sdc',REF49/'g2.sdc')
 assert same_bytes(PNR/'impl/pnr/device.cfg',REF49/'impl/pnr/device.cfg')
 a=(REF49/'impl/pnr/cmd.do').read_text().replace(str(REF49),str(PNR)).replace(REF49.as_posix(),PNR.as_posix());assert a==(PNR/'impl/pnr/cmd.do').read_text()
 assert same_bytes(SYN/'impl/gwsynthesis/g2_top.vg',PNR/'g2_top.vg')
 assert all(metadata(Path(row['path']))==row for row in binding)
 result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),actual_PNR_failed=True,unrouted=1659,no_failed_net_names_or_congestion_coordinates_identified=True,TR_exists=False,all_target_setup_hold_all52_effectiveness_5000coverage_physical_CDC_reset_pulse_IO_unknown=True,runtime=runtime,reports=reports,physical_parser=parser,process_identities=process_identities,actual_related_channels=related,original26_physical_endpoint_connections_preserved=True,actual3clock_roots_preserved=True,actual11LUT3level_tree_preserved=True,actual200leaseFFs_and_logic_preserved=True,memory_CPU_review_separate=True,source_VG_frozen_byte_equal=True,cmd_options_device_CST_SDC_same49=True,global_assignment_changes_not_failure_net_identity_or_unique_cause=True,metadata_bindings=binding,new_EDA_started=False)
 save(out/'result.json',result);print(json.dumps(dict(passed=True,out=str(out),unrouted=1659,actual_lease_FFs=200,actual_tree_LUTs=11,actual_tree_depth=3,actual_related_channels=related,clock_roots=list(roots)),ensure_ascii=False),flush=True)
 return result
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args()
 try:run(a.out)
 except Exception as e:
  if a.out.exists():save(a.out/'review_failed.json',dict(passed=False,error=str(e),traceback=traceback.format_exc(),recorded_at=datetime.now().astimezone().isoformat()))
  raise
