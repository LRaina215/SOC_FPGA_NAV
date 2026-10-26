"""Read-only actual SYN34 RAM/INIT/three-clock/related-transport inspection.
No construction, constraints mutation, PNR, or hardware actions are performed.
"""
from pathlib import Path
import argparse,json,re,sys,time,traceback,collections,datetime
sys.path.insert(0,'D:/Desktop/SOC FPGA/nav/integration/g2')
from synthesis_connectivity import read,CELL,DECL,TOKEN
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from audit_netlist_memories import BSRAM
from audit_routing_topology import primitive_directions
from review_baseline_single_factor_syn import cpu_image
from prepare_no_sfp_clock_plan import construct_shared_logic
from prove_soc_console_remap import Logic,literal,z3
from retry_frozen_synthesis import memory_status

BASE=Path('F:/nav_integration_g2_20261023')
OLD=BASE/'baseline_fusion_only_syn23'
NEW=BASE/'baseline_fusion_model25_NAV50_syn34'
PREP=BASE/'baseline_fusion_model25_NAV50_prepared4'
LIB=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
REF=BASE/'pnr27_lidar_cdc_hold_candidate1/review_result.json'
PREFIX='u_control/model_thread/'
CHANNELS=[('BMI',PREFIX+'bmi_bridge',128,'clk50','model25'),
 ('configuration',PREFIX+'config_bridge/transfer',140,'clk50','model25'),
 ('action',PREFIX+'action_bridge',336,'model25','clk50'),
 ('fusion',PREFIX+'fusion_bridge',145,'model25','clk50'),
 ('status',PREFIX+'status_bridge/transfer',15,'model25','clk50')]

def load(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def save(p,v):Path(p).write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
def meta(p):p=Path(p);s=p.stat();return dict(path=str(p),bytes=s.st_size,mtime_ns=s.st_mtime_ns)
def same(a,b):
 if a.stat().st_size!=b.stat().st_size:return False
 with a.open('rb')as x,b.open('rb')as y:
  while True:
   aa=x.read(1024*1024);bb=y.read(1024*1024)
   if aa!=bb:return False
   if not aa:return True

def module_bindings(modules,targets):
 cache={};found={}
 def definition(kind):
  if kind not in cache:
   body=modules[kind]
   assert not re.search(r'^\s*assign\s',body,re.M)
   decl={n.lstrip('\\'):(d,int(hi)if hi else None,int(lo)if lo else None)for d,hi,lo,n in DECL.findall(body)}
   cache[kind]=(decl,[(t,n.lstrip('\\'),{pin.lstrip('\\'):expr for pin,expr in re.findall(r'\.([A-Za-z_][A-Za-z0-9_]*|\\\S+)\s*\(([^()]*)\)',p)})for t,n,p in CELL.findall(body)])
  return cache[kind]
 def walk(kind,hierarchy,bindings):
  declarations,instances=definition(kind)
  def local(name):return name if name in('VCC','GND')else bindings.get(name,(hierarchy+'/'if hierarchy else'')+name)
  def indices(hi,lo):return range(hi,lo+(1 if lo>=hi else-1),1 if lo>=hi else-1)
  def expand(expr):
   result=[];position=0
   while position<len(expr):
    if not expr[position:].strip():break
    m=TOKEN.match(expr,position);assert m,(kind,hierarchy,expr[position:]);position=m.end()
    width,base,value,name,hi,lo,punct=m.groups()
    if punct:continue
    if width:
     assert not re.search('[xXzZ]',value)
     number=int(value,{'b':2,'h':16,'d':10}[base]);result+=['VCC'if number>>i&1 else'GND'for i in range(int(width)-1,-1,-1)]
    else:
     name=name.lstrip('\\')
     if hi is not None:result+=[local(f'{name}[{i}]')for i in indices(int(hi),int(lo or hi))]
     elif name in declarations and declarations[name][1]is not None:result+=[local(f'{name}[{i}]')for i in indices(*declarations[name][1:])]
     else:result.append(local(name))
   return result
  for child,name,ports in instances:
   full=(hierarchy+'/'+name).strip('/')
   if child not in modules or not any(t==full or t.startswith(full+'/')for t in targets):continue
   connections={p:expand(e)for p,e in ports.items()}
   child_declarations,_=definition(child);mapped={}
   for p,nets in connections.items():
    _,hi,lo=child_declarations[p];names=[p]if hi is None else[f'{p}[{i}]'for i in indices(hi,lo)]
    assert len(names)==len(nets),(full,p,len(names),len(nets));mapped.update(zip(names,nets))
   if full in targets:found[full]=dict(name=full,module_kind=child,ports=connections,internal_declarations=child_declarations)
   walk(child,full,mapped)
 walk('g2_top','',{})
 assert set(found)==set(targets),(set(targets)-set(found))
 return found

def run(out):
 state=dict(passed=False,scope='Actual completed SYN34 structural proof only; no physical timing/function/deployment acceptance',recorded_at=datetime.datetime.now().astimezone().isoformat(),programmed=False,deployment_accepted=False)
 def stage(s,**kw):state.update(phase=s,**kw);save(out/'progress.json',state)
 stage('waiting_for_complete_actual_SYN34')
 while True:
  job=load(NEW/'job.json')
  if job['phase']=='synthesis_complete':break
  if job['phase'].startswith(('failed','cancel')):raise AssertionError(('actual SYN34 failed',job))
  stage('waiting_for_complete_actual_SYN34',actual_SYN_phase=job['phase'])
  time.sleep(13)
 assert job['tool_exit']==0
 while True:
  m=memory_status()
  if m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3:break
  stage('waiting_review_memory',physical=m.avail_phys,available_commit=m.avail_page);time.sleep(13)
 stage('bind_actual_frozen_source')
 def hardware(root):
  paths=[Path(p).relative_to(root).as_posix()for p in load(root/'manifest.json')['source_files']]
  assert len(paths)==len(set(paths));return set(paths)
 original,new=hardware(OLD),hardware(NEW)
 assert len(original)==63 and len(new)==66 and new==hardware(PREP)
 changes=sorted(n for n in original if not same(OLD/n,NEW/n));added=sorted(new-original)
 assert changes==sorted(['g2_top.v','control4500_core.v','control/mem_policy.v','control/imu_fusion.v'])
 assert added==sorted(['control/mem_policy_related25.v','model25_related_mailbox.v','model25_pll/model25_pll.v'])
 assert all(same(PREP/n,NEW/n)for n in new)
 frozen=[meta(NEW/n)for n in sorted(new)]
 asset_sets={};fw={}
 for suffixes,label,count in [({'.hex','.mem','.bin','.mif'},'assets',69),({'.h','.vh'},'headers',31)]:
  paths={p.relative_to(OLD).as_posix():p for p in OLD.rglob('*')if p.is_file()and'impl'not in p.relative_to(OLD).parts and'__pycache__'not in p.parts and p.suffix.lower()in suffixes}
  assert len(paths)==count,(label,len(paths));assert all(same(p,NEW/n)and same(p,PREP/n)for n,p in paths.items());asset_sets[label]=count
 for d in('fw','firmware'):
  paths={p.relative_to(OLD).as_posix():p for p in(OLD/d).rglob('*')if p.is_file()and'__pycache__'not in p.parts}
  assert all(same(p,NEW/n)and same(p,PREP/n)for n,p in paths.items());fw[d]=len(paths)
 stage('parse_actual_VG_memory_and_three_clocks')
 directions=primitive_directions(LIB)
 oldvg=OLD/'impl/gwsynthesis/g2_top.vg';newvg=NEW/'impl/gwsynthesis/g2_top.vg'
 before,after=read(oldvg),read(newvg)
 oldmods,newmods=definitions(oldvg),definitions(newvg)
 oldparams,newparams=parameter_reader(oldmods),parameter_reader(newmods)
 drivers=collections.defaultdict(list);loads=collections.defaultdict(list)
 for n,c in after.items():
  assert c['kind']in directions,c['kind']
  for pin,ns in c['ports'].items():
   for net in ns:
    if directions[c['kind']][pin]=='output'and net not in('VCC','GND'):drivers[net].append((n,pin))
    if directions[c['kind']][pin]in('input','inout'):loads[net].append((n,pin))
 plls={n:c for n,c in after.items()if c['kind']=='PLL'};assert list(plls)==['u_model_pll/PLL_inst']
 pll=plls['u_model_pll/PLL_inst'];pp=newparams(pll)
 expected=dict(FCLKIN='"50"',IDIV_SEL='1',FBDIV_SEL='1',MDIV_SEL='16',ODIV0_SEL='32',MDIV_FRAC_SEL='0',ODIV0_FRAC_SEL='0',CLKOUT0_EN='"TRUE"',CLKFB_SEL='"INTERNAL"')
 assert all(pp.get(k)==v for k,v in expected.items()),pp
 assert all(pp[f'CLKOUT{i}_EN']=='"FALSE"'for i in range(1,7))
 for pin in ['PLLPWD','RESET_I','RESET_O','CLKFB']:
  assert pll['ports'][pin]==['GND'],(pin,pll['ports'][pin])
 assert pll['ports']['ENCLK0']==['VCC']
 def clock(net,seen=()):
  if net in('VCC','GND'):return net,0
  assert net not in seen,(net,seen)
  assert len(drivers[net])==1,(net,drivers[net])
  n,p=drivers[net][0];c=after[n]
  if c['kind']=='IBUF':
   assert c['ports']['I']in(['clk50'],['rgmii_rxc']);return c['ports']['I'][0],0
  if c['kind']=='PLL':
   assert n=='u_model_pll/PLL_inst'and p=='CLKOUT0';assert clock(c['ports']['CLKIN'][0],seen+(net,))==('clk50',0)
   return 'model25',0
  assert c['kind']in('INV','BUF','BUFG'),(net,c)
  root,parity=clock(c['ports']['I'][0],seen+(net,));return root,parity^int(c['kind']=='INV')
 olddrivers=collections.defaultdict(list)
 for name,c in before.items():
  for pin,ns in c['ports'].items():
   if directions[c['kind']][pin]=='output':
    for net in ns:
     if net not in('VCC','GND'):olddrivers[net].append((name,pin))
 def oldclock(net,seen=()):
  if net in('VCC','GND'):return net,0
  assert net not in seen and len(olddrivers[net])==1,(net,olddrivers[net])
  name,pin=olddrivers[net][0];c=before[name]
  if c['kind']=='IBUF':return c['ports']['I'][0],0
  assert c['kind']in('INV','BUF','BUFG'),c
  root,parity=oldclock(c['ports']['I'][0],seen+(net,));return root,parity^int(c['kind']=='INV')
 clock_counts=collections.Counter();clock_nets={}
 for c in after.values():
  for pin,ns in c['ports'].items():
   if re.fullmatch(r'CLK\d*|CLKA|CLKB|CLOCK|WCLK|RCLK',pin):
    for net in ns:
     if net in('VCC','GND'):continue
     root=clock(net);clock_counts[root]+=1;clock_nets[net]=dict(root=root[0],inverted=bool(root[1]),actual_driver=drivers[net])
 assert {r for r,p in clock_counts}=={'clk50','model25','rgmii_rxc'}
 canonical=lambda n:n.replace('u_control/model_thread/model/','u_control/model_thread/',1)if n.startswith('u_control/model_thread/model/')else n
 oldmem={n:dict(name=n,kind=c['kind'],ports=c['ports'],parameters=oldparams(c))for n,c in before.items()if c['kind']in BSRAM or c['kind'].startswith(('RAM16','ROM16'))}
 newmem={canonical(n):dict(name=n,kind=c['kind'],ports=c['ports'],parameters=newparams(c))for n,c in after.items()if c['kind']in BSRAM or c['kind'].startswith(('RAM16','ROM16'))}
 assert len(newmem)==sum(c['kind']in BSRAM or c['kind'].startswith(('RAM16','ROM16'))for c in after.values()),'memory path collision'
 assert set(oldmem)==set(newmem),(set(oldmem)-set(newmem),set(newmem)-set(oldmem))
 relocated=[];memory_pin_changes=[]
 for n,a in oldmem.items():
  b=newmem[n];assert a['kind']==b['kind']and a['parameters']==b['parameters'],('RAM parameters/INIT',n)
  assert set(a['ports'])==set(b['ports'])
  for pin,nets in b['ports'].items():
   assert len(nets)==len(a['ports'][pin])
   if 'CLK'in pin:
    for oldnet,net in zip(a['ports'][pin],nets):
     if net in('VCC','GND'):continue
     expected=('model25',0)if n.startswith('u_control/model_thread/')else oldclock(oldnet)
     assert clock(net)==expected,(n,pin,net,expected)
  if a['name']!=b['name']:relocated.append(dict(old=a['name'],new=b['name'],reason='explicit wrapper model insertion; identical primitive and all parameters/INIT'))
  if a['ports']!=b['ports']:memory_pin_changes.append(n)
 bsram=[m for m in newmem.values()if m['kind']in BSRAM]
 wide=[m['name']for m in bsram if any(m['parameters'].get(k)in('32','36')for k in('BIT_WIDTH','BIT_WIDTH_0','BIT_WIDTH_1'))]
 assert len(bsram)==314 and len(wide)==18
 libtext=LIB.read_text();body=re.search(r'\bmodule\s+SP\b(.*?)\bendmodule',libtext,re.S)[1]
 defaults=dict(re.findall(r'\bparameter\s+(INIT_RAM_\w+)\s*=\s*([^;]+);',body))
 image=cpu_image(newmem,defaults);oldimage=cpu_image(oldmem,defaults)
 binary=(NEW/'firmware/nav_fw.bin').read_bytes();words=b''.join(int(x,16).to_bytes(4,'little')for x in(NEW/'ram_w.hex').read_text().split())
 assert image==oldimage==words==binary+bytes(131072-len(binary))
 assert len(binary)==51712
 for lane in range(4):assert bytes(int(x,16)for x in(NEW/f'ram_b{lane}.hex').read_text().split())==image[lane::4]
 save(out/'actual_memory_binding.json',dict(passed=True,old=oldmem,new=newmem,explicit_wrapper_path_relocations=relocated,all_geometry_and_explicit_INIT_equal=True,memory_port_net_name_changes=memory_pin_changes,all_memory_port_logic_equivalence_claimed=False))
 save(out/'actual_clock_PLL_inventory.json',dict(PLL=pll,actual_PLL_parameters=pp,clock_nets=clock_nets,clock_pin_counts=[dict(root=r,inverted=bool(p),pins=n)for(r,p),n in clock_counts.items()],PLL_input_mhz=50,VCO_mhz=800,model_output_mhz=25,SDC_and_physical_clock_route_signoff=False))
 stage('inspect_actual_related_channels_guard_and_radar')
 # The target-channel inspection is intentionally performed from actual mapped
 # modules/ports, not by assuming RTL register counts survived optimization.
 targets=[r[1]for r in CHANNELS]+['u_control/model_clock_guard']
 modules=module_bindings(newmods,targets)
 save(out/'actual_related_module_ports.json',modules)
 stage('base_inventory_complete_related_proof_pending')
 save(out/'base_inventory.json',dict(passed=True,changed_original_inputs=changes,added_inputs=added,identical_original_inputs=59,identical_assets=asset_sets,firmware_file_counts=fw,actual_BSRAM=314,wide_leaves=18,predicted_split1_BSRAM=332,CPU_image_bytes=131072,original_FW_BIN_bytes=51712,actual_PLL_parameters=pp,true_clock_roots=['clk50','model25','rgmii_rxc'],three_clock_frequencies_mhz=dict(clk50=50,model25=25,rgmii_rxc=100),all_memory_geometry_INIT_preserved=True,all_memory_port_logic_equivalence_claimed=False,original_hardware_bound_before=frozen,original_hardware_bindings_unchanged_at_end=all(meta(Path(row['path']))==row for row in frozen),full_review_complete=False))
 return locals()

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(exist_ok=False)
 try:
  context=run(a.out)
  proof=Path(__file__).with_name('prove_model25_related_transport.py')
  while not proof.exists():time.sleep(1)
  import importlib.util
  spec=importlib.util.spec_from_file_location('model25_actual_protocol_proof',proof)
  extension=importlib.util.module_from_spec(spec);spec.loader.exec_module(extension)
  extension.complete(context)
 except Exception as error:
  save(a.out/'review_failed.json',dict(passed=False,error=str(error),traceback=traceback.format_exc()));raise
