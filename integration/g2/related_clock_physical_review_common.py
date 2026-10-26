"""Reuse the verified PNR50 flat reader and strict primitive/replica comparator.

The comparison body is retained from review_lease_tree_pnr50_final, with input
paths and selected prefixes supplied by the caller. No EDA or source mutation.
"""
from collections import Counter,defaultdict
from pathlib import Path
import re
from revalidate_lidar_cdc import definitions
from synthesis_connectivity import read
from review_small_block_stores import parameter_reader
from audit_routing_topology import primitive_directions
from prove_soc_console_remap import literal
from review_lease_tree_pnr50_final import parse_vo,LIB,BUFFERS,CELL

def make_context(source_vg,physical_vo,extra=(),extra_prefixes=()):
 watched=[Path(source_vg),Path(physical_vo)]
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
 extra=set(extra)|{n for n in source if any(n.startswith(p)for p in extra_prefixes)}
 # The original PNR50 parser always retains every FF but scopes LUT/ALU
 # instances. Include all physical cells in newly selected guard hierarchies,
 # so a new physical combinational primitive cannot disappear from the audit.
 if extra_prefixes:
  physical_text=watched[1].read_text(encoding='utf8')
  extra.update(raw.lstrip('\\') for kind,raw,body in CELL.findall(physical_text)
               if kind!='module' and any(raw.lstrip('\\').startswith(p)for p in extra_prefixes))
  del physical_text
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
  alu_swap=None
  if a['kind']=='ALU'and ea==eb and ea['ALU_MODE']==ea['ADD']==0:
   if a['ports']['I0']==b['ports']['I1']and a['ports']['I1']==b['ports']['I0']and a['ports']['I0']!=b['ports']['I0']:
    # PNR may exchange the two ADD inputs. Check the actual unmodified
    # vendor macro, every binary assignment and both outputs; no generic
    # ALU permutation, subtraction or arbitrary X/Z equivalence is allowed.
    alu=re.search(r'\bmodule\s+ALU\s*\(.*?\bendmodule',libtext,re.S)[0]
    assert re.search(r'assign\s+SUM\s*=\s*S\s*\^\s*CIN\s*;',alu)
    assert re.search(r'assign\s+COUT\s*=\s*S\?\s*CIN\s*:\s*C\s*;',alu)
    add=re.search(r'ADD:\s*begin(.*?)\bend',alu,re.S)[1]
    assert re.search(r'S\s*=\s*I0\s*\^\s*I1\s*;',add)and re.search(r'C\s*=\s*I0\s*;',add)
    truth=[]
    for value in range(16):
     i0,i1,i3,cin=[(value>>i)&1 for i in range(4)]
     def outputs(x,y):
      s=x^y;return(s^cin,cin if s else x)
     original,permuted=outputs(i0,i1),outputs(i1,i0)
     assert original==permuted
     truth.append(dict(I0=i0,I1=i1,I3=i3,CIN=cin,SUM=original[0],COUT=original[1]))
    alu_swap=dict(only_I0_I1_exchanged=True,ALU_MODE_ADD0=True,vendor_macro_source=str(LIB),
                  all_16_binary_assignments_both_outputs_equal=truth,arbitrary_XZ_semantics_claimed=False)
    b=dict(b,ports=dict(b['ports'],I0=b['ports']['I1'],I1=b['ports']['I0']))
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
  return dict(name=n,logical_comparison_kind=a['kind'],source_kind=original_kinds[0],physical_kind=original_kinds[1],buffer_LUT1_reencoding_checked=original_kinds[0]!=original_kinds[1],ALU_ADD_only_input_commutation=alu_swap,all_effective_vendor_parameters_INIT_equal=not bool(parameter_differences),disabled_unused_PLL_output_divider_changes=parameter_differences,source_explicit_parameters=source_params[n],physical_explicit_parameters=pp[n],all_pins_equal_or_proved_buffer_inversion_alias_or_recorded_ADD_input_commutation=True,proved_pin_aliases=diffs)
 return dict(source=source,physical=actual,source_parameters=source_params,
  physical_parameters=pp,clockpins=clockpins,parser=parser,directions=directions,
  source_buffer_drivers=sd,physical_buffer_drivers=pd,canonical=canonical,
  equal_cell=equal_cell,proved_registered_replica_aliases=cloned_aliases,
  effective_parameters=effective,source_modules=modules)
