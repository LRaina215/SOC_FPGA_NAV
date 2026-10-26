"""Read-only PNR51 ring implementation review; default only describes entry.

Run after natural tool exit with --run. Checks the actual flat VO against SYN39
and transfers the already passed 1240 mapped partition checks through strictly
matched primitives/cones. It never invokes an EDA tool or changes PNR inputs.
"""
from pathlib import Path
from collections import Counter, defaultdict
import argparse, datetime, json, re, traceback
import psutil
from synthesis_connectivity import read
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from audit_routing_topology import primitive_directions
from prove_soc_console_remap import literal,z3
import review_lease_tree_pnr50_final as flat

BASE=Path('F:/nav_integration_g2_20261023')
SYN=BASE/'baseline_fusion_model25_observation_ring_syn39'
PNR=BASE/'baseline_fusion_model25_observation_ring_pnr51'
PROOF=BASE/'ring_SYN39_actual_semantics_review6'
OUT=BASE/'pnr51_ring_final_actual_review1'
OBS='u_control/model_thread/model/observation'
POL='u_control/model_thread/model/policy'
LIB=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
BUFFERS={'BUF','BUFG','INV','IBUF'}

def load(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def metadata(p):s=p.stat();return dict(path=str(p),bytes=s.st_size,mtime_ns=s.st_mtime_ns)
def save(p,x):
 assert not p.exists(),('preserve existing review',str(p))
 p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf8')

def run(out):
 runtime=load(PNR/'tool_runtime.json')
 assert 'tool_exit'in runtime, 'PNR51 not final; do not inspect transient implementation'
 start=datetime.datetime.fromisoformat(runtime['started_at']).timestamp()
 pid=runtime['pid'];original_present=False
 if psutil.pid_exists(pid):original_present=abs(psutil.Process(pid).create_time()-start)<5
 assert not original_present,('original EDA tool still exists',pid)
 assert runtime['max_runtime_s']==runtime['max_no_observable_progress_s']==0
 assert runtime['cancelled']is False
 out.mkdir(parents=True,exist_ok=False)
 proof=load(PROOF/'final_review.json');assert proof['passed']and proof['checks']==1240
 vg=SYN/'impl/gwsynthesis/g2_top.vg';vo=PNR/'impl/pnr/g2_top.vo'
 bindings=[metadata(x)for x in (vg,vo,PNR/'tool_runtime.json',PROOF/'final_review.json',PROOF/'proof_checks.json')]
 assert metadata(vg)==proof['actual_VG'], 'SYN39 differs from the proved VG'
 assert vg.read_bytes()==(PNR/'g2_top.vg').read_bytes(),'PNR51 input VG differs'
 anc={'','u_control','u_control/model_thread','u_control/model_thread/model',OBS,POL,POL+'/core'}
 keep=lambda n:n in anc or n.rpartition('/')[0]in anc or n.startswith(OBS+'/')
 source=read(vg,keep);sp=parameter_reader(definitions(vg))
 # Use the proven flat-VO parser, selecting every local cone plus all FFs and
 # buffers. Retain physical copies by scope; never assume a suffix is a copy.
 flat.selected=keep
 actual,pp,_,parser=flat.parse_vo(vo,set(source))
 dirs=primitive_directions(LIB)
 defaults={kind:{k:v.strip()for k,v in re.findall(r'\bparameter\s+(\w+)\s*=\s*([^;]+);',body)}for kind,body in re.findall(r'\bmodule\s+(\w+)\b(.*?)\bendmodule',LIB.read_text(),re.S)}
 def norm(v):
  try:return literal(v)
  except ValueError:return v.strip()
 def effective(c,side):
  d=dict(defaults[c['kind']]);d.update(sp(c)if side=='source'else pp[c['name']]);return{k:norm(v)for k,v in d.items()}
 def drivers(cells):
  d=defaultdict(list)
  for n,c in cells.items():
   for p,ns in c['ports'].items():
    if dirs[c['kind']].get(p)=='output':
     for i,v in enumerate(ns):
      if v not in('VCC','GND'):d[v].append((n,p,i))
  return d
 sd,ad=drivers(source),drivers(actual)
 def canonical(cells,di,net,trail=()):
  assert net not in trail,('buffer cycle',net)
  ds=di.get(net,[])
  if len(ds)!=1:return net,0
  n,p,i=ds[0];c=cells[n]
  invert=int(c['kind']=='INV');pin='I'
  if c['kind']not in BUFFERS:
   if c['kind']!='LUT1':return net,0
   side='source'if cells is source else'actual';truth=effective(c,side)['INIT']
   if truth not in(1,2):return net,0
   invert=int(truth==1);pin='I0';assert p=='F'
  else:assert p=='O'
  assert i==0 and len(c['ports'][pin])==1
  root,phase=canonical(cells,di,c['ports'][pin][0],trail+(net,))
  return root,phase^invert
 checked=set();in_progress=set();rows=[];aliases=[];boundaries=[];alu_swaps=[]
 # Public ALU mode0 is a full adder: exchanging I0/I1 preserves both SUM
 # and COUT, including the carry-dependent branch. Do not generalize this
 # permutation to subtraction or modes whose input roles differ.
 x,y,ci=z3.Bools('ALU_I0 ALU_I1 ALU_CIN')
 def add0(a,b):
  t=z3.Xor(a,b);return z3.Xor(t,ci),z3.If(t,ci,a)
 u,v=add0(x,y);uu,vv=add0(y,x);solver=z3.Solver();solver.add(z3.Or(u!=uu,v!=vv));assert solver.check()==z3.unsat
 def matched_clock(s,a):
  x,y=canonical(source,sd,s),canonical(actual,ad,a)
  assert x==y,('clock root/phase changed',s,a,x,y)
 def compare_net(s,a,where):
  s,si=canonical(source,sd,s);a,ai=canonical(actual,ad,a)
  assert si==ai,('unexpected polarity change',where,s,a,si,ai)
  if s in('GND','VCC')or a in('GND','VCC'):
   assert s==a,('constant differs',where,s,a);return
  ss,aa=sd.get(s,[]),ad.get(a,[])
  if not ss:
   # The SYN39 proof intentionally cuts at signals from other modules. The
   # flat VO parser retains all FFs, so it can know an external driver that
   # was outside the selected SYN graph. Keep precisely that same net cut;
   # no arbitrary different generated alias may be accepted as a boundary.
   assert s==a,('external partition boundary differs',where,s,a,ss,aa)
   boundaries.append(s);return
  assert aa,('actual cone driver missing',where,s,a,ss,aa)
  assert len(ss)==len(aa)==1,('nonunique cone driver',where,s,a,ss,aa)
  sn,sout,sidx=ss[0];an,aout,aidx=aa[0]
  assert(sout,sidx)==(aout,aidx),('primitive output pin/bit differs',where,ss,aa)
  sc,ac=source[sn],actual[an]
  # Opaque arithmetic/memory boundaries are the same ones used by the 1240
  # proof. Their local cells are compared completely below when in scope.
  if sc['kind'].startswith('DFF')and sn!=an:
   assert sn in actual,('physical register copy has no original anchor',sn,an)
   anchor=actual[sn]
   assert ac['kind']==anchor['kind']==sc['kind']
   assert effective(ac,'actual')==effective(anchor,'actual')==effective(sc,'source')
   assert ac['ports'].keys()==anchor['ports'].keys()
   for pin in anchor['ports']:
    if pin=='Q':continue
    assert len(ac['ports'][pin])==len(anchor['ports'][pin])
    assert[canonical(actual,ad,v)for v in ac['ports'][pin]]==[canonical(actual,ad,v)for v in anchor['ports'][pin]],('copy transition differs',sn,an,pin)
   aliases.append(dict(source_FF=sn,physical_copy=an,all_effective_parameters_INIT_and_non_Q_ports_equal=True))
   compare_cell(sn,sn);return
  compare_cell(sn,an)
 def compare_cell(sn,an):
  key=(sn,an)
  if key in checked or key in in_progress:return
  assert sn in source and an in actual,('missing primitive',sn,an)
  s,a=source[sn],actual[an]
  assert s['kind']==a['kind'],('primitive kind differs',sn,an,s['kind'],a['kind'])
  assert effective(s,'source')==effective(a,'actual'),('effective macro parameters/INIT differ',sn,an,sp(s),pp[an])
  assert s['ports'].keys()==a['ports'].keys(),('portset differs',sn,an)
  actual_ports=a['ports']
  if s['kind']=='ALU'and effective(s,'source')['ALU_MODE']==0:
   ca=lambda ns,cs,ds:[canonical(cs,ds,n)for n in ns]
   swapped=ca(s['ports']['I0'],source,sd)==ca(a['ports']['I1'],actual,ad)and ca(s['ports']['I1'],source,sd)==ca(a['ports']['I0'],actual,ad)
   direct=ca(s['ports']['I0'],source,sd)==ca(a['ports']['I0'],actual,ad)and ca(s['ports']['I1'],source,sd)==ca(a['ports']['I1'],actual,ad)
   if swapped and not direct:
    actual_ports=dict(a['ports']);actual_ports['I0'],actual_ports['I1']=a['ports']['I1'],a['ports']['I0']
    alu_swaps.append(dict(source=sn,physical=an,mode=0,permutation='I0/I1',SUM_and_COUT_all_input_states_UNSAT=True,public_library=str(LIB)))
  in_progress.add(key)
  for pin,ns in s['ports'].items():
   ps=actual_ports[pin];assert len(ns)==len(ps),(sn,pin,'port width')
   if dirs[s['kind']].get(pin)=='output':continue
   for i,(x,y)in enumerate(zip(ns,ps)):
    if re.fullmatch(r'CLK\d*|CLKA|CLKB|WCLK|RCLK|CLOCK',pin):matched_clock(x,y)
    else:compare_net(x,y,(sn,an,pin,i))
  in_progress.remove(key);checked.add(key)
  rows.append(dict(source=sn,physical=an,kind=s['kind'],all_effective_parameters_and_INIT_equal=True,all_input_cones_preserved=True,source_explicit_parameters=sp(s),physical_explicit_parameters=pp[an],physical_ports=a['ports']))
 def target(n,c):return bool(c['kind']=='RAM16SDP4'and'/g_ring.frame_mem_'in n or c['kind']=='MULT12X12'and n.startswith(OBS+'/')or c['kind'].startswith('DFF')and(re.fullmatch(re.escape(OBS)+r'/g_ring\.(?:work_slot|live[0-4]|frozen[0-4])_\d+_s\d+',n)or re.fullmatch(re.escape(POL)+r'/snapshot_word_\d+_s\d+',n)))
 targets={n:c for n,c in source.items()if target(n,c)}
 counts=Counter(c['kind']for c in targets.values())
 assert sum(c['kind']=='RAM16SDP4'for c in targets.values())==66
 assert sum(c['kind']=='MULT12X12'for c in targets.values())==3
 assert sum('/g_ring.'in n and c['kind'].startswith('DFF')for n,c in targets.items())==33
 assert sum('/snapshot_word_'in n for n in targets)==24
 for n in sorted(targets):compare_cell(n,n)
 # Compare all local normalizer/policy primitives, transferring corresponding
 # retained arithmetic boundaries as well, rather than only counting targets.
 local={n for n,c in source.items()if n.startswith(OBS+'/')or n.rpartition('/')[0]==POL}
 for n in sorted(local):
  if source[n]['kind']not in BUFFERS|{'VCC','GND'}:compare_cell(n,n)
 physical_ring={n:c for n,c in actual.items()if target(n,c)}
 assert targets.keys()==physical_ring.keys(),('unmatched physical ring primitives',sorted(targets.keys()-physical_ring.keys()),sorted(physical_ring.keys()-targets.keys()))
 assert all(metadata(Path(x['path']))==x for x in bindings),'watched input/output changed during read'
 save(out/'actual_ring_primitives_and_cones.json',dict(passed=True,target_cells=len(targets),RAM16SDP4=66,slot_FFs=33,prefetch_FFs=24,address_MULT12X12=3,compared_primitive_pairs=len(checked),checks=rows,proved_registered_copies=aliases,proved_ALU_mode0_input_swaps=alu_swaps,partition_boundary_nets=sorted(set(boundaries)),parser=parser))
 result=dict(passed=True,recorded_at=datetime.datetime.now().astimezone().isoformat(),tool_exit=runtime['tool_exit'],routing_accepted=False,source_partition_proof=str(PROOF/'final_review.json'),source_partition_checks=1240,actual_target_cells=126,primitive_pairs_compared=len(checked),physical_registered_copy_aliases=len(aliases),RAM_geometry_INIT_and_all_ports=True,metadata_prefetch_CE_reset_and_input_cones=True,address_DSP_modes_and_capture_boundaries=True,bindings=bindings,limits=['Functional partition transfer, not full G2 all-state equivalence','No setup/hold, electrical RAM read timing, reset pulse width or IO signoff','Routing result, total resources, other storage, PLL/three clocks and original26 handled by independent general final reviewer'])
 save(out/'final_review.json',result);print(json.dumps(result,ensure_ascii=False));return result

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');p.add_argument('--out',type=Path,default=OUT);args=p.parse_args()
 if not args.run:
  print(json.dumps(dict(prepared_only=True,PNR=str(PNR),final_review=str(args.out/'final_review.json'),command='D:/Anaconda/python.exe -X utf8 "D:/Desktop/SOC FPGA/nav/integration/g2/review_model25_ring_pnr51_final.py" --run',no_EDA_invocation=True),ensure_ascii=False))
 else:
  try:run(args.out)
  except Exception:
   if args.out.exists()and not(args.out/'final_review.json').exists():save(args.out/'final_review.json',dict(passed=False,error=traceback.format_exc(),frozen_PNR_inputs_unchanged=True))
   raise
