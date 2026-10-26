"""Actual completed SLF128 synthesis clocks, original26 and stop boundaries.

Read-only.  Names are found through actual Q/D connectivity and logical signal
roles, not old generated suffixes.  Physical recovery/removal, MTBF and routed
STA remain separate.  Never writes the synthesis directory or starts EDA.
"""
from pathlib import Path
from datetime import datetime
from collections import Counter
import argparse,json,psutil,re,time,traceback
from synthesis_connectivity import read,CELL,DECL
from revalidate_lidar_cdc import definitions,module_at
from review_small_block_stores import parameter_reader
from audit_routing_topology import primitive_directions
from review_model25_observation_ring_syn39 import index,clock_reader
from review_model50_syn40_clock_stop import trace_clocks
from review_model25_joint_syn import module_bindings
from review_lease_bounded_MAX_joint_mapping import report_paths
from prove_soc_console_remap import Logic,z3,literal
from prepare_baseline_fusion_retime import metadata,same_bytes,save
from retry_frozen_synthesis import memory_status
BASE=Path('F:/nav_integration_g2_20261023');NEW=BASE/'SLF128_storage_safeFW_syn42';OLD=BASE/'baseline_fusion50_ring_MAX_syn40';OUT=BASE/'SLF128_storage_safeFW_syn42_clock_CDC_stop_review4'
LIB=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v');GIB=1024**3
REF=BASE/'baseline_fusion32_CDC52_pnr35/cdc_reference_audit.json';RX='u_nav/u_netl/u_rxf'
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def one(c,p):assert len(c['ports'][p])==1,(c['name'],p);return c['ports'][p][0]
def source_contract():
 paths=['control4500_core.v','control/mem_policy.v','control/rl_policy_tick_v5.v','control/policy_action_map.v','control/ir_remote_control.v','control/motor520_closed_loop.v','control/motor520_hybrid_speed.v','command_lease_guard.v','nav_command_units.v','nav/rtl/plan/safety_guard.v','nav/rtl/common/nav_cdc.v']
 checks={r:same_bytes(OLD/r,NEW/r)for r in paths};assert all(checks.values()),checks
 def topblock(p):
  t=p.read_text();a=t.index('    wire signed [47:0] vx_product');b=t.index('    nav_subsystem #(',a);return t[a:b]
 assert topblock(OLD/'g2_top.v')==topblock(NEW/'g2_top.v')
 assert 'localparam integer CLK_HZ = 50000000;'in(NEW/'g2_top.v').read_text()
 return dict(passed=True,original4500_tick_weights_algorithm_and_stop_source_contract='Unchanged complete control/IR/lease/safety RTL; packed weight storage and SLF/profile changes are separately reviewed',byte_identical=checks,top_units_lease_control_stop_and_feedback_block_identical=True,core_source_CLK_HZ=50000000)

def run():
 assert not OUT.exists();OUT.mkdir();me=psutil.Process();state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),old=str(OLD),new=str(NEW),output=str(OUT),new_EDA_started=False,programmed=False,passed=False)
 def phase(s,**kw):state.update(phase=s,at=datetime.now().astimezone().isoformat(),**kw);save(OUT/'job.json',state)
 sections={}
 def section(name,fn):
  try:r=fn();assert r.get('passed'),r
  except Exception as e:r=dict(passed=False,error=str(e),traceback=traceback.format_exc())
  save(OUT/(name+'.json'),r);sections[name]=dict(passed=r['passed'],path=str(OUT/(name+'.json')));return r
 try:
  while True:
   j=load(NEW/'job.json')
   if j['phase']=='synthesis_complete':assert j['tool_exit']==0;break
   assert not j['phase'].startswith(('failed','cancel')),j
   phase('waiting_completed_synthesis',actual_build=str(NEW),actual_phase=j['phase']);time.sleep(13)
  # Root starts this only after the current storage reader has released its
  # graph. No historical PID is a synchronization dependency for a new build.
  while True:
   mem=memory_status()
   if mem.avail_phys>=2*GIB and mem.avail_page>=4*GIB:break
   phase('waiting_actual_review_memory',physical=mem.avail_phys,commit=mem.avail_page);time.sleep(13)
  section('source_stop_contract',source_contract)
  watched=[OLD/'impl/gwsynthesis/g2_top.vg',NEW/'impl/gwsynthesis/g2_top.vg',LIB,NEW/'g2.sdc',NEW/'g2_top.v',NEW/'command_lease_guard.v']
  bindings=[metadata(p)for p in watched];phase('parse_actual40_and42',admission_physical_bytes=mem.avail_phys,admission_commit_bytes=mem.avail_page)
  om,nm=definitions(watched[0]),definitions(watched[1]);op,np=parameter_reader(om),parameter_reader(nm);directions=primitive_directions(LIB)
  after=read(watched[1]);nd,nl=index(after,directions);nc=clock_reader(after,nd)
  # Old graph is scoped to actual clock/reset/FIFO/stop consumers; every new
  # primitive is retained for clock inventory and first-stage fanout recount.
  scopes=['u_control','u_command_lease','u_command_units','u_nav/u_sg','u_nav/u_rf','u_nav/u_netl/u_rxf','u_nav/u_netl/u_txf','u_lmac','u_rs','u_lrr','u_ltr','u_hrr','u_htr']
  keep=lambda h:'/'not in h or any(h==s or h.startswith(s+'/')or s.startswith(h+'/')for s in scopes)
  before=read(watched[0],keep);od,ol=index(before,directions);oc=clock_reader(before,od)
  def outputs(net):return tuple(nd.get(net,[]))
  clocks=section('actual_two_clocks',lambda:trace_clocks(after,directions,nc,outputs))
  assert not any('PLL'in c['kind']or any('/'+x+'/'in n for x in('bmi_bridge','config_bridge','action_bridge','fusion_bridge','status_bridge'))for n,c in after.items())
  # Nonclock canonical aliases stop at the actual FF role. Numbers such as
  # n123_4 are never treated as equivalent merely because suffixes look alike.
  def transparent(cells,drivers,net,seen=()):
   if net in('GND','VCC'):return(net,0)
   assert net not in seen
   ds=drivers.get(net,[])
   if len(ds)!=1:return(net,0)
   name,pin,bit=ds[0];c=cells[name]
   if c['kind']=='IBUF':return(one(c,'I'),0)
   if c['kind']in('BUF','BUFG','INV'):
    root,p=transparent(cells,drivers,one(c,'I'),seen+(net,));return root,p^int(c['kind']=='INV')
   return(net,0)
  def driver(net):
   ds=nd.get(net,[]);assert len(ds)==1,(net,ds);n,p,b=ds[0];assert p=='Q'and b==0 and after[n]['kind'].startswith('DFF'),(net,ds);return after[n]
  targets=['u_rs','u_lrr','u_ltr','u_hrr','u_htr','u_control/remote_thread','u_command_lease','u_control','u_control/model_thread','u_command_units','u_nav/u_sg'];live=[];pruned=[]
  def fifo_write_state(scope,c):
   q=one(c,'Q');local=c['name'][len(scope)+1:]
   return any(q.startswith(scope+'/'+stem+'[')for stem in('wp_b','wp_g','rp_g_w1','rp_g_w2','rp_b_w'))or bool(re.fullmatch(r'(wp_b|wp_g|rp_g_w1|rp_g_w2|rp_b_w)_\d+_s\d+',local))
  for target in targets:
   try:module_at(nm,target);live.append(target)
   except AssertionError:
    assert target in('u_hrr','u_htr'),('required stop/reset module absent',target)
    scope='u_nav/u_neth/u_rxf'if target=='u_hrr'else'u_nav/u_neth/u_txf'
    fifo_state=[c for n,c in after.items()if n.startswith(scope+'/')and c['kind'].startswith('DFF')]
    relevant=[c for c in fifo_state if fifo_write_state(scope,c)==(target=='u_hrr')]
    assert not relevant,('absent reset wrapper still has live consuming FIFO state',target,scope,relevant)
    pruned.append(dict(module=target,no_actual_consuming_half_FFs=True,scope=scope,other_half_FFs_retained=len(fifo_state),pruned_half='write'if target=='u_hrr'else'read'))
  target_bindings=module_bindings(nm,live)
  save(OUT/'actual_pruned_reset_scopes.json',dict(passed=True,pruned=pruned,no_missing_live_state_accepted=True))
  resetnets={};resetstages=[]
  # Optimization promotes/renames wrapper output ports. Resolve the actual
  # two-FF r1/r2 roles and prove D/clock/PRESET/INIT, instead of assuming .rst.
  for scope,clock in [('u_rs','clk50'),('u_lrr','rgmii_rxc'),('u_ltr','clk50'),('u_hrr','clk50'),('u_htr','clk50')]:
   if scope not in live:continue
   stages=[]
   for r in(1,2):
    matches=[c for n,c in after.items()if re.fullmatch(re.escape(scope)+rf'/r{r}_s\d+',n)]
    assert len(matches)==1,(scope,r,matches)
    c=matches[0];assert c['kind']=='DFFPE'and nc(one(c,'CLK'))==(clock,0)and c['ports']['CE']==['VCC'],c
    assert np(c).get('INIT')=="1'b1",(scope,np(c))
    stages.append(c)
   a,b=stages;assert one(a,'D')=='GND'and one(b,'D')==one(a,'Q'),(scope,stages)
   expected=('key_rst_n',1)if scope=='u_rs'else transparent(after,nd,resetnets['u_rs'])
   assert transparent(after,nd,one(a,'PRESET'))==expected and transparent(after,nd,one(b,'PRESET'))==expected,(scope,stages,expected)
   resetnets[scope]=one(b,'Q');resetstages.append(dict(scope=scope,clock=clock,r1=a,r2=b,preset_source=expected,output_Q=resetnets[scope]))
  save(OUT/'actual_reset_wrapper_roles.json',dict(passed=True,actual_roles=resetstages,physical_recovery_removal_and_pulse_width_pending=True))
  def reset_same(net,expected):return transparent(after,nd,net)==transparent(after,nd,expected)
  def reset_desc(c):
   pin=next((p for p in('CLEAR','PRESET','RESET','SET')if p in c['ports']),None)
   return dict(kind=c['kind'],pin=pin,net=one(c,pin)if pin else None,asynchronous=pin in('CLEAR','PRESET'),parameters=np(c))
  def original26():
   ref=load(REF);assert ref['passed']and len(ref['paths'])==26;rows=[]
   for group,firststem,secondstem,clock_a,clock_b,budget in [('rx_write_gray','wp_g_r1','wp_g_r2','rgmii_rxc','clk50',8.),('core_read_gray','rp_g_w1','rp_g_w2','clk50','rgmii_rxc',18.)]:
    for bit in reversed(range(12)):
     first=driver(f'{RX}/{firststem}[{bit}]');second=driver(f'{RX}/{secondstem}[{bit}]');source=driver(one(first,'D'))
     assert one(second,'D')==one(first,'Q')
     allowed=[f'{RX}/'+('wp_g'if group=='rx_write_gray'else'rp_g')+f'[{bit}]']
     if bit==11:allowed.append(f'{RX}/'+('wp_b'if group=='rx_write_gray'else'rp_b')+f'[{bit}]')
     assert one(source,'Q')in allowed,(group,bit,source)
     expectedreset=resetnets['u_rs'if clock_b=='clk50'else'u_lrr']
     for c in(first,second):
      assert c['kind']=='DFFCE'and reset_same(one(c,'CLEAR'),expectedreset),(c,expectedreset)
     rows.append(dict(group=group,bit=bit,source=source,first=first,second=second,source_clock=clock_a,target_clock=clock_b,budget_ns=budget))
   for group,fq,sq,expectedsource,a,b,budget in [('rx_clock_monitor','u_lmac/rsync[0]','u_lmac/rsync[1]','u_lmac/rdiv[5]','rgmii_rxc','clk50',8.),('rx_sampling_select','u_lmac/neg_s[0]','u_lmac/neg_s[1]',None,'clk50','rgmii_rxc',18.)]:
    f,s=driver(fq),driver(sq);q=driver(one(f,'D'));assert one(s,'D')==one(f,'Q')
    if expectedsource:assert one(q,'Q')==expectedsource
    else:
     # Register-bank output bit4 is promoted through its l_rx_neg output;
     # resolve that actual hierarchical binding, not a presumed local Q name.
     rf=module_bindings(nm,['u_nav/u_rf'])['u_nav/u_rf']
     assert q['ports']['Q']==rf['ports']['l_rx_neg']and re.fullmatch(r'u_nav/u_rf/r_NETL_CTRL_4_s\d+',q['name']),(group,q,rf['ports']['l_rx_neg'])
     oldq=ref['paths'][-1]['actual_source_first_second']['source_register']['primitive']
     assert oldq['ports']['Q']==q['ports']['Q'],('sampling source logical public role changed',oldq,q)
    for c in(f,s):assert reset_desc(c)['net']in(None,'GND'),(group,c)
    rows.append(dict(group=group,source=q,first=f,second=s,source_clock=a,target_clock=b,budget_ns=budget))
   changed=[]
   for row in rows:
    q,f,s=[row[k]for k in('source','first','second')]
    assert nc(one(q,'CLK'))==(row['source_clock'],0)
    for c in(f,s):assert nc(one(c,'CLK'))==(row['target_clock'],0)and c['ports'].get('CE',['VCC'])==['VCC'],c
    assert nl[one(f,'Q')]==[(s['name'],'D',0)],(row,nl[one(f,'Q')])
    row['first_Q_all_primitive_input_loads']=nl[one(f,'Q')]
    for role,c in [('source',q),('first',f),('second',s)]:
     old_candidates=[x for x in before.values()if x['kind'].startswith('DFF')and x['ports'].get('Q')==c['ports']['Q']]
     row[role+'_reset']=reset_desc(c)
     if len(old_candidates)==1:
      oldc=old_candidates[0];oldpin=next((p for p in('CLEAR','PRESET','RESET','SET')if p in oldc['ports']),None)
      diff=dict(role=role,old_name=oldc['name'],new_name=c['name'],old_kind=oldc['kind'],new_kind=c['kind'],old_reset_pin=oldpin,new_reset_pin=row[role+'_reset']['pin'])
      if diff['old_kind']!=diff['new_kind']or diff['old_reset_pin']!=diff['new_reset_pin']:changed.append(diff)
   sdc=[f'# Actual {NEW.name} objects: sourceQ -> firstD only. Stage1->stage2 remains timed.']
   for row in rows:
    a=row['source']['name']+'/Q';b=row['first']['name']+'/D';row['source_pin']=a;row['first_pin']=b
    sdc += [f'set_false_path -from [get_pins {{{a}}}] -to [get_pins {{{b}}}] -hold',f'set_max_delay -from [get_pins {{{a}}}] -to [get_pins {{{b}}}] {row["budget_ns"]:.3f}']
   assert len(rows)==26 and sum(x.startswith('set_')for x in sdc)==52
   (OUT/'actual_SYN42_CDC52_fragment.sdc').write_text('\n'.join(sdc)+'\n',encoding='utf8')
   (OUT/'candidate_PNR_single_file.sdc').write_text('create_clock -name clk50 -period 20 [get_ports {clk50}]\ncreate_clock -name rgmii_rxc -period 10 [get_ports {rgmii_rxc}]\n\n'+'\n'.join(sdc)+'\nreport_timing -setup -max_paths 5000 -max_common_paths 1\n',encoding='utf8')
   # Supply the standard reference schema used to compare the final physical
   # source/first/second endpoints. All names come from this reviewed VG.
   radar=dict(passed=True,netlist=str(NEW/'impl/gwsynthesis/g2_top.vg'),errors=[],
    scope='Actual reviewed26 connectivity only; physical timing and MTBF remain pending',
    paths=[dict(group=r['group'],source_register=r['source']['name'],
      first_register=r['first']['name'],second_register=r['second']['name'],
      source_clock=one(r['source'],'CLK'),target_clock=one(r['first'],'CLK'),
      route_max_delay_ns=r['budget_ns'],passed=True)for r in rows])
   save(OUT/'actual_cdc_reference_audit.json',radar)
   return dict(passed=True,actual_paths=26,CE1_and_first_Q_single_second_D_load=True,new_actual_names_not_copied_old_suffixes=True,rows=rows,intended_reset_type_changes_from_SYN40=changed,candidate_commands=52,physical_delays_effective_status_hold_setup_and_MTBF_proven=False)
  cdc=section('actual_original26_and_CDC52',original26)
  def fifo_resets():
   configs={'u_nav/u_netl/u_rxf':('rgmii_rxc','clk50','u_lrr','u_rs'),'u_nav/u_netl/u_txf':('clk50','clk50','u_rs','u_ltr'),'u_nav/u_neth/u_rxf':('clk50','clk50','u_hrr','u_rs'),'u_nav/u_neth/u_txf':('clk50','clk50','u_rs','u_htr')};groups={};count=0
   for scope,(wc,rc,wr,rr)in configs.items():
    rows=[]
    for name,c in after.items():
     if not name.startswith(scope+'/')or not c['kind'].startswith('DFF'):continue
     q=one(c,'Q');write=fifo_write_state(scope,c);payload=q.startswith(scope+'/q[')or bool(re.fullmatch(re.escape(scope)+r'/u_ob/b[01]\[\d+\]',q))or bool(re.fullmatch(re.escape(scope)+r'/u_ob/b[01]_\d+_s\d+',name))
     clock=nc(one(c,'CLK'));assert clock==((wc if write else rc),0),(name,q,clock)
     if payload:assert c['ports'].get('RESET',['GND'])==['GND']and not any(p in c['ports']for p in('CLEAR','SET','PRESET')),(name,c)
     else:assert c['kind']=='DFFCE'and reset_same(one(c,'CLEAR'),resetnets[wr if write else rr]),(name,q,c,wr,rr)
     rows.append(dict(name=name,Q=q,kind=c['kind'],clock=clock,reset=reset_desc(c),payload_unreset=payload,CE=c['ports'].get('CE',['VCC'])));count+=1
    groups[scope]=dict(actual_FFs=len(rows),live_rows=rows,actual_scope_may_be_pruned_not_assumed_present=not bool(rows))
   assert groups[RX]['actual_FFs']>0
   return dict(passed=True,actual_FIFO_FFs=count,groups=groups,public_nav_fifo_ob_source_unchanged=same_bytes(OLD/'nav/rtl/common/nav_fifo_ob.v',NEW/'nav/rtl/common/nav_fifo_ob.v'),common_reset_assertion_contract_only=True,physical_release_recovery_removal_and_pulsewidth_pending=True)
  fifo=section('actual_FIFO_state_reset_classes',fifo_resets)
  # Stop proofs use actual mapped module local state and matching external
  # input roles.  They do not assume the complete changed NAV/CPU has executed.
  qa={one(c,'Q'):z3.Bool('producer_state:'+n)for n,c in before.items()if c['kind'].startswith('DFF')}
  qb={one(c,'Q'):z3.Bool('producer_state:'+n)for n,c in after.items()if c['kind'].startswith('DFF')}
  la,lb=Logic(before,op,directions,qa,{}),Logic(after,np,directions,qb,{})
  def eq(label,x,y):
   delta=z3.simplify(z3.Xor(x,y))
   if z3.is_false(delta):return'identity'
   s=z3.Solver();s.set(timeout=20000);s.add(delta);answer=s.check();assert answer==z3.unsat,(label,str(answer),str(s.model())if answer==z3.sat else s.reason_unknown());return'UNSAT'
  def module_proof(scope,expected_ffs=None):
   oldbind=module_bindings(om,[scope])[scope];newbind=target_bindings[scope];ab,bb=[mods[bind['module_kind']]for mods,bind in((om,oldbind),(nm,newbind))]
   def decl(body,kind):return{n.lstrip('\\'):(hi,lo)for d,hi,lo,n in DECL.findall(body)if d==kind}
   assert decl(ab,'input')==decl(bb,'input')and decl(ab,'output')==decl(bb,'output'),('actual module role set changed',scope,decl(ab,'input'),decl(bb,'input'))
   boundaries=[]
   for port in set(decl(ab,'input'))|set(decl(ab,'output')):
    a,b=oldbind['ports'][port],newbind['ports'][port];assert len(a)==len(b)
    for bit,(x,y)in enumerate(zip(a,b)):
     if x==y:method='same_bound_net'
     elif transparent(before,od,x)==transparent(after,nd,y):method='actual_buffer_root_equal'
     else:method=eq(scope+'/'+port+f'[{bit}] boundary',la.expr(x),lb.expr(y))
     boundaries.append(dict(port=port,bit=bit,before=x,after=y,method=method))
   def local(mods,bind,label):
    body=mods[bind['module_kind']];todo=[t for t,n,p in CELL.findall(body)if t in mods];extra=set()
    while todo:
     k=todo.pop()
     if k in extra:continue
     extra.add(k);todo +=[t for t,n,p in CELL.findall(mods[k])if t in mods and t not in extra]
    path=OUT/(label+'_actual_'+scope.replace('/','_')+'.v');path.write_text('module g2_top ('+body+'endmodule\n'+''.join('module '+k+' ('+mods[k]+'endmodule\n'for k in sorted(extra)),encoding='utf8')
    cells=read(path);params=parameter_reader(definitions(path));ffs={one(c,'Q'):c for c in cells.values()if c['kind'].startswith('DFF')};return body,cells,params,ffs
   _,ac,ap,af=local(om,oldbind,'SYN40');_,bc,bp,bf=local(nm,newbind,'SYN42');assert af.keys()==bf.keys(),('actual local FF state roles changed',scope,set(af)^set(bf))
   if expected_ffs:assert len(af)==expected_ffs,(scope,len(af))
   states={q:z3.Bool(scope+'/state:'+q)for q in af};a,b=Logic(ac,ap,directions,states,{}),Logic(bc,bp,directions,states,{})
   checks=[]
   for q in sorted(af):
    x,y=af[q],bf[q];assert x['kind']==y['kind']and ap(x)==bp(y)and x['ports']['CLK']==y['ports']['CLK'],('unchanged stop state kind/INIT/clock changed',scope,q,x,y)
    checks.append(dict(role=q,proof=eq(scope+'/'+q+'/next',a.next_state(x),b.next_state(y))))
   for p,(hi,lo)in decl(ab,'output').items():
    bits=[p]if not hi else[p+f'[{i}]'for i in range(int(hi),int(lo)-1,-1)]
    for q in bits:checks.append(dict(role=q+'/public_output',proof=eq(scope+'/'+q+'/output',a.expr(q),b.expr(q))))
   return dict(passed=True,scope=scope,actual_FFs=len(af),actual_ports_boundaries=boundaries,all_actual_next_states_and_outputs_equal=checks,all_original_kinds_INIT_clock_equal=True,binary_partition_proof_only=True)
  ir=section('actual_IR_stop_partition',lambda:module_proof('u_control/remote_thread'))
  lease=section('actual_lease_stop_partition',lambda:module_proof('u_command_lease',200))
  def timing():
   paths=report_paths(NEW);assert paths
   c=next((r for r in paths if'sensor_age_6' in str(r)and'snapshot_word_14'in str(r)),None)
   return dict(passed=True,available_listed_path_count=len(paths),actual_listed_paths=paths,new_ring_prefetch_endpoint_path=c,source_control_byte_unchanged=sections['source_stop_contract']['passed'],synthesis_estimates_not_routed_STA=True,actual_targets_MHz=dict(clk50=50,rgmii_rxc=100),global_frequency_hint_50_not_RX_10ns_acceptance=True,complete_negative_endpoint_coverage=False,timing_passed=False)
  section('actual_synthesis_estimated_paths',timing)
  assert bindings==[metadata(p)for p in watched],'actual input changed during review'
  result=dict(state,phase='review_complete',passed=all(v['passed']for v in sections.values()),at=datetime.now().astimezone().isoformat(),sections=sections,actual_two_clock_roots_only=clocks.get('passed'),actual_original26_proved=cdc.get('passed'),actual_FIFO_reset_types_proved=fifo.get('passed'),actual_IR_partition_proved=ir.get('passed'),actual_200FF_lease_partition_proved=lease.get('passed'),input_bindings=bindings,physical_unknown=['actual all52 Actived commands and endpoints after PNR','sourceQ-to-firstD physical data delay, first-to-second setup resolution/MTBF','reset recovery/removal/min pulse width and stopped-clock restart','whole4500 numerical/CPU/RK busy stop/physical stopping','routed0/setup0/hold0/IO and complete signoff'],deployment_accepted=False)
  save(OUT/'result.json',result);save(OUT/'job.json',result);print(json.dumps(dict(passed=result['passed'],output=str(OUT),sections=sections),ensure_ascii=False));return 0 if result['passed']else 1
 except Exception as e:
  result=dict(state,passed=False,phase='failed_preserved',error=str(e),traceback=traceback.format_exc(),sections=sections);save(OUT/'result.json',result);save(OUT/'job.json',result);print(result['traceback']);return 1
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');p.add_argument('--out',type=Path,default=OUT);p.add_argument('--build',type=Path,default=NEW);a=p.parse_args();assert a.run;OUT=a.out;NEW=a.build;raise SystemExit(run())
