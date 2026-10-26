"""Actual SYN36 guarded-reset, original radar CDC, and mailbox-transition proof.
Uses public Gowin LUT/ALU/register Boolean semantics. It is not sequential whole
system equivalence, physical CDC/payload timing, or vehicle stop acceptance.
"""
import collections,json,re,datetime
from pathlib import Path
from prove_soc_console_remap import z3,Logic,literal
from prepare_no_sfp_clock_plan import construct_shared_logic


def complete(c):
 out=c['out'];save=c['save'] if'save'in c else None
 # run() returns locals, while module-level save/metadata are imported below.
 from review_model25_joint_syn import save,meta,BASE,NEW,OLD,PREP,REF,LIB,CHANNELS
 before,after=c['before'],c['after'];oldparams,newparams=c['oldparams'],c['newparams']
 directions,drivers,loads,clock=c['directions'],c['drivers'],c['loads'],c['clock']
 la,logic,boundaries=construct_shared_logic(before,after,oldparams,newparams,directions)
 checks=[]
 def check(label,a,b,condition=None):
  delta=z3.Xor(a,b)
  if condition is not None:delta=z3.And(condition,delta)
  delta=z3.simplify(delta)
  method='canonical_Boolean_identity'
  if not z3.is_false(delta):
   solver=z3.Solver();solver.set(timeout=10000);solver.add(delta);status=solver.check()
   assert status==z3.unsat,(label,str(status),str(solver.model())[:1400]if status==z3.sat else solver.reason_unknown())
   method='SAT_unsatisfiable_miter'
  checks.append(dict(label=label,passed=True,method=method))
 def expression(net):return logic.expr(net)
 def one(cell,pin):assert len(cell['ports'][pin])==1;return expression(cell['ports'][pin][0])
 def reg(scope,stem,optional=False):
  rows=[x for n,x in after.items()if n.startswith(scope+'/')and '/'not in n[len(scope)+1:]and x['kind'].startswith('DFF')and re.fullmatch(re.escape(stem)+r'_s\d+',n.rsplit('/',1)[1])]
  if optional and not rows:return None
  assert len(rows)==1,(scope,stem,[x['name']for x in rows]);return rows[0]
 def q(cell):return one(cell,'Q')
 def ff_clock(cell,expected):assert clock(cell['ports']['CLK'][0])==(expected,0),(cell['name'],cell['ports']['CLK'],expected)
 def async_clear(cell,clear):
  assert cell['kind']=='DFFCE',(cell['name'],cell['kind'])
  check(cell['name']+'/actual_async_CLEAR',one(cell,'CLEAR'),clear)
 def next_check(cell,want,label=None,condition=None):check(label or cell['name']+'/next',logic.next_state(cell),want,condition)
 def bit_registers(scope,stem):
  rows={}
  for n,x in after.items():
   if not n.startswith(scope+'/')or '/'in n[len(scope)+1:]or not x['kind'].startswith('DFF'):continue
   m=re.fullmatch(re.escape(stem)+r'_(\d+)_s\d+',n.rsplit('/',1)[1])
   if m:assert int(m[1])not in rows;rows[int(m[1])]=x
  return rows

 # Explicitly inspect the actual ignored INIT mapping before channel proofs.
 guard='u_control/model_clock_guard'
 startup=bit_registers('u_control','startup');assert set(startup)==set(range(8))
 for i,ff in startup.items():
  ff_clock(ff,'clk50');assert ff['name']in before
  old=before[ff['name']]
  assert ff['kind']==old['kind']and newparams(ff)==oldparams(old)
  assert literal(newparams(ff).get('INIT',"1'b0"))==0
  check(ff['name']+'/unchanged_mapped_next_state',la.next_state(old),logic.next_state(ff))
 sq=[q(startup[i])for i in range(8)];full=z3.And(*sq)
 reset50=one(startup[0],'RESET')
 for i,ff in startup.items():
  assert ff['kind']=='DFFRE'
  check(ff['name']+'/same_reset',one(ff,'RESET'),reset50)
  increment=z3.Xor(sq[i],z3.And(*sq[:i])if i else z3.BoolVal(True))
  next_check(ff,z3.If(reset50,z3.BoolVal(False),z3.If(full,sq[i],increment)),ff['name']+'/actual_saturating_8bit_startup')
 seen=reg(guard,'seen_lock');clear50=one(seen,'CLEAR')
 check('guard_reset_is_original_reset_OR_startup_not_255',clear50,z3.Or(reset50,z3.Not(full)))
 lf=reg(guard,'lock_fault_async');cf=reg(guard,'clocked_fault')
 assert lf['kind']=='DFFPE'and newparams(lf).get('INIT')=="1'b1"
 ff_clock(lf,'clk50');assert lf['ports']['D']==['GND']
 check('async_lock_latch_enable_is_device_reset_active',one(lf,'CE'),clear50)
 locknet=c['pll']['ports']['LOCK'][0];lock=expression(locknet)
 check('actual_PLL_loss_asynchronous_PRESET',one(lf,'PRESET'),z3.And(q(seen),z3.Not(lock)))
 next_check(lf,z3.If(z3.And(q(seen),z3.Not(lock)),z3.BoolVal(True),z3.If(clear50,z3.BoolVal(False),q(lf))),'PLL_fault_sticky_next_no_INIT0_assumption')
 cold=[(x,z3.BoolVal(False))for x in sq]+[(q(seen),z3.BoolVal(False))]
 subst=lambda expr:z3.simplify(z3.substitute(expr,*cold))
 check('startup_actual_INIT0_holds_device_reset_active',subst(clear50),z3.BoolVal(True))
 check('cold_seen_lock0_makes_async_preset_inactive',subst(one(lf,'PRESET')),z3.BoolVal(False))
 check('first_50MHz_edge_clears_actual_INIT1_fault',subst(logic.next_state(lf)),z3.BoolVal(False))
 lock1,lock2=reg(guard,'lock1'),reg(guard,'lock2')
 for ff in(seen,cf,lock1,lock2):async_clear(ff,clear50);ff_clock(ff,'clk50')
 next_check(seen,z3.If(clear50,z3.BoolVal(False),z3.Or(q(seen),q(lock2))))
 check('PLL_lock_first_stage',one(lock1,'D'),lock);check('PLL_lock_second_stage',one(lock2,'D'),q(lock1))
 releases={}
 for suffix,domain in [('50','clk50'),('25','model25')]:
  a,b=reg(guard,'release'+suffix+'_0'),reg(guard,'release'+suffix+'_1')
  desired_clear=z3.Or(clear50,z3.Not(lock),q(lf),q(cf))
  for ff in(a,b):async_clear(ff,desired_clear);ff_clock(ff,domain);assert ff['ports']['CE']==['VCC']
  assert a['ports']['D']==['VCC'];assert b['ports']['D']==a['ports']['Q']
  check('release'+suffix+'_cold_async_assertion',subst(desired_clear),z3.BoolVal(True))
  releases[domain]=b
 guard_result=dict(passed=True,source_INIT0_ignored=True,actual_fault_primitive=lf,actual_fault_parameters=newparams(lf),
  original_startup_registers=list(startup.values()),original_startup_INIT0_and_counter_logic_preserved=True,
  original_startup_holds_device_reset_active_for_255_50MHz_edges=True,
  fault_INIT1_cleared_on_first_50MHz_edge_while_device_reset_active_and_seen_lock_async_cleared=True,
  proof_does_not_assume_fault_INIT0=True,
  actual_PLL_loss_asynchronous_preset_verified=True,fault_retained_until_physical_reset_low_with_50MHz_edge=True,
  independent_two_edge_50_and25_reset_releases_verified=True,
  PLL_clock_and_reset_physical_recovery_removal_pulse_and_IO_not_signed_off=True,
  startup_assumption='Actual original eight startup FF INIT0 are retained. A physical reset must cover a valid50MHz edge for synchronous startup/latch clearing; electrical reset/GSR pulse and power sequencing remain deployment checks.',checks=checks.copy())
 save(out/'actual_guard_cold_start_and_INIT1_proof.json',guard_result)
 c['stage']('guard_INIT1_cold_clear_proved_inspect_radar26')

 # All original26 are preserved independently of the additional related-clock channels.
 reference=json.loads(REF.read_text(encoding='utf8'));assert reference['passed']and len(reference['paths'])==26
 radar=[]
 for row in reference['paths']:
  for role in('source_register','first_register','second_register'):
   name=row[role];a,b=before[name],after[name]
   assert a['kind']==b['kind']and oldparams(a)==newparams(b),(name,'kind/INIT')
   assert a['ports']['CLK']==b['ports']['CLK']
   for pin in a['ports']:
    assert len(a['ports'][pin])==len(b['ports'][pin])
    if directions[a['kind']][pin]=='input'and pin!='CLK':
     for i,(an,bn)in enumerate(zip(a['ports'][pin],b['ports'][pin])):
      if an!=bn:check(name+'/'+pin+f'[{i}]',la.expr(an),logic.expr(bn))
  source,first,second=[after[row[x]]for x in('source_register','first_register','second_register')]
  assert source['ports']['Q']==first['ports']['D']and first['ports']['Q']==second['ports']['D']
  assert loads[first['ports']['Q'][0]]==[(second['name'],'D')]
  assert source['ports']['CLK']==[row['source_clock']]
  assert first['ports']['CLK']==second['ports']['CLK']==[row['target_clock']]
  assert first['ports']['CE']==second['ports']['CE']==['VCC']
  radar.append(dict(**{k:row[k]for k in('group','source_register','first_register','second_register','source_clock','target_clock','route_max_delay_ns')},passed=True,actual_source=source,actual_first=first,actual_second=second))
 save(out/'actual_original_radar26.json',dict(passed=True,paths=radar,actual_new_input_cones_checked=True,new_timing_exceptions_added=False,physical_CDC_delays_and_MTBF_not_signed_off=True))

 # New private fields intentionally have no reset. Public control/reset state
 # retains the original contract, including destination payload reset.
 rows=[];all_private={}
 for label,scope,width,srcdomain,dstdomain in CHANNELS:
  c['stage']('review_new_private_capture_'+label)
  controls={s:reg(scope,s)for s in('ack1','ack2','req1','req2','request','acknowledge')}
  sr=z3.Not(q(releases[srcdomain]));dr=z3.Not(q(releases[dstdomain]))
  for s in('request','ack1','ack2'):ff_clock(controls[s],srcdomain);async_clear(controls[s],sr)
  for s in('req1','req2','acknowledge'):ff_clock(controls[s],dstdomain);async_clear(controls[s],dr)
  for stem,source in [('ack1','acknowledge'),('ack2','ack1'),('req1','request'),('req2','req1')]:
   assert controls[stem]['ports']['D']==controls[source]['ports']['Q']and controls[stem]['ports']['CE']==['VCC']
  for stem,nextstem in [('ack1','ack2'),('req1','req2')]:assert loads[controls[stem]['ports']['Q'][0]]==[(controls[nextstem]['name'],'D')]
  ready=q(controls['request'])==q(controls['ack2']);commit=z3.Xor(q(controls['req2']),q(controls['acknowledge']))
  bank0=bit_registers(scope,'holding');banks={'holding':bank0}if bank0 else{}
  latest=label in('configuration','status')
  if latest:
   bank1=bit_registers(scope.rsplit('/',1)[0],'last_sent')
   if bank1:banks['last_sent']=bank1
   assert banks
   private=next(iter(next(iter(banks.values())).values()))
   accept=one(private,'CE');valid=accept
   check(label+'/latest_actual_capture_implies_ready',z3.And(accept,z3.Not(ready)),z3.BoolVal(False))
   sent=reg(scope.rsplit('/',1)[0],'sent')
   ff_clock(sent,srcdomain);async_clear(sent,sr)
   next_check(sent,z3.If(sr,False,z3.If(accept,True,q(sent))))
   assert sent['ports']['D']==['VCC']
  else:
   m=c['modules'][scope];pin={'BMI':'data_valid','action':'action_commit25','fusion':'debug_fusion_valid_25'}[label]
   assert pin in m['ports']and len(m['ports'][pin])==1
   valid=expression(m['ports'][pin][0]);accept=z3.And(valid,ready);sent=None
  next_check(controls['request'],z3.If(sr,False,z3.If(accept,z3.Not(q(controls['request'])),q(controls['request']))))
  next_check(controls['acknowledge'],z3.If(dr,False,z3.If(commit,q(controls['req2']),q(controls['acknowledge']))))
  allq={};private_rows=[]
  for bank,ffmap in banks.items():
   for bit,ff in ffmap.items():
    ff_clock(ff,srcdomain)
    assert ff['kind']in('DFF','DFFE','DFFRE'),(ff['name'],'unsupported private primitive',ff['kind'],ff['ports'])
    assert not any(p in ff['ports']for p in('CLEAR','SET','PRESET'))
    check(ff['name']+'/capture_CE_has_no_added_reset',one(ff,'CE'),accept)
    # Gowin may absorb a source-data zero mux into the *synchronous* RESET
    # pin of DFFRE.  This is legal only when that pin cannot alter held data
    # outside capture.  Its effective captured Boolean value must still be
    # independently bound to the complete original source packet.
    synchronous_zero=one(ff,'RESET')if'RESET'in ff['ports']else z3.BoolVal(False)
    check(ff['name']+'/synchronous_data_zero_only_on_capture',z3.And(synchronous_zero,z3.Not(accept)),z3.BoolVal(False))
    capture=z3.And(z3.Not(synchronous_zero),one(ff,'D'))
    next_check(ff,z3.If(accept,capture,q(ff)))
    allq[ff['ports']['Q'][0]]=(bank,bit,ff)
    all_private[ff['name']]=ff
    private_rows.append(dict(bank=bank,bit=bit,cell=ff,actual_parameters=newparams(ff),no_asynchronous_reset=True,synchronous_data_zero_pin=ff['ports'].get('RESET',['GND'])[0],synchronous_zero_implies_capture_proved=True,capture_CE_and_transition_proved=True,full_captured_source_value_independent_review_pending=True))
  dest=bit_registers(scope,'dst_data');dest_rows=[]
  for bit,ff in dest.items():
   ff_clock(ff,dstdomain);async_clear(ff,dr);check(ff['name']+'/public_destination_capture',one(ff,'CE'),commit)
   assert len(ff['ports']['D'])==1
   net=ff['ports']['D'][0]
   if net in allq:
    bank,sbit,src=allq[net];source=dict(bank=bank,bit=sbit,cell=src['name'])
   else:
    assert net in('GND','VCC'),(label,bit,'destination not driven by same-channel private data or literal',net)
    source=dict(constant=net)
   dest_rows.append(dict(bit=bit,cell=ff,source=source))
  for stem,expected in [('dst_valid',z3.If(dr,False,commit)),('src_overflow',z3.If(sr,False,z3.Or(q(reg(scope,'src_overflow',True)),z3.And(valid,z3.Not(ready))))if reg(scope,'src_overflow',True)else None)]:
   ff=reg(scope,stem,True)
   if ff:
    ff_clock(ff,dstdomain if stem=='dst_valid'else srcdomain);async_clear(ff,dr if stem=='dst_valid'else sr);next_check(ff,expected)
  rows.append(dict(name=label,logical_width=width,source_domain=srcdomain,destination_domain=dstdomain,private_banks={k:len(v)for k,v in banks.items()},private_FF_count=sum(len(v)for v in banks.values()),public_sent=sent,controls=controls,private=private_rows,destination=dest_rows,public_destination_FFs=len(dest),same_channel_capture_and_public_control_reset_proved=True))
 save(out/'actual_new_private_source_transport.json',dict(passed=True,channels=rows,actual_private_FFs=len(all_private),private_no_asynchronous_reset_and_same_accept_CE_proved=True,synchronous_data_zero_pins_only_change_data_during_capture_proved=True,public_req_ack_valid_overflow_and_destination_reset_proved=True,actual_word_numeric_alias_identity_independent_review_pending=True))
 save(out/'actual_private_mapping_Boolean_checks.json',dict(passed=True,checks=checks,state_boundaries=boundaries))
 save(out/'private_transport_stage_result.json',dict(passed=True,actual_private_FFs=len(all_private),Boolean_checks=len(checks),all_public_guard_and_radar26_passed=True,word_identity_and_total_reset_load_and_resource_review_pending=True))
 return dict(checks=checks,channels=rows,all_private=all_private,logic=logic,clock=clock,newparams=newparams,after=after)
