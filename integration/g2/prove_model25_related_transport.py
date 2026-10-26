"""Actual SYN34 guarded-reset, original radar CDC, and mailbox-transition proof.
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

 # Prove transition controls on every surviving mapped payload FF. Missing bus
 # bits may be constants or sign-shared and are not silently counted as FFs.
 channel_rows=[]
 for label,scope,width,srcdomain,dstdomain in CHANNELS:
  c['stage']('prove_actual_channel_'+label)
  controls={s:reg(scope,s)for s in('ack1','ack2','req1','req2','request','acknowledge')}
  srcreset=z3.Not(q(releases[srcdomain]));dstreset=z3.Not(q(releases[dstdomain]))
  for s in('request','ack1','ack2'):ff_clock(controls[s],srcdomain);async_clear(controls[s],srcreset)
  for s in('req1','req2','acknowledge'):ff_clock(controls[s],dstdomain);async_clear(controls[s],dstreset)
  for stem,source in [('ack1','acknowledge'),('ack2','ack1'),('req1','request'),('req2','req1')]:
   assert controls[stem]['ports']['D']==controls[source]['ports']['Q']
   assert controls[stem]['ports']['CE']==['VCC']
  for stem,nextstem in [('ack1','ack2'),('req1','req2')]:
   assert loads[controls[stem]['ports']['Q'][0]]==[(controls[nextstem]['name'],'D')]
  ready=(q(controls['request'])==q(controls['ack2']));commit=z3.Xor(q(controls['req2']),q(controls['acknowledge']))
  module=c['modules'][scope]
  source_payload=bit_registers(scope,'holding')
  if label in('configuration','status'):
   source_payload=bit_registers(scope.rsplit('/',1)[0],'last_sent')
   assert source_payload
   actual_accept_net=next(iter(source_payload.values()))['ports']['CE']
   assert len(actual_accept_net)==1
   accept=expression(actual_accept_net[0]);valid=accept
   check(label+'/actual_latest_capture_enable_implies_ready',z3.And(accept,z3.Not(ready)),z3.BoolVal(False))
   actual_accept_binding='actual last_sent register CE, independently proved to imply request==ack2; do not infer an alias from autogenerated send_4'
  else:
   valid_port={'BMI':'data_valid','action':'action_commit25','fusion':'debug_fusion_valid_25'}[label]
   assert valid_port in module['ports']and len(module['ports'][valid_port])==1
   actual_accept_net=module['ports'][valid_port]
   valid=expression(actual_accept_net[0]);accept=z3.And(valid,ready)
   actual_accept_binding='actual event-valid input AND request==ack2'
  next_check(controls['request'],z3.If(srcreset,z3.BoolVal(False),z3.If(accept,z3.Not(q(controls['request'])),q(controls['request']))))
  next_check(controls['acknowledge'],z3.If(dstreset,z3.BoolVal(False),z3.If(commit,q(controls['req2']),q(controls['acknowledge']))))
  destination_payload=bit_registers(scope,'dst_data')
  assert source_payload and destination_payload
  assert all(0<=i<width for i in source_payload.keys()|destination_payload.keys())
  source_by_q={ff['ports']['Q'][0]:(i,ff)for i,ff in source_payload.items()}
  payload=[]
  for bit,ff in source_payload.items():
   ff_clock(ff,srcdomain);async_clear(ff,srcreset)
   check(ff['name']+'/hold_until_ack_accept_only',one(ff,'CE'),accept)
   payload.append(dict(role='source_holding',logical_source_index=bit,cell=ff,actual_parameters=newparams(ff),CE_accept_proved=True))
  destinations=[]
  for bit,ff in destination_payload.items():
   ff_clock(ff,dstdomain);async_clear(ff,dstreset)
   check(ff['name']+'/capture_only_complete_request',one(ff,'CE'),commit)
   assert len(ff['ports']['D'])==1
   net=ff['ports']['D'][0];assert net in source_by_q,(label,bit,ff['name'],net)
   sourcebit,sourceff=source_by_q[net]
   # The mapped word may share sign/constant representatives. Atomicity is
   # checked using the actual same-channel stable holding driver, without
   # claiming suffix indices establish original-word numeric equivalence.
   destinations.append(dict(logical_destination_index=bit,source_index=sourcebit,suffix_indices_equal=(sourcebit==bit),numeric_alias_identity_proved_by_this_review=False,source=sourceff['name'],destination=ff['name'],actual_destination=ff))
  dv=reg(scope,'dst_valid',True)
  if dv:
   ff_clock(dv,dstdomain);async_clear(dv,dstreset)
   next_check(dv,z3.If(dstreset,z3.BoolVal(False),commit))
  ov=reg(scope,'src_overflow',True)
  if ov:
   ff_clock(ov,srcdomain);async_clear(ov,srcreset)
   next_check(ov,z3.If(srcreset,z3.BoolVal(False),z3.Or(q(ov),z3.And(valid,z3.Not(ready)))))
  channel_rows.append(dict(name=label,scope=scope,RTL_logical_width=width,source_domain=srcdomain,destination_domain=dstdomain,actual_control_registers=controls,actual_accept_net=actual_accept_net,actual_accept_binding=actual_accept_binding,latest_change_detection_complete_word_equivalence_proof_pending=label in('configuration','status'),actual_retained_source_payload_FFs=len(source_payload),actual_retained_destination_payload_FFs=len(destination_payload),source_payload=payload,destination_source_bit_connections=destinations,actual_2stage_req_ack_CE_reset_and_payload_hold_proved=True,full_original_word_bit_alias_equivalence_not_proved=True,physical_setup_hold_and_bundle_stability_delays_unverified=True,destination_valid_pulse_FF_retained=dv is not None,overflow_FF_retained=ov is not None))
 save(out/'actual_five_related_channels.json',dict(passed=True,channels=channel_rows,related_clocks_not_asynchronous=True,all_retained_payload_update_enables_and_same_channel_holding_drivers_proved=True,logical_word_widths_not_reduced_in_source=True,full_original_word_bit_alias_numeric_equivalence_claimed=False,physical_payload_timing_and_metastability_resolution_not_signed_off=True))
 save(out/'actual_Boolean_checks.json',dict(passed=True,checks=checks,actual_state_boundaries=boundaries,full_sequential_equivalence_claimed=False))
 base=json.loads((out/'base_inventory.json').read_text())
 final=dict(passed=True,scope='Completed actual SYN34 source/memory/INIT/PLL3clock/original26/5retained-payload-transport/guard proof, not physical or whole-model signoff',recorded_at=datetime.datetime.now().astimezone().isoformat(),base=base,actual_RAM314_width18_CPU128KiB_FW_preserved=True,original_radar_CDC26_passed=True,actual_five_related_atomic_channels_retained_controls_passed=True,guard_INIT1_cold_start_clearing_passed=True,actual_fault_INIT=1,source_fault_INIT0_was_ignored=True,Boolean_proved_endpoints=len(checks),actual_channel_counts=[{k:r[k]for k in('name','RTL_logical_width','actual_retained_source_payload_FFs','actual_retained_destination_payload_FFs')}for r in channel_rows],all_original_word_bit_alias_numeric_equivalence=False,physical_global_resources_routes_setup_hold_CDC_bundle_reset_pulse_IO_unverified=True,full4500_end_to_end_numerical_function_and125Hz_acceptance_pending=True,full_stop_and_deployment_accepted=False,no_SYN_PNR_started_by_review=True,programmed=False,metadata=[meta(NEW/'impl/gwsynthesis/g2_top.vg'),meta(Path(__file__)),meta(Path(__file__).with_name('review_model25_joint_syn.py'))])
 save(out/'final_review.json',final);c['stage']('actual_structural_review_passed',passed=True)
 print(json.dumps({k:final[k]for k in('passed','Boolean_proved_endpoints','guard_INIT1_cold_start_clearing_passed','actual_channel_counts')},ensure_ascii=False))
