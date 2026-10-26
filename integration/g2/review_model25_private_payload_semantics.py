"""Actual SYN36 complete surviving mailbox words and final consumer proof.

Bind data-zero synchronous DFFRE functions, not bare D pins.  Reuse only the
previous frozen producer arithmetic proof after bytewise source binding; no
private INIT, suffix alias, finite4500, or physical timing assumption is used.
"""
from pathlib import Path
import argparse,json,re,datetime,traceback
from review_model25_joint_syn import *
BASELINE=BASE/'baseline_fusion_model25_NAV50_syn34'
NEW=BASE/'baseline_fusion_model25_private_source_noreset_syn36'
PREP=BASE/'baseline_fusion_model25_private_source_noreset_prepared2'
OLDPROOF=BASE/'model25_SYN34_actual_review1'
PROTOCOL=BASE/'private_source_SYN36_actual_review3'

def run(out):
 old_full=load(OLDPROOF/'payload_semantics_review1/final_review.json');assert old_full['passed']
 old=load(OLDPROOF/'actual_review4/actual_five_related_channels.json');assert old['passed']
 new=load(PROTOCOL/'actual_new_private_source_transport.json');assert new['passed']
 assert load(PROTOCOL/'base_inventory.json')['passed']
 checks=[]
 def check(label,a,b,condition=None):
  delta=z3.Xor(a,b)
  if condition is not None:delta=z3.And(condition,delta)
  delta=z3.simplify(delta)
  if not z3.is_false(delta):
   solver=z3.Solver();solver.set(timeout=20000);solver.add(delta);result=solver.check()
   assert result==z3.unsat,(label,str(result),str(solver.model())if result==z3.sat else solver.reason_unknown())
  checks.append(dict(label=label,passed=True,method='canonical_or_UNSAT_Boolean_miter'))
 # The prior all24bit action/all32bit gravity/all16bit gyro proof remains
 # applicable only to these exact source expressions and frozen producer RTL.
 bindings=[]
 for rel in ('control/policy_action_map.v','control/imu_fusion.v','control/mem_policy.v','control/mem_policy_related25.v','control4500_core.v'):
  assert same(BASELINE/rel,NEW/rel)and same(PREP/rel,NEW/rel)
  bindings.append(meta(NEW/rel))
 oldvg=BASELINE/'impl/gwsynthesis/g2_top.vg';newvg=NEW/'impl/gwsynthesis/g2_top.vg'
 keep=lambda n:n=='u_control'or n.startswith('u_control/')
 before,after=read(oldvg,keep),read(newvg,keep)
 olddefs,newdefs=definitions(oldvg),definitions(newvg)
 pa,pb=parameter_reader(olddefs),parameter_reader(newdefs)
 directions=primitive_directions(LIB)
 la,lb,boundaries=construct_shared_logic(before,after,pa,pb,directions)
 oldch={x['name']:x for x in old['channels']};newch={x['name']:x for x in new['channels']}
 oldsrc={n:{x['logical_source_index']:x['cell']for x in c['source_payload']}for n,c in oldch.items()}
 newpriv={n:{x['bit']:x['cell']for x in c['private']}for n,c in newch.items()}
 source_of_q={n:{x['cell']['ports']['Q'][0]:x['bit']for x in c['private']}for n,c in newch.items()}
 def ex(net):return lb.expr(net)
 def capture(ff):return z3.And(z3.Not(lb.expr(ff['ports'].get('RESET',['GND'])[0])),lb.expr(ff['ports']['D'][0]))
 fields={}
 for b,field,w in [(0,'inference_cycles_25',32),(32,'completed_frames_25',32),(208,'servo_rk_us_25',16),(224,'servo_rh_us_25',16),(240,'servo_lk_us_25',16),(256,'servo_lh_us_25',16),(272,'target_right_mmps_25',32),(304,'target_left_mmps_25',32)]:
  for j in range(w):fields[b+j]=(field,j)
 fusion_fields=[(1,'debug_gz_25'),(25,'debug_gy_25'),(49,'debug_gx_25'),(73,'debug_wz_25'),(97,'debug_wy_25'),(121,'debug_wx_25')]
 status_alias={10:13,5:11,4:12}
 def refbit(label,bit):
  if bit in oldsrc[label]:return bit
  if label=='configuration'and bit==139:return 'VCC'
  if label=='action':
   if 64<=bit<208:return 'UNUSED_OUTER'
   field,j=fields[bit]
   if field.startswith('servo_')and j>=12:return 'GND'
   if field.startswith('target_')and j>=23:return bit-j+23
  if label=='fusion'and bit:
   b,field=next((b,f)for b,f in fusion_fields if b<=bit<b+24)
   assert (bit-b)>= (15 if '_g'in field else 20)
   return b+23
  if label=='status'and bit in status_alias:return status_alias[bit]
  raise AssertionError(('unclassified_original_word_bit',label,bit))
 def old_value(label,bit):
  k=refbit(label,bit)
  if k in ('GND','VCC'):return z3.BoolVal(k=='VCC')
  assert k!='UNUSED_OUTER'
  return la.expr(oldsrc[label][k]['ports']['D'][0])
 def accept(c):return ex(c['private'][0]['cell']['ports']['CE'][0])
 source_rows=[]
 for label,c in newch.items():
  ac=accept(c)
  for bit,ff in newpriv[label].items():
   check(label+f'/capture_word_bit{bit}',capture(ff),old_value(label,bit),ac)
   source_rows.append(dict(channel=label,logical_bit=bit,actual=ff,original_reference_bit=refbit(label,bit),effective_DFFRE_data_function_includes_synchronous_zero=True,capture_condition_proved=True))
 assert len(source_rows)==551
 save(out/'actual_private551_source_word_binding.json',dict(passed=True,source_bindings=bindings,prior_full_numeric_proof=str(OLDPROOF/'payload_semantics_review1/final_review.json'),private551=source_rows,cold_private_initial_value_assumed=False))
 # Check every real destination DFF input against complete-word semantics,
 # including changed sign representatives.  A matching cell suffix is not a
 # source identity proof: original source Boolean values are compared.
 destinations=[];packet={}
 for label,c in newch.items():
  packet[label]={}
  for row in c['destination']:
   bit,ff=row['bit'],row['cell'];net=ff['ports']['D'][0]
   if net in source_of_q[label]:
    sb=source_of_q[label][net]
    check(label+f'/source_alias_to_destination{bit}',old_value(label,sb),old_value(label,bit))
    src=dict(private_bit=sb,private_source=net)
   else:
    assert net in('VCC','GND');check(label+f'/constant_destination{bit}',ex(net),old_value(label,bit));src=dict(constant=net)
   packet[label][bit]=ff['ports']['Q'][0]
   destinations.append(dict(channel=label,logical_destination_bit=bit,actual_destination=ff,source=src,full_word_identity_proved=True))
 save(out/'actual_complete_destination_words.json',dict(passed=True,rows=destinations))
 # Configuration yaw bits24..47 are real hard-DSP AREG state.  The remaining
 # controls/reset use the same destination commit already Boolean-proved.
 dc=next(x for n,x in after.items()if n=='u_control/model_thread/model/observation/n1150_s')
 assert dc['kind']=='MULTALU27X18';p=pb(dc);a=list(reversed(dc['ports']['A']))
 assert a[:24]==[newpriv['configuration'][24+j]['ports']['Q'][0]for j in range(24)]
 assert a[24:]==[a[23]]*3
 for key,val in {'AREG_CLK':'"CLK0"','AREG_CE':'"CE0"','AREG_RESET':'"RESET0"','MULT_RESET_MODE':'"ASYNC"','BREG_CLK':'"BYPASS"','DREG_CLK':'"BYPASS"','PREG_CLK':'"BYPASS"','OREG_CLK':'"BYPASS"'}.items():assert p.get(key)==val,(key,p.get(key))
 d0=newch['configuration']['destination'][0]['cell']
 for pin,dpin in [('CLK','CLK'),('CE','CE'),('RESET','CLEAR')]:assert list(reversed(dc['ports'][pin]))[0]==d0['ports'][dpin][0]
 assert sorted(set(range(140))-set(packet['configuration']))==list(range(24,48))
 save(out/'actual_configuration_DSP_AREG24.json',dict(passed=True,actual_DSP=dc,parameters=p,logical_bits=list(range(24,48)),CLK_CE_RESET_match_public_destination=True))
 # Both latest producers must implement the complete retained source word
 # mismatch.  Capture CE was deliberately not used as its own specification.
 latest=[]
 for label in('configuration','status'):
  c=newch[label];ctrl=c['controls'];sent=c['public_sent'];ready=ex(ctrl['request']['ports']['Q'][0])==ex(ctrl['ack2']['ports']['Q'][0])
  changed=z3.Or(*[z3.Xor(old_value(label,i),ex(ff['ports']['Q'][0]))for i,ff in newpriv[label].items()])
  check(label+'/latest_complete_retained_word_change_no_reset_gate',accept(c),z3.And(ready,z3.Or(z3.Not(ex(sent['ports']['Q'][0])),changed)))
  latest.append(dict(channel=label,retained_private=len(newpriv[label]),public_sent=sent,complete_change_detector_and_no_added_capture_reset_gate_proved=True))
 save(out/'actual_latest_full_word_CE_no_reset_gate.json',dict(passed=True,channels=latest))
 # Final consumer transitions are checked from actual destination Qs, keeping
 # the original latest fault/enable/feed-forward and output cycle behavior.
 def packet_q(label,i):
  if i in packet[label]:return ex(packet[label][i])
  if label=='action':
   field,j=fields[i]
   if field.startswith('servo_')and j>=12:return z3.BoolVal(False)
   if field.startswith('target_')and j>=23:
    k=next(k for k in packet[label]if k in fields and fields[k][0]==field and fields[k][1]>=23)
    return ex(packet[label][k])
  if label=='fusion'and i:
   b,field=next((b,f)for b,f in fusion_fields if b<=i<b+24)
   assert i-b>= (15 if '_g'in field else 20)
   k=next(k for k in packet[label]if b<=k<b+24 and k-b>= (15 if '_g'in field else 20))
   return ex(packet[label][k])
  raise AssertionError(('missing_destination_value',label,i))
 reset=ex('u_control/n2375_7');fault=z3.Or(ex('u_control/clocked_fault'),ex('u_control/lock_fault_async'))
 stopped=z3.Or(z3.Not(packet_q('status',14)),fault,old_value('configuration',136),z3.Not(old_value('configuration',139)))
 ac=ex('u_control/model_thread/action_commit50');fc=ex('u_control/model_thread/fusion_commit50')
 consumers=[]
 for name,ff in after.items():
  m=re.fullmatch(r'u_control/model_thread/(action|fusion)_hold50_(\d+)_s\d+',name)
  if not m:continue
  label,i=m[1],int(m[2]);data,current=packet_q(label,i),ex(ff['ports']['Q'][0]);assert ff['ports']['CLK']==['clk50_d']
  if label=='action':
   field,j=fields[i]
   if field.startswith('target_'):
    target='model_left_mmps'if 'left'in field else'model_right_mmps'
    projected=re.fullmatch('u_control/'+target+r'\[(\d+)\]',ff['ports']['Q'][0]);assert projected,(name,ff['ports']['Q'],target)
    physical_bit=int(projected[1]);assert physical_bit==j or min(physical_bit,j)>=23
    want=z3.If(reset,False,z3.If(stopped,False,z3.If(ac,data,current)))
   else:
    if field.startswith('servo_'):
     target=field.replace('servo_','model_').replace('_25','')
    else:target='model_'+field.replace('_25','')
    assert ff['ports']['Q']==['u_control/'+target+f'[{j}]'],(name,ff['ports']['Q'],target,j)
    initial=bool(1500>>j&1)if field.startswith('servo_')else False
    want=z3.If(reset,initial,z3.If(z3.And(z3.Not(stopped),ac),data,current))
  else:
   if i:
    b,field=next((b,f)for b,f in fusion_fields if b<=i<b+24)
    target=field.replace('_25','');projected=re.fullmatch('u_control/'+target+r'\[(\d+)\]',ff['ports']['Q'][0]);assert projected,(name,ff['ports']['Q'],target)
    physical_bit=int(projected[1]);lo=15 if '_g'in field else 20
    assert physical_bit==i-b or min(physical_bit,i-b)>=lo
   else:assert ff['ports']['Q']==['u_control/fusion_hold50[0]']
   initial=bool(i in range(13,25));want=z3.If(reset,initial,z3.If(z3.And(fc,z3.Not(fault)),data,current))
  check(name+'/actual_final_consumer_next',lb.next_state(ff),want)
  consumers.append(dict(channel=label,logical_bit=i,actual_consumer=ff,transition_proved=True))
 assert sum(x['channel']=='action'for x in consumers)==160 and sum(x['channel']=='fusion'for x in consumers)==112
 pulse=after['u_control/model_thread/fusion_pulse_s0'];check('fusion_valid_final_pulse',lb.next_state(pulse),z3.If(reset,False,z3.And(fc,z3.Not(fault))))
 assert pulse['ports']['Q']==['u_control/debug_fusion_valid']
 accel=after['u_control/telemetry_thread/n2158_s0']
 assert accel['kind']=='LUT4'and pb(accel)=={'INIT':"16'hF088"}
 assert accel['ports']['I0']==['u_control/debug_fusion_valid']and accel['ports']['I1']==['u_control/fusion_hold50[0]']
 check('accel_rejected_packet_bit0_and_actual_telemetry_projection',ex(accel['ports']['F'][0]),z3.If(ex(accel['ports']['I3'][0]),ex(accel['ports']['I2'][0]),z3.And(ex(accel['ports']['I0'][0]),ex(accel['ports']['I1'][0]))))
 # Configuration enable is a literal1 source. Its old shared private sent bit
 # was split into a public resettable sent, not silently made reset-free.
 assert oldsrc['configuration'][139]['ports']['D']==['VCC']
 assert newch['configuration']['public_sent']['kind']=='DFFCE'
 save(out/'actual_final_consumer272_transitions.json',dict(passed=True,consumers=consumers,actual_Q_projection_into_named_original_fields_checked=True,accel_rejected_and_valid_actual_telemetry_projection_checked=True,source_enable_literal1=True,public_sent_still_resettable=True))
 save(out/'actual_payload_Boolean_checks.json',dict(passed=True,checks=checks,actual_state_boundaries=boundaries,all_source551_and_destination_words_and_latest_CE_and_final272_checked=True))
 result=dict(passed=True,recorded_at=datetime.datetime.now().astimezone().isoformat(),actual_source=str(NEW),actual_VG_metadata=meta(newvg),reference_VG_metadata=meta(oldvg),structural_protocol=str(PROTOCOL/'actual_new_private_source_transport.json'),prior_full_signed_math_and_feedback_proof=str(OLDPROOF/'payload_semantics_review1/final_review.json'),byte_bound_frozen_producer_sources=bindings,private551_capture_functions_proved=True,all_actual_destination_DFF_words_and_hard_DSP24_AREG_proved=True,full_latest_change_detection_without_added_reset_gate_proved=True,final_consumer_transitions272_proved=True,Boolean_checks=len(checks),unknown_word_bits=[],whole_system_sequential_equivalence_claimed=False,physical_setup_hold_bundle_delay_reset_pulse_IO_accepted=False,no_SYN_PNR_started=True)
 save(out/'final_review.json',result);print(json.dumps(result,ensure_ascii=False));return result
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(exist_ok=False)
 try:run(a.out)
 except BaseException:save(a.out/'failed.json',dict(passed=False,traceback=traceback.format_exc()));raise
