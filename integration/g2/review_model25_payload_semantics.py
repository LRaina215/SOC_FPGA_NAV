"""Read-only supplement for actual SYN34 mailbox word aliases and DSP AREG.
Retains the earlier checker counterexamples; no frozen input/build mutation.
"""
from pathlib import Path
import sys,re,json,datetime,traceback
sys.path.insert(0,str(Path(__file__).parent))
from review_model25_joint_syn import *
OUT=None
B=BASE/'model25_SYN34_actual_review1/actual_review4'
checks=[]
def prove(label,delta,**context):
 s=z3.Solver();s.set(timeout=20000);s.add(z3.simplify(delta));r=s.check()
 if r!=z3.unsat:raise AssertionError((label,str(r),str(s.model())if r==z3.sat else s.reason_unknown()))
 checks.append(dict(label=label,passed=True,method='full_symbolic_unsatisfiable_miter',**context))
def bit(v,n):return z3.Extract(n,n,v)
def bv(n,w):return z3.BitVecVal(n,w)
def norm(s):return re.sub(r'\s+','',s)
def sourcebind(path,fragments):
 assert same(PREP/path,NEW/path),(path,'actual frozen source binding')
 s=(PREP/path).read_text(encoding='utf-8-sig');t=norm(s)
 for f in fragments:assert norm(f)in t,(path,f)
 return dict(path=str(PREP/path),fragments=fragments,metadata=meta(PREP/path))

def run():
 base=load(B/'final_review.json');assert base['passed']
 channels={c['name']:c for c in load(B/'actual_five_related_channels.json')['channels']}
 source_bindings=[]
 source_bindings.append(sourcebind('control/policy_action_map.v',[
  'clip=a>4096 ?24\'sd4096:a< -4096 ?-24\'sd4096:a;',
  'product=$signed(clip(a))*gain;',
  'scale=product<0 ? -((-product+2048)>>>12) : ((product+2048)>>>12);',
  'pulse=v<500 ?500:v>2500?2500:v;',
  'left_mmps<=scale(actions[48+:24],-600);','right_mmps<=scale(actions[120+:24],600);']))
 # Every possible signed24 policy action, not a finite enumerated clip sample.
 a=z3.BitVec('arbitrary_action24',24)
 ca=z3.If(a>bv(4096,24),bv(4096,24),z3.If(a<bv(-4096,24),bv(-4096,24),a))
 wheel_math=[]
 for gain in(-600,600):
  p=z3.SignExt(40,ca)*bv(gain,64)
  scaled=z3.If(p<bv(0,64),-((-p+bv(2048,64))>>12),(p+bv(2048,64))>>12)
  w=z3.Extract(31,0,scaled)
  prove('wheel_signed24_all_states_gain'+str(gain),z3.Or(w>bv(600,32),w<bv(-600,32)),input_states=2**24)
  prove('wheel_high_bits23_to31_are_equal_gain'+str(gain),z3.Or(*[bit(w,i)!=bit(w,31)for i in range(23,31)]),input_states=2**24)
  wheel_math.append(dict(gain=gain,range_min=-600,range_max=600,all_bits23_through31_equal=True))
 v=z3.BitVec('arbitrary_pulse_integer32',32)
 pulse=z3.If(v<bv(500,32),bv(500,32),z3.If(v>bv(2500,32),bv(2500,32),v))
 prove('servo_pulse_all32bit_inputs_high15_to12_zero',z3.Extract(15,12,pulse)!=bv(0,4),input_states=2**32)
 # Original fusion expressions have32bit arithmetic multiplication context.
 sf=sourcebind('control/imu_fusion.v',[
  'wire signed [31:0] recip_q;',
  'gravity_x <= (egx * recip_q) >>> 16;',
  'gravity_y <= (egy * recip_q) >>> 16;',
  'gravity_z <= (egz * recip_q) >>> 16;',
  'avx <= ((gyro_x - bias_x) * GYRO_MUL) >>> GYRO_SHIFT;',
  'ang_vel_x <= avx;'])
 source_bindings.append(sf)
 product=z3.BitVec('arbitrary_gravity_product32',32)
 gravity=z3.Extract(23,0,product>>16)
 prove('all_gravity_product32_high_bits15_to23_equal',z3.Or(*[bit(gravity,i)!=bit(gravity,23)for i in range(15,23)]),input_states=2**32)
 gyro,bias=z3.BitVecs('arbitrary_gyro16 arbitrary_bias16',16)
 angular=z3.Extract(23,0,((z3.SignExt(16,gyro)-z3.SignExt(16,bias))*bv(4466,32))>>10)
 prove('angular_all16bit_sensor_bias_high_bits20_to23_equal',z3.Or(*[bit(angular,i)!=bit(angular,23)for i in range(20,23)]),input_states=2**32)
 source_bindings.append(sourcebind('control/mem_policy.v',[
  'assign status_flags={active,stop_latched,external_fault,(policy_fault!=0),(sensor_age>=SENSOR_TIMEOUT_CLKS),bias_suspect,calibrating,sensor_ready};',
  '.last_executed_actions(executed_actions)',
  '.last_actions(executed_actions),.have_action(have_action)',
  'assign debug_wx=wx;assign debug_wy=wy;assign debug_wz=wz;assign debug_gx=gx;assign debug_gy=gy;assign debug_gz=gz;']))
 wrap=(PREP/'control/mem_policy_related25.v').read_text(encoding='utf8')
 packs={
 'BMI':['sample_ticks','gyro_z','gyro_y','gyro_x','accel_z','accel_y','accel_x'],
 'configuration':['enable','sensor_ready','sensor_error','external_fault','left_encoder_delta','right_encoder_delta','command_forward_q12','command_yaw_q12','command_height_q12'],
 'action':['target_left_mmps_25','target_right_mmps_25','servo_lh_us_25','servo_lk_us_25','servo_rh_us_25','servo_rk_us_25','executed_actions_25','completed_frames_25','inference_cycles_25'],
 'fusion':['debug_wx_25','debug_wy_25','debug_wz_25','debug_gx_25','debug_gy_25','debug_gz_25','accel_rejected_25'],
 'status':['ready_25','active_25','calibrating_25','bias_suspect_25','status_flags_25','policy_fault_25']}
 for name,fields in packs.items():assert norm('.src_data({'+','.join(fields)+'})')in norm(wrap),(name,'source pack')
 for literal_pack in[
  'assign {target_left_mmps,target_right_mmps,servo_lh_us,servo_lk_us,servo_rh_us,servo_rk_us,executed_actions,completed_frames,inference_cycles}=action_hold50;',
  'assign {debug_wx,debug_wy,debug_wz,debug_gx,debug_gy,debug_gz,accel_rejected_snapshot}=fusion_hold50;',
  'assign {ready_raw,active_raw,calibrating_raw,bias_suspect_raw,status_flags_raw,policy_fault_raw}=status_data50;']:
  assert norm(literal_pack)in norm(wrap)
 # Parse actual mapped cells and exact module connectivity. Primary names alone
 # are not accepted as evidence for a logical bit representative.
 vg=NEW/'impl/gwsynthesis/g2_top.vg';defs=definitions(vg);params=parameter_reader(defs)
 cells=read(vg,lambda n:n=='u_control'or n.startswith('u_control/'))
 directions=primitive_directions(LIB)
 _,logic,_=construct_shared_logic({},cells,params,params,directions)
 module_ports=module_bindings(defs,['u_control/model_thread','u_control/model_thread/model','u_control/model_thread/model/actuator_map','u_control/model_thread/model/fusion','u_control/model_thread/model/observation'])
 save(OUT/'actual_model_module_ports.json',module_ports)
 def ex(net):return logic.expr(net)
 def eq(label,a,b):prove(label,z3.Xor(a,b))
 # Explicit signed aliases at source and destination are classified from pack,
 # actual source D drivers, and numeric invariant. No suffix equality shortcut.
 action=channels['action'];src={x['logical_source_index']:x for x in action['source_payload']}
 aliases=[];retained_bindings=[]
 expected={}
 for b0,field,width in[(0,'inference_cycles_25',32),(32,'completed_frames_25',32),(208,'servo_rk_us_25',16),(224,'servo_rh_us_25',16),(240,'servo_lk_us_25',16),(256,'servo_lh_us_25',16),(272,'target_right_mmps_25',32),(304,'target_left_mmps_25',32)]:
  for j in range(width):expected[b0+j]=(field,j)
 for i,x in src.items():
  field,j=expected[i];net=x['cell']['ports']['D'][0]
  literal='u_control/model_thread/'+field+'['+str(j)+']'
  if net==literal:method='exact_source_pack_driver'
  else:
   m=re.fullmatch(r'u_control/model_thread/'+re.escape(field)+r'\[(\d+)\]',net)
   assert m and field.startswith('target_')and j>=23 and int(m[1])>=23,(i,net,literal)
   # Actual same model actuator output, with all high bits equal symbolically.
   assert net in sum(module_ports['u_control/model_thread/model/actuator_map']['ports'].values(),[])
   method='all24bit_action_signed_range_proof_high_bit_alias'
   aliases.append(dict(logical_source_bit=i,actual_source_D=net,original_bit=j,actual_output_bit=int(m[1]),method=method))
  retained_bindings.append(dict(logical_source_bit=i,field=field,field_bit=j,actual_source_D=net,method=method))
 for pair in action['destination_source_bit_connections']:
  si,di=pair['source_index'],pair['logical_destination_index']
  if si!=di:
   fs,bs=expected[si];fd,bd=expected[di]
   assert fs==fd and fs.startswith('target_')and min(bs,bd)>=23
   aliases.append(dict(logical_source_bit=si,logical_destination_bit=di,source_cell=pair['source'],destination_cell=pair['destination'],actual_source_D=src[si]['cell']['ports']['D'],actual_destination_D=pair['actual_destination']['ports']['D'],method='same_original_wheel_value_all_high23_to31_equal'))
 action_missing_source=sorted(set(range(336))-src.keys())
 unused=list(range(64,208));servo_missing=[b+j for b in(208,224,240,256)for j in range(12,16)]
 wheel_missing=[b+j for b in(272,304)for j in range(24,32)]
 assert action_missing_source==sorted(unused+servo_missing+wheel_missing)
 oldcore=(OLD/'control4500_core.v').read_text(encoding='utf8');newcore=(PREP/'control4500_core.v').read_text(encoding='utf8')
 for code in(oldcore,newcore):
  clean=re.sub(r'//[^\n]*|/\*.*?\*/','',code,flags=re.S)
  assert len(re.findall(r'\bmodel_executed_actions\b',clean))==2
  assert re.search(r'wire\s*\[143:0\]\s*model_executed_actions\s*;',clean)
  assert '.executed_actions(model_executed_actions)'in norm(clean)
 # This is a source-level unused outer output, while inner observation feedback
 # remains physically present, independently inspect actual primitive ports.
 last_action_ff=[c for n,c in cells.items()if n.startswith('u_control/model_thread/model/actuator_map/last_actions')and c['kind'].startswith('DFF')]
 assert len(last_action_ff)==96
 observation_ports=module_ports['u_control/model_thread/model/observation']['ports']
 feedback=[]
 for ff in last_action_ff:
  net=ff['ports']['Q'][0]
  pins=[p for p,nets in observation_ports.items()if net in nets]
  assert pins and all('executed_actions'in p for p in pins),(ff['name'],pins)
  feedback.append(dict(actual_action_register=ff,actual_observation_input_ports=pins))
 save(OUT/'actual_inner_executed_action_feedback96.json',dict(passed=True,outer_unused144_bits_do_not_remove_internal_feedback=True,feedback=feedback))
 action_result=dict(passed=True,logical_width=336,retained_source_DFF=len(src),retained_destination_DFF=len(action['destination_source_bit_connections']),source_word_fields=packs['action'],retained_driver_binding=retained_bindings,signed_alias_connections=aliases,source_missing_bits=action_missing_source,
  missing_executed_actions_outer_output_bits=unused,outer_executed_actions_unused_old_and_new=True,inner_last_actions_actual_FFs=len(last_action_ff),inner_observation_feedback_source_retained=True,
  missing_servo_bits_zero_proved=servo_missing,missing_wheel_bits_sign_alias_proved=wheel_missing,full_bit_identity_unproved=[])
 save(OUT/'actual_action_word_aliases.json',action_result)
 # Fusion logical fields: low bit accel; 24bit vectors gZ,gY,gX,wZ,wY,wX.
 fusion=channels['fusion'];fsrc={x['logical_source_index']:x for x in fusion['source_payload']}
 ffields=[(1,'debug_gz_25'),(25,'debug_gy_25'),(49,'debug_gx_25'),(73,'debug_wz_25'),(97,'debug_wy_25'),(121,'debug_wx_25')]
 fmissing=[]
 for b0,field in ffields:
  for j in range(24):
   i=b0+j
   if i in fsrc:
    actual_D=fsrc[i]['cell']['ports']['D'][0]
    expected_D='u_control/model_thread/'+field+f'[{j}]'
    if actual_D!=expected_D:
     assert field.startswith('debug_g') and actual_D=='u_control/model_thread/'+field[6:8]+f'[{j}]',(i,actual_D,expected_D)
     assert actual_D in sum(module_ports['u_control/model_thread/model/fusion']['ports'].values(),[])
   else:
    lo=15 if '_g'in field else 20
    assert lo<=j<23,(field,j);fmissing.append(dict(logical_bit=i,field=field,field_bit=j,sign_representative_bit=b0+23,method='arbitrary32bit_product_arithmetic_shift16_sign_extension'if lo==15 else'all16bit_gyro_bias_constant4466_arithmetic_shift10'))
 assert len(fmissing)==33 and 0 in fsrc
 assert set(range(145))-fsrc.keys()=={x['logical_bit']for x in fmissing}
 save(OUT/'actual_fusion_word_aliases.json',dict(passed=True,logical_width=145,retained_source_DFF=112,retained_destination_DFF=112,sign_aliases=fmissing,full_bit_identity_unproved=[]))
 # Configuration24 missing destination bits are actual yaw DSP AREG, not absent.
 config=channels['configuration'];srcconf={x['logical_source_index']:x for x in config['source_payload']}
 dc=next(c for n,c in cells.items()if n=='u_control/model_thread/model/observation/n1150_s')
 dp=params(dc);assert dc['kind']=='MULTALU27X18'
 lowA=list(reversed(dc['ports']['A']))
 for j in range(24):assert lowA[j]==srcconf[24+j]['cell']['ports']['Q'][0],(j,lowA[j])
 assert lowA[24:]==[lowA[23]]*3
 for k,v in {'AREG_CLK':'"CLK0"','AREG_CE':'"CE0"','AREG_RESET':'"RESET0"','MULT_RESET_MODE':'"ASYNC"','BREG_CLK':'"BYPASS"','DREG_CLK':'"BYPASS"','PREG_CLK':'"BYPASS"','OREG_CLK':'"BYPASS"'}.items():assert dp.get(k)==v,(k,dp.get(k),v)
 d0=config['destination_source_bit_connections'][0]['actual_destination']
 assert list(reversed(dc['ports']['CLK']))[0]==d0['ports']['CLK'][0]
 assert list(reversed(dc['ports']['CE']))[0]==d0['ports']['CE'][0]
 assert list(reversed(dc['ports']['RESET']))[0]==d0['ports']['CLEAR'][0]
 missing_config=sorted(set(range(140))-{x['logical_destination_index']for x in config['destination_source_bit_connections']})
 assert missing_config==list(range(24,48))
 config_result=dict(passed=True,missing_DFF_destination_bits=missing_config,actual_DSP=dc,parameters=dp,source_yaw_holding_to_actual_A_bit=[dict(logical_payload_bit=24+j,DSP_A_bit=j,actual_source_Q=lowA[j])for j in range(24)],actual_AREG_CLK_CE_RESET_same_as_proved_destination_commit=True,actual_AREG_asynchronous_reset=True,actual_A_sign_extension_preserved=True,claimed_DSP_whole_arithmetic_equivalence=False)
 save(OUT/'actual_configuration_DSP_AREG24.json',config_result)

 # Final50MHz consumer FFs are checked against original captured-word/stop/
 # reset behavior. This also records how packet signed aliases reach actual
 # outputs, instead of stopping the proof at a holding suffix.
 def packet_q(ch):return{x['logical_destination_index']:x['actual_destination']['ports']['Q'][0]for x in ch['destination_source_bit_connections']}
 aq,fq,sq=packet_q(action),packet_q(fusion),packet_q(channels['status'])
 reset=ex('u_control/n2375_7')
 fault=z3.Or(ex('u_control/clocked_fault'),ex('u_control/lock_fault_async'))
 external=ex(srcconf[136]['cell']['ports']['D'][0])
 enabled=ex(srcconf[139]['cell']['ports']['D'][0])
 stopped=z3.Or(z3.Not(ex(sq[14])),fault,external,z3.Not(enabled))
 action_commit=ex('u_control/model_thread/action_commit50')
 fusion_commit=ex('u_control/model_thread/fusion_commit50')
 consumers=[]
 def output_next(ff,want):
  assert ff['ports']['CLK']==['clk50_d']
  eq(ff['name']+'/actual_final_consumer_next_state',logic.next_state(ff),want)
 for name,ff in cells.items():
  m=re.fullmatch(r'u_control/model_thread/action_hold50_(\d+)_s\d+',name)
  if m:
   i=int(m[1]);assert i in aq,(name,'unexpected mapped consumer')
   data=ex(aq[i]);current=ex(ff['ports']['Q'][0])
   field,j=expected[i]
   if field.startswith('target_'):
    target='model_left_mmps'if 'left'in field else'model_right_mmps'
    match=re.fullmatch('u_control/'+target+r'\[(\d+)\]',ff['ports']['Q'][0]);assert match
    actualbit=int(match[1]);assert actualbit==j or min(actualbit,j)>=23
    want=z3.If(reset,z3.BoolVal(False),z3.If(stopped,z3.BoolVal(False),z3.If(action_commit,data,current)))
   else:
    if field.startswith('servo_'):
     output_name=field.replace('servo_','model_').replace('_25','')
     assert ff['ports']['Q']==['u_control/'+output_name+f'[{j}]']
     reset_value=z3.BoolVal(bool(1500>>j&1))
    else:reset_value=z3.BoolVal(False)
    want=z3.If(reset,reset_value,z3.If(z3.And(z3.Not(stopped),action_commit),data,current))
   output_next(ff,want)
   consumers.append(dict(kind='action',logical_packet_bit=i,source_field=field,field_bit=j,actual_consumer=ff,next_state_proved=True))
  m=re.fullmatch(r'u_control/model_thread/fusion_hold50_(\d+)_s\d+',name)
  if m:
   i=int(m[1]);assert i in fq
   current=ex(ff['ports']['Q'][0]);data=ex(fq[i])
   if i:
    b0,field=next((b,f)for b,f in ffields if b<=i<b+24)
    assert ff['ports']['Q']==['u_control/'+field.replace('_25','')+f'[{i-b0}]'],(name,ff['ports']['Q'])
   else:assert ff['ports']['Q']==['u_control/fusion_hold50[0]']
   initial=bool(i in range(13,25)) # reset GZ=-4096, all other vectors0.
   want=z3.If(reset,z3.BoolVal(initial),z3.If(z3.And(fusion_commit,z3.Not(fault)),data,current))
   output_next(ff,want)
   consumers.append(dict(kind='fusion',logical_packet_bit=i,actual_consumer=ff,next_state_proved=True))
 assert sum(x['kind']=='action'for x in consumers)==160
 assert sum(x['kind']=='fusion'for x in consumers)==112 # includes actual scalar accel-rejected payload FF.
 # The accel-reject scalar is kept in the same word; valid is separately
 # registered and still masked by the original domain-fault condition.
 assert cells['u_control/model_thread/fusion_pulse_s0']['ports']['Q']==['u_control/debug_fusion_valid']
 output_next(cells['u_control/model_thread/fusion_pulse_s0'],z3.If(reset,z3.BoolVal(False),z3.And(fusion_commit,z3.Not(fault))))
 accel_lut=cells['u_control/telemetry_thread/n2158_s0']
 assert accel_lut['kind']=='LUT4'and params(accel_lut)=={'INIT':"16'hF088"}
 assert accel_lut['ports']['I0']==['u_control/debug_fusion_valid']and accel_lut['ports']['I1']==['u_control/fusion_hold50[0]']
 eq('fusion_accel_reject_complete_word_bit0_and_pulse_folded_into_actual_telemetry_mux',ex(accel_lut['ports']['F'][0]),z3.If(ex(accel_lut['ports']['I3'][0]),ex(accel_lut['ports']['I2'][0]),z3.And(ex(accel_lut['ports']['I0'][0]),ex(accel_lut['ports']['I1'][0]))))
 clock_roles={}
 for name,ff in cells.items():
  if ff['kind'].startswith('DFF'):
   clock=ff['ports']['CLK'][0]
   if name.startswith('u_control/model_thread/model/'):
    assert clock=='clk_model25',(name,clock)
    role='full_model_internal25'
   elif clock=='clk_model25':
    assert (name.startswith('u_control/model_thread/')or re.fullmatch(r'u_control/model_clock_guard/(?:hb25|release25_\d+)_s\d+',name)),(name,clock)
    role='related25_mailbox_or_guard'
   else:
    assert clock=='clk50_d',(name,clock);role='control_io_guard50'
   clock_roles[role]=clock_roles.get(role,0)+1
 assert clock_roles['full_model_internal25']==12639
 save(OUT/'actual_final_consumers_and_clock_roles.json',dict(passed=True,source_pack_and_final_assign_literals_bound=True,all160_action_final_consumer_transitions_proved=True,all_retained_fusion_consumer_transitions_proved=True,consumer_count=len(consumers),consumers=consumers,model_full_DFF_clock_roles=clock_roles,read_only_actual_control_FF_clock_count=sum(clock_roles.values()),physical_clock_tree_timing_not_signed_off=True))
 # Latest accept logic, full retained word mismatch, not inferred send alias.
 latest=[]
 for label in('configuration','status'):
  ch=channels[label];data=ch['source_payload'];ctrl=ch['actual_control_registers'];parent=ch['scope'].rsplit('/',1)[0]
  ready=ex(ctrl['request']['ports']['Q'][0])==ex(ctrl['ack2']['ports']['Q'][0])
  sent_rows=[x for n,x in cells.items()if re.fullmatch(re.escape(parent)+r'/sent_s\d+',n)]
  if sent_rows:assert len(sent_rows)==1;sent=sent_rows[0]
  else:
   assert label=='configuration';sent=srcconf[139]['cell'];assert sent['ports']['D']==['VCC']
   assert sent['ports']['CLEAR']==data[0]['cell']['ports']['CLEAR']and sent['ports']['CE']==data[0]['cell']['ports']['CE']
  changed=z3.Or(*[z3.Xor(ex(x['cell']['ports']['D'][0]),ex(x['cell']['ports']['Q'][0]))for x in data])
  eq(label+'/latest_full_retained_word_change_or_unsent_and_ready',ex(ch['actual_accept_net'][0]),z3.And(ready,z3.Or(z3.Not(ex(sent['ports']['Q'][0])),changed)))
  latest.append(dict(name=label,retained_word_bits=len(data),actual_sent=sent,complete_retained_word_change_detection_proved=True,missing_status_bits_are_same_payload_duplicates=label=='status'))
 # Status aliases are exact repeats in the source pack, not numeric assumptions.
 status_aliases=[dict(missing=10,retained=13,expression='active'),dict(missing=5,retained=11,expression='bias_suspect'),dict(missing=4,retained=12,expression='calibrating')]
 for row in status_aliases:
  # Packet positions are fixed by the literal source status tuple above.
  assert row['missing']not in{x['logical_source_index']for x in channels['status']['source_payload']}
  assert row['retained']in{x['logical_source_index']for x in channels['status']['source_payload']}
 save(OUT/'actual_status_latest_word_proof.json',dict(passed=True,status_aliases=status_aliases,latest_send_proofs=latest,checks=checks))
 save(OUT/'actual_payload_math_checks.json',dict(passed=True,checks=checks,source_bindings=source_bindings,all_wheel24_sensor16_and_product32_inputs_symbolic=True,full_end_to_end_sequential_equivalence_claimed=False))
 final=dict(passed=True,recorded_at=datetime.datetime.now().astimezone().isoformat(),structural_review=str(B/'final_review.json'),supplementary_reviews=[str(p)for p in OUT.glob('*.json')if p.name not in('failed.json','final_review.json')],actual_configuration_missing24_are_DSP_AREG=True,actual_action_two_cross_index_signed_aliases_proved=True,action_144_unused_outer_observer_bits_preserve_inner_feedback=True,fusion33_sign_alias_bits_proved=True,status3_duplicate_bits_proved=True,latest_complete_retained_word_change_detection_proved=True,unproved_payload_alias_bits=[],whole_model_sequential_equivalence_proved=False,full4500_numerical_125Hz_and_vehicle_stop_accepted=False,physical_setup_hold_bundle_timing_reset_pulse_IO_accepted=False,no_build_or_frozen_input_modified=True)
 save(OUT/'final_review.json',final)
 print(json.dumps(final,ensure_ascii=False))
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--base-review',type=Path,default=B);ap.add_argument('--out',type=Path,required=True);args=ap.parse_args()
 B=args.base_review;OUT=args.out;OUT.mkdir(exist_ok=False)
 try:run()
 except BaseException:
  save(OUT/'failed.json',dict(passed=False,error=traceback.format_exc()));raise

