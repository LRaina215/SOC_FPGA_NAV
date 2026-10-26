"""Inspect actual SYN39 ring address, state and word cones without EDA mutation.

Mapped LUT/ALU/FF functions use the public Gowin macro semantics. The three new
MULT12X12 address operators are modeled explicitly; distributed RAM contents
remain arbitrary. This is a mapped partition proof, not routed timing signoff.
"""
from pathlib import Path
import argparse, json, re, sys, traceback, datetime
sys.path.insert(0, str(Path(__file__).parent))
from review_model25_joint_syn import (BASE, LIB, load, save, meta, same,
    definitions, parameter_reader, read, primitive_directions, construct_shared_logic)
from prove_soc_console_remap import z3, literal

OLD=BASE/'baseline_fusion_model25_bounded_MAX_syn37'
NEW=BASE/'baseline_fusion_model25_observation_ring_syn39'
PREP=BASE/'baseline_fusion_model25_observation_ring_prepared2'
OBS='u_control/model_thread/model/observation'
POL='u_control/model_thread/model/policy'
OUT=BASE/'ring_SYN39_actual_semantics_reproduction'

def run(out):
 out.mkdir(parents=True,exist_ok=True); checks=[];debug={}
 def prove(label,actual,expected,condition=None):
  delta=actual!=expected
  if condition is not None:delta=z3.And(condition,delta)
  s=z3.Solver();s.set(timeout=20000);s.add(z3.simplify(delta));r=s.check()
  if r!=z3.unsat:
   if r==z3.sat:
    m=s.model();save(out/'counterexample.json',dict(label=label,actual=str(m.eval(actual,model_completion=True)),expected=str(m.eval(expected,model_completion=True)),observations={k:str(m.eval(v,model_completion=True))for k,v in debug.items()},model=str(m)))
   raise AssertionError((label,str(r),str(s.model())[:1800]if r==z3.sat else s.reason_unknown()))
  checks.append(dict(label=label,passed=True,method='symbolic_unsatisfiable_miter'))
 def bv(n,w):return z3.BitVecVal(n,w)
 def boolbit(v,b):return z3.Extract(b,b,v)==bv(1,1)
 def vector(bits):return z3.Concat(*[z3.If(b,bv(1,1),bv(0,1))for b in bits])if len(bits)>1 else z3.If(bits[0],bv(1,1),bv(0,1))
 anc={'','u_control','u_control/model_thread','u_control/model_thread/model',OBS,POL,POL+'/core'}
 keep=lambda n:n in anc or n.rpartition('/')[0]in anc or n.startswith(OBS+'/')
 a=read(OLD/'impl/gwsynthesis/g2_top.vg',keep);b=read(NEW/'impl/gwsynthesis/g2_top.vg',keep)
 pa=parameter_reader(definitions(OLD/'impl/gwsynthesis/g2_top.vg'));pb=parameter_reader(definitions(NEW/'impl/gwsynthesis/g2_top.vg'))
 la,lb,boundaries=construct_shared_logic(a,b,pa,pb,primitive_directions(LIB))
 ff={c['ports']['Q'][0]:c for c in b.values()if c['kind'].startswith('DFF')}
 def ex(n):return lb.expr(n)
 def vec(n,width):return vector([ex(n+'['+str(i)+']')for i in range(width-1,-1,-1)])
 def pvec(c,p):return vector([ex(n)for n in c['ports'][p]])
 def regs(prefix,width):
  result={}
  for n,c in b.items():
   m=re.fullmatch(re.escape(prefix)+r'_(\d+)_s\d+',n)
   if m:result[int(m[1])]=c
  assert set(result)==set(range(width)),(prefix,sorted(result))
  return [result[i]for i in range(width-1,-1,-1)]
 def qvec(cs):return vector([ex(c['ports']['Q'][0])for c in cs])
 def nxt(cs):return vector([lb.next_state(c)for c in cs])
 ost=vec(OBS+'/state',3);pst=vec(POL+'/state',3);lane=vec(OBS+'/lane',5)
 work=vec(OBS+'/g_ring.work_slot',3);reset=ex(OBS+'/n331_5');preset=ex('u_control/model_thread/n1113_6')
 init=ex(OBS+'/history_initialized');commit=ost==bv(4,3)
 invariant=z3.And(z3.ULE(work,bv(5,3)),z3.ULE(lane,bv(27,5)))
 # The work-address DSP duplicates the public work-slot transition internally.
 # Its input register and the explicit three FFs have the same async reset,
 # CE/update function and initial zero, so equality is an inductive relation.
 dsps={n:c for n,c in b.items()if c['kind']=='MULT12X12'and n.startswith(OBS+'/')}
 assert set(dsps)=={OBS+'/'+x for x in ('n22065_s','n22100_s','n22165_s')}
 dsp_rows=[]
 for n,c in dsps.items():
  p=pb(c);assert all(p[k]=='"BYPASS"'for k in ('BREG_CLK','PREG_CLK','OREG_CLK'))
  prove(n+' B=28',pvec(c,'B'),bv(28,12))
  if p['AREG_CLK']=='"CLK0"':
   assert n.endswith('/n22065_s') and p['MULT_RESET_MODE']=='"ASYNC"'
   assert c['ports']['CLK']==['GND','clk_model25']
   prove(n+' reset matches observer',ex(c['ports']['RESET'][1]),reset)
   prove(n+' CE exactly COMMIT',ex(c['ports']['CE'][1]),commit)
   expected=z3.If(work==bv(5,3),bv(0,3),work+bv(1,3))
   prove(n+' A next slot including all valid slots',pvec(c,'A'),z3.ZeroExt(9,expected),invariant)
   cs=regs(OBS+'/g_ring.work_slot',3)
   prove('explicit work slot complete next-state',nxt(cs),z3.If(reset,bv(0,3),z3.If(commit,expected,work)),invariant)
   # Apply only the just-proved invariant for the current registered operand.
   operand=z3.ZeroExt(9,work)
   kind='AREG work-slot duplicate; induction from common async reset/zero and matching transition'
  else:
   assert p['AREG_CLK']=='"BYPASS"'
   operand=pvec(c,'A');kind='fully combinational signed12 multiplication'
  product=z3.SignExt(12,operand)*z3.SignExt(12,pvec(c,'B'))
  for i,net in enumerate(c['ports']['DOUT']):
   lb.opaque_symbols[(n,'DOUT',i)]=boolbit(product,23-i)
   lb.memo.pop(net,None)
  dsp_rows.append(dict(instance=n,ports=c['ports'],parameters=p,model=kind))
 # Metadata: three work, fifteen live and fifteen frozen bits; initialized
 # and all published-valid controls retain their actual reset roots.
 live=[vec(OBS+'/g_ring.live'+str(i),3)for i in range(5)]
 frozen=[vec(OBS+'/g_ring.frozen'+str(i),3)for i in range(5)]
 tick=vec(POL+'/tick_count',32);fault=vec('u_control/model_thread/policy_fault_25',3)
 runok=z3.And(ex('u_control/model_thread/model/fault_0_8'),ex('u_control/model_thread/config_data[139]'),ex('u_control/model_thread/model/fusion_seen'),fault==bv(0,3))
 deadline=z3.And(tick==bv(199999,32),pst!=bv(0,3))
 framevalid=ex('u_control/model_thread/model/frame_valid')
 freeze=z3.And(pst==bv(1,3),framevalid,runok,z3.Not(deadline))
 for i in range(5):
  cs=regs(OBS+'/g_ring.live'+str(i),3)
  dest=work if i==4 else z3.If(init,live[i+1],work)
  prove('live'+str(i)+' complete next-state',nxt(cs),z3.If(reset,bv(0,3),z3.If(commit,dest,live[i])))
  cs=regs(OBS+'/g_ring.frozen'+str(i),3)
  prove('frozen'+str(i)+' complete next-state',nxt(cs),z3.If(reset,bv(0,3),z3.If(z3.And(freeze,init),live[i],frozen[i])))
 for c in [x for x in b.values()if x['kind'].startswith('DFF')and('g_ring.'in x['name']or'/snapshot_word_'in x['name'])]:
  assert c['ports']['CLK']==['clk_model25']
  if 'g_ring.'in c['name']:prove(c['name']+' reset root',lb.one(c,'CLEAR'),reset)
  else:prove(c['name']+' reset root',lb.one(c,'CLEAR'),preset)
 # Every bank has six four-bit cells and shares its true word address.
 rams={}
 for n,c in b.items():
  if c['kind']!='RAM16SDP4' or 'g_ring.frame_mem_'not in n:continue
  m=re.fullmatch(re.escape(OBS)+r'/g_ring.frame_mem_g_ring.frame_mem_(\d+)_(\d+)_s',n);assert m,n
  bank,group=map(int,m.groups());rams[bank,group]=c
 assert set(rams)=={(bank,group)for bank in range(11)for group in range(6)}
 address=vec(OBS+'/g_ring.write_address',8)
 prove('full write address work*28+lane',address,z3.ZeroExt(5,work)*bv(28,8)+z3.ZeroExt(3,lane),invariant)
 # Direct lanes have denominators 1. The unchanged source chooses the other
 # ten lanes for restoring division; only PREPARE direct or SAVE can write.
 direct=z3.And(*[lane!=bv(i,5)for i in (0,1,2,7,10,11,13,14,18,21)])
 writing=z3.And(z3.Not(reset),z3.Or(z3.And(ost==bv(1,3),direct),ost==bv(3,3)))
 for (bank,group),c in rams.items():
  assert c['ports']['CLK']==['clk_model25']
  prove(c['name']+' low write address',pvec(c,'WAD'),z3.Extract(3,0,address))
  prove(c['name']+' bank write enable',ex(c['ports']['WRE'][0]),z3.And(writing,z3.Extract(7,4,address)==bv(bank,4)),invariant)
  prove(c['name']+' shared low read address',pvec(c,'RAD'),z3.Extract(3,0,vec(OBS+'/g_ring.read_address',8)))
 # Build the actual word permutation by its relation to unchanged legacy
 # normalizer writes. This is a mapped partition proof at corresponding
 # actual arithmetic registers/DSP outputs, not an assertion about net names.
 oldwords={}
 for n,c in a.items():
  m=re.fullmatch(re.escape(OBS)+r'/words_words_RAMREG_(\d+)_G\[(\d+)\]_s\d+',n)
  if m and c['kind'].startswith('DFF'):oldwords[int(m[1]),int(m[2])]=c
 assert len(oldwords)==672,len(oldwords)
 permutation=[]
 for group in range(6):
  c=rams[0,group]
  for pos,n in enumerate(c['ports']['DI']):
   bitno=group*4+3-pos
   # Tool groups follow little-endian four-bit slices. Verify all 28 lanes,
   # rather than relying on the generated group suffix as semantic evidence.
   for ln in range(28):
    old=oldwords[ln,bitno]
    prove('RAM DI group%d pos%d lane%d'%(group,pos,ln),ex(n),la.one(old,'D'),z3.And(writing,lane==bv(ln,5)))
   permutation.append(dict(group=group,pin_position_MSB_first=pos,logical_word_bit=bitno,actual_DI=n))
  for bank in range(1,11):
   for pos,n in enumerate(rams[bank,group]['ports']['DI']):prove('bank%d group%d DI%d same word'%(bank,group,pos),ex(n),ex(c['ports']['DI'][pos]))
 # Reading remains truly asynchronous at the RAM boundary. Give each bank
 # arbitrary contents; no INIT or first-write value is assumed here.
 memory=z3.Array('arbitrary_ring_word_memory',z3.BitVecSort(8),z3.BitVecSort(24))
 for (bank,group),c in rams.items():
  ra=z3.Concat(bv(bank,4),pvec(c,'RAD'));word=z3.Select(memory,ra)
  for pos,net in enumerate(c['ports']['DO']):
   lb.opaque_symbols[(c['name'],'DO',pos)]=boolbit(word,group*4+3-pos);lb.memo.pop(net,None)
 # Clear dependent caches after replacing opaque RAM outputs. State/constant
 # expressions retain exactly the same interpretation.
 lb.memo={'GND':z3.BoolVal(False),'VCC':z3.BoolVal(True)}
 index=vec('u_control/model_thread/model/observation_read_index',8)
 segment=z3.If(z3.ULT(index,bv(28,8)),bv(0,3),z3.If(z3.ULT(index,bv(56,8)),bv(1,3),z3.If(z3.ULT(index,bv(84,8)),bv(2,3),z3.If(z3.ULT(index,bv(112,8)),bv(3,3),z3.If(z3.ULT(index,bv(140,8)),bv(4,3),bv(5,3))))))
 prove('actual segment decode',vec(OBS+'/g_ring.segment',3),segment)
 rr=[z3.If(freeze,live[i],frozen[i])for i in range(5)]
 slot=z3.If(segment==bv(0,3),rr[4],z3.If(segment==bv(1,3),rr[0],z3.If(segment==bv(2,3),rr[1],z3.If(segment==bv(3,3),rr[2],z3.If(segment==bv(4,3),rr[3],rr[4])))))
 read_active=z3.And(runok,z3.Not(deadline),z3.Or(z3.And(pst==bv(1,3),framevalid),pst==bv(2,3)))
 prove('read slot selects live on freeze otherwise frozen at active read',vec(OBS+'/g_ring.read_slot',3),slot,read_active)
 ra=z3.ZeroExt(5,slot)*bv(28,8)+index-z3.ZeroExt(5,segment)*bv(28,8)
 prove('complete read address at active read',vec(OBS+'/g_ring.read_address',8),ra,read_active)
 raw=z3.Select(memory,ra);clip=z3.If(raw>bv(409600,24),bv(409600,24),z3.If(raw<bv(-409600,24),bv(-409600,24),raw))
 result=z3.If(z3.UGE(index,bv(168,8)),bv(0,24),z3.If(segment==bv(0,3),clip,raw))
 valid_slots=z3.And(*[z3.ULE(x,bv(5,3))for x in live+frozen])
 prove('all24 output bits arbitrary RAM values/current exact clip/history raw',vec('u_control/model_thread/model/observation_read_data',24),result,z3.And(valid_slots,read_active))
 # Verify the actual private prefetch FF, including abort/enable/deadline CE.
 snap=regs(POL+'/snapshot_word',24);ld=regs(POL+'/load_index',8);load=qvec(ld)
 capture=z3.And(runok,z3.Not(deadline),z3.Or(z3.And(pst==bv(1,3),framevalid),z3.And(pst==bv(2,3),load!=bv(167,8))))
 debug.update(policy_state=pst,load=load,tick=tick,run_ok=runok,fault=fault,reset=preset,capture=capture,frame_valid=framevalid,deadline=deadline,frame_freeze=freeze,ce=lb.one(snap[0],'CE'),actual_D=pvec(snap[0],'D'),actual_Q=pvec(snap[0],'Q'))
 prove('optimized snapshot select agrees on permitted consumption',ex('u_control/model_thread/model/snapshot_word_23_16'),freeze,read_active)
 for c in snap:
  bitno=int(re.search(r'/snapshot_word_(\d+)_s',c['name'])[1])
  prove(c['name']+' data exact output bit',lb.one(c,'D'),boolbit(vec('u_control/model_thread/model/observation_read_data',24),bitno))
  # The mapper may suppress CE while async CLEAR dominates. Require the
  # complete next-state below for every reset value; CE identity only matters
  # outside the asserted reset interval.
  prove(c['name']+' prefetch CE outside dominating async clear',lb.one(c,'CE'),capture,z3.Not(preset))
 prove('prefetch24 complete next state',nxt(snap),z3.If(preset,bv(0,24),z3.If(capture,result,qvec(snap))),valid_slots)
 prove('prefetch index first0 otherwise load+1',index,z3.If(pst==bv(1,3),bv(0,8),load+bv(1,8)))
 save(out/'actual_address_DSP_modes.json',dict(passed=True,library=str(LIB),public_macro='MULT12X12 lines6723-7028',DSPs=dsp_rows))
 save(out/'actual_RAM_word_permutation.json',dict(passed=True,cells=66,logical_words=168,physical_words=176,word_width=24,permutation=permutation,arbitrary_initial_contents=True))
 save(out/'proof_checks.json',dict(passed=True,count=len(checks),boundaries=boundaries,checks=checks))
 result=dict(passed=True,recorded_at=datetime.datetime.now().astimezone().isoformat(),actual_VG=meta(NEW/'impl/gwsynthesis/g2_top.vg'),checks=len(checks),RAM16SDP4=66,new_address_MULT12X12=3,source_reset_slot_induction=True,all_RAM_words_arbitrary_initial_values=True,limits=['mapped partition proof with corresponding retained arithmetic FF/DSP boundary','physical place/routing and setup/hold not evaluated','distributed RAM electrical/hold behavior not measured on board'],automatic_PNR=False)
 save(out/'final_review.json',result);print(json.dumps(result,ensure_ascii=False))
 return result

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=OUT);args=p.parse_args()
 try:run(args.out)
 except Exception:
  args.out.mkdir(parents=True,exist_ok=True);save(args.out/'final_review.json',dict(passed=False,error=traceback.format_exc(),actual_input_unchanged=True));raise
