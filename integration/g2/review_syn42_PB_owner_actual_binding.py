"""Local SYN42 PB port/next-state binding, with explicit caller reachability limits."""
from pathlib import Path
from datetime import datetime
from collections import Counter
import argparse,json,re,traceback
from prepare_baseline_fusion_retime import load,save,metadata
from synthesis_connectivity import read,MODULE
from review_stage3_slf128_memories import Tree,LIBRARY
from audit_routing_topology import primitive_directions
from prove_soc_console_remap import Logic,literal,z3

ROOT=Path('F:/nav_integration_g2_20261023')
LOCAL=ROOT/'SLF128_storage_safeFW_syn42_local_memory_mapping1'
BUILD=ROOT/'SLF128_storage_safeFW_syn42'
NAMES=('nav_subsystem','slf_match','slf_extract','xt_frame_rx','nav_ram_sdp_18','nav_fifo_ob_2','pc_project')

def review(out):
 out=out.resolve();assert not out.exists();out.mkdir(parents=True)
 result=dict(passed=False,scope='Local actual ports and one-step transition facts; not an unrestricted reachable-state safety proof',SYN_started=False,RTL_started=False,PNR_started=False)
 try:
  sources=[LOCAL/(n+'.v')for n in NAMES];text='\n'.join(p.read_text(encoding='utf8')for p in sources)
  text,count=re.subn(r'\bmodule\s+nav_subsystem\s*\(', 'module g2_top(',text,count=1);assert count==1
  view=out/'local_actual_parser_view.v';view.write_text(text,encoding='utf8')
  tree=Tree(view);allcells=read(view);directions=primitive_directions(LIBRARY)
  # Omitted custom modules are explicit free boundaries. Their data/control
  # outputs are never guessed from names or treated as an already-proved stream.
  omitted={n:c for n,c in allcells.items()if c['kind']not in directions}
  cells={n:c for n,c in allcells.items()if c['kind']in directions}
  qcells={q:c for c in cells.values()if c['kind'].startswith('DFF')for q in c['ports']['Q']}
  qs={q:z3.Bool('actual_FF:'+q)for q in qcells}
  supported=lambda k:bool(re.fullmatch(r'LUT[1-6]',k))or k.startswith('MUX2_LUT')or k in('ALU','INV','IBUF','BUFG','VCC','GND')or k.startswith('DFF')
  opaque={(n,p,i):z3.Bool(f'actual_opaque:{n}:{p}:{i}')for n,c in cells.items()if not supported(c['kind'])for p,ns in c['ports'].items()if directions[c['kind']].get(p)=='output'for i in range(len(ns))}
  logic=Logic(cells,lambda c:tree.params[c['name']],directions,qs,opaque);E=logic.expr
  queries=[]
  def query(name,formula,expect=None):
   s=z3.Solver();s.set(timeout=10000);s.add(formula);status=str(s.check());row=dict(name=name,result=status,scope='Arbitrary current FF state and explicit free boundaries')
   if status=='sat':
    m=s.model();row['witness']={str(v):str(m.eval(v,model_completion=True))for v in z3.z3util.get_vars(formula)}
   queries.append(row)
   if expect and status!=expect:
    save(out/'partial_queries.json',queries)
    raise AssertionError((name,status))
   return row
  def same(name,a,b):query(name,z3.Xor(a,b),'unsat')
  def nextq(q):assert q in qcells,('actual FF Q not found',q);return logic.next_state(qcells[q])
  rst=E('r2');fuse=E('r_LIDAR_CTRL_Z[6]');hold=E('r_LIDAR_CTRL_Z[5]');enabled=E('r_MATCH_CTRL_Z[0]')
  point=E('point_owned');pending=E('points_pending');exbusy=E('ex_busy');we=E('pb_we_Z');ready=E('mt_ready')
  raw=E('u_xt/raw_frame_accept');begin=z3.And(raw,z3.Not(fuse));frame_en=E('mt_frame_en')
  sf=z3.And(E('pt_valid'),E('pt_ready'),z3.Not(fuse));sof=z3.And(sf,E('pt_sof'));extract=E('u_match/extract_en');sof_extract=z3.And(sof,extract)
  same('actual n8165_6 binds accepted non-FUSE raw frame_begin',E('n8165_6'),begin)
  same('latched extraction enable priority',extract,z3.If(E('u_match/extract_armed'),E('u_match/accepted_extract_en'),frame_en))
  producer=z3.Or(pending,exbusy,we,z3.And(begin,frame_en),sof_extract)
  source=(BUILD/'nav/rtl/match/slf_match.v').read_text(encoding='utf8')
  states={k:int(v)for k,v in re.findall(r'\b(S_[A-Z0-9_]+)\s*=\s*(\d+)',source)}
  def state(n,next_=False):return z3.And(*[(nextq(f'u_match/st[{i}]')if next_ else E(f'u_match/st[{i}]'))==z3.BoolVal(bool(n>>i&1))for i in range(5)])
  idle=state(states['S_IDLE']);wait=state(states['S_WAIT_POINTS'])
  cmd_eval=E('n12374_4');cmd_update=z3.And(E('n12374_7'),E('wd_q[1]'));cmd_clear=E('raw_frame_accept_5')
  same('raw_frame_accept_5 is mapped CLEAR command, not raw begin',cmd_clear,z3.And(E('n12374_7'),E('wd_q[2]')))
  lf_busy=E('lf_busy');pwbranch=z3.And(E('u_match/pw_pend'),z3.Not(lf_busy));prior=z3.And(cmd_update,E('r_MATCH_CTRL_Z[1]'))
  command=z3.And(idle,z3.Not(cmd_clear),z3.Not(pwbranch),z3.Not(prior),z3.Or(cmd_eval,cmd_update))
  admit=z3.And(idle,z3.Not(lf_busy),z3.Not(pending),z3.Not(exbusy),z3.Not(we),z3.Not(point),z3.Not(cmd_eval),z3.Not(cmd_update),z3.Not(cmd_clear))
  expected_raw=z3.And(E('pose_wait'),z3.Or(fuse,admit),z3.Or(z3.Not(E('r_LIDAR_CTRL_Z[3]')),E('p_LIDAR_POSE_COMMIT_Z'),E('u_xt/pose_pending')))
  same('actual RAW accept = waiting && (FUSE || ownership admit) && pose request',raw,expected_raw)
  query('non-FUSE raw begin cannot coexist with owner pending extraction final write or commands',z3.And(begin,z3.Or(point,pending,exbusy,we,lf_busy,z3.Not(idle),cmd_eval,cmd_update,cmd_clear)),'unsat')
  same('points_pending complete priority including same-edge raw reservation',nextq('points_pending'),z3.If(rst,z3.BoolVal(False),z3.If(z3.Or(sof_extract,z3.And(begin,frame_en)),z3.BoolVal(True),z3.If(E('u_match/ex_done'),z3.BoolVal(False),pending))))
  same('extract_armed accepted raw/SOF priority',nextq('u_match/extract_armed'),z3.If(rst,z3.BoolVal(False),z3.If(sof,z3.BoolVal(False),z3.If(begin,z3.BoolVal(True),E('u_match/extract_armed')))))
  same('accepted extraction enable capture',nextq('u_match/accepted_extract_en'),z3.If(rst,z3.BoolVal(False),z3.If(begin,frame_en,E('u_match/accepted_extract_en'))))
  same('HOLD history capture',nextq('u_match/hold_prev'),z3.If(rst,z3.BoolVal(False),hold))
  owner=z3.If(z3.Or(z3.Not(enabled),z3.And(E('u_match/hold_prev'),z3.Not(hold))),z3.BoolVal(False),point)
  owner=z3.If(z3.And(begin,frame_en),hold,owner)
  owner=z3.If(z3.And(idle,cmd_clear),z3.BoolVal(False),owner)
  owner=z3.If(z3.And(idle,z3.Not(cmd_clear),z3.Not(pwbranch),prior,z3.Not(producer)),z3.BoolVal(False),owner)
  owner=z3.If(command,z3.BoolVal(True),owner)
  owner=z3.If(z3.And(wait,z3.Not(producer),E('u_match/wait_release')),z3.BoolVal(False),owner)
  owner=z3.If(z3.And(state(states['S_LFW']),E('u_match/lf_done')),z3.BoolVal(False),owner)
  same('point_owned complete source-priority next state',nextq('point_owned'),z3.If(rst,z3.BoolVal(False),owner))
  query('accepted raw or extracted SOF clears public ready on that edge',z3.And(z3.Or(begin,sof_extract),nextq('mt_ready')),'unsat')
  query('early accepted command waits and does not start a PB read',z3.And(z3.Not(rst),command,producer,z3.Or(z3.Not(state(states['S_WAIT_POINTS'],True)),nextq('u_match/ac_re'))),'unsat')
  query('WAIT_POINTS retains wait and no PB read through final write',z3.And(z3.Not(rst),wait,producer,z3.Or(z3.Not(state(states['S_WAIT_POINTS'],True)),nextq('u_match/ac_re'))),'unsat')
  end=z3.And(E('u_xt/rd_act'),E('x_valid'),E('x_ready'),E('x_eof'))
  same('RAW rd_act complete accept/end/reset priority',nextq('u_xt/rd_act'),z3.If(z3.Or(rst,end),z3.BoolVal(False),z3.If(raw,z3.BoolVal(True),E('u_xt/rd_act'))))
  same('RAW pending request retained until actual accept',nextq('u_xt/pose_pending'),z3.If(z3.Or(rst,raw),z3.BoolVal(False),z3.If(E('p_LIDAR_POSE_COMMIT_Z'),z3.BoolVal(True),E('u_xt/pose_pending'))))
  rows=[c for c in cells.values()if c['kind']=='SDPX9B'and c['name'].startswith('u_match/u_pb/')];assert len(rows)==8
  details=[];collisions=[]
  pb_re=E('u_match/ac_re')
  for c in rows:
   p=tree.params[c['name']];assert literal(p['BIT_WIDTH_0'])==literal(p['BIT_WIDTH_1'])==18 and literal(p['READ_MODE'])==0
   bank=int(re.search(r'g_point_bank\[(\d)\]',c['name'])[1]);expected_w=z3.And(we,*[E(f'u_match/pb_wa[{10+i}]')==z3.BoolVal(bool(bank>>i&1))for i in range(2)])
   expected_r=z3.And(pb_re,*[E(f'u_match/pb_ra[{10+i}]')==z3.BoolVal(bool(bank>>i&1))for i in range(2)])
   active=lambda pin,select,ce:z3.And(E(c['ports'][ce][0]),*[E(n)==z3.BoolVal(bool(literal(p[select])>>(2-i)&1))for i,n in enumerate(c['ports'][pin])])
   actual_w=active('BLKSELA','BLK_SEL_0','CEA');actual_r=active('BLKSELB','BLK_SEL_1','CEB')
   same(c['name']+' physical write gate',actual_w,expected_w);same(c['name']+' physical read gate',actual_r,expected_r)
   assert c['ports']['ADA']==[f'u_match/pb_wa[{i}]'for i in range(9,-1,-1)]+['GND','GND','VCC','VCC']
   assert c['ports']['ADB']==[f'u_match/pb_ra[{i}]'for i in range(9,-1,-1)]+['GND']*4
   equal=z3.And(*[E(a)==E(b)for a,b in zip(c['ports']['ADA'][:10],c['ports']['ADB'][:10])])
   collision=z3.And(actual_w,actual_r,equal);collisions.append(collision)
   details.append(dict(cell=c,parameters=p,bank=bank,actual_write=str(z3.simplify(actual_w)),actual_read=str(z3.simplify(actual_r))))
  free=query('arbitrary FF/free upstream SOF collision is an obligation, not a reachable trace',z3.Or(*collisions))
  keys=['point_owned','points_pending','ex_busy','pb_we_Z','mt_ready','u_match/extract_armed','u_match/accepted_extract_en','u_match/hold_prev','u_match/wait_release','u_match/ac_re','u_xt/rd_act','u_xt/pose_pending','pose_wait']+[f'u_match/st[{i}]'for i in range(5)]
  keyrows=[dict(net=q,cell=qcells[q],parameters=tree.params[qcells[q]['name']],next_state=str(nextq(q)))for q in keys if q in qcells]
  source_paths=[BUILD/'nav/rtl'/p for p in('match/slf_match.v','match/slf_extract.v','lidar/xt_frame_rx.v','top/nav_subsystem.v','common/nav_ram_sdp.v')]
  excerpts=[]
  terms=('frame_accept','frame_can_start','points_pending','point_owned','producer_pending','frame_begin','read_admit','pose_pending','S_WAIT_POINTS','pb_we','fuse_only','MATCH_ONLY')
  for p in source_paths:
   lines=p.read_text(encoding='utf8').splitlines();excerpts.append(dict(path=str(p),lines=[dict(line=i,text=s)for i,s in enumerate(lines,1)if any(t in s for t in terms)]))
  save(out/'actual_ports_FFs_queries_and_source.json',dict(PB_planes=details,key_FF_next_states=keyrows,queries=queries,omitted_custom_boundaries=[dict(name=n,kind=c['kind'])for n,c in omitted.items()],
   source_excerpts=excerpts,inputs=[metadata(p)for p in sources+source_paths+[LIBRARY,Path(__file__).resolve()]]))
  result.update(passed=True,recorded_at=datetime.now().astimezone().isoformat(),actual_build=str(BUILD),expanded_modules=list(NAMES),local_native_cells=len(cells),omitted_custom_modules=len(omitted),
   physical_PB_planes=8,width=18,READ_MODE=0,all_CEA_CEB_BLKSEL_and_address_bindings_proved=True,
   complete_owner_pending_RAW_next_state_bindings_proved=True,RAW_admission_blocks_new_nonFUSE_producer_during_ownership=True,
   early_command_WAIT_and_last_write_drain_proved=True,Boolean_queries=len(queries),UNSAT_queries=sum(q['result']=='unsat'for q in queries),free_state_collision_query=free['result'],
   arbitrary_SOF_collision_is_not_a_real_caller_trace=True,unconditional_full_reachability_collision_safety_proved=False,
   conditional_real_caller_contract=['Only actual accepted non-FUSE RAW pass can introduce extraction SOF. Project/transform emits one ordered SOF per accepted pass; no fabricated independent SOF.',
    'Original single-frame stable mode/config contract holds from accepted RAW start through its point/EOF pipeline drain. FUSE_ONLY bypass admits only a non-writing pass and cannot be switched into writing midway.',
    'Same-edge RAW acceptance reserves pending; command sees producer pending/ex_busy/pb_we/SOF and remains in WAIT_POINTS until the last PB write has drained.',
    'Reset/soft-clear/disable/HOLD release follows the frozen source transitions; no arbitrary reset of only part of the producer/consumer is assumed.'],
   prior_targeted_RTL_proof=str(ROOT/'stage2B_point_ownership_probe1/result.json'),new_RTL_or_EDA_run=False,
   details=str(out/'actual_ports_FFs_queries_and_source.json'),full_G2_or_board_collision_acceptance=False)
 except Exception as error:result.update(error=str(error),traceback=traceback.format_exc())
 save(out/'result.json',result);return result

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);a=p.parse_args();r=review(a.out)
 print(json.dumps(r,ensure_ascii=False,indent=2));raise SystemExit(0 if r['passed']else 1)
