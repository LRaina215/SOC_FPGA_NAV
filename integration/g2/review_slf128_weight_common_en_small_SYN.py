"""Read only the new small actual weight map; hand official macro simulation on."""
from pathlib import Path
from datetime import datetime
from collections import Counter
import argparse,json,re,traceback
import psutil
from prepare_baseline_fusion_retime import load,save,metadata
from prepare_slf128_weight_common_en_small_SYN import unchanged
from synthesis_connectivity import read
from review_stage3_slf128_memories import Tree,LIBRARY,init_vector,library_defaults
from audit_routing_topology import primitive_directions
from prove_soc_console_remap import Logic,literal,z3

def review(build,out):
 build=build.resolve();out=out.resolve();assert out.is_dir()and not(out/'result.json').exists()
 result=dict(passed=False,scope='Small actual35-ROM INIT/port/CE/selector structural review; official macro values remain separate',PNR_started=False)
 try:
  tool=load(build/'tool_result.json');assert tool['tool_passed'];unchanged(load(build/'frozen_inputs.json')['inputs'])
  vg=build/'impl/gwsynthesis/g2_top.vg';tree=Tree(vg);cells=read(vg);directions=primitive_directions(LIBRARY);defaults=library_defaults()
  roms={n:c for n,c in cells.items()if c['kind']in('pROM','pROMX9')};assert Counter(c['kind']for c in roms.values())==dict(pROM=15,pROMX9=20)
  ff={c['ports']['Q'][0]:c for c in cells.values()if c['kind'].startswith('DFF')};assert len(ff)==8
  qs={q:z3.Bool('actual_FF:'+q)for q in ff};opaque={(n,'DO',i):z3.Bool(f'ROM:{n}:{i}')for n,c in roms.items()for i in range(len(c['ports']['DO']))}
  logic=Logic(cells,lambda c:tree.params[c['name']],directions,qs,opaque)
  en=logic.expr('en');addr=[logic.expr(f'addr[{i}]')for i in range(16)]
  payloads={p.name:[int(x,16)for x in p.read_text().split()]for p in(build/'src/mem').glob('weight9_*.mem')};assert len(payloads)==35
  selected=set();rows=[];queries=[]
  def prove(label,left,right):
   s=z3.Solver();s.set(timeout=10000);s.add(z3.Xor(left,right));status=str(s.check());queries.append(dict(label=label,result=status));assert status=='unsat',(label,status)
  for n,c in roms.items():
   p=tree.params[n];width=literal(p['BIT_WIDTH']);assert width in(8,9)and literal(p['READ_MODE'])==0 and p['RESET_MODE']=='"SYNC"'
   assert c['ports']['CLK']==['clk50_d']and c['ports']['RESET']==['GND']
   value,bits=init_vector(c['kind'],p,defaults);assert bits==2048*width
   words=[(value>>(width*i))&((1<<width)-1)for i in range(2048)]
   matches=[name for name,data in payloads.items()if data==words];assert len(matches)==1 and matches[0]not in selected,(n,matches)
   payload=matches[0];selected.add(payload);actual=logic.expr(c['ports']['CE'][0])
   if width==8:
    assert payload.startswith('weight9_high');prove(n+' high CE = common en',actual,en)
    expected=[f'addr_d[{i}]'for i in range(12,1,-1)]+['GND']*3
   else:
    assert payload.startswith('weight9_low');prove(n+' low CE cannot read on en0',z3.And(actual,z3.Not(en)),z3.BoolVal(False))
    expected=[f'addr_d[{i}]'for i in range(10,-1,-1)]+['GND']*3
   assert c['ports']['AD']==expected,(n,'actual address mismatch',c['ports']['AD'])
   rows.append(dict(cell=c,parameters=p,payload=metadata(build/'src/mem'/payload),width=width,words=2048,CE_expression=str(z3.simplify(actual))))
  assert selected==set(payloads)
  selector_rows=[]
  expected_bits={'selected_low[0]':11,'selected_low[1]':12,'selected_low[2]':13,'selected_low[3]':14,'selected_high[2]':15,'selected_lane[0]':0,'selected_lane[1]':1}
  for q,c in ff.items():
   local=q.split('/',1)[1];assert c['ports']['CLK']==['clk50_d']and c['ports']['RESET']==['GND']
   prove(c['name']+' CE = common en',logic.expr(c['ports']['CE'][0]),en)
   if local=='bit14_flip':
    target=z3.Or(*[z3.And(*[addr[i]==z3.BoolVal(bool(n>>i&1))for i in range(16)])for n in(675,3055,11735)])
   else:assert local in expected_bits;target=addr[expected_bits[local]]
   prove(c['name']+' capture data',logic.expr(c['ports']['D'][0]),target)
   selector_rows.append(dict(cell=c,parameters=tree.params[c['name']],next_state=str(logic.next_state(c))))
  outputs=tree.boundary['u_weights']['value_d'];assert len(outputs)==16
  dependencies={}
  for index,net in enumerate(outputs):
   names=sorted(str(x)for x in z3.z3util.get_vars(logic.expr(net)))
   assert all(n.startswith(('actual_FF:','ROM:'))for n in names),('public output depends on live input',net,names)
   dependencies[15-index]=dict(net=net,state_dependencies=names)
  runtime=load(build/'small_SYN_runtime.json');ids=[dict(pid=tool['controller_pid'],created_epoch=tool['controller_created_epoch'],name='controller')]+runtime['processes']
  for row in ids:
   try:row['original_identity_alive']=abs(psutil.Process(row['pid']).create_time()-row['created_epoch'])<.01
   except psutil.NoSuchProcess:row['original_identity_alive']=False
  assert not any(row['original_identity_alive']for row in ids)
  save(out/'actual_35_ROMs_selectors_and_CE.json',dict(ROMs=rows,selectors=selector_rows,Boolean_queries=queries,public_output_state_dependencies=dependencies,
   all_primitives_with_parameters=[dict(c,parameters=tree.params[c['name']])for c in cells.values()],process_identities=ids,
   inputs=[metadata(p)for p in(vg,LIBRARY,Path(__file__).resolve(),build/'frozen_inputs.json')]))
  result.update(passed=True,recorded_at=datetime.now().astimezone().isoformat(),actual_build=str(build),actual_VG=str(vg),actual_module=str(out/'actual_fixed4500_weight_rom.v'),
   synthesis_exit0=True,one_small_C_SYN_only=True,actual_ROMs=35,high_pROM8=15,low_pROMX9_9=20,all35_payloads_unique_exact=True,
   high15_actual_CE_common_en=True,all_low_CE_false_when_en0=True,all_actual_ROM_address_bits_correct=True,READ_MODE=0,RESET_MODE='SYNC',
   physical_selector_correction_FFs=8,logical_selector_capture_aliases='selected_high[0:1] share selected_low[2:3]; selected_low[4] shares selected_high[2], all from the same en-captured address bits.',
   all8_selector_and_flip_capture_functions_checked=True,selector_and_ROM_CE_hold_on_en0=True,public_output_only_ROM_and_FF_state=True,
   Boolean_queries=len(queries),all_queries_UNSAT=True,all_frozen_inputs_unchanged=True,all_recorded_original_identities_exited=True,
   details=str(out/'actual_35_ROMs_selectors_and_CE.json'),official_primitive_all40704_and_idle_validation_pending=True,
   four_state_postmap_values_full4500_or_physical_timing_accepted=False,programmed=False)
 except Exception as error:result.update(error=str(error),traceback=traceback.format_exc())
 save(out/'result.json',result);return result

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--build',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
 result=review(a.build,a.out);print(json.dumps(result,ensure_ascii=False,indent=2));raise SystemExit(0 if result['passed']else 1)
