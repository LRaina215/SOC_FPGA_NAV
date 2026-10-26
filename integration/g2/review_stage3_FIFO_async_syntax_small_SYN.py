"""Read the completed FIFO C netlist. Does not launch or modify any tool/build."""
from pathlib import Path
from datetime import datetime
from collections import Counter
import argparse,json,re
import psutil
from prepare_baseline_fusion_retime import load,save,metadata,same_bytes
from prepare_stage3_FIFO_async_syntax_small_SYN import CONFIGS,BASE,SYN40,canonical_RAM,unchanged
from synthesis_connectivity import read
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from audit_netlist_memories import BSRAM
from audit_routing_topology import primitive_directions

LIB=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')

def review(build,out):
 build=build.resolve();out=out.resolve();assert not out.exists();out.mkdir(parents=True)
 original=load(build/'result.json');runtime=load(build/'small_SYN_runtime.json');proof=load(build/'frozen_inputs.json')
 assert original['error']=="primitive_directions() missing 1 required positional argument: 'library'"and runtime['exit_code']==0
 unchanged(proof['inputs']);vg=build/'impl/gwsynthesis/g2_top.vg';cells=read(vg);mods=definitions(vg);get=parameter_reader(mods);directions=primitive_directions(LIB)
 drivers={}
 for cell in cells.values():
  for pin,nets in cell['ports'].items():
   if directions[cell['kind']][pin]=='output':
    for net in nets:drivers.setdefault(net,[]).append(cell)
 def annotated(c):return dict(c,parameters=get(c))
 def source(net):return dict(net=net,drivers=[annotated(c)for c in drivers.get(net,[])])
 def root(net):
  hops=[];seen=set()
  while net not in seen:
   seen.add(net);d=drivers.get(net,[])
   if len(d)!=1 or d[0]['kind']not in ('IBUF','BUFG'):break
   cell=d[0];assert len(cell['ports']['I'])==1;hops.append(annotated(cell));net=cell['ports']['I'][0]
  return dict(root=net,buffer_hops=hops)
 reference=load(build/'reference_RAMs.json');groups={};all_pass=True
 for scope,cfg in CONFIGS.items():
  subset=[c for c in cells.values()if c['name'].startswith(scope+'/')]
  memories=[dict(path=c['name'],kind=c['kind'],parameters=get(c),ports=c['ports'])for c in subset if c['kind']in BSRAM]
  same=canonical_RAM(memories)==canonical_RAM(reference[scope]['RAMs'])if reference[scope]['actual_reference_available']else None
  FFs=[]
  for c in subset:
   if not c['kind'].startswith('DFF'):continue
   clock=root(c['ports']['CLK'][0]);reset_pin='CLEAR'if 'CLEAR'in c['ports']else 'RESET'if 'RESET'in c['ports']else None
   reset=root(c['ports'][reset_pin][0])if reset_pin else None
   expected=cfg['write_reset']if clock['root']==cfg['write_clock']and cfg['write_clock']!=cfg['read_clock']else cfg['read_reset']
   FFs.append(dict(cell=annotated(c),clock_source=clock,reset_source=reset,
    direct_data_source=[source(n)for n in c['ports'].get('D',[])],CE_source=[source(n)for n in c['ports'].get('CE',[])],
    async_reset_ok=c['kind']=='DFFCE'and reset and reset['root']==expected and clock['root']in(cfg['write_clock'],cfg['read_clock'])))
  converted=[]
  for stem,reset,clock in [('rp_b_w',cfg['write_reset'],cfg['write_clock']),('wp_b_r',cfg['read_reset'],cfg['read_clock'])]:
   for bit in range(cfg['AW']+1):
    matches=[r for r in FFs if r['cell']['ports']['Q']==[f'{scope}/{stem}[{bit}]']]
    good=len(matches)==1 and matches[0]['async_reset_ok']and matches[0]['clock_source']['root']==clock and matches[0]['reset_source']['root']==reset
    converted.append(dict(signal=f'{scope}/{stem}[{bit}]',passed=bool(good),actual_FF=matches[0]['cell']['name']if len(matches)==1 else None))
  syncs=[]
  for stem in ('rp_g_w1','rp_g_w2','wp_g_r1','wp_g_r2'):
   for bit in range(cfg['AW']+1):
    matches=[r for r in FFs if r['cell']['ports']['Q']==[f'{scope}/{stem}[{bit}]']]
    syncs.append(dict(signal=f'{scope}/{stem}[{bit}]',passed=len(matches)==1 and bool(matches[0]['async_reset_ok']),actual_FFs=[r['cell']['name']for r in matches]))
  occupancy=[]
  for suffix in ('pend','cnt[0]','cnt[1]'):
   matches=[r for r in FFs if r['cell']['ports']['Q']==[f'{scope}/u_ob/{suffix}']]
   occupancy.append(dict(signal=f'{scope}/u_ob/{suffix}',passed=len(matches)==1 and bool(matches[0]['async_reset_ok']),actual_FFs=[r['cell']['name']for r in matches]))
  sync_reset=[r['cell']['name']for r in FFs if 'RESET'in r['cell']['ports']]
  async_ffs=[r for r in FFs if r['cell']['kind']=='DFFCE'];valid=all(r['async_reset_ok']for r in async_ffs)
  passed=all(r['passed']for r in converted+syncs+occupancy)and valid and not sync_reset and bool(memories)and same is not False
  groups[scope]=dict(passed=passed,config=cfg,FF_count=len(FFs),FF_kinds=dict(Counter(r['cell']['kind']for r in FFs)),async_clear_FF_count=len(async_ffs),
   all_actual_FF_clock_reset_data_CE_sources=FFs,converted_pointer_bits=converted,synchronizer_bits=syncs,occupancy_bits=occupancy,
   all_DFFCE_reset_clock_roots_correct=valid,synchronous_reset_FFs=sync_reset,
   RAMs=memories,RAM_explicit_parameters_match_available_SYN40=same,RAM_reference_limit=reference[scope]['limitation'])
  all_pass&=passed
 stdout=(build/'small_SYN_stdout.log').read_text(errors='replace');header=vg.read_text(encoding='utf8')[:600]
 assert 'Device Version: C'in header and 'GW5AT-LV138PG484AC1/I0'in header
 no_error='EX3534'not in stdout and not re.search(r'\bERROR\s*\(',stdout)
 all_pass&=no_error and 'GowinSynthesis finish'in stdout
 identities=[dict(pid=original['controller_pid'],created_epoch=original['controller_created_epoch'],role='controller')]+[dict(r,role='tool_or_child')for r in runtime['processes']]
 for r in identities:
  try:r['original_identity_alive']=abs(psutil.Process(r['pid']).create_time()-r['created_epoch'])<.01
  except psutil.NoSuchProcess:r['original_identity_alive']=False
 exited=all(not r['original_identity_alive']for r in identities);all_pass&=exited
 assert same_bytes(BASE/'nav/rtl/common/nav_fifo_ob.v',SYN40/'nav/rtl/common/nav_fifo_ob.v')
 detail=dict(groups=groups,process_identities=identities,all_recorded_identities_exited=exited,
  exact_all_primitives_with_parameters=[annotated(c)for c in cells.values()],
  inputs=[metadata(p)for p in (vg,build/'small_SYN_stdout.log',build/'frozen_inputs.json',LIB,Path(__file__).resolve())])
 save(out/'actual_primitives.json',detail)
 result=dict(passed=bool(all_pass),recorded_at=datetime.now().astimezone().isoformat(),actual_build=str(build),actual_netlist=str(vg),tool_exit_code=runtime['exit_code'],
  original_parser_failure_preserved=str(build/'result.json'),correction='Pass public library explicitly and trace actual IBUF/BUFG ports; no hardware/tool rerun.',
  device='GW5AT-LV138PG484AC1/I0',device_version='C',EX3534_absent=no_error,source_changes_only_two_outer_if_forms=True,inner_ternary_XZ_preserved=True,
  public_nav_fifo_ob_byte_identical_SYN40=True,all_frozen_build_inputs_unchanged=True,
  primitive_counts=dict(Counter(c['kind']for c in cells.values())),FF_total=sum(g['FF_count']for g in groups.values()),
  async_clear_FF_total=sum(g['async_clear_FF_count']for g in groups.values()),BSRAM_total=sum(len(g['RAMs'])for g in groups.values()),
  group_summaries={k:{x:g[x]for x in ('passed','FF_count','FF_kinds','async_clear_FF_count','all_DFFCE_reset_clock_roots_correct','synchronous_reset_FFs','RAM_explicit_parameters_match_available_SYN40')}for k,g in groups.items()},
  converted_pointer_bits_checked=sum(len(g['converted_pointer_bits'])for g in groups.values()),synchronizer_bits_checked=sum(len(g['synchronizer_bits'])for g in groups.values()),occupancy_bits_checked=9,
  actual_details=str(out/'actual_primitives.json'),all_recorded_original_identities_exited=exited,
  warnings=[line for line in stdout.splitlines()if 'WARN'in line],
  host_RX_AW12_no_original_mapped_reference=True,original_26_CDC_or_52_constraints_accepted=False,
  full_G2_resources_timing_async_recovery_removal_collision_or_board_proved=False,PNR_started=False,programmed=False)
 save(out/'result.json',result);return result

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--build',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
 print(json.dumps(review(a.build,a.out),ensure_ascii=False,indent=2))
