"""Completed40->52 physical preservation review; no EDA or source writes.

Default prints configuration; --review uses a fresh output. --wait-for-memory
only waits for actual2GiB physical/4GiB commit, never for another tool to exit.
"""
from pathlib import Path
from datetime import datetime
from collections import Counter,defaultdict
from types import SimpleNamespace
import argparse,json,re,time,traceback,psutil
from synthesis_connectivity import CELL,PORT
from revalidate_lidar_cdc import definitions
from audit_cdc_first_stages import sdc_fragment
from prepare_baseline_fusion_retime import metadata,same_bytes
from related_clock_physical_review_common import make_context
from review_model25_ring_system_pnr51_final import verify_extra_guard_replicas
from review_lease_tree_pnr50_final import report as resource_report,CELL as VO_CELL
from review_three_clock_TR import review_timing
from retry_frozen_synthesis import memory_status
import review_pnr50_bsram_init as bsram
import review_pnr50_distributed_init as distributed

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'baseline_fusion50_ring_MAX_syn40'
PHYSICAL=BASE/'baseline_fusion50_ring_MAX_pnr52'
CONTROL=BASE/'baseline_fusion50_ring_MAX_pnr_one_shot'
DEFAULT_OUT=BASE/'pnr52_system_final_actual_review1'

def load(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def save(p,v):
 assert not p.exists(),('Evidence exists',str(p))
 p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf8')

def finished():
 rt=load(PHYSICAL/'tool_runtime.json')
 assert 'ended_at'in rt and'tool_exit'in rt and rt['cancelled']is False
 assert rt['max_runtime_s']==rt['max_no_observable_progress_s']==0
 created=datetime.fromisoformat(rt['started_at']).timestamp();pid=rt['pid']
 current=psutil.Process(pid).create_time()if psutil.pid_exists(pid)else None
 assert current is None or abs(current-created)>2,('Original physical tool still alive',pid)
 assert Path(load(CONTROL/'job.json')['output']).resolve()==PHYSICAL.resolve()
 return rt,dict(pid=pid,original_created_epoch=created,current_created_epoch=current,original_identity_exited=True)

def memories(out):
 folder=out/'BSRAM_CPU';folder.mkdir()
 paths=[SOURCE/'impl/gwsynthesis/g2_top.vg',PHYSICAL/'g2_top.vg',PHYSICAL/'impl/pnr/g2_top.vo',
        PHYSICAL/'impl/pnr/g2_top.rpt.txt',PHYSICAL/'impl/pnr/cmd.do',PHYSICAL/'g2_gate.json',bsram.LIB,
        SOURCE/'firmware/nav_fw.bin',SOURCE/'ram_w.hex']+[SOURCE/f'ram_b{i}.hex'for i in range(4)]
 before=[bsram.metadata(p)for p in paths]
 s,p,defaults=bsram.extract(SOURCE,PHYSICAL)
 bsram.save(folder/'actual_bsram_parameters_ports.json',dict(source=s,physical=p,library_defaults=defaults))
 result=bsram.check(SOURCE,PHYSICAL,s,p,defaults)
 result['legacy_reader_unrouted1659_literal_found']=result.pop('routing_failure_recorded_in_report')
 result['legacy_reader_key_note']='frozen_physical_input_VG_equals_SYN37 uses its supplied source argument SYN40, not literal SYN37.'
 result['actual_input_VG_equals_completed_SYN40']=same_bytes(SOURCE/'impl/gwsynthesis/g2_top.vg',PHYSICAL/'g2_top.vg')
 result['inputs_before']=before
 result['inputs_unchanged_after_read_only_review']=before==[bsram.metadata(p)for p in paths]
 result['passed'] &= result['inputs_unchanged_after_read_only_review']
 bsram.save(folder/'result.json',result);assert result['passed']
 dist=out/'distributed_INIT';dist.mkdir()
 dr=distributed.review(SimpleNamespace(source=SOURCE,physical=PHYSICAL,bsram_proof=folder/'result.json',out=dist))
 assert dr['passed'];return result,dr

def selected_preservation(out,reference):
 mods=definitions(SOURCE/'impl/gwsynthesis/g2_top.vg');prefixes={}
 def visit(kind,parent):
  for child,raw,body in CELL.findall(mods[kind]):
   name=(parent+'/'+raw.lstrip('\\')).strip('/')
   if child in('command_lease_guard','safety_guard'):prefixes[name]=child
   if child in mods:visit(child,name)
 visit('g2_top','');assert set(prefixes.values())=={'command_lease_guard','safety_guard'}
 prefixes['u_control/remote_thread']='IR_remote_boundary'
 roles={row[k]for row in reference['paths']for k in('source_register','first_register','second_register')}
 ctx=make_context(SOURCE/'impl/gwsynthesis/g2_top.vg',PHYSICAL/'impl/pnr/g2_top.vo',extra=roles,extra_prefixes=[p+'/'for p in prefixes])
 before,after=ctx['source'],ctx['physical'];checks=[];scopes=[]
 for prefix,kind in prefixes.items():
  a={n for n,c in before.items()if n.startswith(prefix+'/')and c['kind']not in('GND','VCC')}
  b={n for n,c in after.items()if n.startswith(prefix+'/')and c['kind']not in('GND','VCC')}
  assert a and a<=b,('Selected stop primitive missing',prefix,sorted(a-b))
  checks.extend(ctx['equal_cell'](n)for n in sorted(a))
  extras=verify_extra_guard_replicas(ctx,a,b-a)
  scopes.append(dict(instance=prefix,kind=kind,source_primitives=len(a),physical_additions_checked=extras))
 checks.extend(ctx['equal_cell'](n)for n in sorted(roles))
 first={}
 for r in reference['paths']:
  a,b,c=[after[r[k]]for k in('source_register','first_register','second_register')]
  assert a['ports']['Q']==b['ports']['D']and b['ports']['Q']==c['ports']['D']
  first[b['ports']['Q'][0]]=c['name']
 loads=defaultdict(list)
 for kind,raw,body in VO_CELL.findall((PHYSICAL/'impl/pnr/g2_top.vo').read_text()):
  if kind=='module':continue
  for pin,expr in PORT.findall(body):
   if ctx['directions'][kind][pin]=='output':continue
   net=re.sub(r'\s+(?=\[)','',expr.strip().lstrip('\\'))
   if net in first:loads[net].append((raw.lstrip('\\'),pin))
 for q,name in first.items():assert loads[q]==[(name,'D')],('Original26 first-Q fanout',q,loads[q])
 clocks=[];counts=Counter()
 for key,net in ctx['clockpins'].items():
  root,phase=ctx['canonical'](after,ctx['physical_buffer_drivers'],net)
  assert root in('clk50','rgmii_rxc'),('New physical clock root',key,root)
  counts[root,phase]+=1;clocks.append(dict(instance=key[0],pin=key[1],bit=key[2],net=net,root=root,inverted=bool(phase)))
 assert {k for k,p in counts}=={'clk50','rgmii_rxc'}
 assert not any(c['kind']=='PLL'for c in after.values())
 value=dict(passed=True,original26_paths=26,original26_firstQ_unique_secondD_load=True,selected_stop_IR_scopes=scopes,
   selected_cell_checks=checks,actual_clock_pin_count=len(clocks),actual_clock_pin_traces=clocks,
   actual_two_clock_counts=[dict(root=r,inverted=bool(p),pins=n)for(r,p),n in sorted(counts.items())],
   no_PLL_or_model25_clock=True,proved_registered_replica_aliases=list(ctx['proved_registered_replica_aliases'].values()),
   physical_CDC_payload_delays_reset_pulse_IO_and_vehicle_stop_signed_off=False,
   ring_data_address_prefetch_DSP_functional_cones_reproved=False)
 save(out/'actual_two_clocks_original26_stop_IR.json',value);return value

def run(out):
 rt,identity=finished();assert not out.exists();out.mkdir(parents=True)
 watched=[SOURCE/'impl/gwsynthesis/g2_top.vg',PHYSICAL/'g2_top.vg',PHYSICAL/'impl/pnr/g2_top.vo',
          PHYSICAL/'impl/pnr/g2_top.rpt.txt',PHYSICAL/'impl/pnr/cmd.do',PHYSICAL/'impl/pnr/device.cfg',
          PHYSICAL/'g2.cst',PHYSICAL/'g2.sdc',PHYSICAL/'tool_runtime.json']
 bindings=[metadata(p)for p in watched]
 assert same_bytes(SOURCE/'impl/gwsynthesis/g2_top.vg',PHYSICAL/'g2_top.vg')
 assert same_bytes(PHYSICAL/'impl/pnr/device.cfg',PHYSICAL/'expected_from35_device.cfg')
 assert same_bytes(PHYSICAL/'impl/pnr/cmd.do',PHYSICAL/'expected_from35_cmd.do')
 assert same_bytes(SOURCE/'g2.cst',PHYSICAL/'g2.cst')
 reference=load(PHYSICAL/'cdc_reference_audit.json');assert reference['passed']and len(reference['paths'])==26
 sdc=[x.strip()for x in(PHYSICAL/'g2.sdc').read_text().splitlines()if x.strip()and not x.strip().startswith('#')]
 assert [x for x in sdc if x.startswith('set_')]==[x for x in sdc_fragment(reference).splitlines()if x.startswith('set_')]
 assert len([x for x in sdc if x.startswith('create_clock ')])==2 and not any(x.startswith('create_generated_clock')for x in sdc)
 assert 'create_clock -name clk50 -period 20 [get_ports {clk50}]'in sdc and'create_clock -name rgmii_rxc -period 10 [get_ports {rgmii_rxc}]'in sdc
 print('52 all BSRAM split INIT CPU and distributed INIT',flush=True);m,d=memories(out)
 print('52 physical stop IR original26 and actual two clocks',flush=True);c=selected_preservation(out,reference)
 t=review_timing(PHYSICAL,out/'actual_timing_review.json',targets=dict(clk50=50,rgmii_rxc=100))
 reports={tag:resource_report(BASE/name)for tag,name in[('35','baseline_fusion32_CDC52_pnr35'),('51','baseline_fusion_model25_observation_ring_pnr51'),('52',PHYSICAL.name)]}
 raw=(PHYSICAL/'impl/pnr/g2_top.rpt.txt').read_text();unrouted=re.findall(r'total\s+(\d+)\s+unrouted nets',raw)
 result=dict(passed=True,preservation_review_completed=True,recorded_at=datetime.now().astimezone().isoformat(),
   source=str(SOURCE),physical=str(PHYSICAL),runtime=rt,original_process_identity=identity,
   actual_resources=reports,actual_unrouted_nets=int(unrouted[-1])if unrouted else None,
   BSRAM=m['physical_BSRAM'],wide_split_pairs=m['split_pairs'],CPU=m['CPU64'],all_memory_geometry_effective_INIT_preserved=m['passed']and d['passed'],
   actual_two_clocks_original26_stop_IR=c,timing=t,
   selected_guard_source_equivalence='SYN40_guard_input_roles_review3/result.json',
   root_input_cmd_device_CST_clock52requests_bound=True,metadata_bindings=bindings,
   bound_input_metadata_unchanged=all(metadata(Path(r['path']))==r for r in bindings),
   no_failed_net_identity_or_congestion_cause_claim=True,new_EDA_started=False,programmed=False,
   physical_complete_CDC_reset_IO_and_vehicle_stop_signed_off=False,deployment_accepted=False)
 assert result['bound_input_metadata_unchanged'];save(out/'result.json',result);return result

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--review',action='store_true');ap.add_argument('--wait-for-memory',action='store_true')
 ap.add_argument('--out',type=Path,default=DEFAULT_OUT);a=ap.parse_args()
 if not a.review:print(json.dumps(dict(prepared=True,source=str(SOURCE),physical=str(PHYSICAL),out=str(a.out),starts_EDA=False)))
 else:
  while a.wait_for_memory:
   m=memory_status()
   if m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3:break
   print(json.dumps(dict(phase='waiting_for_actual_review_memory',physical=m.avail_phys,commit=m.avail_page)),flush=True);time.sleep(15)
  try:
   r=run(a.out);print(json.dumps(dict(passed=r['passed'],actual_unrouted=r['actual_unrouted_nets'],result=str(a.out/'result.json'),deployment_accepted=False)))
  except BaseException as e:
   if a.out.exists():save(a.out/'review_failed.json',dict(passed=False,error=str(e),traceback=traceback.format_exc(),recorded_at=datetime.now().astimezone().isoformat()))
   raise
