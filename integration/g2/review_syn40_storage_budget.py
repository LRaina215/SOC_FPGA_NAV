"""Read completed SYN40 hierarchy and leaf memory counts for migration budget.

No port/cone flattening, EDA, source mutation or physical PNR52 output read.
The 21MB frozen input is parsed once; only counters/selected memory geometry
are retained. A matching input is byte compared, never hashed.
"""
from collections import Counter,defaultdict
import argparse
from functools import lru_cache
import json
from pathlib import Path
import re
import shutil
from xml.etree import ElementTree as ET
from audit_netlist_memories import MODULE,INST,PARAM,BSRAM

BASE=Path('F:/nav_integration_g2_20261023')
BUILD=BASE/'baseline_fusion50_ring_MAX_syn40'
PNR=BASE/'baseline_fusion50_ring_MAX_pnr52'
OUT=BASE/'SYN40_storage_migration_budget_review2'
PREFIXES=['u_soc','u_control/model_thread','u_control/model_thread/observation','u_control/model_thread/policy/core',
 'u_control/model_thread/policy/core/weights','u_nav/u_map/u_tv','u_nav/u_map/u_tv/u_bm','u_nav/u_map/u_tv/u_ct',
 'u_nav/u_mbx','u_nav/u_mbx/u_injb','u_nav/u_mbx/u_rxb','u_nav/u_mbx/u_txb','u_nav/u_mbx/u_txs',
 'u_nav/u_match','u_nav/u_match/u_ex/u_rb','u_nav/u_match/u_pb','u_nav/u_match/u_sm','u_nav/u_match/u_lf',
 'u_nav/u_match/u_touch','u_nav/u_match/u_slist','u_nav/u_match/u_acc','u_nav/u_match/u_cand','u_nav/u_match/u_res',
 'u_nav/u_match/u_sin','u_nav/u_match/u_cos','u_nav/u_tr/u_sin','u_nav/u_tr/u_cos']+[f'u_soc/g_con[{i}].u_con' for i in range(4)]

def save(name,value):
 p=OUT/name;assert not p.exists();p.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def meta(p):
 s=p.stat();return dict(path=str(p),bytes=s.st_size,mtime_ns=s.st_mtime_ns)
def same(a,b):
 if a.stat().st_size!=b.stat().st_size:return False
 with a.open('rb') as x,b.open('rb') as y:
  while True:
   u=x.read(1024*1024);v=y.read(1024*1024)
   if u!=v:return False
   if not u:return True
def summary(counts):
 return dict(FF=sum(v for k,v in counts.items() if k.startswith('DFF')),
  LUT_leaf=sum(v for k,v in counts.items() if re.fullmatch(r'LUT[1-6]',k)),
  MUX_LUT_leaf=sum(v for k,v in counts.items() if k.startswith('MUX2_LUT')),
  BSRAM=sum(v for k,v in counts.items() if k in BSRAM),RAM16=sum(v for k,v in counts.items() if k.startswith('RAM16')),
  ROM16=counts['ROM16'],ALU=counts['ALU'],primitive_counts=dict(counts))

def budget(inventory):
 def row(name,path,bits,blocks,delta,ports,note,kind='candidate'):
  group=inventory['selected_leaf_groups'].get(path,{})
  return dict(name=name,actual_scope=path,classification=kind,logical_bits=bits,
    actual_SYN_leaf_counts=group,current_physical_BSRAM=blocks,proposed_physical_BSRAM_delta=delta,
    port_and_cycle_contract=ports,initialization='Preserve every initialized word and held output; most writable nav stores start zero. No added service cycle is assumed.',
    local_logic_recovery_upper_bound_policy='RAM16 removal is budgeted at6 report logic units/leaf, ROM16 at1; local LUT/MUX counts are only an eligibility ceiling. Added glue and actual CLS packing are unmeasured.',note=note)
 rows=[
 row('SLF256_to128_structure_and_touch','u_nav/u_match/u_sm',65536*19,68,-51,
   'Keep each structure/touch store1W+1R and enabled1-cycle read; reset/clear scans and all address consumers require the separate SLF contract.',
   'Only structure18bit and touch1bit arrays quarter:64+4 source/native blocks ->16+1. Delta -48-3=-51. This is a changed navigation window, not a capacity-preserving migration. Current no-map128 evidence is not accepted by this budget.','conditional_semantic_change'),
 row('packed_weight9','u_control/model_thread/policy/core/weights',40704*16,40,-5,
   'One enabled weight read/cycle, identical latency and hold; no extra MAC cycle.',
   'Existing isolated and joint evidence40->35 blocks, not9 saved; all40704 values preserved including3 bit14 exceptions. Rebase INIT/address proof onto current40 required.','existing_numeric_and_SYN_proof_not_current_PNR'),
 row('traversability_four_row_caches_and_column_state','u_nav/u_map/u_tv',4*130*34+128*(5+24),10,0,
   'Four simultaneous row-chain old values and column history; preserve same-address read-before-write and pipeline alignment.',
   'Already5 wide BSRAM leaves: two36bit, three32bit, becoming10 physical under split1. Their existing BSRAM occupancy is not a new migration gain. W/pipeline FFs are not these RAM arrays.','already_BSRAM'),
 row('traversability_bitmap','u_nav/u_map/u_tv/u_bm',16384,0,1,
   '16384x1 simple dual port,1W+1R with enabled1-cycle read/hold and old-data collision semantics.',
   'Actual1024 RAM16SDP1 plus1704 local LUT; source wrapper uses distributed at the exact block-depth boundary. One native16384x1 block is the proposed geometry.'),
 row('traversability_column_scratch','u_nav/u_map/u_tv/u_ct',22*128,0,2,
   '128x22 synchronous1W+1R; original T1 request/T2 consume contract must remain.',
   'Actual48 RAM16SDP4. Conservative two native<=18bit planes; capacity bits alone would undercount wide parallel ports.'),
 row('mailbox_injection_and_TX_shadow','u_nav/u_mbx',2*512*32,0,4,
   'Two independent512x32 SDP stores, each one write and one enabled read; retain original injection/RX timestamps and2-stage caller alignment.',
   'Only u_injb/u_txs remain, each256 RAM16SDP4. u_txb has zero actual leaves and cannot be counted a third time. u_rxb is already2BSRAM. Budget two16/18bit planes per retained store.'),
 row('SLF_written_slot_list','u_nav/u_match/u_slist',1024*16,0,1,
   '1024 entries, one write and one enabled read; old1-cycle hold and list-clear ordering.',
   'Actual256 RAM16SDP4 +600 local LUT. LOG2N7 changes word16->14 only (14336bits), depth1024 stays; BSRAM still1, RAM16 leaf-count reduction is not guaranteed.'),
 row('SLF_accept_mask','u_nav/u_match/u_acc',1024,0,1,
   '1024x1 separate gate-write/score-read store with original enabled1-cycle behavior.',
   'Actual64 RAM16SDP1; LOG2N shrink does not change point-index depth.'),
 row('SLF_candidate_and_result_tables','u_nav/u_match/u_cand',2*512*32,0,4,
   'Two512x32 stores with independent host/FSM accesses;2 physical narrow planes each, no extra result bus wait.',
   'Both actual256 RAM16SDP4 and548 LUT each. LOG2N shrink does not change these candidate counts; u_res is a separate same-sized actual scope.'),
 row('SLF_extractor_row_cache','u_nav/u_match/u_ex/u_rb',160*98,0,6,
   '160x98, wide one write and one synchronous read per accepted pixel; retain upstream backpressure and flush order.',
   'Actual275 RAM16SDP4. W160/UW8 is independent of SLF LOG2N. Six<=18bit planes conservatively preserve98bit/cycle; a bit-capacity quotient of1 block is invalid.'),
 row('console_four_byte_lanes','u_soc',4096*8,0,4,
   'Four1024x8 lanes, independently byte-enabled writes and four same-address reads forming a32bit debug word.',
   'Actual512 RAM16SDP4; lane LUT totals407+328+328+328=1391. Four native8bit blocks preserve byte writes; claiming2 blocks requires a separately proved byte-enable/word-assembly redesign.'),
 row('four_trig_lookup_ports','u_nav/u_match/u_sin',4*1024*16,0,4,
   'SLF and trajectory each read sin+cos together: four enabled lookup ports total, one clock/read latency and hold unchanged.',
   'Actual4x737=2948 ROM16 and4 local LUT totals1694. Existing prior trigblock joint SYN measured +4BSRAM,-2948ROM16,-1660T_Lut,-104FF,-2ALU. Those are prior-context results, not current CLS savings.'),
 row('inference_bias','u_control/model_thread/policy/core',425*26,0,2,
   'One425x26 enabled lookup in BIAS_REQ; BIAS_CAPTURE must see same signed32bit extension at the same edge.',
   'Actual702 ROM16 in core; ELU and weights are already BSRAM. Conservative16+10bit planes use2 blocks. Do not count all1925 core-local LUT as bias-only selectors.'),
 row('inference_ELU','u_control/model_thread/policy/core',2307*13,4,0,
   'Existing ACT_WAIT enabled read plus constant -4096 tail must remain cycle-exact.',
   'Already4 pROM BIT_WIDTH4 leaves. No LUT-to-BSRAM migration saving remains; any further packing is a separate optimization and not credited.','already_BSRAM'),
 row('SLF_LF_four_row_caches','u_nav/u_match/u_lf',4*258*19,8,0,
   'Four simultaneous old row values; preserve T1/T1b column-moment alignment.',
   'Already4 wide32bit BSRAM leaves ->8 physical. LOG2N8->7 logical row capacity19608->9880 bits (about half), not quarter. Physical blocks are conservatively held at8 until actual mapping.','already_BSRAM'),
 row('point_buffer_explicit18bit_planes','u_nav/u_match/u_pb',4096*36,16,-8,
   'Same4096x36 capacity,1W+1R, enabled1-cycle read, read-first and output hold.',
   'Current8x36bit leaves split to16physical. Old isolated slf_point_planes_probe1 really measured16->8 with capacity/latency retained; current40 has not adopted planes. Rebase and recheck all actual geometry/INIT/port binding; prior source was sfp_lane2_obsreuse_syn9.','optional_prior_isolated_physical_proof')]
 def combined(names,direct=False):
  counts=Counter();table=inventory['actual_direct_named_array_leaves'] if direct else inventory['selected_leaf_groups']
  for name in names:counts.update(table[name]['primitive_counts'])
  return summary(counts)
 pooled={
  'SLF256_to128_structure_and_touch':['u_nav/u_match/u_sm','u_nav/u_match/u_touch'],
  'mailbox_injection_and_TX_shadow':['u_nav/u_mbx/u_injb','u_nav/u_mbx/u_txs'],
  'SLF_candidate_and_result_tables':['u_nav/u_match/u_cand','u_nav/u_match/u_res'],
  'console_four_byte_lanes':[f'u_soc/g_con[{i}].u_con' for i in range(4)],
  'four_trig_lookup_ports':['u_nav/u_match/u_sin','u_nav/u_match/u_cos','u_nav/u_tr/u_sin','u_nav/u_tr/u_cos']}
 for r in rows:
  if r['name'] in pooled:r['actual_SYN_leaf_counts']=combined(pooled[r['name']])
  if r['name']=='traversability_four_row_caches_and_column_state':
   r['actual_SYN_leaf_counts']=combined(['u_nav/u_map/u_tv/'+n for n in ('lb1','lb3','vc')],True)
  if r['name']=='SLF_LF_four_row_caches':
   r['actual_SYN_leaf_counts']=combined(['u_nav/u_match/u_lf/'+n for n in ('lb1','lb3')],True)
  if r['name']=='inference_ELU':r['actual_SYN_leaf_counts']=combined(['u_control/model_thread/policy/core/elu_mem'],True)
  if r['name']=='inference_bias':r['actual_SYN_leaf_counts']=dict(ROM16=702,RAM16=0,BSRAM=0,local_selector_LUT='not separately attributed from shared core logic')
  c=r['actual_SYN_leaf_counts'];known_data=6*c.get('RAM16',0)+c.get('ROM16',0)
  local=c.get('LUT_leaf');r['known_current_storage_report_logic_units']=known_data
  r['current_local_LUT_eligibility_ceiling']=local
  r['current_storage_plus_local_LUT_upper_bound']=known_data+local if local is not None else None
  r['actual_recovered_logic_or_CLS']='unmeasured; bounds are not measured savings'
 return dict(passed=True,phase='read_only_budget_not_implementation_acceptance',baseline='Actual PNR52 completed report, including already-realized ring gain',
   current=inventory['physical_report'],rows=rows,
   SLF128=dict(structure_blocks_64_to16=-48,touch_blocks_4_to1=-3,credited_physical_blocks=-51,
     structure_touch_bits=[65536*19,16384*19],LF_row_bits=[4*258*19,4*130*19],slot_list_bits=[1024*16,1024*14],
     unchanged='Point buffer4096x36, extractor160x98, candidate/result512x32, accept mask1024 and four trig tables; CPU/prior cache not budgeted here.',
     algorithm_acceptance='not accepted; first no-map128 scenario is reported degraded, so no128 RTL is recommended for immediate use'),
   already_realized_ring=dict(SYN23_FF=38183,SYN40_FF=29642,FF_delta=-8541,SYN23_T_Lut=53059,SYN40_T_Lut=45967,T_Lut_delta=-7092,
     added_RAM16=66,added_MULT12X12=3,subtract_again=False,physical_baseline_already_includes_this=True),
   budget_formulas=dict(current_physical='314 source BSRAM +18 wide extra =332 actual PNR52',
     existing_report_logic='45825 LUT +9944 ALU +3650 ROM16 +6*3921 RAM16 =82945',
     requested_migrations_physical_add='bitmap1 +column2 +mailbox4 +slist1 +console4 +trig4 +bias2 =18',
     no_128_no_point_planes_all_requested='332 -5(weight9) +18 =345 >340',
     no_128_with_point_planes_all_requested='332 -5 -8(point planes) +18 =337; within340 hardware capacity but fails existing deployment policy<=336',
     no_128_policy_compliant_subset='332 -5 -8 +10(trig/bitmap/mailbox/slist) +6(bias/console) =335; omit column scratch2',
     conditional_128_requested='332 -51(SLF) -5(weight9) +18 =294;46 spare (not yet accepted)',
     conditional_128_plus_point_planes='294 -8 =286;54 spare',
     optional_other_SLF_lists='accept1 +candidate/result4 +extractor row6 =11 additional physical blocks',
     CLS='No measured per-candidate CLS saving. Logic totals/eligible local selectors must not be divided into a promised CLS reduction.'),
   proposed_review_batches=[
     dict(order=1,items=['weight9','capacity-preserving point planes'],physical_after=319,scope='Independent exact-numeric/SDP proofs on current40 before any large migration; no map-size semantic change.'),
     dict(order=2,items=['trig block4','bitmap1','two actual mailbox buffers4','slot list1'],physical_after=329,scope='Existing source-specific methods may be rebased; prove read-first, hold, INIT, bus latency and all simultaneous ports, then actual mapping/PNR.'),
     dict(order=3,items=['bias2','four console lanes4'],physical_after=335,scope='Only after previous measured results; omit column scratch2 to respect<=336 policy. Budget not routability evidence.'),
     dict(order='conditional',items=['SLF128'],physical_delta=-51,scope='Deferred to independent protocol/geometry/behavior contract; current degraded no-map result prevents assuming equivalence.')],
   evidence=dict(weight9='D:/Desktop/SOC FPGA/nav/docs/4_design/evidence/g2_20261023/weight9_synthesis_comparison.json',
     weight9_joint='D:/Desktop/SOC FPGA/nav/docs/4_design/evidence/g2_20261023/weight9_joint_synthesis_complete.json',
     weight9_primitive_checks='D:/Desktop/SOC FPGA/nav/docs/4_design/evidence/g2_20261023/weight9_netlist_functional.json',
     trig='D:/Desktop/SOC FPGA/nav/docs/4_design/evidence/g2_20261023/weight9_gyro17_trigblock_joint_synthesis.json',
     point_planes='D:/Desktop/SOC FPGA/nav/docs/4_design/evidence/g2_20261023/slf_point_planes_contract_20261024.json'),
   no_EDA=True,no_production_source_or_plan_changes=True,physical_CLS_saving='unknown',deployment_accepted=False)

def run(out=OUT):
 global OUT
 OUT=Path(out)
 OUT.mkdir(exist_ok=False)
 vg=BUILD/'impl/gwsynthesis/g2_top.vg';xml=BUILD/'impl/gwsynthesis/g2_top_syn_rsc.xml';pv=PNR/'g2_top.vg';rpt=PNR/'impl/pnr/g2_top.rpt.txt'
 watched=[vg,xml,rpt]+([pv] if pv.exists() else []);before=[meta(p) for p in watched]
 modules=dict(MODULE.findall(vg.read_text(encoding='utf-8')))
 @lru_cache(None)
 def children(kind):return [(t,n.lstrip('\\')) for t,n in INST.findall(modules[kind])]
 @lru_cache(None)
 def parameters(kind):
  result=defaultdict(dict)
  for name,key,value in PARAM.findall(modules[kind]):result[name.strip().lstrip('\\')][key]=value.strip()
  return result
 totals=Counter();groups={p:Counter() for p in PREFIXES};memory=[];direct_arrays=defaultdict(Counter)
 def walk(kind,path=''):
  for child,local in children(kind):
   name=(path+'/'+local).strip('/')
   if child in modules:walk(child,name);continue
   totals[child]+=1
   for prefix in PREFIXES:
    if name.startswith(prefix+'/'):groups[prefix][child]+=1
   if child in BSRAM or child.startswith(('RAM16','ROM16')):
    params=parameters(kind).get(local,{})
    init={k:v for k,v in params.items() if k.startswith('INIT')}
    memory.append(dict(name=name,kind=child,parent_module_kind=kind,
      parameters={k:v for k,v in params.items() if not k.startswith('INIT')},explicit_INIT_parameters=len(init)))
   if path in ('u_nav/u_map/u_tv','u_nav/u_match/u_lf','u_control/model_thread/policy/core'):
    match=re.match(r'(lb[1-4]|vc|dhist|bias_mem|elu_mem|input_mem|bank_a|bank_b)_',local)
    if match:direct_arrays[path+'/'+match[1]][child]+=1
 walk('g2_top')
 hierarchy={}
 def report(e,path=''):
  name=(path+'/'+e.attrib['name']).strip('/')
  hierarchy[name]=dict(e.attrib)
  for child in e:report(child,name)
 report(ET.parse(xml).getroot())
 relevant={k:v for k,v in hierarchy.items() if k=='g2_top' or any(k=='g2_top/'+p for p in PREFIXES)}
 memory_parents=defaultdict(Counter)
 for row in memory:memory_parents[row['name'].rsplit('/',1)[0]][row['kind']]+=1
 wide=[r for r in memory if r['kind'] in BSRAM and any(r['parameters'].get(k) in ('32','36') for k in ('BIT_WIDTH','BIT_WIDTH_0','BIT_WIDTH_1'))]
 result=dict(scope='Read-only completed SYN40 storage migration inventory; PNR52 frozen input only, no in-flight physical output',
   SYN40=str(BUILD),PNR52=str(PNR),new_EDA_started=False,physical_PNR52_CLS_and_timing='unknown',
   synthesis_hierarchy_report=relevant,leaf_totals=summary(totals),selected_leaf_groups={p:summary(c) for p,c in groups.items()},
   actual_direct_named_array_leaves={p:summary(c) for p,c in direct_arrays.items()},
   actual_memory_by_parent={p:dict(c) for p,c in memory_parents.items()},
   BSRAM_leaf_count=sum(c for k,c in totals.items() if k in BSRAM),wide32_36_source_leaves=len(wide),
   predicted_physical_BSRAM_if_wide_split1=sum(c for k,c in totals.items() if k in BSRAM)+len(wide),
   PNR52_input_VG_byte_identical_to_SYN40=same(vg,pv) if pv.exists() else None,
   inputs_before=before,inputs_metadata_unchanged=before==[meta(p) for p in watched],
   distinctions='Hierarchy T_Lut is the vendor report total. LUT_leaf/MUX_LUT_leaf/RAM16/ROM16 are separate primitive counts, not additive CLS savings. Predicted physical block split is not a new PNR measurement.')
 report_text=rpt.read_text(encoding='utf-8-sig')
 resource={}
 for label in ('Logic','Register','CLS','BSRAM'):
  m=re.search(r'^\s*'+label+r'\s*\|\s*(\d+)/(\d+)',report_text,re.M);assert m,label
  resource[label]=dict(used=int(m[1]),capacity=int(m[2]))
 m=re.search(r'(\d+)\((\d+) LUT, (\d+) ALU, (\d+) ROM16\)',report_text);assert m
 resource.update(LUT=int(m[2]),ALU=int(m[3]),ROM16=int(m[4]),RAM16=int(re.search(r'--SSRAM\(RAM16\)\s*\|\s*(\d+)',report_text)[1]),
   unrouted_nets=int(re.search(r'total (\d+) unrouted nets',report_text)[1]),TR_available=(PNR/'impl/pnr/g2_top.tr').exists(),report=str(rpt))
 assert resource['Logic']['used']==resource['LUT']+resource['ALU']+resource['ROM16']+6*resource['RAM16']
 result['physical_report']=resource;result['physical_PNR52_CLS_and_timing']='CLS measured; timing unknown without TR;1026 unrouted failure'
 save('inventory.json',result);save('memory_geometry.json',memory)
 save('budget.json',budget(result))
 source_patterns={
  'nav/rtl/common/nav_ram_sdp.v':['BAW=','DEPTH>','syn_ramstyle','if(INIT_FILE','if(re)','if(we)'],
  'nav/rtl/match/slf_match.v':['LOG2N  =','PB_AW =','DW(18)','DW(1)','DW(AW)','DW(32)','DW(36)','u_slist','u_sin','u_cos'],
  'nav/rtl/match/slf_lf.v':['LB =','CW =','reg [CW-1:0] lb'],
  'nav/rtl/map/trav_v3.v':['LB =','CW =','reg [CW-1:0] lb','u_bm','u_ct','vc [','dhist ['],
  'nav/rtl/net/host_mbx.v':['RX_AW  =','TX_WAW =','u_injb','u_rxb','u_txb','u_txs','DW(96)'],
  'nav/rtl/soc/nav_soc.v':['控制台环形','DW(8), .AW(10)','con_wp[1:0]','con_q;','con_ra <=','d_rdata <= d_ram'],
  'nav/rtl/plan/traj_eval.v':['u_sin','u_cos','正弦表'],
  'nav/rtl/match/slf_extract.v':['W = 160','RB =','DW(RB)'],
  'fc_v4_top.v':['bias_mem [','elu_mem [','BIAS_REQ) bias_q','ACT_WAIT) elu_q','$readmemh'],
  'fixed4500_weight_rom.v':['bank0 [','bank39 [']}
 source_lines={name:[dict(line=i,text=t.strip()) for i,t in enumerate((BUILD/name).read_text(encoding='utf-8-sig').splitlines(),1) if any(p in t for p in patterns)] for name,patterns in source_patterns.items()}
 save('source_lines.json',source_lines)
 shutil.copy2(Path(__file__),OUT/Path(__file__).name)
 print(json.dumps(dict(out=str(OUT),leaf_totals=result['leaf_totals'],BSRAM=result['BSRAM_leaf_count'],wide=result['wide32_36_source_leaves'],predicted_physical=result['predicted_physical_BSRAM_if_wide_split1'],PNR52_input_equal=result['PNR52_input_VG_byte_identical_to_SYN40'])))

if __name__=='__main__':
 parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,default=OUT);args=parser.parse_args();run(args.out)
