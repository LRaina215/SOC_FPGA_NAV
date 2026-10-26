"""Supplement a passed PNR51 ring review with full-VO fanout counts; no EDA."""
from pathlib import Path
from collections import Counter,defaultdict
import argparse,json,re,datetime
import review_model25_ring_pnr51_final as ring
import review_lease_tree_pnr50_final as flat
from synthesis_connectivity import read,DECL,TOKEN,PORT
from audit_routing_topology import primitive_directions

def run(review,out):
 proof=ring.load(review/'final_review.json');assert proof['passed']
 audit=ring.load(review/'actual_ring_primitives_and_cones.json')
 vo=ring.PNR/'impl/pnr/g2_top.vo';vg=ring.SYN/'impl/gwsynthesis/g2_top.vg'
 before=[ring.metadata(vo),ring.metadata(vg)]
 expected={x['path']:x for x in proof['bindings']}
 assert all(expected[x['path']]==x for x in before)
 anc={'','u_control','u_control/model_thread','u_control/model_thread/model',ring.OBS,ring.POL,ring.POL+'/core'}
 keep=lambda n:n in anc or n.rpartition('/')[0]in anc or n.startswith(ring.OBS+'/')
 source=read(vg,keep);flat.selected=keep
 actual,params,_,_=flat.parse_vo(vo,set(source));dirs=primitive_directions(ring.LIB)
 roles=defaultdict(set)
 selected={n:c for n,c in actual.items()if n.startswith(ring.OBS+'/')or n.rpartition('/')[0]==ring.POL}
 for n,c in selected.items():
  if c['kind'].startswith('DFF')and('g_ring.'in n or'/snapshot_word_'in n):
   for net in c['ports']['Q']:roles[net].add('slot_Q'if'g_ring.'in n else'prefetch_Q')
   for p in('CE','CLEAR','RESET','SET','PRESET'):
    for net in c['ports'].get(p,[]):roles[net].add('selected_FF_'+p)
  if c['kind']=='RAM16SDP4'and'g_ring.frame_mem_'in n:
   for p,ns in c['ports'].items():
    if p!='CLK':
     for net in ns:roles[net].add('ring_RAM_'+p)
  if c['kind']=='MULT12X12'and n.startswith(ring.OBS+'/'):
   for p,ns in c['ports'].items():
    if p!='CLK':
     for net in ns:roles[net].add('address_DSP_'+p)
 for net in('VCC','GND'):roles.pop(net,None)
 wanted=set(roles);counts=Counter();loads=defaultdict(list)
 text=vo.read_text(encoding='utf8')
 declarations={n.lstrip('\\'):(d,int(h)if h else None,int(l)if l else None)for d,h,l,n in DECL.findall(text)}
 def expand(expr):
  pos=0;result=[]
  while pos<len(expr):
   if not expr[pos:].strip():break
   m=TOKEN.match(expr,pos);assert m,('unsupported existing VO expression',expr[pos:]);pos=m.end()
   w,b,v,n,h,l,p=m.groups()
   if p or w:continue
   n=n.lstrip('\\')
   if h is not None:
    hi,lo=int(h),int(l or h);result +=[n+'['+str(i)+']'for i in range(hi,lo+(1 if lo>=hi else-1),1 if lo>=hi else-1)]
   elif n in declarations and declarations[n][1]is not None:
    _,hi,lo=declarations[n];result +=[n+'['+str(i)+']'for i in range(hi,lo+(1 if lo>=hi else-1),1 if lo>=hi else-1)]
   else:result.append(n)
  return result
 for kind,n,body in flat.CELL.findall(text):
  if kind=='module':continue
  n=n.lstrip('\\')
  for p,expr in PORT.findall(body):
   p=p.lstrip('\\')
   if dirs[kind][p]=='output':continue
   for index,net in enumerate(expand(expr)):
    if net in wanted:
     counts[net]+=1;loads[net].append(dict(instance=n,kind=kind,pin=p,bit_index_MSB_first=index))
 unique=sorted({(r['source_FF'],r['physical_copy'])for r in audit['proved_registered_copies']})
 copy_count=Counter(s for s,a in unique)
 resource=lambda cells:dict(FF=sum(c['kind'].startswith('DFF')for c in cells.values()),LUT=sum(bool(re.fullmatch(r'LUT[1-6]',c['kind']))for c in cells.values()),ALU=sum(c['kind']=='ALU'for c in cells.values()),RAM16SDP4=sum(c['kind']=='RAM16SDP4'for c in cells.values()),MULT12X12=sum(c['kind']=='MULT12X12'for c in cells.values()))
 source_local={n:c for n,c in source.items()if n.startswith(ring.OBS+'/')or n.rpartition('/')[0]==ring.POL}
 rows=[dict(net=n,roles=sorted(roles[n]),actual_input_pin_loads=counts[n],loads=loads[n])for n in sorted(wanted,key=lambda n:(-counts[n],n))]
 localrows=[r for r in rows if not any(x.startswith('selected_FF_')for x in r['roles'])and not any(x in('address_DSP_RESET','address_DSP_CE')for x in r['roles'])]
 assert all(ring.metadata(Path(x['path']))==x for x in before),'physical output changed during read'
 out.mkdir(parents=True,exist_ok=False)
 detail=dict(passed=True,full_VO_input_pin_scan=True,net_rows=rows,registered_copy_pairs=[dict(source=s,physical=a)for s,a in unique],proved_copy_counts_by_source=dict(copy_count),proof_review=str(review/'final_review.json'))
 ring.save(out/'full_VO_fanout_and_copies.json',detail)
 summary=dict(passed=True,recorded_at=datetime.datetime.now().astimezone().isoformat(),routing_passed=False,unrouted=1288,no_TR=True,source_selected_resources=resource(source_local),physical_selected_resources=resource(selected),proved_unique_registered_copies=len(unique),copy_references=len(audit['proved_registered_copies']),ALU_mode0_input_swaps=len(audit['proved_ALU_mode0_input_swaps']),copy_counts_by_source=dict(copy_count),target_nets_counted=len(wanted),largest_actual_local_ring_data_address_fanouts=[{k:v for k,v in r.items()if k!='loads'}for r in localrows[:15]],largest_including_CE_reset=[{k:v for k,v in r.items()if k!='loads'}for r in rows[:10]],logical_pin_load_counts_only_not_route_delay_or_failed_net_identity=True,proof_review=str(review/'final_review.json'),bindings=before)
 ring.save(out/'summary.json',summary);print(json.dumps(summary,ensure_ascii=False));return summary

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--review',type=Path,default=ring.BASE/'pnr51_ring_final_actual_review2');p.add_argument('--out',type=Path,default=ring.BASE/'pnr51_ring_physical_metrics1');a=p.parse_args();run(a.review,a.out)
