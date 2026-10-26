"""Read-only reconstruction of SYN34's reported age-to-command cone.

This audits synthesis primitives and estimates, never a routed failure-net
identity, congestion cause, or physical timing signoff.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,re
from bs4 import BeautifulSoup
from prepare_baseline_fusion_retime import metadata,save

def inspect(syn,out):
 out.mkdir(parents=True,exist_ok=False)
 source=syn/'command_lease_guard.v';vg=syn/'impl/gwsynthesis/g2_top.vg';report=syn/'impl/gwsynthesis/g2_top_syn.rpt.html'
 text=vg.read_text();start=text.index('module command_lease_guard (');text=text[start:];text=text[:text.index('endmodule')]
 cells={};drivers={}
 for m in re.finditer(r'^\s*(\w+) (\S+) \(([\s\S]+?)\);',text,re.M):
  ports=dict(re.findall(r'\.(\w+)\(([^\n]+?)\)',m[3]));params={}
  for p,v in re.findall(r'defparam '+re.escape(m[2])+r'\.(\w+)=([^;]+);',text):params[p]=v
  row=dict(type=m[1],name=m[2],ports=ports,params=params);cells[m[2]]=row
  if 'F'in ports:drivers[ports['F']]=row
 visited={}
 def and_leaves(net):
  if re.fullmatch(r'age_ms\[(?:[0-9]|[12][0-9]|3[01])\]',net):return {net},0
  c=drivers[net];width=int(c['type'].removeprefix('LUT'))
  match=re.fullmatch(r"\d+'h([0-9a-fA-F]+)",c['params']['INIT']);assert match
  assert int(match[1],16)==1<<((1<<width)-1),(c['name'],c['params'])
  leaves=set();depth=0
  for i in range(width):
   sub,levels=and_leaves(c['ports']['I'+str(i)]);leaves|=sub;depth=max(depth,levels)
  visited[c['name']]=c;return leaves,depth+1
 leaves,levels=and_leaves(cells['age_ms_31_s6']['ports']['F'])
 assert leaves=={'age_ms['+str(i)+']'for i in range(32)}and levels==12
 html=report.read_text();part=html.split('Path Summary:',1)[1]
 values=[x.get_text(' ',strip=True)for x in BeautifulSoup(part.split('</table>',1)[0],'html.parser').find_all('td')]
 summary=dict(zip(values[::2],values[1::2]));assert summary['From']=='u_command_lease/age_ms_0_s3'and summary['To']=='u_command_lease/out_forward_q12_2_s1'
 arrival=part.split('Data Arrival Path:',1)[1].split('Data Required Path:',1)[0]
 rows=[]
 for tr in BeautifulSoup(arrival,'html.parser').find_all('tr'):
  row=[x.get_text(' ',strip=True)for x in tr.find_all('td')]
  if len(row)==6:rows.append(dict(zip(('arrival_ns','delay_ns','type','edge','fanout','node'),row)))
 assert rows and any(r['node']=='u_command_lease/age_ms_31_s6/F'and r['arrival_ns']=='10.843'for r in rows)
 names=['age_ms_0_s3','n1007_s3','n1004_s4','n1001_s4','n998_s4','n996_s4','n995_s3','n992_s4','n991_s3','n987_s3','n983_s3','n980_s4','age_ms_31_s6','n4094_s69','n4094_s4','n590_s3','n641_s','n628_s','n1099_s10','n1099_s9','n1099_s5','n1099_s4','n1098_s3','n1097_s3','n1097_s2','out_forward_q12_2_s1']
 result=dict(recorded_at=datetime.now().astimezone().isoformat(),read_only=True,source_synthesis=str(syn),
  scope='Actual SYN34 netlist/report structure and estimated delay only; not PNR49 actual critical path, unrouted-net identity or unique congestion cause.',
  source_anchors=[dict(line=i,text=line.strip())for i,line in enumerate(source.read_text().splitlines(),1)
   if any(t in line for t in['wire deadline=','wire stop_mode=','out_forward_q12<=approach','if(changing)','else if(estop)','else if(source_fault || deadline'])],
  first_reported_path=summary,arrival_path_rows=rows,
  actual_all_ones_cone=dict(root='age_ms_31_s6/F',function='AND of exactly age_ms[31:0]',LUT_logic_levels=levels,
   leaves=sorted(leaves),actual_AND_primitives=list(visited.values()),Q_arrival_ns=.715,AND_root_arrival_ns=10.843,
   elapsed_Q_to_all_ones_ns=10.128),
  actual_path_primitives=[cells[n]for n in names],
  interpretation='32bit increment/wrap all-ones cone reaches deadline/stop selection, then target selection before ramp subtraction/comparison/output selection. Source deadline rewrite aims to remove increment arithmetic from this decision; balancing or reducing its new mapping is not established.',
  minimum_candidate='Only rewrite default200ms age+1>=200 as exact32bit range >=199 AND age!=FFFFFFFF, with timeout0 retaining original arithmetic; no count/ramp/register/clock/priority edits.',
  mapped_improvement_verified=False,PNR49_modified=False,new_SYN_PNR_started=False,
  bindings=[metadata(source),metadata(vg),metadata(report)])
 save(out/'result.json',result);return result

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--syn',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
 result=inspect(a.syn,a.out);print(json.dumps(dict(result=str(a.out/'result.json'),first_reported_path=result['first_reported_path'],AND_levels=result['actual_all_ones_cone']['LUT_logic_levels']),indent=2))
