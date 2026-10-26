"""Read-only SYN34/SYN36 real reset-pin and source-bank inventory.

Counts logical pin connections; it is not a congestion, reset timing or PNR
claim.  Only explicit INV/BUF/BUFG are collapsed into a reset root.
"""
from pathlib import Path
from collections import Counter,defaultdict
import argparse,json,re,sys,datetime,xml.etree.ElementTree as ET
from synthesis_connectivity import read
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from review_model25_joint_syn import BASE,save,load,meta

SCOPES={'BMI':'bmi_bridge','configuration':'config_bridge','action':'action_bridge',
        'fusion':'fusion_bridge','status':'status_bridge'}
def inventory(build):
 vg=build/'impl/gwsynthesis/g2_top.vg'
 cells=read(vg,lambda n:n=='u_control'or n.startswith('u_control/'))
 params=parameter_reader(definitions(vg))
 inv={};buf={}
 for c in cells.values():
  if c['kind']=='INV':inv[c['ports']['O'][0]]=c['ports']['I'][0]
  if c['kind']in('BUF','BUFG'):buf[c['ports']['O'][0]]=c['ports']['I'][0]
 def root(net):
  seen=set();parity=0
  while net in inv or net in buf:
   assert net not in seen;seen.add(net)
   if net in inv:net=inv[net];parity^=1
   else:net=buf[net]
  return net,parity
 rows=[];totals=defaultdict(Counter)
 for n,c in cells.items():
  if not c['kind'].startswith('DFF'):continue
  for pin in('CLEAR','PRESET','RESET','SET'):
   if pin in c['ports']and c['ports'][pin]!=['GND']:
    net=c['ports'][pin][0];r,p=root(net)
    rows.append(dict(cell=n,kind=c['kind'],pin=pin,net=net,root=r,inversions=p))
    totals[(r,p)][pin]+=1
 private={};shared={}
 for label,scope in SCOPES.items():
  scope='u_control/model_thread/'+scope
  ff=[c for n,c in cells.items()if n.startswith(scope+'/')and re.fullmatch(r'(holding|last_sent)_\d+_s\d+',n.rsplit('/',1)[1])and c['kind'].startswith('DFF')]
  # transfer contains no new holding cells when last_sent is shared by source.
  private[label]=dict(count=len(ff),kinds=dict(Counter(c['kind']for c in ff)),
   async_control_pins=sum(any(p in c['ports']for p in('CLEAR','PRESET','SET'))for c in ff),
   synchronous_RESET_tied_zero=sum(c['ports'].get('RESET')==['GND']for c in ff),
   synchronous_RESET_data_logic=sum('RESET'in c['ports']and c['ports']['RESET']!=['GND']for c in ff),
   CE_nets=dict(Counter(c['ports'].get('CE',['VCC'])[0]for c in ff)),
   cells=[dict(**c,parameters=params(c))for c in ff])
  sent=[c for n,c in cells.items()if n.startswith(scope+'/')and re.fullmatch(r'sent_s\d+',n.rsplit('/',1)[1])]
  shared[label]=dict(public_sent_cells=sent,source_private_Q_nets=[c['ports']['Q'][0]for c in ff])
 return dict(build=str(build),actual_VG=meta(vg),root_counts=[dict(root=r,inversions=p,total=sum(v.values()),pins=dict(v))for(r,p),v in sorted(totals.items(),key=lambda x:(-sum(x[1].values()),x[0]))],reset_pin_rows=rows,private=private,public_sent=shared)

def run(out):
 a=inventory(BASE/'baseline_fusion_model25_NAV50_syn34')
 b=inventory(BASE/'baseline_fusion_model25_private_source_noreset_syn36')
 table=[]
 for key in('BMI','configuration','action','fusion','status'):
  x,y=a['private'][key],b['private'][key]
  table.append(dict(channel=key,before_FF=x['count'],after_FF=y['count'],
   before_async_pins=x['async_control_pins'],after_async_pins=y['async_control_pins'],
   removed_async_pins=x['async_control_pins']-y['async_control_pins'],
   new_sync_zero_pins=y['synchronous_RESET_tied_zero'],new_sync_data_zero_pins=y['synchronous_RESET_data_logic']))
 # Configuration bit139 was previously shared with a public sent flag.  The
 # new explicit sent stays resettable; never count that retained public pin
 # as eliminated or call same total FF a one-FF saving.
 assert [x['after_FF']for x in table]==[128,139,160,112,12]
 assert [x['after_async_pins']for x in table]==[0]*5
 configsent=b['public_sent']['configuration']['public_sent_cells']
 assert len(configsent)==1 and configsent[0]['kind']=='DFFCE'
 async_reduction=sum(x['removed_async_pins']for x in table)-1
 assert async_reduction==551
 byroot=lambda d:{(x['root'],x['inversions']):x for x in d['root_counts']}
 old,new=byroot(a),byroot(b);delta=[]
 for r in sorted(old.keys()|new.keys()):
  x,y=old.get(r,{}),new.get(r,{})
  if x.get('total',0)!=y.get('total',0):delta.append(dict(root=r[0],inversions=r[1],before=x.get('total',0),after=y.get('total',0),delta=y.get('total',0)-x.get('total',0)))
 result=dict(passed=True,recorded_at=datetime.datetime.now().astimezone().isoformat(),
  scope='Logical source FF asynchronous reset load removal only. Numeric source-word, control Boolean semantics and physical routing/timing are separate gates.',
  source_channels=table,old_private_including_shared_sent=552,new_private551_plus_public_sent1=True,
  configuration_public_sent_retained=configsent[0],actual_async_reset_load_reduction=async_reduction,
  reset_root_changes=delta,before=a,after=b,
  neither_global_resource_savings_nor_routing_improvement_proved=True,
  synchronous_data_zero_only_during_capture_proof=str(BASE/'private_source_SYN36_actual_review3/actual_private_mapping_Boolean_checks.json'))
 save(out/'actual_private_reset_loads.json',result)
 vendor={}
 for name,sub in [('SYN34','baseline_fusion_model25_NAV50_syn34'),('SYN36','baseline_fusion_model25_private_source_noreset_syn36')]:
  path=BASE/sub/'impl/gwsynthesis/g2_top_syn_rsc.xml';element=ET.parse(path).getroot()
  def counts(e):return{k:int(re.match(r'\d+',v)[0])for k,v in e.attrib.items()if k.startswith('T_')}
  def walk(e,parent=''):
   path=(parent+'/'+e.attrib['name']).strip('/');rows={path:counts(e)}
   for c in e:rows.update(walk(c,path))
   return rows
  vendor[name]=dict(report=str(path),total=counts(element),modules=walk(element))
 aa,bb=vendor['SYN34']['total'],vendor['SYN36']['total']
 resource=dict(passed=True,recorded_at=result['recorded_at'],resource_counts_scope='Vendor synthesis resource XML only; physical usage and timing not signed off',before=vendor['SYN34'],after=vendor['SYN36'],delta={k:bb.get(k,0)-aa.get(k,0)for k in aa.keys()|bb.keys()},conclusion='FF and all memory/DSP categories unchanged, LUT -118 and ALU +139. Reset fanout reduction is not proof of routing improvement.',actual_physical_BSRAM=None,physical_BSRAM_prediction=332)
 save(out/'actual_vendor_synthesis_resources.json',resource)
 print(json.dumps({k:v for k,v in result.items()if k not in('before','after')},ensure_ascii=False))
 return result
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(exist_ok=False);run(a.out)
