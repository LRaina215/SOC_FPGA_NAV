"""Read completed frozen stage2B RTL contracts; no simulator or EDA launch."""
from pathlib import Path
from datetime import datetime
import argparse,json,re,psutil
from prepare_baseline_fusion_retime import metadata

BASE=Path('F:/nav_integration_g2_20261023')
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))

def original_identity(pid,created):
 try:
  q=psutil.Process(pid);now=q.create_time()
  same=abs(now-created)<.01
 except(psutil.NoSuchProcess,psutil.AccessDenied):now=None;same=False
 return dict(pid=pid,original_created_epoch=created,current_created_epoch=now,original_identity_exited=not same)

def collect(profile):
 candidate=BASE/('baseline_fusion50_'+profile+'_prepared1')
 folder=BASE/('stage2B_'+profile+'_contract_probe1')
 r=load(folder/'result.json');f=load(candidate/'prepared_source_review.json')
 assert r['passed']and r['phase']=='RTL_complete'
 assert all(t['exit_code']==0 for t in r['runtimes'])
 for row in f['bindings']+f['validation_bindings']:assert metadata(Path(row['path']))==row
 text=(folder/'xsim.log').read_text(errors='replace')
 assert 'TEST PASSED'in text and not re.search(r'\bFatal:',text)
 markers=[x for x in text.splitlines()if x.startswith(('TEST PASSED','WEIGHT9 SCAN','POINT PASS','$finish called'))]
 identities=[original_identity(r['controller_pid'],r['controller_created_epoch'])]
 for t in r['runtimes']:
  identities.append(original_identity(t['wrapper_pid'],t['wrapper_created_epoch']))
  identities += [original_identity(x['pid'],x['created_epoch'])for x in t.get('processes',[])]
 assert all(x['original_identity_exited']for x in identities)
 tools=[dict(tool=t['tool'],command=t['command'],wrapper_pid=t['wrapper_pid'],wrapper_created_epoch=t['wrapper_created_epoch'],started_at=t['started_at'],ended_at=t['ended_at'],exit_code=t['exit_code'],admission_physical_bytes=t['admission_physical_bytes'],admission_commit_bytes=t['admission_commit_bytes'],observed_processes=t.get('processes',[]))for t in r['runtimes']]
 warnings=[]
 for p in sorted(folder.glob('*.log')):
  for line in p.read_text(errors='replace').splitlines():
   if 'WARNING:'in line or 'CRITICAL WARNING:'in line:warnings.append(dict(log=str(p),line=line))
 if profile=='weight9':
  scan=re.search(r'WEIGHT9 SCAN reads=(\d+) holds=(\d+)',text);assert scan
  assert int(scan[1])==40960 and int(scan[2])==3151
  assert '3 frames,1275 neurons,cycle-exact actions'in text
  counts=dict(legal_ROM_word_reads=40960,held_address_changes=3151,real4500_frames=3,real4500_neuron_values=1275,trained_weights_preserved=40704,zero_padded_words_preserved=256,explicit_unknown_rejection=True,cycle_exact_status_actions=True,inference_substitute=False,scheduler_125Hz_not_exercised_in_this_core_fixture=True)
 else:
  rows=re.findall(r'POINT PASS IV=([0-9a-fA-F]+) checks=(\d+) reads=(\d+) writes=(\d+) collisions=(\d+) holds=(\d+)',text);assert len(rows)==2
  counts=[]
  for iv,*values in rows:
   values=list(map(int,values));assert values==[28682,20469,8145,4098,8213]
   counts.append(dict(INIT_VAL='36h'+iv,cycles=values[0],reads=values[1],writes=values[2],same_address_read_write_collisions=values[3],read_enable_holds=values[4],all4096_addresses_covered=True,independent_scoreboard_read_first=True,explicit_unknown_rejection=True))
 return dict(profile=profile,passed=True,candidate=str(candidate),actual_result=str(folder/'result.json'),actual_xsim_log=str(folder/'xsim.log'),markers=markers,checks=counts,tools=tools,original_process_identities=identities,frozen_source_and_fixture_metadata_unchanged=True,warnings=warnings,scope='Finite actual RTL and reference comparisons only; no new vendor netlist, fullG2, physical timing or board acceptance')

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args()
 rows=[collect(x)for x in ['weight9','pointplanes']]
 assert rows[0]['tools'][-1]['ended_at']<rows[1]['tools'][0]['started_at']
 result=dict(recorded_at=datetime.now().astimezone().isoformat(),passed=True,scope='Stage2B packedweight9 then pointplanes finite RTL contracts',profiles=rows,simulations_sequential_one_at_a_time=True,minimum_admission_physical_bytes=2*1024**3,minimum_admission_commit_bytes=4*1024**3,no_frozen_RTL_TB_or_assets_modified=True,no_retry=True,no_Gowin_synthesis_or_PNR_started=True,no_new_firmware_build=True,programmed=False,component_contracts_only=True,new_combined_resource_mapping_and_physical_acceptance_pending=True,physical_BSRAM319_is_prediction=True,no_SHA_used=True)
 a.out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
 print(json.dumps(dict(passed=True,output=str(a.out),profiles=[dict(profile=x['profile'],passed=x['passed'],warnings=len(x['warnings']))for x in rows]),ensure_ascii=False))
