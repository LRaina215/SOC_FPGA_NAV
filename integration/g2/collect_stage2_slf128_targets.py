"""Collect completed limited proofs; never starts or terminates a process."""
from pathlib import Path
from datetime import datetime
import argparse,json,re
import psutil
from prepare_baseline_fusion_retime import metadata,same_bytes,save
BASE=Path('F:/nav_integration_g2_20261023');ROOT=Path(__file__).resolve().parents[2]
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def collect():
 cases=[BASE/x for x in ['stage2_SLF128_frontend_probe1','stage2_SLF128_bitwise_probe2','stage2_SLF256_bitwise_control_probe1','stage2_SLF128_FW_components_probe2','stage2_SLF128_forward_HAL_math_probe1']]
 records=[];identities={}
 def add(pid,epoch,role):
  key=(int(pid),float(epoch));identities[key]=dict(pid=key[0],created_epoch=key[1],role=role)
 for d in cases:
  r=load(d/'result.json');assert r['passed'],d
  if'controller_pid'in r:add(r['controller_pid'],r['controller_created_epoch'],str(d)+' controller')
  rt=r.get('runtimes',[])
  for run in rt:
   assert run['exit_code']==0 and run['ended_at'],run
   if'wrapper_pid'in run:add(run['wrapper_pid'],run['wrapper_created_epoch'],str(d)+' wrapper')
   if'pid'in run:add(run['pid'],run['created_epoch'],str(d)+' tool')
   for p in run.get('processes',[]):add(p['pid'],p['created_epoch'],p.get('name','descendant'))
   assert run['admission_physical_bytes']>=2*1024**3 and run['admission_commit_bytes']>=4*1024**3
  logs=[d/'xsim.log']if(d/'xsim.log').exists()else[d/'execute.log']
  for p in logs:
   text=p.read_text(encoding='utf8',errors='replace');assert re.search(r'TEST PASSED',text,re.I),p
   assert not re.search(r'Fatal:|\$fatal|TEST FAILED',text,re.I),p
  records.append(dict(output=str(d),result=metadata(d/'result.json'),passed=True,counts=r.get('counts'),markers=r.get('markers'),proof_scope=r.get('not_proven',['literal frontend or isolated matcher RTL only; not whole G2/CPU/physical timing'])))
 for name in ['stage2_SLF128_targeted_validation1','stage2_SLF128_targeted_validation2','stage2_SLF128_FW_components_probe1','stage2_SLF128_safeFW_RV_compile1']:
  p=BASE/name/'job.json'
  if p.exists():
   r=load(p)
   if'controller_pid'in r:add(r['controller_pid'],r['controller_created_epoch'],name+' controller')
 for p in identities.values():
  try:
   q=psutil.Process(p['pid']);alive=abs(q.create_time()-p['created_epoch'])<0.01
  except psutil.NoSuchProcess:alive=False
  p['original_identity_exited']=not alive;assert not alive,p
 a=BASE/'stage2_SLF128_forward_safeFW_prepared7';b=BASE/'stage2_SLF128_forward_safeFW_prepared8'
 unchanged=['nav/rtl/common/nav_ram_sdp.v','nav/rtl/match/slf_extract.v','nav/rtl/match/slf_lf.v','nav/rtl/match/slf_match.v']
 assert all(same_bytes(a/n,b/n)for n in unchanged)
 assert all(same_bytes(p,b/p.relative_to(a))for p in(a/'fw').rglob('*')if p.is_file())
 priorfail=load(BASE/'stage2_SLF128_targeted_validation1/job.json');assert priorfail['phase']=='failed_preserved_no_retry'
 out=dict(recorded_at=datetime.now().astimezone().isoformat(),passed=True,stage='limited stage2 RTL/C contracts; not joint synthesis or deployment',production_SLF_LOG2N=7,SLF256_only_validation_control=True,records=records,original_process_identities=list(identities.values()),all_original_tools_exited=True,all_tools_admitted_2GiB_physical_4GiB_commit=True,first_fixture_failures_preserved=[dict(path=str(BASE/'stage2_SLF128_bitwise_probe1'),reason='Windows .bat generic argument split before elaboration; no RTL simulation entered; explicit fixed wrapper in probe2 only'),dict(path=str(BASE/'stage2_SLF128_FW_components_probe1'),reason='native PE linker retained unrelated nav_app functions; missing full-app dependencies, no test run; probe2 uses literal public-setter cut with explicit substitutes')],reference_source=str(a),safety_source=str(b),matcher_inputs_and_all_FW_identical_between7_8=True,safety_rebase=metadata(b/'safety_rebase_review.json'),target_matcher_scope=dict(frames=5,candidates_per_frame=245,all_map_words_per_frame_128=16384,all_map_words_per_frame_256=65536,full_CLEAR_including_highest_slot=True,prior_scoring_and_UPDATE_hold=True,LF_centers_changed_after_UPDATE_accept_and_snapshot_preserved=True,negative_world_coordinates_wrap_tags=True),remaining=['whole new safe FW on actual Pico with populated map/busy A-star and RK feedback/stop/independent ESTOP','new CPU INIT reconstructed and bound into the final storage candidate','unique joint actual memory primitives/INIT/CEB/reset/2 clocks/CDC/fanout/timing/IO','all target clocks setup/hold, successful routing and board/physical stopping'],SYN_started=False,PNR_started=False,programmed=False)
 target=ROOT/'docs/4_design/evidence/g2_20261023/stage2_SLF128_targeted_contracts_20261026.json';save(target,out);print(json.dumps(dict(passed=True,result=str(target),completed_proof_groups=len(records),original_process_identities=len(identities)),ensure_ascii=False));return out
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--collect',action='store_true');a=p.parse_args();assert a.collect;collect()
