"""Read-only archive of the FIFO syntax correction, C mapping and source6."""
from pathlib import Path
from datetime import datetime
import argparse,json
import psutil
from prepare_baseline_fusion_retime import metadata,same_bytes,save
BASE=Path('F:/nav_integration_g2_20261023');ROOT=Path(__file__).resolve().parents[2]
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def collect():
 candidate=BASE/'stage3_SLF128_storage_RTL_prepared6';fixture=BASE/'stage3_FIFO_async_reset_syntax_candidate1';rtlpath=BASE/'stage3_FIFO_async_reset_syntax_RTL_probe1/result.json';cpath=BASE/'stage3_FIFO_async_reset_syntax_small_SYN_review3/result.json';repairpath=BASE/'stage3_FIFO_manifest_binding_repair1/result.json'
 rtl=load(rtlpath);mapped=load(cpath);repaired=load(repairpath);assert rtl['passed']and mapped['passed']and repaired['passed']
 bound=load(candidate/'firmware_binding_review.json')
 for b in bound['complete_frozen_input_bindings']:assert metadata(Path(b['path']))==b,b
 assert same_bytes(candidate/'nav/rtl/common/nav_async_fifo.v',fixture/'nav_async_fifo.v')
 assert mapped['device_version']=='C'and mapped['EX3534_absent']and mapped['one_SYN_only']and mapped['synthesis_exit_code']==0
 assert mapped['converted_pointer_bits_checked']==74 and mapped['synchronizer_bits_checked']==148 and mapped['occupancy_bits_checked']==9
 assert all(g['all_DFFCE_reset_clock_roots_correct']and g['payload_RESET_all_tied_GND']for g in mapped['group_summaries'].values())
 ids=[]
 def exited(pid,epoch,role):
  try:alive=abs(psutil.Process(pid).create_time()-epoch)<.01
  except psutil.NoSuchProcess:alive=False
  assert not alive,(pid,epoch,role);ids.append(dict(pid=pid,created_epoch=epoch,role=role,original_identity_exited=True))
 exited(rtl['controller_pid'],rtl['controller_created_epoch'],'RTL controller')
 for rt in rtl['runtimes']:
  assert rt['exit_code']==0;exited(rt['wrapper_pid'],rt['wrapper_created_epoch'],rt['tool'])
  for p in rt.get('processes',[]):exited(p['pid'],p['created_epoch'],p['name'])
 old=load(BASE/'stage3_FIFO_async_reset_syntax_small_SYN_prepared1/job.json');assert old['runtime']['exit_code']==0
 exited(old['controller_pid'],old['controller_created_epoch'],'small C SYN controller')
 for p in old['runtime'].get('processes',[]):exited(p['pid'],p['created_epoch'],p['name'])
 out=dict(recorded_at=datetime.now().astimezone().isoformat(),passed=True,scope='Only repair actual C frontend EX3534; not whole G2 resource or timing acceptance',source=str(candidate),only_changed_RTL_from_prepared5=['nav/rtl/common/nav_async_fifo.v'],two_always_only=True,change='Canonical outer if(reset)clear; else retains original reset?0:g2b. Same sensitivity/nonblocking assignments, with original four-state merge retained.',source4state_identity_cases=16,RTL_result=metadata(rtlpath),RTL_profiles=rtl['metrics'],total_edges=sum(x['edges']for x in rtl['metrics']),total_writes=sum(x['writes']for x in rtl['metrics']),total_reads=sum(x['reads']for x in rtl['metrics']),actual_C_result=metadata(cpath),actual_C_exit_code=0,actual_C_EX3534_absent=True,actual_C_FF=mapped['FF_total'],actual_C_async_CLEAR_FF=mapped['async_clear_DFFCE'],actual_C_resetless_payload_FF=mapped['resetless_payload_DFFRE'],actual_C_BSRAM=mapped['BSRAM_total'],actual_C_checked_converted_bits=74,actual_C_checked_sync_bits=148,actual_C_checked_occupancy_bits=9,actual_C_RAM_scope='All explicit RX11/TX11 primitive parameters equal available SYN40; RX12 three width4 SDPB leaves have no old actual comparison because full G2 pruned the unused host input.',metadata_repair=metadata(repairpath),metadata_repair_scope='Exactly one stale manifest row in four proof arrays, caused by recording it before the expected manifest update; 247 other inputs and all RTL/FW/assets unchanged, all old JSON preserved. Only corresponding proof references updated.',final_binding=metadata(candidate/'firmware_binding_review.json'),complete_input_count=len(bound['complete_frozen_input_bindings']),all_old_original_process_identities_exited=ids,first_joint_failure_preserved=str(BASE/'SLF128_storage_safeFW_syn41'),all_small_reader_failures_preserved=[mapped['original_parser_failure_preserved'],mapped['overstrict_review2_preserved']],new_joint_SYN42_owned_by_root=str(BASE/'SLF128_storage_safeFW_syn42'),whole_G2_accepted=False,not_proven=['new full system primitive modes/CEB/OCE/AD, RAM/CPU INIT and resources','new actual clocks/reset loads/original26 CDC/52 effective constraints','physical reset pulse/recovery-removal/CDC/MTBF and IO','routed0/setup0/hold0 and board/real RK busy stop/physical stopping'],PNR_started_by_this_task=False,programmed=False)
 path=ROOT/'docs/4_design/evidence/g2_20261023/stage3_FIFO_reset_syntax_correction_20261026.json';save(path,out);print(json.dumps(dict(passed=True,source=str(candidate),result=str(path),old_process_identities=len(ids)),ensure_ascii=False));return out
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--collect',action='store_true');a=p.parse_args();assert a.collect;collect()
