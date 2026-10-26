"""Archive the same-ELF firmware export and final frozen source binding."""
from pathlib import Path
from datetime import datetime
import argparse,json
import psutil
from prepare_baseline_fusion_retime import metadata,same_bytes,save
BASE=Path('F:/nav_integration_g2_20261023');ROOT=Path(__file__).resolve().parents[2]
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def collect():
 package=BASE/'stage2_SLF128_safeFW_RV_finalize2';candidate=BASE/'stage3_SLF128_storage_RTL_prepared5';proof=load(package/'result.json');bound=load(candidate/'firmware_binding_review.json')
 assert proof['passed']and bound['all11_CPU_assets_consistent']and bound['source63_and_all_FW_unchanged']
 for b in proof['strict_all_input_bindings']+proof['output_bindings']+bound['complete_frozen_input_bindings']:assert metadata(Path(b['path']))==b,b
 strict_path=BASE/'stage2_SLF128_safeFW_RV_compile1_strict_stack_review2/result.json';strict=load(strict_path)
 assert strict['complete']and strict['passed']and not strict['unresolved_indirect']and strict['all_SU_and_ltrans_included']
 identities=[]
 def exited(pid,created):
  try:alive=abs(psutil.Process(pid).create_time()-created)<.01
  except psutil.NoSuchProcess:alive=False
  assert not alive,(pid,created);identities.append(dict(pid=pid,created_epoch=created,original_identity_exited=True))
 exited(proof['controller_pid'],proof['controller_created_epoch'])
 for rt in proof['runtimes']:
  assert rt['exit_code']==0 and rt['ended_at'];exited(rt['pid'],rt['created_epoch'])
 # The replayer uses Windows text newlines; the archived component uses LF.
 # Compare complete decoded text with only universal-newline normalization.
 assert(candidate/'nav/rtl/common/nav_async_fifo.v').read_text(encoding='utf8')==(BASE/'fifo_async_assert_candidate3/nav_async_fifo.v').read_text(encoding='utf8')
 assert(candidate/'nav/rtl/net/rgmii_mac100.v').read_text(encoding='utf8')==(BASE/'mac_rx_boundary_candidate2/rgmii_mac100.v').read_text(encoding='utf8')
 assert "assign i_IMU_RX_DATA  = iu_valid ? {1'b1, 22'd0, iu_first, iu_data} : 32'd0;"in(candidate/'nav/rtl/top/nav_subsystem.v').read_text()
 assert(candidate/'run.tcl').read_text().lstrip().startswith('error')
 result=dict(recorded_at=datetime.now().astimezone().isoformat(),passed=True,scope='same linked ELF strict stack review, original objcopy/INIT export and source-only combined binding; no new joint synthesis',candidate=str(candidate),manifest=metadata(candidate/'manifest.json'),final_input_count=len(bound['complete_frozen_input_bindings']),full_bindings_file=metadata(candidate/'firmware_binding_review.json'),firmware_package=str(package),BIN_bytes=proof['image_bytes'],BSS_bytes=proof['BSS_bytes'],static_stack_bound_bytes=proof['static_stack_bound_bytes'],reserved_stack_bytes=proof['stack_reserved_bytes'],linked_symbols=proof['linked_symbols'],remaining_RAM_after_BSS_and_reserved_stack_bytes=proof['remaining_after_6KiB_stack_bytes'],CPU_reconstruction=dict(image_bytes=131072,each_byte_lane_entries=32768,all4_lanes_and_word_image_exact=True,remaining_bytes_all_zero=True,changed_CPU_assets=bound['CPU_assets_changed']),strict_stack=dict(report=metadata(strict_path),complete=True,all_LTO_partitions_SU_and_assembly_included=True,actual_local_jr=strict['actual_jr_count'],actual_jalr=strict['actual_jalr_count'],all_unknown_indirect_rejected=True,accepted_new_switch='Bounded actual CFG/reaching-def through callee-saved s4, unsigned guard dominates scale4/lw/jr; actual ELF read-only table targets all local legal instruction boundaries. No fixed-address whitelist.',original_incomplete_stack_report_preserved=str(BASE/'stage2_SLF128_safeFW_RV_compile1/firmware/stack_analysis.json')),original_process_identities=identities,source63_and_FW_unchanged_from_storage3=True,safe8_repairs_preserved=True,first_binding_preparation_failure_preserved=str(BASE/'stage3_SLF128_storage_RTL_prepared4/binding_preparation_failure.json'),old_run_entry_harderror=True,combined128_RTL_regression_still_required=True,not_proven=proof['not_proven'],SYN_started=False,PNR_started=False,programmed=False)
 target=ROOT/'docs/4_design/evidence/g2_20261023/stage2_SLF128_same_ELF_FW_and_final_binding_20261026.json';save(target,result);print(json.dumps(dict(passed=True,result=str(target),candidate=str(candidate),image_bytes=proof['image_bytes']),ensure_ascii=False));return result
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--collect',action='store_true');a=p.parse_args();assert a.collect;collect()
