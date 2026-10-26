"""New final composition: only the proved FIFO syntax differs from prepared5."""
from pathlib import Path
from datetime import datetime
import argparse,json,shutil
from prepare_baseline_fusion_retime import metadata,relocate,same_bytes,save
BASE=Path('F:/nav_integration_g2_20261023');SRC=BASE/'stage3_SLF128_storage_RTL_prepared5';OUT=BASE/'stage3_SLF128_storage_RTL_prepared6';FIX=BASE/'stage3_FIFO_async_reset_syntax_candidate1';FIFO='nav/rtl/common/nav_async_fifo.v'
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def prepare():
 assert not OUT.exists();bound=load(SRC/'firmware_binding_review.json');review=load(SRC/'prepared_source_review.json');proof=load(BASE/'stage3_FIFO_async_reset_syntax_RTL_probe1/result.json');assert proof['passed']
 for b in bound['complete_frozen_input_bindings']:assert metadata(Path(b['path']))==b,b
 fix_review=load(FIX/'prepared_review.json');assert metadata(SRC/FIFO)==fix_review['source_binding']
 for b in fix_review['bindings']:assert metadata(Path(b['path']))==b,b
 shutil.copytree(SRC,OUT);shutil.copyfile(FIX/'nav_async_fifo.v',OUT/FIFO)
 m=relocate(load(SRC/'manifest.json'),SRC,OUT)
 names=[Path(p)for p in m['source_files']];changed=[str(p.relative_to(OUT)).replace('\\','/')for p in names if not same_bytes(SRC/p.relative_to(OUT),p)]
 assert changed==[FIFO],changed
 for n in ['run.tcl','effective_options.tcl']:
  if(OUT/n).exists():(OUT/n).write_text(relocate((OUT/n).read_text(),SRC,OUT),encoding='utf8')
 assert(OUT/'run.tcl').read_text().lstrip().startswith('error')
 composition=relocate(load(SRC/'composition_review.json'),SRC,OUT)
 # manifest is itself one of the frozen inputs: finalize it before recording
 # its metadata.  The first prepared6 record was made in the opposite order.
 m.update(FIFO_reset_syntax_candidate=str(FIX),modified_rtl_inputs=composition['changed_RTL_relative_to_SYN40'],automatic_SYN=False,automatic_PNR=False,EDA_authorized=False,firmware_rebuild_pending=False,inherited_CPU_INIT_not_new_FW=False)
 save(OUT/'manifest.json',m)
 all_names=[Path(b['path']).relative_to(SRC)for b in bound['complete_frozen_input_bindings']];bindings=[metadata(OUT/p)for p in all_names]
 composition.update(candidate=str(OUT),ancestor_before_FIFO_syntax=str(SRC),bindings=bindings,final_for_EDA=False,FIFO_reset_syntax='Only two always clauses now expose canonical outer if(reset); else retains original ternary for full X/Z semantics. No sensitivity/RAM/clock/latency/parameters changed.',blockers=['Root final isolated C mapping and frozen source review; then explicit authorized joint synthesis','Actual full primitive CEB/OCE/AD and <=300 BSRAM/LUT/FF gates after joint synthesis'])
 save(OUT/'composition_review.json',composition)
 review=relocate(review,SRC,OUT);review.update(candidate=str(OUT),ancestor_before_FIFO_syntax=str(SRC),hardware_bindings=[metadata(p)for p in names],complete_frozen_input_bindings=bindings,whole_combined128_RTL_regression_pending=False,combined128_RTL_regression='Inherited prepared5 exact hardware apart from same-state FIFO syntax; source5 full combined probe2 passed, FIFO replacement old/new probe1 passed',FIFO_syntax_RTL_proof=str(BASE/'stage3_FIFO_async_reset_syntax_RTL_probe1/result.json'))
 # All firmware files and all eleven image objects remain exact prepared5 bytes.
 fw_names=[Path(b['path']).relative_to(SRC)for b in load(SRC/'prepared_source_review.json')['all_FW_bindings']]
 review['all_FW_bindings']=[metadata(OUT/p)for p in fw_names]
 assert all(same_bytes(SRC/p,OUT/p)for p in fw_names)
 save(OUT/'prepared_source_review.json',review)
 fb=relocate(bound,SRC,OUT);fb.update(candidate=str(OUT),ancestor=str(SRC),source63_and_all_FW_unchanged=False,changed_RTL_from_ancestor=[FIFO],source62_and_all_FW_unchanged=True,complete_frozen_input_bindings=bindings,composition_review=metadata(OUT/'composition_review.json'),source_review=metadata(OUT/'prepared_source_review.json'),RTL_regression_pending=False,FIFO_RTL_equivalence=str(BASE/'stage3_FIFO_async_reset_syntax_RTL_probe1/result.json'),full_combined128_ancestor_result=str(BASE/'stage3_SLF128_storage_regression_probe2/result.json'))
 save(OUT/'firmware_binding_review.json',fb)
 r=dict(prepared=True,at=datetime.now().astimezone().isoformat(),candidate=str(OUT),ancestor=str(SRC),changed_RTL=changed,source62_all_FW_CPU_assets_weights_headers_identical=True,complete_frozen_input_bindings=bindings,manifest=metadata(OUT/'manifest.json'),source_review=metadata(OUT/'prepared_source_review.json'),composition_review=metadata(OUT/'composition_review.json'),same_state_FIFO_proof=metadata(BASE/'stage3_FIFO_async_reset_syntax_RTL_probe1/result.json'),ancestor_combined128_regression=metadata(BASE/'stage3_SLF128_storage_regression_probe2/result.json'),actual_C_small_SYN_review_pending=True,run_entry_remains_disabled=True,automatic_SYN=False,automatic_PNR=False,programmed=False)
 for b in bound['complete_frozen_input_bindings']:assert metadata(Path(b['path']))==b,b
 save(OUT/'FIFO_syntax_rebase_review.json',r);print(json.dumps(dict(candidate=str(OUT),changed_RTL=changed,complete_inputs=len(bindings),same_FW_BIN_bytes=bound['firmware_BIN_bytes']),ensure_ascii=False));return r
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--prepare',action='store_true');a=p.parse_args();assert a.prepare;prepare()
