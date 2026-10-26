"""Bind one accepted same-ELF export into a new, source-identical composition."""
from pathlib import Path
from datetime import datetime
import argparse,json,shutil
import psutil
from prepare_baseline_fusion_retime import metadata,relocate,same_bytes,save
BASE=Path('F:/nav_integration_g2_20261023');SRC=BASE/'stage3_SLF128_storage_RTL_prepared3';OUT=BASE/'stage3_SLF128_storage_RTL_prepared5';PACKAGE=BASE/'stage2_SLF128_safeFW_RV_finalize2';SAFE=BASE/'stage2_SLF128_forward_safeFW_prepared8'
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def prepare():
 assert not OUT.exists();proof=load(PACKAGE/'result.json');assert proof['passed']and proof['all_four_lanes_and_word_image_reconstruct_BIN_and_zero_padding']
 assert proof['static_stack_bound_bytes']<=3072 and proof['stack_reserved_bytes']==6144 and proof['remaining_after_6KiB_stack_bytes']>=0
 for b in proof['output_bindings']:assert metadata(Path(b['path']))==b,b
 composition=load(SRC/'composition_review.json');assert composition['SLF_LOG2N']==7 and composition['source_files']==63
 for b in composition['bindings']:assert metadata(Path(b['path']))==b,b
 active=[dict(pid=p.pid,name=p.info['name'])for p in psutil.process_iter(['name'])if(p.info['name']or'').lower()in{'xsim.exe','xsimk.exe','xelab.exe','xvlog.exe','vsim.exe','vsimk.exe'}]
 assert not active,('new source must freeze with no RTL input running',active)
 # Do not carry historical ELF/LTO products as if they belonged to this image.
 def ignore(directory,names):return{'firmware'}if Path(directory)==SRC else set()
 shutil.copytree(SRC,OUT,ignore=ignore);shutil.copytree(PACKAGE/'firmware',OUT/'firmware')
 cpu=[f'ram_b{b}.hex'for b in range(4)]+['ram_w.hex']
 for n in cpu:shutil.copyfile(PACKAGE/'firmware'/n,OUT/n)
 cpu_changes=cpu+['firmware/'+n for n in cpu]+['firmware/nav_fw.bin'];assert len(cpu_changes)==11
 for n in cpu_changes:assert not same_bytes(SRC/n,OUT/n),('expected fresh CPU image',n)
 m=relocate(load(SRC/'manifest.json'),SRC,OUT)
 names=[Path(p)for p in m['source_files']];assert len(names)==63
 assert all(same_bytes(SRC/p.relative_to(OUT),p)for p in names)
 fw_paths=[p.relative_to(SRC)for p in(SRC/'fw').rglob('*')if p.is_file()]
 assert all(same_bytes(SRC/p,OUT/p)and same_bytes(SAFE/p,OUT/p)for p in fw_paths)
 # All assets except the explicit11 CPU objects are unchanged, including the
 # newly packed weight9 files.  This is a byte comparison, not a digest.
 old_paths=[Path(b['path']).relative_to(SRC)for b in composition['bindings']]
 changed=[str(n).replace('\\','/')for n in old_paths if not same_bytes(SRC/n,OUT/n)]
 assert set(changed)==set(cpu_changes),changed
 for n in ['run.tcl','effective_options.tcl']:
  if(OUT/n).exists():(OUT/n).write_text(relocate((OUT/n).read_text(),SRC,OUT),encoding='utf8')
 assert(OUT/'run.tcl').read_text().lstrip().startswith('error'),'old preparation entry must remain disabled'
 m.update(firmware_rebuild_pending=False,inherited_CPU_INIT_not_new_FW=False,new_firmware_RV_ELF_BIN_128KiB_lanes_bound=True,firmware_package=str(PACKAGE),firmware_image_bytes=proof['image_bytes'],source_preparation_only=True,automatic_SYN=False,automatic_PNR=False,EDA_authorized=False)
 save(OUT/'manifest.json',m)
 # Refresh every original244 proof input, then add exact ELF/stack/source
 # identities.  Never relabel an old CPU binding as the new image.
 extra=[Path('firmware')/n for n in ['nav_fw.elf','nav_fw.map','stack_analysis.json','strict_linked_stack_review.json']]
 all_paths=sorted(set(old_paths+extra),key=lambda p:str(p));bindings=[metadata(OUT/n)for n in all_paths]
 c=relocate(composition,SRC,OUT);c.update(final_for_EDA=False,ancestor_storage=str(SRC),candidate=str(OUT),bindings=bindings,inherited_assets_byte_identical_before_new_FW_binding=composition['inherited_assets_byte_identical'],inherited_assets_byte_identical=composition['inherited_assets_byte_identical']-11,new_FW_RV_ELF_BIN_128KiB_lanes_bound=True,CPU_INIT='New same-ELF accepted export, each byte/word reconstructs BIN plus zero padding to128KiB',CPU_assets_replaced=cpu_changes,firmware_package=str(PACKAGE),new_FW_BIN_bytes=proof['image_bytes'],static_stack_bound_bytes=proof['static_stack_bound_bytes'],stack_reserved_bytes=6144,remaining_after_stack_bytes=proof['remaining_after_6KiB_stack_bytes'],blockers=['Final combined-source128 RTL contract regression and root source review','Actual primitive CEB/OCE/AD and <=300 noncamera BSRAM/LUT/FF gates after one authorized combined synthesis'],synthesis_started=False,PNR_started=False,programmed=False)
 save(OUT/'composition_review.json',c)
 # Old generic source review in the parent was a selected-base snapshot.
 # Preserve it as provenance and replace the authoritative4 review with the
 # actual combined hardware/firmware inventory.
 inherited=load(SAFE/'prepared_source_review.json');save(OUT/'ancestor_selected_base_review.json',inherited)
 review=dict(prepared=True,candidate=str(OUT),selected_safe_base=str(SAFE),source=str(SRC),source_file_count=63,hardware_bindings=[metadata(p)for p in names],all_FW_bindings=[metadata(OUT/p)for p in fw_paths],complete_frozen_input_bindings=bindings,SLF_LOG2N=7,firmware_package=str(PACKAGE),firmware_RV_ELF_BIN_128KiB_lanes_bound=True,firmware_rebuild_pending=False,whole_combined128_RTL_regression_pending=True,automatic_SYN=False,automatic_PNR=False,SYN_started=False,PNR_started=False,programmed=False)
 save(OUT/'prepared_source_review.json',review)
 # Verify duplicated lane locations, not merely that the copied files exist.
 assert all(same_bytes(OUT/n,OUT/'firmware'/n)for n in cpu)
 image=(OUT/'firmware/nav_fw.bin').read_bytes();padded=image+bytes(131072-len(image));lanes=[bytes(int(t,16)for t in(OUT/f'ram_b{b}.hex').read_text().split())for b in range(4)]
 assert bytes(v for word in zip(*lanes)for v in word)==padded
 assert b''.join(int(t,16).to_bytes(4,'little')for t in(OUT/'ram_w.hex').read_text().split())==padded
 result=dict(prepared=True,recorded_at=datetime.now().astimezone().isoformat(),candidate=str(OUT),ancestor=str(SRC),safe128_base=str(SAFE),firmware_package=str(PACKAGE),CPU_assets_changed=cpu_changes,source63_and_all_FW_unchanged=True,all_other_proof_inputs_unchanged=True,firmware_BIN_bytes=len(image),BSS_bytes=proof['BSS_bytes'],static_stack_bound_bytes=proof['static_stack_bound_bytes'],stack_reserved_bytes=6144,remaining_after_stack_bytes=proof['remaining_after_6KiB_stack_bytes'],all11_CPU_assets_consistent=True,complete_frozen_input_bindings=bindings,composition_review=metadata(OUT/'composition_review.json'),source_review=metadata(OUT/'prepared_source_review.json'),source8_safety_repairs_inherited=True,run_tcl_harderror_retained=True,RTL_regression_pending=True,EDA_started=False,programmed=False)
 for b in composition['bindings']:assert metadata(Path(b['path']))==b,b
 save(OUT/'firmware_binding_review.json',result);print(json.dumps({k:result[k]for k in ['candidate','firmware_BIN_bytes','BSS_bytes','static_stack_bound_bytes','remaining_after_stack_bytes','source63_and_all_FW_unchanged','all11_CPU_assets_consistent']},ensure_ascii=False));return result
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--prepare',action='store_true');a=p.parse_args();assert a.prepare;prepare()
