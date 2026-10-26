"""Export the already-linked ELF after a strict same-image stack re-review.

No compilation or firmware execution.  The original failed stack gate, ELF,
all LTO files and proof directories are preserved unchanged.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,os,psutil,shutil,subprocess,time,traceback
from prepare_baseline_fusion_retime import metadata,same_bytes,save
from retry_frozen_synthesis import memory_status
BASE=Path('F:/nav_integration_g2_20261023');BUILD=BASE/'stage2_SLF128_safeFW_RV_compile1'
REVIEW=BASE/'stage2_SLF128_safeFW_RV_compile1_strict_stack_review2/result.json'
SRC=BASE/'stage2_SLF128_forward_safeFW_prepared8';OLD=BASE/'stage2_SLF128_forward_safeFW_prepared7'
DEST=BASE/'stage2_SLF128_safeFW_RV_finalize2';TOOL=Path('D:/tools/xpack-riscv-none-elf-gcc-15.2.0-1/bin');GIB=1024**3
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def run():
 assert not DEST.exists();DEST.mkdir();report=load(REVIEW)
 assert report['passed']and report['complete']and report['actual_static_bound_bytes']<=3072
 assert report['stack_reserved_bytes']==6144 and report['irq_disabled']
 assert not report['unresolved_indirect']and not report['nonstatic_frames']and not report['unresolved_sp_writes']
 assert report['actual_jalr_count']==0 and report['all_SU_and_ltrans_included']
 assert not report['compile_performed']and not report['firmware_executed']and not report['input_files_changed']
 assert not load(BUILD/'firmware/stack_analysis.json')['complete'],'preserve original incomplete stack gate'
 calls=load(BUILD/'tool_calls.json');assert calls[0]['returncode']==0 and any('riscv-none-elf-gcc' in x for x in calls[0]['command'])
 for b in load(OLD/'prepared_source_review.json')['all_FW_bindings']:
  p=Path(b['path']);assert metadata(p)==b and same_bytes(p,SRC/p.relative_to(OLD)),p
 inputs=report['inputs']
 def unchanged():
  for b in inputs:assert metadata(Path(b['path']))==b,b
 unchanged();original_elf=BUILD/'firmware/nav_fw.elf';assert Path(inputs[0]['path'])==original_elf
 fw=DEST/'firmware';fw.mkdir();shutil.copyfile(original_elf,fw/'nav_fw.elf');shutil.copyfile(BUILD/'firmware/nav_fw.map',fw/'nav_fw.map')
 shutil.copyfile(REVIEW,fw/'strict_linked_stack_review.json')
 accepted=dict(report,static_bound_bytes=report['actual_static_bound_bytes'],original_incomplete_report=str(BUILD/'firmware/stack_analysis.json'))
 save(fw/'stack_analysis.json',accepted)
 me=psutil.Process();state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),source=str(SRC),output=str(DEST),old_build=str(BUILD),strict_review=str(REVIEW),no_recompile=True,original_failure_preserved=True,firmware_executed=False,SYN_started=False,PNR_started=False,programmed=False)
 def phase(n,**kw):state.update(phase=n,at=datetime.now().astimezone().isoformat(),**kw);save(DEST/'job.json',state)
 def tool(label,cmd):
  while True:
   mem=memory_status();phase('waiting_memory_'+label,physical=mem.avail_phys,commit=mem.avail_page)
   if mem.avail_phys>=2*GIB and mem.avail_page>=4*GIB:break
   time.sleep(17)
  with(DEST/(label+'.log')).open('w')as log:
   p=subprocess.Popen([str(x)for x in cmd],cwd=DEST,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
   rt=dict(command=[str(x)for x in cmd],pid=p.pid,created_epoch=psutil.Process(p.pid).create_time(),admission_physical_bytes=mem.avail_phys,admission_commit_bytes=mem.avail_page,started_at=datetime.now().astimezone().isoformat());phase('running_'+label,runtime=rt);p.wait();rt.update(exit_code=p.returncode,ended_at=datetime.now().astimezone().isoformat());save(DEST/(label+'_runtime.json'),rt)
  assert p.returncode==0,(label,p.returncode)
  return rt
 try:
  runs=[tool('objcopy',[TOOL/'riscv-none-elf-objcopy.exe','-O','binary',fw/'nav_fw.elf',fw/'nav_fw.bin']),tool('symbols',[TOOL/'riscv-none-elf-nm.exe','-n',fw/'nav_fw.elf'])]
  wanted={'RAM_SIZE','STACK_RESERVE','STACK_SIZE','_bss_start','_bss_end','_stack_top'}
  symbols={a[-1]:int(a[0],16)for line in(DEST/'symbols.log').read_text().splitlines()if len(a:=line.split())>=3 and a[-1]in wanted}
  assert symbols['RAM_SIZE']==131072 and symbols['STACK_SIZE']==6144 and symbols['_stack_top']==131072
  remaining=131072-symbols['_bss_end']-6144;assert remaining>=0
  image=(fw/'nav_fw.bin').read_bytes();assert 0<len(image)<=131072
  padded=image+bytes(131072-len(image));nw=32768
  # Same token spelling and lane order as the frozen build_rv.py export code.
  for b in range(4):(fw/f'ram_b{b}.hex').write_text('\n'.join(f'{padded[4*i+b]:02x}'for i in range(nw))+'\n')
  (fw/'ram_w.hex').write_text('\n'.join(padded[4*i:4*i+4][::-1].hex()for i in range(nw))+'\n')
  lanes=[bytes(int(t,16)for t in(fw/f'ram_b{b}.hex').read_text().split())for b in range(4)]
  assert all(len(x)==32768 for x in lanes)
  assert bytes(v for word in zip(*lanes)for v in word)==padded
  words=[int(t,16)for t in(fw/'ram_w.hex').read_text().split()]
  assert len(words)==32768 and b''.join(w.to_bytes(4,'little')for w in words)==padded
  assert padded[len(image):]==bytes(131072-len(image))
  unchanged();assert same_bytes(original_elf,fw/'nav_fw.elf')
  result=dict(state,phase='firmware_export_complete',passed=True,firmware_directory=str(fw),image_bytes=len(image),BSS_bytes=next(x['bytes']for x in report['ELF_sections']if x['name']=='.bss'),linked_symbols=symbols,remaining_after_6KiB_stack_bytes=remaining,static_stack_bound_bytes=report['actual_static_bound_bytes'],stack_reserved_bytes=6144,CPU_image_bytes=131072,each_byte_lane_entries=32768,all_four_lanes_and_word_image_reconstruct_BIN_and_zero_padding=True,strict_all_input_bindings=inputs,original_compile_and_stack_failure_unchanged=True,source8_FW_byte_equal_to_compiled_source7=True,original_ELF_metadata=metadata(original_elf),output_bindings=[metadata(p)for p in sorted(fw.iterdir())if p.is_file()],runtimes=runs,not_proven=['real Pico boot and busy host-loss with this new image','runtime stack watermark/real RK and physical stop','new CPU primitive INIT in joint synthesis and physical mapping','routing/setup/hold/CDC/reset/IO/board'])
  save(DEST/'result.json',result);save(DEST/'job.json',result);print(json.dumps(dict(passed=True,image_bytes=len(image),BSS_bytes=result['BSS_bytes'],remaining=remaining,stack_bound=result['static_stack_bound_bytes'],firmware=str(fw)),ensure_ascii=False));return 0
 except Exception as e:
  r=dict(state,phase='failed_preserved',passed=False,error=str(e),traceback=traceback.format_exc());save(DEST/'result.json',r);save(DEST/'job.json',r);print(r['traceback']);return 1
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');a=p.parse_args();assert a.run;raise SystemExit(run())
