"""Freeze/run one combined SLF128 regression, without firmware builds or EDA mapping."""
from pathlib import Path
from datetime import datetime
import argparse,json,os,re,shutil,subprocess,time,traceback
import psutil
from prepare_baseline_fusion_retime import load,save,metadata,same_bytes
from retry_frozen_synthesis import memory_status

ROOT=Path('F:/nav_integration_g2_20261023')
OLD=ROOT/'stage2_SLF128_bitwise_probe2'
HERE=Path(__file__).resolve().parent
VIV=Path('E:/2026.1/Vivado/bin');GIB=1024**3
VECTORS=['pts.hex','frm.hex','cand.hex','res.hex','sm.hex','smm.hex','prior.hex','pres.hex','counts.txt','sin_lut.hex']
TOOLS={'xvlog.exe','xelab.exe','xsim.exe','xsimk.exe','vsim.exe','vsimk.exe'}

def unchanged(rows):
 for row in rows:assert metadata(Path(row['path']))==row,('frozen input changed',row)

def prepare(candidate,inputs):
 candidate=candidate.resolve();inputs=inputs.resolve();assert not inputs.exists()
 proof=load(candidate/'firmware_binding_review.json');manifest=load(candidate/'manifest.json')
 assert proof['prepared'] and proof['all11_CPU_assets_consistent'] and proof['source63_and_all_FW_unchanged']
 assert proof['source8_safety_repairs_inherited'] and proof['run_tcl_harderror_retained']
 rows=proof['complete_frozen_input_bindings'];unchanged(rows)
 sources=[str(Path(p).resolve())for p in manifest['source_files']];assert len(sources)==63 and manifest['SLF_LOG2N']==7
 assert all(Path(p).is_relative_to(candidate)for p in sources)
 assert load(OLD/'result.json')['passed']
 inputs.mkdir(parents=True)
 for name in VECTORS:
  shutil.copyfile(OLD/name,inputs/name);assert same_bytes(OLD/name,inputs/name)
 source=(OLD/'tb.sv').read_text(encoding='utf8')
 assert source.count('module tb_slf #(parameter integer SLF_LOG2N=7);')==1
 source=source.replace('module tb_slf #(parameter integer SLF_LOG2N=7);','module tb_slf #(parameter integer SLF_LOG2N=7)(output reg done=0);')
 anchor="        .frame_en(1'b1), .cfg_zlo(zlo)"
 assert source.count(anchor)==1
 source=source.replace(anchor,"        .frame_begin(1'b0), .frame_hold(1'b1), .match_enable(1'b1), .frame_can_start(),\n"+anchor)
 assert source.count('        $finish;')==1
 source=source.replace('        $finish;','        $display("STAGE3_SLF128_MODEL_PASS"); done=1;')
 (inputs/'tb_slf.sv').write_text(source,encoding='utf8',newline='\n')
 for name in ('tb_stage3_storage_branches.sv','tb_stage3_empty_imu_bus.sv'):shutil.copyfile(HERE/name,inputs/name)
 (inputs/'wrapper.sv').write_text('''`timescale 1ns/1ps
module stage3_combined;
wire slf_done,ram_done,bus_done;
tb_slf #(.SLF_LOG2N(7)) model(slf_done);
stage3_storage_branches stores(ram_done);
stage3_empty_imu_bus bus_test(bus_done);
initial begin wait(slf_done&&ram_done&&bus_done);#1;$display("STAGE3_COMBINED_SLF128_STORAGE_PASS");$finish;end
initial begin #300000000;$fatal(1,"combined finite watchdog");end
endmodule
''',encoding='utf8')
 expected=[int(x)for x in re.findall(r'SLF_TIMING .*?cycles=(\d+)',(OLD/'xsim.log').read_text(errors='replace'))]
 assert len(expected)==7
 include=str(candidate/'nav/rtl/regs')
 commands=[['xvlog',['-sv','-i',include,*sources,'tb_slf.sv','tb_stage3_storage_branches.sv','tb_stage3_empty_imu_bus.sv','wrapper.sv']],
  ['xelab',['-debug','off','-timescale','1ns/1ps','stage3_combined','-s','stage3_combined']],['xsim',['stage3_combined','-runall']]]
 entry=dict(candidate=str(candidate),hardware_source_count=63,firmware_binding=str(candidate/'firmware_binding_review.json'),
  reused_vectors_from=str(OLD),vector_generation_repeated=False,SLF_LOG2N=7,expected_eval_cycles=expected,
  ownership_ports=dict(frame_begin=0,frame_hold=1,match_enable=1,frame_can_start='explicit unconsumed output'),
  direct_stream_scope='No raw commit in this fixture; accepted-raw ownership proof remains separate.',
  original_math_checks_preserved=True,normal_eval_publication_cycles_compared=True,
  common_RAM_cases=9,common_RAM_simultaneous_read_write=False,IMU_bus_real_nav_subsystem=True,
  IMU_invalid_payload_forced_X=True,CPU_executed=False,CPU_startup_proved=False,
  physical_CEB_collision_timing_or_board_proved=False,source_safety_overlay_reused=True,
  historical_safety_proofs=['fifo_async_assert_candidate3','mac_rx_boundary_candidate2','preview_MAC_UDP_XT_reset_probe2'],
  admission_min_physical_bytes=2*GIB,admission_min_commit_bytes=4*GIB,commands=commands,Gowin_commands=[],automatic_retry=False)
 save(inputs/'entry.json',entry)
 extra=[candidate/'firmware_binding_review.json',Path(__file__).resolve(),HERE/'tb_stage3_storage_branches.sv',HERE/'tb_stage3_empty_imu_bus.sv']
 extra.extend(p for p in inputs.iterdir()if p.is_file())
 bound={row['path']:row for row in rows}
 for p in extra:bound[str(p)]=metadata(p)
 save(inputs/'frozen_bindings.json',dict(inputs=list(bound.values()),source_binding_count=len(rows),final_new_FW_assets_bound=True))
 save(inputs/'job.json',dict(phase='all_inputs_frozen_not_started',candidate=str(candidate),source_count=63,RTL_started=False,SYN_started=False,PNR_started=False))
 return entry

def run(inputs,out):
 inputs=inputs.resolve();out=out.resolve();assert not out.exists()
 entry=load(inputs/'entry.json');rows=load(inputs/'frozen_bindings.json')['inputs'];unchanged(rows)
 assert not entry['Gowin_commands']
 shutil.copytree(inputs,out);me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
 state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),candidate=entry['candidate'],input_directory=str(inputs),output=str(out),
  synthesis_started=False,PNR_started=False,programmed=False,automatic_retry=False,CPU_executed=False)
 with(out/'run_claim.json').open('x',encoding='utf8')as f:json.dump(state,f,ensure_ascii=False)
 def status(phase,**kw):state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw);save(out/'job.json',state)
 runtimes=[]
 try:
  for index,(tool,args)in enumerate(entry['commands']):
   assert tool in ('xvlog','xelab','xsim')
   while True:
    m=memory_status();status('waiting_memory_before_'+tool,available_physical_bytes=m.avail_phys,available_commit_bytes=m.avail_page)
    if m.avail_phys>=2*GIB and m.avail_page>=4*GIB:break
    time.sleep(17)
   unchanged(rows);cmd=['cmd','/c',str(VIV/(tool+'.bat')),*args]
   with(out/f'{index}_{tool}_stdout.log').open('w',encoding='utf8')as log:
    p=subprocess.Popen(cmd,cwd=out,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW|subprocess.BELOW_NORMAL_PRIORITY_CLASS)
    rt=dict(tool=tool,command=cmd,wrapper_pid=p.pid,wrapper_created_epoch=psutil.Process(p.pid).create_time(),started_at=datetime.now().astimezone().isoformat(),
     admission_physical_bytes=m.avail_phys,admission_commit_bytes=m.avail_page,processes=[])
    seen={};status('running_'+tool,runtime=rt)
    while p.poll()is None:
     try:
      for child in psutil.Process(p.pid).children(recursive=True):
       row=dict(pid=child.pid,created_epoch=child.create_time(),name=child.name(),command=child.cmdline());seen[(child.pid,row['created_epoch'])]=row
     except(psutil.NoSuchProcess,psutil.AccessDenied):pass
     rt['processes']=list(seen.values());save(out/f'{index}_{tool}_runtime.json',rt);time.sleep(1)
    rt.update(exit_code=p.returncode,ended_at=datetime.now().astimezone().isoformat());runtimes.append(rt);save(out/f'{index}_{tool}_runtime.json',rt)
   assert p.returncode==0,(tool,p.returncode)
   logtext=(out/f'{index}_{tool}_stdout.log').read_text(errors='replace')
   assert not re.search(r'(?:^|\n)\s*(?:FATAL_ERROR|ERROR:)',logtext),(tool,'error despite exit0')
  text=(out/'xsim.log').read_text(errors='replace')
  assert 'STAGE3_COMBINED_SLF128_STORAGE_PASS'in text and 'STAGE3_SLF128_MODEL_PASS'in text and 'STAGE3_EMPTY_IMU_BUS_PASS'in text
  assert not re.search(r'\bFatal:|TEST FAILED|ERROR:',text)
  actual=[int(x)for x in re.findall(r'SLF_TIMING .*?cycles=(\d+)',text)]
  assert actual==entry['expected_eval_cycles'],('eval publication cycles changed',actual,entry['expected_eval_cycles'])
  ram=re.findall(r'STAGE3_RAM_BRANCH_PASS id=(\d+).*?checks=(\d+)',text);assert len(ram)==9 and {int(x[0])for x in ram}==set(range(9))
  unchanged(rows)
  result=dict(state,phase='combined_regression_complete',passed=True,runtimes=runtimes,source_count=63,
   unchanged_frozen_inputs=len(rows),eval_cycles=actual,eval_cycles_equal_original128=True,RAM_cases=9,RAM_checks=sum(int(x[1])for x in ram),
   markers=[line for line in text.splitlines()if line.startswith(('SLF_TIMING','STAGE3_','$finish'))],
   original_five_frame_245_candidate_all_structure_clear_prior_checks=True,
   actual_Pico_CPU_startup_proved=False,physical_CEB_OCE_AD_collision_safety_proved=False,mapping_resources_timing_proved=False,deployment_accepted=False)
  save(out/'result.json',result);save(out/'job.json',result);return 0
 except Exception as error:
  result=dict(state,phase='failed_preserved_no_retry',passed=False,error=str(error),traceback=traceback.format_exc(),runtimes=runtimes)
  save(out/'result.json',result);save(out/'job.json',result);return 1

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--candidate',type=Path);p.add_argument('--inputs',type=Path,required=True);p.add_argument('--out',type=Path)
 p.add_argument('--prepare',action='store_true');p.add_argument('--run',action='store_true');a=p.parse_args();assert a.prepare!=a.run
 if a.prepare:
  assert a.candidate;print(json.dumps(prepare(a.candidate,a.inputs),ensure_ascii=False,indent=2))
 else:
  assert a.out;raise SystemExit(run(a.inputs,a.out))
