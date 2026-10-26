"""Prepare, or explicitly start once, the real-Pico/raw-UART drain fixture.

--prepare copies a completed UART/G2 fixture and the frozen52824B firmware,
generates finite scenarios and does not run any compiler/simulator/EDA tool.
--start is a separate explicit one-shot operation. This never feeds PNR51.
"""
import argparse
import ctypes
from datetime import datetime
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import time
import zlib

BASE=Path('F:/nav_integration_g2_20261023')
OUT=BASE/'CPU250_bounded_drain_UART_Pico_prepared1'
PRIOR=BASE/'preview_full_G2_UART_CPU250_probe2_emptyIMU_modelsim'
HARDWARE=BASE/'baseline_fusion_preview_safety250_emptyIMU_prepared3'
FW=BASE/'CPU250_host_drain_isolated_validation1'
REPO=Path(__file__).resolve().parents[2]
MODELSIM=Path('E:/modelsim_ae/win32aloem')
LICENSE=Path('C:/Users/Li/Downloads/LR-204362_License.dat')
GIB=1024**3


def save(path,obj):
    temp=path.with_name(path.name+'.tmp');temp.write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');temp.replace(path)
def meta(p):
    s=p.stat();return dict(path=str(p),bytes=s.st_size,mtime_ns=s.st_mtime_ns)
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def memory_status():
    class MemoryStatus(ctypes.Structure):
        _fields_=[('length',ctypes.c_ulong),('load',ctypes.c_ulong),
          ('total_phys',ctypes.c_ulonglong),('avail_phys',ctypes.c_ulonglong),
          ('total_page',ctypes.c_ulonglong),('avail_page',ctypes.c_ulonglong),
          ('total_virtual',ctypes.c_ulonglong),('avail_virtual',ctypes.c_ulonglong),
          ('avail_extended_virtual',ctypes.c_ulonglong)]
    assert os.name=='nt','This prepared licensed runner targets Windows only'
    value=MemoryStatus();value.length=ctypes.sizeof(value)
    assert ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(value))
    return value
def symbols(path):
    data=path.read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',data)
    sections=[struct.unpack_from('<IIIIIIIIII',data,h[6]+i*h[11]) for i in range(h[12])];result={}
    for section in sections:
        if section[1]!=2:continue
        strings=sections[section[6]];names=data[strings[4]:strings[4]+strings[5]]
        for pos in range(section[4],section[4]+section[5],section[9]):
            name,value,size,info,other,index=struct.unpack_from('<IIIBBH',data,pos)
            key=names[name:].split(b'\0',1)[0].decode()
            if key:result[key]=dict(address=value,size=size,type=info&15)
    return result


def prepare():
    assert not OUT.exists(),'Unique output already exists; never overwrite its inputs or evidence'
    old=load(PRIOR/'result.json');compile_proof=load(FW/'cross_compile_result.json');native=load(FW/'native_result.json')
    assert old['passed'] and old['actual_PicoRV32_and_firmware'] and compile_proof['passed'] and native['passed']
    assert compile_proof['image_bytes']==52824 and compile_proof['linked_symbols']['RAM_SIZE']==131072
    assert (FW/'firmware/nav_fw.bin').stat().st_size==52824
    for row in old['source_bindings']:assert meta(Path(row['path']))==row
    manifest=load(HARDWARE/'manifest.json');inputs=[Path(p).relative_to(HARDWARE) for p in manifest['source_files']]
    assert len(inputs)==63
    template=Path(__file__).with_name('tb_cpu250_bounded_uart_scenarios.svh')
    watch=[PRIOR/'result.json',FW/'cross_compile_result.json',FW/'native_result.json',FW/'firmware/nav_fw.bin',FW/'firmware/nav_fw.elf',Path(__file__).resolve(),template]
    watch += [PRIOR/n for n in inputs]
    before=[meta(p) for p in watch]
    OUT.mkdir()
    copied=[]
    for n in inputs:
        src=PRIOR/n;dest=OUT/n;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dest);copied.append(dest)
    for p in PRIOR.rglob('*'):
        if p.is_file() and 'work' not in p.relative_to(PRIOR).parts and p.suffix.lower() in ('.vh','.h','.hex','.mem','.mif'):
            dest=OUT/p.relative_to(PRIOR);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest);copied.append(dest)
    for name in ['nav_fw.bin','nav_fw.elf','stack_analysis.json','ram_w.hex']+[f'ram_b{i}.hex' for i in range(4)]:
        dest=OUT/name;shutil.copy2(FW/'firmware'/name,dest);copied.append(dest)
    image=(OUT/'nav_fw.bin').read_bytes();lanes=[bytes(int(x,16) for x in (OUT/f'ram_b{i}.hex').read_text().split()) for i in range(4)]
    assert all(len(x)==32768 for x in lanes)
    reconstructed=bytes(lanes[i%4][i//4] for i in range(131072))
    assert reconstructed==image+bytes(131072-len(image))
    syms=symbols(OUT/'nav_fw.elf');needed={'GUARD_ADDRESS':'g_host_link_guard','INTERRUPTED_ADDRESS':'g_nav_search_interrupted',
        'DRAIN_ENTRY':'drain_host_messages','FRONTIER_ENTRY':'explore_frontier.constprop.0',
        'CHECKPOINT_ENTRY':'nav_app_search_checkpoint','STEP_ENTRY':'task_step_local.constprop.0'}
    assert syms['g_host_link_guard']['size']==24 and syms['g_nav_search_interrupted']['size']==1
    (OUT/'guard_symbol.vh').write_text(f"localparam integer GUARD_ADDRESS=32'h{syms['g_host_link_guard']['address']:08x};\n")
    lines=['localparam integer HOST_TIMEOUT_US=250000;','localparam integer MAX_ZERO_AGE_US=350000;']
    lines += [f"localparam integer {k}=32'h{syms[v]['address']:08x};" for k,v in needed.items() if k!='GUARD_ADDRESS']
    (OUT/'guard_probe_limits.vh').write_text('\n'.join(lines)+'\n')
    previous_tb=(PRIOR/'tb_preview_full_uart_cpu.sv').read_text()
    prefix=previous_tb.split('  initial begin : stimulus')[0]
    assert 'task wr32' in prefix and 'always #10 clk' in prefix and 'BIT_NS=1e9/921600.0' in prefix
    tb=OUT/'tb_preview_full_uart_cpu.sv';tb.write_text(prefix+template.read_text(encoding='utf-8'),encoding='utf-8');copied.append(tb)
    # A legal dense64x64 prior. Half-map camera seen/unseen boundaries are
    # produced by real firmware explore_seen_mark, never written into CPU RAM.
    body=bytes(4096);header=struct.pack('<4sHHiiIiiHHII',b'NVMP',2,0,0,0,0,0,0,64,64,zlib.crc32(body),len(body))
    assert len(header)==40
    prior=header+body;(OUT/'prior_image.hex').write_text('\n'.join(f'{b:02x}' for b in prior)+'\n');copied.append(OUT/'prior_image.hex')
    (OUT/'run.do').write_text('onerror {quit -code 2}\nonbreak {quit -code 3}\nrun -all\nquit -code 0\n')
    commands=[['vlib',['work']],['vlog',['-work','work',*[f'+incdir+{p}' for p in (OUT,OUT/'nav/rtl/regs',OUT/'control')],*[str(OUT/n) for n in inputs]]],
              ['vlog',['-sv','-work','work','+incdir+'+str(OUT),str(tb)]],['vsim',['-c','-t','1ps','work.tb_preview_full_uart_cpu','-do','run.do']]]
    # Run the frozen F-drive entry directly; no live-repo import is needed.
    shutil.copy2(Path(__file__).resolve(),OUT/'run_once.py');copied.append(OUT/'run_once.py')
    record=dict(prepared=True,simulator_started=False,compiled=False,phase='prepared_not_simulated',
      source_completed_UART_fixture=str(PRIOR),source_hardware=str(HARDWARE),frozen_FW_proof=str(FW),
      hardware_inputs=63,exact_image_bytes=52824,CPU_RAM_bytes=131072,unmodified_image_bound_to_INIT=True,
      actual_CPU_and_raw_UART=True,clock_hz=50000000,UART_baud=921600,UART_bits_per_byte=10,
      firmware_instrumented=False,CPU_or_timebase_substituted=False,hardware_time_accelerated=False,
      driven_inputs='Only key reset and raw UART921600 8N1; other external sensor pins remain prior-fixture idle. MAP_CMD is a real UART register transaction. No CPU hold/debug RAM write, force or synthetic MMIO.',
      read_only_observers='Actual CPU PC/RA/a0, RAM guard/interrupted bytes, MBX RX_POP/TX publication, final SAFE_CMD_W pulse. No observed state is driven.',
      symbol_bindings={k:dict(name=v,**syms[v]) for k,v in needed.items()},commands=commands,
      admission=dict(minimum_available_physical_bytes=2*GIB,minimum_available_commit_bytes=4*GIB,one_simulator_session=True,automatic_wait_or_start=False),
      simulation_budget=dict(planned_us=[900000,1200000],planned_50MHz_cycles=[45000000,60000000],finite_watchdog_us=1600000,finite_watchdog_cycles=80000000,
        prior_measured_simulated_us=290004,prior_measured_wall_seconds=2967,rough_wall_hours=[2.56,3.41],worst_watchdog_wall_hours_at_prior_rate=4.55,
        estimate_scope='Scaling the previous full-G2 ModelSim run only; machine load, new firmware and monitor cost may change throughput. No reservation or execution occurred.'),
      cases=[
        dict(name='finite_continuous_UART_refill',inputs='120 legal MAP_CLEAR packets spaced at least500us on actual wire; observe actual ordinary drain returns with backlog and budget<=4, loop service and no timeout solely from quota. Drops are measured rather than assumed absent.'),
        dict(name='MAP_ACK_order_and_payload',inputs='4136B NVMP prior,512B MAP_DATA chunks; stop-and-wait raw-UART TX-shadow ACK reads; duplicate offset0 and out-of-order offset1024 must ACK unchanged512; final CRC-valid END must ACK4136/READY.'),
        dict(name='busy_real_arrival_loss',inputs='SET_POSE3000,3000; real hardware MAP_CMD sweep; wait two actual idle planner entries; send SEARCH20ms after observed prior tick and then stop actual UART host traffic. Require actual frontier PC entry, frontier active at actual250ms latch, interrupted flag and new zero SAFE_CMD_W commit by350ms from actual last_rx.'),
        dict(name='latch_recovery',inputs='Fresh no-target heartbeat cannot heal; fresh IDLE explicitly recovers via actual UART.')],
      ACK_contract='UART bridge K confirms register transactions. Mailbox TX shadow is latest-only; map upload is paced and sequence-stability checked. Read-only CPU publication logs do not imply every overwritten ACK reached host.',
      prerequisites_and_limits=['Busy entry/250ms-in-frontier are runtime obligations, not yet demonstrated. Missing either is a failure, never an idle busy-pass.',
        'No physical FPGA/SOC delay, real USB throughput, real RK/IMU/lidar/balance inference, full WCET or standstill conclusion.',
        'Old fixture uses simulation-only declaration ordering moves already recorded in its result; copied simulation RTL is reused, not merged to hardware.',
        'This52824B candidate and its test are separate from SYN39/PNR51; no image or source is copied into51.'],
      source_metadata_before=before,source_inputs_unchanged=before==[meta(p) for p in watch],
      prepared_bindings=[meta(p) for p in dict.fromkeys(copied+[OUT/'guard_symbol.vh',OUT/'guard_probe_limits.vh',OUT/'run.do'])])
    save(OUT/'preparation.json',record);save(OUT/'job.json',dict(phase='prepared_not_simulated',programmed=False,simulator_started=False))
    assert record['source_inputs_unchanged'];print(json.dumps({k:record[k] for k in ('prepared','simulator_started','compiled','exact_image_bytes','simulation_budget','admission')},ensure_ascii=False))


def start():
    import psutil
    proof=load(OUT/'preparation.json');assert proof['prepared'] and proof['exact_image_bytes']==52824
    assert not (OUT/'launch_claim.json').exists(),'One-shot launch already claimed; no automatic retry'
    for row in proof['prepared_bindings']+proof['source_metadata_before']:assert meta(Path(row['path']))==row
    m=memory_status();assert m.avail_phys>=2*GIB and m.avail_page>=4*GIB,('Admission requires2GiB physical/4GiB commit',m.avail_phys,m.avail_page)
    assert not any((p.info['name'] or '').lower() in ('vsim.exe','vsimk.exe') for p in psutil.process_iter(['name'])),'Existing licensed simulator session; not starting'
    assert LICENSE.is_file()
    me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    with (OUT/'launch_claim.json').open('x') as f:json.dump(dict(pid=me.pid,created_epoch=me.create_time()),f)
    env=os.environ.copy();env['MGLS_LICENSE_FILE']=str(LICENSE);env['LM_LICENSE_FILE']=str(LICENSE)
    state=dict(phase='starting',programmed=False,controller_pid=me.pid,admission_physical=m.avail_phys,admission_commit=m.avail_page)
    try:
      for index,(tool,args) in enumerate(proof['commands']):
        command=[str(MODELSIM/(tool+'.exe')),*args];state.update(phase='running_'+tool,command=command,updated_at=datetime.now().astimezone().isoformat());save(OUT/'job.json',state)
        with (OUT/f'{index}_{tool}.log').open('w',encoding='utf-8') as stream:
          child=subprocess.Popen(command,cwd=OUT,env=env,stdout=stream,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
          state['child_pid']=child.pid;save(OUT/'job.json',state)
          while child.poll() is None:
            state['updated_at']=datetime.now().astimezone().isoformat();save(OUT/'job.json',state);time.sleep(5)
          assert child.returncode==0,(tool,child.returncode)
      text=(OUT/'3_vsim.log').read_text(errors='replace');marker=re.search(r'BOUNDED_DRAIN_UART_PICO PREPARED_CASES_PASSED[^\r\n]*',text)
      assert marker and not re.search(r'FATAL|Fatal:|ERROR:',text),'No successful real-CPU scenario marker'
      metrics={k:int(v) for k,v in re.findall(r'(\w+)=(\d+)',marker[0])}
      assert metrics['backlog_returns']>0 and metrics['frontier_entries']>0 and 250000<=metrics['zero_age_us']<=350000
      unchanged=all(meta(Path(row['path']))==row for row in proof['source_metadata_before']);assert unchanged
      save(OUT/'result.json',dict(passed=True,metrics=metrics,scope='Finite actual raw-UART/G2/Pico/full52824B firmware only; no timing closure, realUSB/WCET or physical stop acceptance',
        actual_CPU=True,raw_UART=True,hardware_time_accelerated=False,firmware_instrumented=False,source_inputs_unchanged=unchanged,physical_standstill_proven=False,deployment_accepted=False))
      state.update(phase='finite_actual_UART_CPU_passed',metrics=metrics);save(OUT/'job.json',state)
    except Exception as error:
      import traceback
      state.update(phase='failed_preserved_no_retry',error=str(error),traceback=traceback.format_exc());save(OUT/'job.json',state);raise


if __name__=='__main__':
    p=argparse.ArgumentParser();g=p.add_mutually_exclusive_group(required=True);g.add_argument('--prepare',action='store_true');g.add_argument('--start',action='store_true');a=p.parse_args()
    prepare() if a.prepare else start()
