"""Run the frozen whole G2 RTL with its real PicoRV32 UART-loss image.

Only external UART/reset and idle sensor pins are driven. Hardware time is not
accelerated. This finite idle-navigation case does not establish busy-planner
WCET, physical timing, complete RL inference, real RK feedback or vehicle safety.
"""
from datetime import datetime
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import time
import psutil
from prepare_baseline_fusion_retime import metadata,save,same_bytes
from retry_frozen_synthesis import memory_status
from prepare_baseline_fusion_uart_loss import decode_ram,read_elf

ROOT=Path(__file__).resolve().parents[2]
SOURCE=Path('F:/nav_integration_g2_20261023/baseline_fusion_preview_FIFO_MAC_UART_prepared1')
VIV=Path('E:/2026.1/Vivado/bin')
EXPECTED_TIMEOUT_US=1000000
MAX_ZERO_AGE_US=0

def symbol(path,wanted):
    data=path.read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',data)
    sections=[struct.unpack_from('<IIIIIIIIII',data,h[6]+i*h[11])for i in range(h[12])]
    for s in sections:
        if s[1]!=2:continue
        strings=sections[s[6]];names=data[strings[4]:strings[4]+strings[5]]
        for off in range(s[4],s[4]+s[5],s[9]):
            name,value,size,info,other,index=struct.unpack_from('<IIIBBH',data,off)
            if names[name:].split(b'\0',1)[0].decode()==wanted:
                assert size==24 and value%4==0 and value+size<131072
                return value
    raise AssertionError(('ELF symbol not found',wanted))

def run(out,wait_for_memory=False,portable_oled_declaration=False,simulator='vivado'):
    assert not out.exists();out.mkdir(parents=True)
    me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),source=str(SOURCE),output=str(out),
        active_PNR40_modified=False,new_SYN_PNR=False,programmed=False,physical_timing_or_vehicle_acceptance=False)
    def status(phase,**fields):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**fields)
        t=out/'job.tmp';t.write_text(json.dumps(state,indent=2));t.replace(out/'job.json')
    try:
        status('checking_frozen_actual_G2_RTL_firmware_and_admission')
        review=json.loads((SOURCE/'prepared_source_review.json').read_text());assert review['passed']
        for row in review['bindings']:assert metadata(Path(row['path']))==row
        timeout_header=(SOURCE/'fw/nav/host_link_guard.h').read_text()
        timeout=re.search(r'^#define HOST_LINK_TIMEOUT_US (\d+)u',timeout_header,re.M)
        assert timeout and int(timeout[1])==EXPECTED_TIMEOUT_US
        while True:
            mem=memory_status()
            if mem.avail_phys>=2*1024**3 and mem.avail_page>=4*1024**3:break
            assert wait_for_memory,('memory admission',mem.avail_phys,mem.avail_page)
            status('waiting_for_actual_memory_admission',available_physical_bytes=mem.avail_phys,
                   available_commit_bytes=mem.avail_page,minimum_physical_bytes=2*1024**3,minimum_commit_bytes=4*1024**3)
            time.sleep(15)
        for row in review['bindings']:assert metadata(Path(row['path']))==row
        if EXPECTED_TIMEOUT_US==1000000:
            image,elf=read_elf(SOURCE/'firmware/nav_fw.elf')
        else:
            from prepare_uart_loss_250 import read_image
            assert EXPECTED_TIMEOUT_US==250000 and review['host_timeout_us']==250000
            image,elf=read_image(SOURCE/'firmware/nav_fw.elf')
            assert len(image)==review['new_image_bytes']and elf==review['image_layout']
        assert image==(SOURCE/'firmware/nav_fw.bin').read_bytes()
        assert decode_ram(SOURCE)==image+bytes(131072-len(image))
        address=symbol(SOURCE/'firmware/nav_fw.elf','g_host_link_guard')
        manifest=json.loads((SOURCE/'manifest.json').read_text());inputs=[Path(s)for s in manifest['source_files']];assert len(inputs)==63
        copied=[]
        for p in inputs:
            n=p.relative_to(SOURCE);q=out/n;q.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,q);copied.append(n)
        declaration_order_review=[]
        if portable_oled_declaration:
            # Vivado diagnoses the old OLED part-select before its later net
            # declaration. Move only those two unchanged continuous net
            # declarations in this simulation copy; never patch hardware RTL
            # or relax width/type/error checking globally.
            p=out/'control/oled.v';original=p.read_text()
            axis='    wire [2:0] axis_index=segment-3\'d1;\n'
            value='    wire [15:0] current_value=frame_values[axis_index*16 +: 16];\n'
            assert original.count(axis)==original.count(value)==1
            stripped=original.replace(axis,'',1).replace(value,'',1)
            anchor='    wire [15:0] magnitude=current_value[15]'
            assert stripped.count(anchor)==1
            adapted=stripped.replace(anchor,axis+value+anchor,1)
            assert adapted.replace(axis,'',1).replace(value,'',1)==stripped
            p.write_text(adapted,newline='\n')
            declaration_order_review.append(dict(file='control/oled.v',moved_unchanged_declarations=[axis.strip(),value.strip()],
                all_remaining_text_identical=True,expressions_widths_and_assignments_unchanged=True,
                simulation_copy_only=True,hardware_source_modified=False))
            p=out/'control4500_core.v';original=p.read_text()
            lines=[line for line in original.splitlines(True)if re.match(r'    localparam signed \[31:0\] (LEFT|RIGHT)_TARGET_MMPS=',line)]
            assert len(lines)==2
            diagnostic='    reg [2:0] diagnostic_left_motor_fault,diagnostic_right_motor_fault;\n'
            assert original.count(diagnostic)==1;lines.append(diagnostic)
            stripped=original
            for line in lines:assert stripped.count(line)==1;stripped=stripped.replace(line,'',1)
            anchor='    wire signed [31:0] wheel_left_target_mmps='
            assert stripped.count(anchor)==1
            adapted=stripped.replace(anchor,''.join(lines)+anchor,1)
            inverse=adapted
            for line in lines:inverse=inverse.replace(line,'',1)
            assert inverse==stripped
            p.write_text(adapted,newline='\n')
            declaration_order_review.append(dict(file='control4500_core.v',moved_unchanged_declarations=[s.strip()for s in lines],
                all_remaining_text_identical=True,expressions_widths_and_assignments_unchanged=True,
                simulation_copy_only=True,hardware_source_modified=False))
            p=out/'g2_top.v';original=p.read_text()
            line='    wire l_rx_rst, l_tx_rst;\n';anchor='    rgmii_mac100 #(.CLK_HZ(CLK_HZ)) u_lmac ('
            assert original.count(line)==original.count(anchor)==1
            stripped=original.replace(line,'',1);adapted=stripped.replace(anchor,line+anchor,1)
            assert adapted.replace(line,'',1)==stripped
            p.write_text(adapted,newline='\n')
            declaration_order_review.append(dict(file='g2_top.v',moved_unchanged_declarations=[line.strip()],
                all_remaining_text_identical=True,expressions_widths_and_assignments_unchanged=True,
                simulation_copy_only=True,hardware_source_modified=False))
        for p in SOURCE.rglob('*'):
            if p.is_file()and p.suffix.lower()in {'.vh','.h','.hex','.mem','.bin','.mif'}:
                n=p.relative_to(SOURCE);q=out/n;q.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,q)
        assert all(same_bytes(SOURCE/n,out/n)for n in copied if not(portable_oled_declaration and n.as_posix()in ('control/oled.v','control4500_core.v','g2_top.v')))
        tb=ROOT/'integration/g2/tb_preview_full_uart_cpu.sv';shutil.copyfile(tb,out/tb.name)
        (out/'guard_symbol.vh').write_text(f'localparam integer GUARD_ADDRESS=32\'h{address:08x};\n')
        (out/'guard_probe_limits.vh').write_text(
            f'localparam integer HOST_TIMEOUT_US={EXPECTED_TIMEOUT_US};\n'
            f'localparam integer MAX_ZERO_AGE_US={MAX_ZERO_AGE_US};\n')
        source_rows=[metadata(SOURCE/n)for n in copied]
        status('source_and_image_bound_compiling',hardware_inputs=63,CPU_image_bytes=len(image),guard_symbol=address,
               admission_physical_bytes=mem.avail_phys,admission_commit_bytes=mem.avail_page,simulation_declaration_order_review=declaration_order_review)
        includes=sorted({str(out),str(out/'nav/rtl/regs'),str(out/'control')})
        env=os.environ.copy()
        if simulator=='modelsim':
            assert not any((p.info['name']or'').lower()in('vsim.exe','vsimk.exe')for p in psutil.process_iter(['name'])), 'single licensed simulation session is already occupied'
            license_file=Path('C:/Users/Li/Downloads/LR-204362_License.dat');assert license_file.is_file()
            env['MGLS_LICENSE_FILE']=str(license_file);env['LM_LICENSE_FILE']=str(license_file)
            (out/'run.do').write_text('onerror {quit -code 2}\nonbreak {quit -code 3}\nrun -all\nquit -code 0\n')
            cmds=[('vlib',['work']),('vlog',['-work','work',*['+incdir+'+p for p in includes],*[str(out/n)for n in copied]]),
                  ('vlog',['-sv','-work','work','+incdir+'+str(out),str(out/tb.name)]),
                  ('vsim',['-c','-t','1ps','work.tb_preview_full_uart_cpu','-do','run.do'])]
        else:
            cmds=[('xvlog',[arg for p in includes for arg in ['-i',p]]+[str(out/n)for n in copied]),
                  ('xvlog',['-sv','-i',str(out),str(out/tb.name)]),
                  ('xelab',['-debug','off','-timescale','1ns/1ps','tb_preview_full_uart_cpu','-s','full_uart_cpu']),
                  ('xsim',['full_uart_cpu','-runall'])]
        runs=[]
        for tool,args in cmds:
            log=out/(f'{len(runs)}_{tool}.log')
            command=([str(Path('E:/modelsim_ae/win32aloem')/(tool+'.exe')),*args]if simulator=='modelsim'else['cmd','/c',str(VIV/(tool+'.bat')),*args])
            with log.open('w')as f:
                child=subprocess.Popen(command,cwd=out,env=env,stdout=f,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
                status('running_'+tool,child_pid=child.pid,child_created_epoch=psutil.Process(child.pid).create_time(),command=command)
                code=child.wait()
            runs.append(dict(tool=tool,exit_code=code,log=str(log)))
            assert code==0,(tool,str(log))
        text=(out/('3_vsim.log'if simulator=='modelsim'else'xsim.log')).read_text(errors='replace');line=re.search(r'FULL_G2_UART_CPU TEST PASSED[^\r\n]*',text)
        assert line and not re.search(r'FATAL|Fatal:|ERROR:',text)
        metrics={k:int(v)for k,v in re.findall(r'(\w+)=(\d+)',line[0])}
        assert metrics['checks']>=8 and metrics['timeout_elapsed_us']>=EXPECTED_TIMEOUT_US
        if MAX_ZERO_AGE_US:
            assert metrics['zero_publication_age_us']<=MAX_ZERO_AGE_US
        for row in source_rows:assert metadata(Path(row['path']))==row
        result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),source=str(SOURCE),probe=str(out),metrics=metrics,
            hardware_inputs=63,CPU_image_bytes=len(image),image_bound_to_128KiB_INIT=True,actual_PicoRV32_and_firmware=True,
            actual_G2_top_UART_bridge_bus_arb_NAV_mailbox_SAFE_and_command_lease=True,
            host_initial_lock_timeout_rejection_and_explicit_idle_rearm=True,hardware_ESTOP_separate_from_mode4=True,
            simulation_core_MHz=50,hardware_time_not_accelerated=True,external_idle_sensors_only=True,no_force_or_CPU_stub=True,
            full_4500_inference_busy_planner_WCET_physical_or_vehicle_acceptance=False,programmed=False,new_SYN_PNR=False,
            source_bindings=source_rows,firmware_binding=metadata(SOURCE/'firmware/nav_fw.elf'),TB_binding=metadata(out/tb.name),
            simulation_declaration_order_review=declaration_order_review,
            simulator=simulator,license_child_environment_only=(simulator=='modelsim'),
            configured_host_timeout_us=EXPECTED_TIMEOUT_US,
            configured_zero_publication_age_limit_us=MAX_ZERO_AGE_US,
            physical_standstill_deadline_proven=False,
            runs=runs,scope='Finite real boot/UART/mailbox/CPU/SAFE/lease integration with idle external sensors and no populated map; not moving navigation, heavy A-star or full inference proof')
        save(out/'result.json',result);status('finite_actual_G2_CPU_UART_probe_passed',metrics=metrics);return result
    except Exception as error:
        import traceback
        failure=dict(error=str(error),traceback=traceback.format_exc());save(out/'failed.json',failure)
        status('failed_preserved_no_automatic_retry',**failure);raise

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--wait-for-memory',action='store_true');p.add_argument('--portable-oled-declaration',action='store_true');p.add_argument('--simulator',choices=['vivado','modelsim'],default='vivado')
    p.add_argument('--source',type=Path,default=SOURCE)
    p.add_argument('--timeout-us',type=int,choices=[250000,1000000],default=1000000)
    p.add_argument('--max-zero-age-us',type=int,default=0)
    a=p.parse_args();SOURCE=a.source.resolve();EXPECTED_TIMEOUT_US=a.timeout_us;MAX_ZERO_AGE_US=a.max_zero_age_us
    assert MAX_ZERO_AGE_US==0 or EXPECTED_TIMEOUT_US<MAX_ZERO_AGE_US<=500000
    r=run(a.out,a.wait_for_memory,a.portable_oled_declaration,a.simulator);print(json.dumps(dict(passed=r['passed'],metrics=r['metrics'])))
