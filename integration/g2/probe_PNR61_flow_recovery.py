"""Current MAC/FIFO/XT: admission, queued pose, long stall and clock recovery.

Only public TB inputs/clocks change. Production RTL, current FPGA and the
previous passed probe remain untouched. Finite sequences are not analog STA.
"""
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess

from probe_mac_udp_xt_reset import NAMES,VIV
from retry_frozen_synthesis import memory_status

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'K_timing_syn49'
REF=BASE/'PNR61_MAC_UDP_XT_pipeline_probe2'
OUT=BASE/'PNR61_flow_reset_clock_recovery_probe2'


def once(s,a,b):
    assert s.count(a)==1,(a,s.count(a));return s.replace(a,b,1)


def main():
    assert not OUT.exists();m=memory_status()
    assert m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3
    OUT.mkdir()
    for n in NAMES:shutil.copyfile(SOURCE/n,OUT/Path(n).name)
    for name in ('wire.hex','packets.hex','groups.hex','pixels.hex'):shutil.copyfile(REF/name,OUT/name)
    s=(REF/'tb_mac_udp_xt_reset.sv').read_text()
    s=once(s,'reg clk=0,rxc=0,rx_run=1,arst=0;',
        'reg clk=0,rxc=0,rx_run=1,core_run=1,arst=0;\n  reg admit=1,wait_pose=0,pose=0;\n  wire accepted,pose_waiting;\n  integer accept_count=0,admit_checks=0,pose_checks=0,clock_checks=0;\n  integer stall_left=0;reg long_stall_done=0,clock_pause_done=0;')
    s=once(s,'reg held=0;reg [33:0] held_pixel;',
        'reg held=0;reg [33:0] held_pixel;\n  always @(posedge clk)if(!rst && accepted)accept_count=accept_count+1;\n  always @(negedge clk)begin\n    if(stall_left>0)stall_left=stall_left-1;\n    else if(expected_sn==3 && monitor_pixel && pixel_index>=97 && !long_stall_done)begin stall_left=640;long_stall_done=1;end\n  end')
    s=once(s,'always #10 clk=~clk;','always #10 if(core_run)clk=~clk;')
    s=once(s,'wire ready=(ticks[2:0]!=0 && ticks[2:0]!=3);','wire ready=(stall_left==0 && ticks[2:0]!=0 && ticks[2:0]!=3);')
    s=once(s,".cfg_wait_pose(1'b0),.read_admit(1'b1),.frame_accept(),.pose_commit(1'b0),.cfg_retain(1'b0),",
        ".cfg_wait_pose(wait_pose),.read_admit(admit),.frame_accept(accepted),.pose_commit(pose),.cfg_retain(1'b0),.pose_waiting(pose_waiting),")
    s=once(s,'v=wire_bytes[off+j];if(fcs_bad && j==n-1)v=v^1;\n        nibble(v[3:0],1);nibble(v[7:4],1);',r'''v=wire_bytes[off+j];if(fcs_bad && j==n-1)v=v^1;
        nibble(v[3:0],1);nibble(v[7:4],1);
        if(index==groups[3][31:16] && j==10 && !clock_pause_done)begin
          // Pause the reader with RX active; fewer than2048 bytes can arrive.
          @(negedge clk);core_run=0;
          fork begin #30000;core_run=1;clock_checks=clock_checks+1;end join_none
        end
        if(index==groups[3][31:16] && j==500 && !clock_pause_done)begin
          // Pause PHY and its stimulus together without reset, then resume.
          @(negedge rxc);rx_run=0;#6000;rx_run=1;
          clock_checks=clock_checks+1;clock_pause_done=1;
        end''')
    s=once(s,'expected_sn=k+1;pixel_base=k*9600;pixel_index=0;monitor_pixel=!corrupt;',r'''expected_sn=k+1;pixel_base=k*9600;pixel_index=0;monitor_pixel=!corrupt;
      admit=(k>=2);wait_pose=(k==1);pose=0;''')
    s=once(s,'for(integer j=0;j<n;j=j+1)packet(first+j,corrupt&&j==n-1);\n      repeat(20000)@(negedge clk);',r'''for(integer j=0;j<n;j=j+1)packet(first+j,corrupt&&j==n-1);
      if(k<2)begin
        wait(pose_waiting);
        if(k==1)begin @(negedge clk);pose=1;@(negedge clk);pose=0;end
        repeat(512)@(negedge clk);
        if(pixel_index!=0 || accepted)$fatal(1,"frame escaped before owner admission");
        admit_checks=admit_checks+1;
        @(negedge clk);admit=1;
        if(k==1)pose_checks=pose_checks+1;
      end
      repeat(20000)@(negedge clk);''')
    s=once(s,'if(frame_checks!=4 || pixel_checks!=38400 || reset_checks!=2 || stalls==0)$fatal(1,"coverage incomplete");',
        'if(frame_checks!=4 || pixel_checks!=38400 || reset_checks!=2 || stalls==0 || admit_checks!=2 || pose_checks!=1 || clock_checks!=2 || !long_stall_done || accept_count!=4)$fatal(1,"extended flow/recovery coverage incomplete");\n    $display("FLOW RECOVERY PASSED admissions=%0d pending_pose=%0d clock_pauses=%0d frame_accepts=%0d",admit_checks,pose_checks,clock_checks,accept_count);')
    (OUT/'tb_mac_udp_xt_reset.sv').write_text(s,encoding='utf8')
    state=dict(started_at=datetime.now().astimezone().isoformat(),phase='running',source=str(SOURCE),
        reference=str(REF),production_RTL_modified=False,hardware_access=False,
        scope='Public owner admission/pose pulse,640-cycle output stall,30us core pause,6us RX pause,existing two common resets and FCS rejection')
    (OUT/'job.json').write_text(json.dumps(state,indent=2))
    for i,(tool,args) in enumerate([
        ('xvlog',[str(OUT/Path(n).name) for n in NAMES]),('xvlog',['-sv',str(OUT/'tb_mac_udp_xt_reset.sv')]),
        ('xelab',['-debug','off','-timescale','1ns/1ps','tb_mac_udp_xt_reset','-s','flow']),('xsim',['flow','-runall'])]):
        with (OUT/f'{i}_{tool}.log').open('w') as f:
            r=subprocess.run(['cmd','/c',str(VIV/(tool+'.bat')),*args],cwd=OUT,stdout=f,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
        assert r.returncode==0,(tool,OUT/f'{i}_{tool}.log')
    log=(OUT/'xsim.log').read_text(errors='replace')
    assert 'FLOW RECOVERY PASSED' in log and 'MAC UDP XT RESET PASSED' in log and 'Fatal:' not in log
    metrics={k:int(v) for k,v in re.findall(r'(\w+)=(\d+)',log)}
    assert all((SOURCE/n).read_bytes()==(OUT/Path(n).name).read_bytes() for n in NAMES)
    state.update(phase='complete',passed=True,metrics=metrics,ended_at=datetime.now().astimezone().isoformat(),
        all_current_RTL_bytes_bound=True,whole_G2_or_physical_clock_loss_fail_safe_proved=False)
    (OUT/'result.json').write_text(json.dumps(state,indent=2));(OUT/'job.json').write_text(json.dumps(state,indent=2))
    print(json.dumps(dict(passed=True,metrics=metrics,out=str(OUT))))


if __name__=='__main__':main()
