"""Freeze a minimal RX re-arm candidate and finite wire/clock/reset tests.

Only a new independent F: directory is populated.  No running source, synthesis,
PnR, board image or service is modified.  Core50MHz and RX25MHz are simulated;
the implementation's more conservative RX100MHz SDC is not the PHY stimulus.
"""
import argparse
from datetime import datetime
import difflib
import json
from pathlib import Path
import shutil
import subprocess

from prepare_baseline_fusion_retime import metadata, save, same_bytes

ROOT=Path(__file__).resolve().parents[2]
BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'baseline_fusion_only_syn23'
FIFO=BASE/'fifo_async_assert_candidate3'
OUT=BASE/'mac_rx_boundary_candidate1'
VIV=Path('E:/2026.1/Vivado/bin')
MAC='nav/rtl/net/rgmii_mac100.v'
OVERRIDE="""        // The shared RX reset is asserted even with a stopped RX clock and
        // released only after RX clock edges.  Discard any current tail, then
        // use existing R_BAD to wait for real idle before preamble/SFD search.
        // Keep payload/CRC updates and normal-case publication edges intact.
        if (rx_rst) begin
            rs <= R_BAD;
            hold_v <= 1'b0;
            rx_valid <= 1'b0;
        end
"""

TB=r'''`timescale 1ns/1ps
module tb_mac_rx_boundary;
    reg clk=0,rxc=0,rx_run=1,arst=0;
    always #10 clk=~clk;
    always #20 if(rx_run)rxc=~rxc;
    wire rst,rxrst;
    nav_rst_sync u_rs(.clk(clk),.arst(arst),.rst(rst));
    nav_rst_sync u_lrr(.clk(rxc),.arst(rst),.rst(rxrst));
    reg [3:0] rxd=0;
    reg ctl=0,rx_neg=0;
    wire old_valid,new_valid,old_last,new_last,old_error,new_error;
    wire [7:0] old_data,new_data;
    rgmii_mac100_original original(.clk(clk),.rst(rst),.rgmii_rxc(rxc),.rgmii_rx_ctl(ctl),.rgmii_rxd(rxd),
        .mdio_i(1'b1),.rx_neg(rx_neg),.tx_valid(1'b0),.tx_data(8'd0),.tx_last(1'b0),
        .rx_valid(old_valid),.rx_data(old_data),.rx_last(old_last),.rx_error(old_error));
    rgmii_mac100 candidate(.clk(clk),.rst(rst),.rx_rst(rxrst),.rgmii_rxc(rxc),.rgmii_rx_ctl(ctl),.rgmii_rxd(rxd),
        .mdio_i(1'b1),.rx_neg(rx_neg),.tx_valid(1'b0),.tx_data(8'd0),.tx_last(1'b0),
        .rx_valid(new_valid),.rx_data(new_data),.rx_last(new_last),.rx_error(new_error));
    // Exact current net_port RX admission text, followed by archived FIFO3.
    wire [11:0] rx_wfree;
    reg in_frame,dropping;
    wire start_b=new_valid & ~in_frame;
    wire admit=start_b ? (rx_wfree>1536) : ~dropping;
    always @(posedge rxc)begin
        if(rxrst)begin in_frame<=0;dropping<=0;end
        else if(new_valid)begin
            if(start_b)dropping<=~(rx_wfree>1536);
            in_frame<=~new_last;
        end
    end
    wire fifo_valid;
    wire [9:0] fifo_data;
    nav_async_fifo #(.DW(10),.AW(11)) fifo(
        .wclk(rxc),.wrst(rxrst),.s_valid(new_valid&admit),.s_data({new_error,new_last,new_data}),.s_ready(),.wfree(rx_wfree),
        .rclk(clk),.rrst(rst),.m_valid(fifo_valid),.m_data(fifo_data),.m_ready(1'b1));
    reg [7:0] packet[0:1517];
    integer packet_len=0,mac_index=0,fifo_index=0;
    integer mac_frames=0,fifo_frames=0,normal_frames=0,reset_cases=0,stop_cases=0;
    integer byte_checks=0,cycle_checks=0,tail_checks=0,old_tail_bytes=0;
    reg compare=0,expect_packet=0,reject_tail=0,reject_fifo=0;
    function [31:0] crc_byte(input [31:0] c,input [7:0] data);
        reg [31:0] x;
        begin
            x=c^{24'd0,data};
            for(integer k=0;k<8;k=k+1)x=x[0]?((x>>1)^32'hedb88320):(x>>1);
            crc_byte=x;
        end
    endfunction
    task make_packet(input integer total,input integer tag);
        reg [31:0] crc;
        integer sum,iplen,udplen;
        begin
            packet_len=total;
            for(integer i=0;i<total;i=i+1)packet[i]=(i*29+tag*31)^(i>>2);
            // Legal Ethernet/IPv4/UDP packet; UDP checksum0 is permitted in IPv4.
            packet[0]=8'h02;packet[1]=8'h4e;packet[2]=8'h56;packet[3]=0;packet[4]=0;packet[5]=1;
            packet[6]=8'h02;packet[7]=8'h13;packet[8]=8'h37;packet[9]=0;packet[10]=0;packet[11]=2;
            packet[12]=8'h08;packet[13]=0;
            iplen=total-18;udplen=iplen-20;
            packet[14]=8'h45;packet[15]=0;packet[16]=iplen>>8;packet[17]=iplen;
            packet[18]=tag>>8;packet[19]=tag;packet[20]=0;packet[21]=0;packet[22]=64;packet[23]=17;
            packet[24]=0;packet[25]=0;
            packet[26]=192;packet[27]=168;packet[28]=0;packet[29]=101;
            packet[30]=192;packet[31]=168;packet[32]=0;packet[33]=100;
            sum=0;
            for(integer i=14;i<34;i=i+2)sum=sum+{packet[i],packet[i+1]};
            sum=(sum&65535)+(sum>>16);sum=(sum&65535)+(sum>>16);sum=(~sum)&65535;
            packet[24]=sum>>8;packet[25]=sum;
            packet[34]=8'h1e;packet[35]=8'h07;packet[36]=8'h1e;packet[37]=8'h07;
            packet[38]=udplen>>8;packet[39]=udplen;packet[40]=0;packet[41]=0;
            crc=32'hffffffff;
            for(integer i=0;i<total-4;i=i+1)crc=crc_byte(crc,packet[i]);
            crc=~crc;
            for(integer i=0;i<4;i=i+1)packet[total-4+i]=crc>>(8*i);
        end
    endtask
    // Actual100Mb RX: one nibble per40ns cycle, changes2ns after rising RXC;
    // both edge sampling paths receive the same real-wire waveform.
    task nibble(input [3:0] data,input bit dv);
        begin @(posedge rxc);#2;rxd=data;ctl=dv;end
    endtask
    task idle(input integer n);
        begin for(integer i=0;i<n;i=i+1)nibble(0,0);end
    endtask
    task preamble;
        begin for(integer i=0;i<15;i=i+1)nibble(5,1);nibble(13,1);end
    endtask
    task send_complete(input integer total,input integer tag);
        begin
            make_packet(total,tag);mac_index=0;fifo_index=0;expect_packet=1;compare=1;
            preamble();
            for(integer i=0;i<total;i=i+1)begin nibble(packet[i][3:0],1);nibble(packet[i][7:4],1);end
            // Exactly12-byte IFG:24 nibbles, not a25ns RX clock stimulus.
            idle(24);
            if(mac_index!=total || fifo_index!=total)$fatal(1,"fresh packet count mismatch mode=%0d mac=%0d fifo=%0d expected=%0d",rx_neg,mac_index,fifo_index,total);
            if(candidate.rs!=0)$fatal(1,"fresh packet did not return to idle");
            expect_packet=0;compare=0;normal_frames=normal_frames+1;
        end
    endtask
    always @(posedge rxc)begin
        #1;
        if(compare && !rxrst)begin
            cycle_checks=cycle_checks+1;
            if(new_valid!==old_valid)$fatal(1,"valid cycle mismatch mode=%0d",rx_neg);
            if(new_valid && {new_data,new_last,new_error}!=={old_data,old_last,old_error})$fatal(1,"wire byte/cycle mismatch mode=%0d",rx_neg);
        end
        if(reject_tail && !rxrst)begin
            tail_checks=tail_checks+1;
            if(new_valid)$fatal(1,"candidate accepted post-reset DV-high tail");
            if(old_valid)old_tail_bytes=old_tail_bytes+1;
        end
        if(expect_packet && new_valid && !rxrst)begin
            // With this measured +2ns PHY waveform the original rising-edge
            // path compares delayed dv_p with newer ctl_n at the final edge.
            // Preserve its final error1; falling-edge mode must have error0.
            if(mac_index>=packet_len || new_data!==packet[mac_index] || new_error!==(!rx_neg && mac_index==packet_len-1) || new_last!==(mac_index==packet_len-1))
                $fatal(1,"fresh MAC payload/FCS/last/error mismatch idx=%0d got=%h last=%b err=%b",mac_index,new_data,new_last,new_error);
            if(new_last && (candidate.rcrc!==32'hdebb20e3 || candidate.rerr!==!rx_neg))
                $fatal(1,"final CRC/residual or original sampling-mode error explanation mismatch");
            mac_index=mac_index+1;byte_checks=byte_checks+1;
            if(new_last)mac_frames=mac_frames+1;
        end
    end
    always @(posedge clk)begin
        if(!rst && fifo_valid)begin
            if(reject_fifo)$fatal(1,"FIFO replayed pre-reset or rejected-tail data");
            if(expect_packet)begin
                if(fifo_index>=packet_len || fifo_data[7:0]!==packet[fifo_index] || fifo_data[9]!==(!rx_neg && fifo_index==packet_len-1) || fifo_data[8]!==(fifo_index==packet_len-1))
                    $fatal(1,"FIFO fresh byte/last/error mismatch idx=%0d",fifo_index);
                fifo_index=fifo_index+1;
                if(fifo_data[8])fifo_frames=fifo_frames+1;
            end
        end
    end
    function phase_hit(input integer p,input integer byte_index);
        case(p)
          0:phase_hit=(candidate.rs==1);
          1:phase_hit=(candidate.rs==2 && candidate.nib==0 && candidate.hold_v==0);
          2:phase_hit=(candidate.rs==2 && candidate.nib==1 && candidate.hold_v==0);
          3:phase_hit=(candidate.rs==2 && candidate.nib==0 && candidate.hold_v==1);
          4:phase_hit=(candidate.rs==2 && candidate.nib==1 && candidate.hold_v==1);
          5:phase_hit=(candidate.rs==2 && byte_index>=packet_len-2);
          default:phase_hit=(candidate.rx_valid && candidate.rx_last);
        endcase
    endfunction
    task drive_to_phase(input integer p,input integer tag);
        begin : find_phase
            make_packet(128,tag);
            for(integer i=0;i<16;i=i+1)begin
                nibble(i==15?13:5,1);if(phase_hit(p,0))disable find_phase;
            end
            for(integer i=0;i<packet_len;i=i+1)begin
                nibble(packet[i][3:0],1);if(phase_hit(p,i))disable find_phase;
                nibble(packet[i][7:4],1);if(phase_hit(p,i))disable find_phase;
            end
            for(integer i=0;i<8;i=i+1)begin nibble(0,0);if(phase_hit(p,packet_len))disable find_phase;end
            $fatal(1,"RX phase not reached p=%0d mode=%0d",p,rx_neg);
        end
    endtask
    task fake_tail;
        begin
            // No DV-low delimiter occurs; apparent5/D and header-like bytes
            // are still part of the interrupted wire frame, never a fresh SOF.
            for(integer i=0;i<16;i=i+1)nibble(i==15?13:5,1);
            for(integer i=0;i<64;i=i+1)begin
                nibble(packet[i][3:0],1);nibble(packet[i][7:4],1);
            end
        end
    endtask
    task reset_running_case(input integer p,input integer tag);
        begin
            expect_packet=0;compare=0;reject_tail=0;reject_fifo=0;
            idle(24);drive_to_phase(p,tag);
            #3;arst=1;reject_tail=1;reject_fifo=1;
            repeat(8)@(negedge clk);#3;arst=0;
            wait(!rst && !rxrst);
            if(candidate.rs!==3 || candidate.hold_v!==0 || new_valid!==0)$fatal(1,"reset did not arm existing R_BAD wait-idle state");
            // p6 occurs after the real DV-low frame delimiter.  Its next
            // preamble is a legitimate new frame, not an interrupted tail.
            if(p!=6)fake_tail();
            idle(24);
            if(candidate.rs!==0)$fatal(1,"real idle did not re-arm preamble search");
            reject_tail=0;reject_fifo=0;
            send_complete(128,tag+100);reset_cases=reset_cases+1;
        end
    endtask
    task reset_stopped_case(input integer p,input integer tag);
        begin
            expect_packet=0;compare=0;reject_tail=0;reject_fifo=0;
            idle(24);drive_to_phase(p,tag);
            @(negedge rxc);#3;rx_run=0;arst=1;reject_tail=1;reject_fifo=1;
            repeat(8)@(negedge clk);#3;arst=0;
            repeat(24)@(negedge clk);
            if(rxrst!==1 || fifo.wp_b!==0 || fifo.rp_b!==0)$fatal(1,"stopped-clock reset did not hold FIFO closed");
            rx_run=1;wait(!rxrst);#1;
            if(candidate.rs!==3 || candidate.hold_v!==0 || new_valid!==0)$fatal(1,"RX resume released current tail");
            fake_tail();idle(24);
            reject_tail=0;reject_fifo=0;
            send_complete(128,tag+100);stop_cases=stop_cases+1;
        end
    endtask
    initial begin
        #1;arst=1;repeat(8)@(negedge clk);#3;arst=0;
        wait(!rst && !rxrst);idle(24);
        // Both modes, minimum/representative/MTU frame and minimum IFG.
        for(integer mode=0;mode<2;mode=mode+1)begin
            rx_neg=mode;idle(24);
            send_complete(64,mode*10+1);send_complete(128,mode*10+2);
            send_complete(1518,mode*10+3);send_complete(64,mode*10+4);
        end
        for(integer mode=0;mode<2;mode=mode+1)begin
            rx_neg=mode;idle(24);
            for(integer p=0;p<7;p=p+1)reset_running_case(p,200+mode*20+p);
            reset_stopped_case(0,400+mode*20);reset_stopped_case(4,401+mode*20);
        end
        if(reset_cases!=14 || stop_cases!=4 || normal_frames!=26 || mac_frames!=26 || fifo_frames!=26)
            $fatal(1,"coverage counts incomplete");
        if(old_tail_bytes==0)$fatal(1,"original RX-tail behavior not exercised");
        $display("MAC RX BOUNDARY TEST PASSED normal_frames=%0d mac_frames=%0d FIFO_frames=%0d byte_checks=%0d cycle_checks=%0d running_reset_cases=%0d stopped_RX_reset_cases=%0d reject_tail_cycle_checks=%0d original_tail_bytes=%0d candidate_tail_bytes=0 RX_modes=2 min_IFG_bytes=12 core_period_ns=20 RX_period_ns=40",
            normal_frames,mac_frames,fifo_frames,byte_checks,cycle_checks,reset_cases,stop_cases,tail_checks,old_tail_bytes);
        $finish;
    end
    initial begin #3000000;$fatal(1,"finite MAC recovery watchdog");end
endmodule
'''


def load(path):return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def prepare(out=OUT):
    out=Path(out).resolve();assert not out.exists();out.mkdir(parents=True)
    source=(SOURCE/MAC).read_text(encoding='utf-8')
    port='    input  wire       rst,\n'
    assert source.count(port)==1
    marker='''        default: if (!dv_s) rs <= R_IDLE;
        endcase
    end
    initial begin rs = R_IDLE;'''
    assert source.count(marker)==1
    candidate=source.replace(port,port+'    input  wire       rx_rst,       // existing RX-domain reset from top\n',1)
    candidate=candidate.replace(marker,marker.replace('    end\n    initial',OVERRIDE+'    end\n    initial'),1)
    inverse=candidate.replace(port+'    input  wire       rx_rst,       // existing RX-domain reset from top\n',port,1).replace(OVERRIDE,'',1)
    assert inverse==source
    top=(SOURCE/'g2_top.v').read_text(encoding='utf-8')
    start=top.index('    rgmii_mac100 #(.CLK_HZ(CLK_HZ)) u_lmac (')
    stop=top.index('    );',start)+len('    );')
    block=top[start:stop]
    assert block.count('.clk(clk), .rst(rst),')==1
    newtop=top[:start]+block.replace('.clk(clk), .rst(rst),','.clk(clk), .rst(rst), .rx_rst(l_rx_rst),',1)+top[stop:]
    assert newtop.replace('.clk(clk), .rst(rst), .rx_rst(l_rx_rst),','.clk(clk), .rst(rst),',1)==top
    assert 'nav_rst_sync u_lrr (.clk(l_rx_clk), .arst(rst), .rst(l_rx_rst));' in top
    ff=load(FIFO/'result.json');assert ff['passed'] and ff['modified_inputs']==['nav/rtl/common/nav_async_fifo.v']
    assert ff['public_nav_fifo_ob_byte_identical'] and ff['metrics']['candidate_stopped_write_replays']==0
    for item in ff['candidate_bindings']:
        assert metadata(Path(item['path']))==item
    for name,text in [('rgmii_mac100.v',candidate),('g2_top.v',newtop)]:
        (out/name).write_text(text,encoding='utf-8',newline='\n')
    shutil.copyfile(SOURCE/MAC,out/'original_rgmii_mac100.v')
    (out/'compiled_original_mac.v').write_text(source.replace('module rgmii_mac100 #(',
         'module rgmii_mac100_original #(',1),encoding='utf-8',newline='\n')
    shutil.copyfile(FIFO/'nav_async_fifo.v',out/'nav_async_fifo.v')
    shutil.copyfile(SOURCE/'nav/rtl/common/nav_cdc.v',out/'nav_cdc.v')
    assert same_bytes(FIFO/'nav_async_fifo.v',out/'nav_async_fifo.v')
    (out/'tb_mac_rx_boundary.sv').write_text(TB,encoding='utf-8',newline='\n')
    patches=[]
    for name,old,new in [(MAC,source,candidate),('g2_top.v',top,newtop)]:
        patches.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='a/'+name,tofile='b/'+name))
    (out/'MAC_RX_only.patch').write_text(''.join(patches),encoding='utf-8',newline='\n')
    runs=[]
    for tool,args in [('xvlog',[str(out/'compiled_original_mac.v'),str(out/'rgmii_mac100.v'),
                     str(out/'nav_async_fifo.v'),str(out/'nav_cdc.v')]),
                     ('xvlog',['-sv',str(out/'tb_mac_rx_boundary.sv')]),
                     ('xelab',['-debug','off','-timescale','1ns/1ps','tb_mac_rx_boundary','-s','snap']),
                     ('xsim',['snap','-runall'])]:
        logfile=out/(str(len(runs))+'_'+tool+'_stdout.log')
        with logfile.open('w',encoding='utf-8') as log:
            result=subprocess.run(['cmd','/c',str(VIV/(tool+'.bat')),*args],cwd=out,stdout=log,
                                  stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
        runs.append(dict(tool=tool,exit_code=result.returncode,log=str(logfile)))
        assert result.returncode==0,(tool,str(logfile))
    log=(out/'xsim.log').read_text(encoding='utf-8',errors='replace')
    import re
    assert 'MAC RX BOUNDARY TEST PASSED' in log and 'Fatal:' not in log
    summary=re.search(r'MAC RX BOUNDARY TEST PASSED[^\r\n]*',log)[0]
    metrics={key:int(value) for key,value in re.findall(r'(\w+)=(\d+)',summary)}
    assert metrics['running_reset_cases']==14 and metrics['stopped_RX_reset_cases']==4
    assert metrics['normal_frames']==metrics['mac_frames']==metrics['FIFO_frames']==26
    assert metrics['candidate_tail_bytes']==0 and metrics['original_tail_bytes']>0
    result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),source=str(SOURCE),candidate=str(out),
        modified_MAC_top_inputs=[MAC,'g2_top.v'],existing_reset_used='g2_top.l_rx_rst from u_lrr',
        unchanged_RX_body_except_last_priority_override=True,original_initial_register_values_retained=True,
        unchanged_TX_MDIO_samplers_clock_monitor_and_public_FPGA_IO=True,added_source_register_bits=0,
        physical_resource_mapping_not_measured=True,paired_FIFO_candidate=str(FIFO),
        archived_FIFO_bytes_identical=True,admission_fragment_matches_actual_net_port=True,
        metrics=metrics,runs=runs,root_and_RX_reset_topology_matches_actual_source=True,
        wire_stimulus=dict(core_period_ns=20,RX_period_ns=40,PHY_data_change_ns_after_RXC_rising=2,
                          actual_RX_mode='100Mb25MHz',implementation_SDC_RX_10ns_not_used_as_PHY_stimulus=True),
        proof_scope='Finite original/candidate byte-and-cycle comparison, falling-edge valid Ethernet/IPv4/UDP/FCS and rising-edge preservation of the original final RX-error behavior, MAC reset recovery and paired repaired FIFO; not full UDP/XT/application or physical acceptance',
        rising_edge_mode_diagnostic='Original and candidate retain final error1 for the +2ns PHY waveform: CRC residual is correct, rerr1 comes from dv_p/ctl_n sampling alignment. Falling-edge mode requires error0.',
        source_bindings=[metadata(SOURCE/p) for p in (MAC,'g2_top.v','nav/rtl/net/net_port.v','nav/rtl/common/nav_cdc.v')],
        candidate_bindings=[metadata(out/p) for p in ('rgmii_mac100.v','g2_top.v','nav_async_fifo.v','tb_mac_rx_boundary.sv')],
        FIFO_proof_binding=metadata(FIFO/'result.json'),no_internal_clock_pointer_or_calibration_force=True,
        existing_PNR37_PNR38_or_other_inputs_modified=False,new_SYN_or_PNR_started=False,
        serial_opened=False,programmed=False,deployment_accepted=False,
        remaining=['Actual RX-control FF/CE/INIT/reset fanout and new joint CDC/IO/timing/resource review',
                   'Whole UDP/XT/CPU behavior and physical PHY reset/idle sequencing',
                   'All possible asynchronous edge/reset/input sequences and MTBF are not proved by finite simulation'])
    save(out/'result.json',result);return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,default=OUT);args=parser.parse_args()
    import psutil
    proc=psutil.Process();proc.nice(psutil.IDLE_PRIORITY_CLASS);proc.cpu_affinity([18])
    result=prepare(args.out)
    print(json.dumps({key:result[key] for key in ('passed','candidate','metrics','added_source_register_bits','new_SYN_or_PNR_started')},ensure_ascii=False))
