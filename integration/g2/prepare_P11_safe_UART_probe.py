"""Prepare finite serial/actual-regfile probe as strings, without invoking EDA.

Default output is JSON only. Explicit --materialize is gated by PNR54 and SYN44
natural exit and creates a fresh test overlay. The test itself is unexecuted.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,shutil
from prepare_baseline_fusion_retime import metadata,save
import prepare_P11_safe_UART_RTL_overlay as hw

TB=r'''`timescale 1ns/1ps
module p11_safe_uart_tb;
 reg clk=0,clock_on=1;always #10 if(clock_on)clk=~clk;
 reg rst=1,rx=1,wr=0,rd=0;reg[15:0]addr=0;reg[31:0]wdata=0;
 reg[5:0]us_div=0;reg tick_us=0;reg[31:0]time_us=0;
 always @(posedge clk)begin
   if(rst)begin us_div<=0;tick_us<=0;time_us<=0;end
   else if(us_div==49)begin us_div<=0;tick_us<=1;time_us<=time_us+1;end
   else begin us_div<=us_div+1;tick_us<=0;end
 end
 wire[31:0]rdata,cmd,stamp,seq,ovf,tsdata;wire hit,tx,ready,rv,first,tv;
 wire[7:0]data;wire[9:0]count;
 p11_csr_probe_harness h(.clk(clk),.rst(rst),.tick_us(tick_us),.time_us(time_us),
   .uart_rx(rx),.wr(wr),.rd(rd),.addr(addr),.wdata(wdata),.rdata(rdata),.hit(hit),
   .uart_tx(tx),.source_ready(ready),.cmd(cmd),.stamp(stamp),.seq(seq),.ovf(ovf),
   .rx_count(count),.rx_valid(rv),.rx_first(first),.ts_valid(tv),.rx_data(data),.ts_data(tsdata));
 // Actual original receiver, sharing inputs and generated CSR pop/config/TX.
 // Compare RX only before first administrative flush; TX for the whole run.
 wire ref_tx,ref_rv,ref_first,ref_tv,ref_busy,ref_full,ref_irq;
 wire[7:0]ref_data;wire[9:0]ref_count;wire[31:0]ref_ts,ref_ovf;
 imu_uart_reference refu(.clk(clk),.rst(rst),.time_us(time_us),.tick_us(tick_us),
   .cfg_div(h.r_IMU_BAUD_DIV[15:0]),.cfg_idle_us(h.r_IMU_IDLE_US[15:0]),
   .uart_rx(rx),.uart_tx(ref_tx),.rx_valid(ref_rv),.rx_data(ref_data),.rx_first(ref_first),
   .rx_pop(h.p_IMU_RX_DATA&ref_rv),.rx_count(ref_count),.ts_valid(ref_tv),.ts_data(ref_ts),
   .ts_pop(h.p_IMU_RX_TS&ref_tv),.st_ovf(ref_ovf),.irq(ref_irq),
   .tx_we(h.p_IMU_TX_DATA),.tx_data(wdata[7:0]),.tx_busy(ref_busy),.tx_full(ref_full));
 integer checks=0,serial_chars=0,frames=0,flushes=0,csr_reads=0,ts_reads=0;
 reg compare_raw=0;
 task check;input bit value;input[255:0]label;
 begin if(value!==1'b1)$fatal(1,"P11_FAIL %0s time%0t",label,$time);checks=checks+1;end endtask
 always @(negedge clk)if(!rst)begin
   check(tx===ref_tx,"TX literal miter");
   check(!$isunknown({ready,cmd,stamp,seq,ovf}),"known snapshot/status");
   if(compare_raw)begin
     check({rv,tv,count,ovf}==={ref_rv,ref_tv,ref_count,ref_ovf},"raw status/count miter");
     if(rv)check({first,data}==={ref_first,ref_data},"raw DATA/FIRST miter");
     if(tv)check(tsdata===ref_ts,"raw TS miter");
   end
 end
 task clocks;input integer n;integer k;begin for(k=0;k<n;k=k+1)@(negedge clk);end endtask
 task write_csr;input[15:0]a;input[31:0]v;
 begin @(negedge clk);addr=a;wdata=v;wr=1;rd=0;@(negedge clk);wr=0;clocks(2);end endtask
 task read_csr;input[15:0]a;output[31:0]v;
 begin @(negedge clk);addr=a;wr=0;rd=1;#1;check(hit===1,"CSR hit");v=rdata;
   @(negedge clk);rd=0;csr_reads=csr_reads+1;end endtask
 task byte_tx;input[7:0]b;input bit stop_ok;integer k;
 begin @(negedge clk);rx=0;clocks(217);
   for(k=0;k<8;k=k+1)begin rx=b[k];clocks(217);end
   rx=stop_ok;clocks(217);rx=1;serial_chars=serial_chars+1;
 end endtask
 function[15:0]crc_byte;input[15:0]old;input[7:0]b;reg[15:0]c;integer k;
 begin c=old^{b,8'd0};for(k=0;k<8;k=k+1)c=c[15]?(c<<1)^16'h1021:(c<<1);crc_byte=c;end endfunction
 task packet;input bit legacy;input signed[15:0]v,w;input bit badcrc;reg[15:0]crc;
 begin crc=crc_byte(16'hffff,8'h10);crc=crc_byte(crc,legacy?8'h02:8'h04);
   crc=crc_byte(crc,v[7:0]);crc=crc_byte(crc,v[15:8]);
   if(!legacy)begin crc=crc_byte(crc,w[7:0]);crc=crc_byte(crc,w[15:8]);end
   byte_tx(8'ha5,1);byte_tx(8'h5a,1);byte_tx(8'h10,1);byte_tx(legacy?8'h02:8'h04,1);
   byte_tx(v[7:0],1);byte_tx(v[15:8],1);
   if(!legacy)begin byte_tx(w[7:0],1);byte_tx(w[15:8],1);end
   byte_tx(crc[7:0]^(badcrc?8'h01:0),1);byte_tx(crc[15:8],1);clocks(8);frames=frames+1;
 end endtask
 task qualify;integer k;
 begin k=0;while(!ready&&k<160000)begin clocks(1);k=k+1;end
   check(ready===1,"bounded 2ms cleanup readiness");end endtask
 task snap;input[31:0]expected_seq,expected_cmd;reg[31:0]a,b,c,d;
 begin read_csr(16'h602c,a);read_csr(16'h6024,b);read_csr(16'h6028,c);read_csr(16'h602c,d);
   check(a===expected_seq&&d===a,"coherent four-read SEQ");check(b===expected_cmd,"snapshot command exact");
   check(c===stamp,"snapshot actual TS");end endtask
 integer i,phase;reg[31:0]v,oldseq,start_us,oldstamp;
 initial begin
   clocks(8);rst=0;clocks(4);write_csr(16'h6000,217);write_csr(16'h6018,500);
   compare_raw=1;
   // Disabled parser leaves raw/WIT behavior intact; no fabricated snapshot.
   byte_tx(8'h55,1);byte_tx(8'h52,1);byte_tx(8'h01,1);clocks(2000);
   check(seq===0,"boot parser disabled");read_csr(16'h6004,v);read_csr(16'h600c,v);
   write_csr(16'h6010,8'ha6);write_csr(16'h6010,8'h5b);
   compare_raw=0;write_csr(16'h6020,2);flushes=flushes+1;
   check(!ready&&seq===0,"CTRL2 current write data flush");
   rx=0;clocks(110000);check(!ready,"LOW never ready");rx=1;start_us=time_us;
   clocks(45000);rx=0;clocks(1);rx=1;start_us=time_us;
   clocks(90000);check(!ready,"between-tick LOW restarts");qualify();
   check((time_us-start_us)>=2000,"actual high duration minimum");
   check(!rv&&!tv&&count===0,"all old RX queues cleared");
   write_csr(16'h6020,1);check(ready,"enable-only retains READY");
   clocks(26000);start_us=time_us;packet(0,16'sd600,-16'sd2900,0);
   snap(1,{16'(-2900),16'd600});check(stamp>=start_us&&stamp<=start_us+1,"FIRST start timestamp");
   oldstamp=stamp;packet(0,-16'sd600,16'sd2900,0);
   check(seq===1&&stamp===oldstamp,"noidle duplicate burst cannot renew");
   clocks(26000);packet(1,-16'sd600,0,0);snap(2,{16'd0,16'(-600)});
   clocks(26000);packet(0,16'sd601,0,0);check(seq===2,"range reject forward");
   clocks(26000);packet(0,0,16'sd2901,0);check(seq===2,"range reject yaw");
   clocks(26000);packet(0,0,0,1);check(seq===2,"CRC reject");
   clocks(26000);start_us=time_us;byte_tx(8'hcc,1);packet(0,0,0,0);
   snap(3,0);check(stamp>=start_us&&stamp<=start_us+1,"leadingnoise conservativeTS");
   // Real full raw queues do not block the tap/parser. No force to queue state.
   for(i=0;i<270;i=i+1)byte_tx(i[7:0],1);
   check(count>=256,"production raw queue naturally full");check(ovf!=0,"real DATA overflow counted");
   clocks(26000);packet(0,1,2,0);snap(4,{16'd2,16'd1});
   // Independently fill the TS queue by real idle-separated FIRST characters.
   // Original nav_fifo capacity is RAM32 plus two FWFT slots, not merely32.
   for(i=0;i<36;i=i+1)begin clocks(26000);byte_tx(8'hcc,1);end
   clocks(26000);packet(0,2,3,0);snap(5,{16'd3,16'd2});
   for(i=0;i<34;i=i+1)begin check(tv,"naturally full TS queue");read_csr(16'h600c,v);ts_reads=ts_reads+1;end
   clocks(8);check(!tv,"actual34 TS capacity drained");
   // Flush every physical character phase. The old reference TX keeps running.
   for(phase=0;phase<10;phase=phase+1)begin
     write_csr(16'h6010,phase);oldseq=seq;
     fork begin byte_tx(8'ha5,1);end
          begin clocks(phase*217+5);write_csr(16'h6020,2);flushes=flushes+1;end join
     check(seq===0&&!ready,"flush overrides in-flight RX/snapshot");
     qualify();check(!rv&&!tv&&count===0,"no old queue after phaseflush");
     write_csr(16'h6020,1);clocks(26000);packet(0,0,0,0);snap(1,0);
   end
   // Config changes while isolated restart qualification; ACTIVE normal LOW
   // retains READY. Clock pause is not simulated time_us advancement.
   write_csr(16'h6020,2);flushes=flushes+1;clocks(90000);
   write_csr(16'h6018,501);start_us=time_us;clocks(80000);check(!ready,"cfg restart");qualify();
   check(time_us-start_us>=2000,"cfg high-time requalification");
   rx=0;clocks(20);check(ready,"ACTIVE start keeps sticky READY");rx=1;
   @(negedge clk);clock_on=0;#10000;clock_on=1;clocks(4);check(ready,"clock pause no cleanup replay");
   check(!wr&&!rd,"bus idle at finish");
   $display("P11_SAFE_UART_PASS checks=%0d serial_chars=%0d frames=%0d flushes=%0d csr_reads=%0d ts_reads=%0d",checks,serial_chars,frames,flushes,csr_reads,ts_reads);
   $finish;
 end
 initial begin #100000000;$fatal(1,"finite100ms stimulus watchdog");end
endmodule
'''

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--source',type=Path,default=hw.SOURCE);p.add_argument('--materialize',action='store_true');a=p.parse_args()
    assert not a.out.exists();gates=dict(PNR54=hw.gate54(a.materialize),SYN44=hw.gate44(a.materialize))
    review,uart,top,cfg=hw.source_review(a.source)
    if a.materialize:
        hw.materialize(a.source,a.out,review,uart,top,cfg)
        (a.out/'test/p11_safe_uart_tb.sv').write_text(TB,encoding='utf8')
        for rel in review['support']:
            shutil.copy2(a.source/rel,a.out/'test'/Path(rel).name)
    else:a.out.mkdir()
    r=dict(at=datetime.now().astimezone().isoformat(),source=str(a.source),gates=gates,
      RTL_materialized=a.materialize,tools_started=False,FW_recompiled=False,
      frozen_strings=dict(TB_bytes=len(TB.encode()),actual_generated_regfile_harness=True,original_UART_miter=True),
      test_scope='actualserial217clkbit/50MHz, actualgeneratednav_regfile, production256byte/32TS RAM+twoFWFTslots; noCPU/wholeNAV/physicalpinclaim',
      intended_cases=['disabledparserrawmiter','TXallcyclesmiter','CTRL2wdnotoldr','LOWforever/noREADY',
        'LOWbetweenus ticks','2001HIGHqualification','FIRSTstarttimestamp','legacy/newCRC/range',
        'noidleburstsinglepublication','leadingnoiseearlyTS','naturallyfullrawandTSFIFO','10serialphaseflushes',
        'cfgqualificationrestart','stickREADYafterACTIVEstart','clockpause'],
      not_yet_covered=['arbitraryfour-readpublicationcuts/saturationformal','fullNAVbusACK/CPUHOLDexercise',
        'allCRCsingle-bitmutations/everyrangevalue','allindependentDATA-TSfullpermutations',
        'actualLUTFF/CDC/reset/STA','selectedFWCPUbusy250/350msandphysical500ms'],
      future_commands=[['xvlog.bat','--sv','test/nav_cdc.v','test/nav_ram_sdp.v','test/nav_fifo_ob.v','test/nav_fifo.v','test/imu_uart_reference.v',
        'nav/rtl/io/imu_uart.v','nav/rtl/io/p11_command_snapshot.v','nav/rtl/regs/nav_regfile.v',
        'test/p11_csr_probe_harness.sv','test/p11_safe_uart_tb.sv'],
        ['xelab.bat','work.p11_safe_uart_tb','-s','p11_safe_uart'],['xsim.bat','p11_safe_uart','-runall']],
      execution_gate='noautomatictool; rootadmissionafterallRTL/TBfrozen, eachtool2GiBphys/4GiBcommit, actualPID/create/log/PASS/noFatalmustbecollected',
      no_test_result_claimed=True,deployment_ready=False)
    hw.save(a.out/'result.json',r)
    if not a.materialize:
        assert not list(a.out.rglob('*.v'))and not list(a.out.rglob('*.sv'))
        save(hw.REPO/'docs/4_design/evidence/g2_20261023/P11_safe_UART_serial_CSR_probe_plan_20261026.json',r)
    print(json.dumps(dict(output=str(a.out),materialized=a.materialize,tools=False)))

if __name__=='__main__':main()
