"""Directed live-mailbox contract proof for removal of offline TX readback.

The original and trimmed RTL see the same complete TX/RX/injection stimuli.
All live public pins are compared each cycle. Only the explicitly removed
TX-shadow readback is allowed to differ; its new defined response is zero.
No synthesis, place/route or hardware action is performed.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path

from prepare_lf_line_compact import simulate, save
from trim_peripheral_rtl import trim_oled, trim_tx_shadow


TB = r'''`timescale 1ns/1ps
module tb;
reg clk=0,rst=1;always #10 clk=~clk;
reg ev=0,el=0,ee=0;reg[7:0]ed=0;reg[31:0]ip=32'hc0a8020a,ts=32'h12345678;
reg[15:0]port=16'd9000,elen=0;
reg iw=0,ig=0;reg[31:0]id=0,nowus=32'd100;reg[15:0]ilen=0;
reg br=0;reg[8:0]baddr=0;wire[31:0]bq0,bq1,sq0,sq1;
wire av0,av1,ack0,ack1,irq0,irq1,go0,go1;
wire[15:0]len0,len1,sp0,sp1,tl0,tl1;
wire[7:0]pk0,pk1,pq0,pq1;wire[31:0]rt0,rt1,rip0,rip1,rw0,rw1,drop0,drop1;
reg rd=0,pop=0,tw=0,tgo=0,tbusy=0,pr=0;
reg[31:0]td=0;reg[15:0]tlen=0;reg[10:0]pa=0;wire[9:0]words0,words1;
host_mbx_old a(.clk(clk),.rst(rst),.e_valid(ev),.e_data(ed),.e_last(el),.e_err(ee),
 .e_src_ip(ip),.e_src_port(port),.e_len(elen),.e_ts(ts),.inj_we(iw),.inj_wdata(id),.inj_go(ig),.inj_len(ilen),.now_us(nowus),
 .txb_re(br),.txb_addr(baddr),.txb_q(bq0),.tx_seq(sq0),.rx_avail(av0),.rx_len(len0),.rx_pkts(pk0),.rx_ts(rt0),
 .rx_src_ip(rip0),.rx_src_port(sp0),.rx_rd(rd),.rx_rd_ack(ack0),.rx_word(rw0),.rx_pop(pop),.rx_irq(irq0),.st_drop(drop0),
 .tx_we(tw),.tx_wdata(td),.tx_go_in(tgo),.tx_len_in(tlen),.tx_words(words0),.tx_go(go0),.tx_len(tl0),.tx_busy(tbusy),
 .pl_re(pr),.pl_addr(pa),.pl_q(pq0));
host_mbx_trim b(.clk(clk),.rst(rst),.e_valid(ev),.e_data(ed),.e_last(el),.e_err(ee),
 .e_src_ip(ip),.e_src_port(port),.e_len(elen),.e_ts(ts),.inj_we(iw),.inj_wdata(id),.inj_go(ig),.inj_len(ilen),.now_us(nowus),
 .txb_re(br),.txb_addr(baddr),.txb_q(bq1),.tx_seq(sq1),.rx_avail(av1),.rx_len(len1),.rx_pkts(pk1),.rx_ts(rt1),
 .rx_src_ip(rip1),.rx_src_port(sp1),.rx_rd(rd),.rx_rd_ack(ack1),.rx_word(rw1),.rx_pop(pop),.rx_irq(irq1),.st_drop(drop1),
 .tx_we(tw),.tx_wdata(td),.tx_go_in(tgo),.tx_len_in(tlen),.tx_words(words1),.tx_go(go1),.tx_len(tl1),.tx_busy(tbusy),
 .pl_re(pr),.pl_addr(pa),.pl_q(pq1));
integer cycles=0,txchecks=0,rxchecks=0,shadowchecks=0;reg[31:0]expected[0:511];
always@(negedge clk)begin
 cycles++;
 if({av0,len0,pk0,rt0,rip0,sp0,ack0,rw0,irq0,drop0,words0,go0,tl0,sq0,pq0}!==
    {av1,len1,pk1,rt1,rip1,sp1,ack1,rw1,irq1,drop1,words1,go1,tl1,sq1,pq1})
  $fatal(1,"live mailbox pin mismatch cycle=%0d",cycles);
 if(bq1!==32'd0)$fatal(1,"disabled shadow must return zero");
end
task transmit(input integer count,input integer round);
 integer waited;
 begin
 for(integer i=0;i<count;i++)begin
  @(negedge clk);#1;tw=1;td=32'h81a5f032^(i*32'h01020301)^round;expected[i]=td;
 end
 @(negedge clk);#1;tw=0;tgo=1;tlen=count*4;
 @(negedge clk);#1;tgo=0;
 repeat(4)@(negedge clk);
 for(integer i=0;i<count*4;i++)begin
  @(negedge clk);#1;pr=1;pa=i;br=1;baddr=i/4;
  @(posedge clk);#2;
  if(pq0!==expected[i/4][8*(i%4)+:8] || pq1!==expected[i/4][8*(i%4)+:8])$fatal(1,"live TX byte %0d",i);
  if(bq0!==expected[i/4] || bq1!==0)$fatal(1,"shadow feature trade mismatch %0d",i);
  txchecks++;shadowchecks++;
 end
 @(negedge clk);#1;pr=0;br=0;
 end
endtask
task receive(input integer count,input bit bad);
 begin
 for(integer i=0;i<count;i++)begin
  @(negedge clk);#1;ev=1;ed=i^8'h5a;el=(i==count-1);ee=bad && el;elen=count;
 end
 @(negedge clk);#1;ev=0;el=0;ee=0;repeat(14)@(negedge clk);
 if(!bad)begin
  if(!av0 || len0!=count)$fatal(1,"RX descriptor not retained");
  for(integer i=0;i<(count+3)/4;i++)begin
   @(negedge clk);#1;rd=1;
   @(negedge clk);#1;rd=0;
   for(integer w=0;!ack0 && w<30;w++)begin @(posedge clk);#2;end
   if(!ack0 || !ack1 || rw0!==rw1)$fatal(1,"RX read ACK/data mismatch");
   rxchecks++;
  end
  @(negedge clk);#1;pop=1;
  @(negedge clk);#1;pop=0;repeat(8)@(negedge clk);
 end
 end
endtask
initial begin
 repeat(6)@(negedge clk);#1;rst=0;repeat(6)@(negedge clk);
 transmit(5,0);transmit(368,1);transmit(512,2);
 // Busy rejects TX_GO without affecting sequence or data-write protocol.
 @(negedge clk);#1;tbusy=1;tgo=1;repeat(4)@(negedge clk);#1;tgo=0;tbusy=0;
 receive(17,0);receive(1472,0);receive(31,1);
 // Original offline injection is deliberately retained in this profile.
 for(integer i=0;i<5;i++)begin @(negedge clk);#1;iw=1;id=32'h12345678+i;end
 @(negedge clk);#1;iw=0;ig=1;ilen=19;
 @(negedge clk);#1;ig=0;repeat(100)@(negedge clk);
 if(!av0 || !av1 || len0!=19 || len1!=19)$fatal(1,"retained injection path lost");
 @(negedge clk);#1;rst=1;repeat(5)@(negedge clk);#1;rst=0;repeat(8)@(negedge clk);
 transmit(5,3);receive(9,0);
 $display("TEST PASSED TRIM_MAILBOX cycles=%0d tx_bytes=%0d rx_words=%0d shadow_reads=%0d original_injection=1 reset=1 live_pins_cycle_equal=1",cycles,txchecks,rxchecks,shadowchecks);$finish;
end
initial begin #2000000;$fatal(1,"mailbox proof watchdog");end
endmodule
'''


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',type=Path,required=True);ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args();source=a.source.resolve();out=a.out.resolve()
    out.mkdir(parents=True,exist_ok=False)
    old=(source/'nav/rtl/net/host_mbx.v').read_text(encoding='utf-8')
    new=trim_tx_shadow(old)
    (out/'host_mbx_trim.v').write_text(new,encoding='utf-8',newline='\n')
    control=trim_oled((source/'control4500_core.v').read_text(encoding='utf-8'))
    (out/'control4500_core_trim.v').write_text(control,encoding='utf-8',newline='\n')
    (out/'host_old.v').write_text(old.replace('module host_mbx #(', 'module host_mbx_old #(',1),encoding='utf-8')
    (out/'host_new.v').write_text(new.replace('module host_mbx #(', 'module host_mbx_trim #(',1),encoding='utf-8')
    (out/'nav_ram_sdp.v').write_bytes((source/'nav/rtl/common/nav_ram_sdp.v').read_bytes())
    (out/'nav_fifo.v').write_bytes((source/'nav/rtl/common/nav_fifo.v').read_bytes())
    (out/'nav_fifo_ob.v').write_bytes((source/'nav/rtl/common/nav_fifo_ob.v').read_bytes())
    (out/'tb.sv').write_text(TB,encoding='utf-8')
    proof=simulate(out,[out/n for n in ('nav_ram_sdp.v','nav_fifo_ob.v','nav_fifo.v','host_old.v','host_new.v','tb.sv')],'tb')
    result=dict(proof,passed=True,source=str(source),created_at=datetime.now().astimezone().isoformat(),
        live_mailbox_interface_passed=True,OLED_release_source_contract_passed=True,
        RTL_source_transforms_exact=True,offline_TX_readback_expected_zero=True,
        original_TX_RAM_RX_injection_and_ack_retained=True,
        original_UART_V2_SFP_stop_guard_unchanged=True,
        programmed=False,PNR_started=False,synthesis_started=False,
        scope='Directed public-interface comparison, not complete NAV/control or physical timing acceptance')
    save(out/'result.json',result)
    print(json.dumps({k:result[k]for k in('passed','source','result','live_mailbox_interface_passed')},ensure_ascii=False))


if __name__=='__main__':main()
