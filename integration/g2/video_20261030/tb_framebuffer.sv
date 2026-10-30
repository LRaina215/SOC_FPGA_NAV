`timescale 1ns/1ps
module tb_framebuffer;
 reg wc=0,rc=0,run_read=1,reset_n=0,pv=0,wb=0,req=0,fs=0,re=0,inv=0;
 reg [16:0] wa=0;reg [14:0] ra=0;reg [15:0] wd=0;
 wire ack,ready;wire [15:0] q;
 integer checks=0,i;
 always #5 wc=~wc;
 always #7 if(run_read)rc=~rc;
 g2_video_framebuffer dut(.write_clk(wc),.write_reset_n(reset_n),.pixel_valid(pv),
  .write_bank(wb),.pixel_addr(wa),.pixel_data(wd),.frame_req(req),
  .read_clk(rc),.read_reset_n(reset_n),.frame_start(fs),.read_enable(re),.read_addr(ra),
  .invalidate(inv),.frame_ack(ack),.display_ready(ready),.read_data(q));
 function [15:0] pattern(input integer a,input integer epoch);
  pattern=(a*17)^(epoch*16'h359b);
 endfunction
 task fill(input integer epoch);
  begin
   for(integer a=0;a<19200;a=a+1)begin
    @(negedge wc);pv=1;wa=a;wd=pattern(a,epoch);
   end
   @(negedge wc);pv=0;req=~req;
  end
 endtask
 task swap;
  begin
   repeat(4)@(negedge rc);
   if(ack===req)$fatal(1,"swapped before frame start");
   fs=1;@(negedge rc);fs=0;
   if(ack!==req || ready!==1)$fatal(1,"missing frame-boundary commit");
   checks=checks+1;
  end
 endtask
 task read_frame(input integer epoch);
  begin
   for(integer a=0;a<19200;a=a+1)begin
    @(negedge rc);re=1;ra=a;@(posedge rc);#1;
    if(q!==pattern(a,epoch))$fatal(1,"pixel a=%0d epoch=%0d q=%h",a,epoch,q);
    checks=checks+1;
   end
   @(negedge rc);re=0;
  end
 endtask
 initial begin
  #31;reset_n=1;repeat(4)@(negedge rc);
  if(ready!==0)$fatal(1,"unwritten RAM exposed");
  fill(1);swap();
  wb=1;
  fork fill(2);read_frame(1);join
  if(ack===req)$fatal(1,"published without frame boundary");
  swap();read_frame(2);
  // Stopped LCD clock holds ownership/ack while the other bank completes.
  @(negedge rc);run_read=0;wb=0;fill(3);
  repeat(12)@(negedge wc);
  if(ack===req)$fatal(1,"ack advanced during stopped read clock");
  run_read=1;swap();read_frame(3);
  // Invalid write address must neither wrap nor corrupt a valid pixel.
  wb=0;@(negedge wc);pv=1;wa=17'd65536;wd=16'hdead;
  @(negedge wc);wa=17'd19200;@(negedge wc);pv=0;
  @(negedge rc);ra=0;re=1;@(posedge rc);#1;
  if(q!==pattern(0,3))$fatal(1,"invalid write wrapped");checks=checks+1;
  @(negedge rc);inv=1;@(negedge rc);inv=0;
  if(ready!==0)$fatal(1,"invalidation failed");
  // Common reset invalidates retained RAM. A new full frame is required.
  reset_n=0;req=0;pv=0;re=0;#37;reset_n=1;wb=0;
  repeat(5)@(negedge rc);if(ready!==0)$fatal(1,"reset exposed retained pixels");
  fill(4);swap();read_frame(4);
  $display("FRAMEBUFFER_PASS checks=%0d full_pixels=76800",checks);$finish;
 end
 initial begin #10000000;$fatal(1,"watchdog");end
endmodule
