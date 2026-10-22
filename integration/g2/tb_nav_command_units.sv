`timescale 1ns/1ps
module tb_nav_command_units;
 reg clk=0;always #10 clk=~clk;
 reg reset_n=0,kill=0,valid=0;reg [15:0] seq=0;
 reg signed [15:0] vx=0,wz=0;
 wire ov;wire [15:0] os;wire signed [23:0] oqv,oqw;
 nav_command_units dut(clk,reset_n,kill,valid,seq,vx,wz,ov,os,oqv,oqw);
 reg [3:0] ev=0;reg [15:0] es[0:3];reg signed[23:0] eqv[0:3],eqw[0:3];
 integer checked=0,flushed=0,corrections=0;
 function signed [23:0] original(input signed [15:0] x);
  reg signed [63:0] p;
  begin p=$signed(x)*64'sd4096;original=(p<0?p-64'sd500:p+64'sd500)/64'sd1000;end
 endfunction
 always @(posedge clk or negedge reset_n)begin
  if(reset_n && !kill && dut.v2)begin
   if(dut.scaled_v2>{1'b0,dut.nv2})corrections++;
   if(dut.scaled_w2>{1'b0,dut.nw2})corrections++;
  end
  if(!reset_n || kill)begin
   if(ev!=0)flushed=flushed+1;
   ev<=0;for(integer j=0;j<4;j++)begin es[j]<=0;eqv[j]<=0;eqw[j]<=0;end
  end else begin
   ev<={ev[2:0],valid};es[0]<=seq;eqv[0]<=original(vx);eqw[0]<=original(wz);
   for(integer j=1;j<4;j++)begin es[j]<=es[j-1];eqv[j]<=eqv[j-1];eqw[j]<=eqw[j-1];end
  end
 end
 always @(negedge clk)begin
  if(ov!==ev[3])$fatal(1,"valid latency mismatch");
  if(ov)begin
   if({os,oqv,oqw}!=={es[3],eqv[3],eqw[3]})$fatal(1,"numeric/sequence mismatch seq=%h",os);
   checked=checked+1;
  end
 end
 // Drive away from the scoreboard edge; exercise every value at full throughput.
 initial begin
  repeat(5)@(posedge clk);#1;reset_n=1;
  for(integer x=-32768;x<=32767;x++)begin
   @(posedge clk);#1;valid=1;vx=x;wz=~x;seq=seq+1'b1;
  end
  @(posedge clk);#1;valid=0;repeat(6)@(posedge clk);#1;
  if(checked!=65536)$fatal(1,"exhaustive conversion frames missing: %0d",checked);
  for(integer k=0;k<100;k++)begin
   @(posedge clk);#1;valid=k%7!=0;vx=k*317;wz=-k*299;seq=seq+1'b1;
   kill=(k%13==5);reset_n=!(k==55 || k==56);
  end
  @(posedge clk);#1;valid=0;kill=0;reset_n=1;repeat(7)@(posedge clk);
  @(negedge clk);#1;if(flushed<5)$fatal(1,"flush scenarios not exercised");
  if(corrections==0)$fatal(1,"reciprocal correction not exercised");
  $display("TEST PASSED exhaustive=65536 checked=%0d flushed=%0d corrections=%0d",checked,flushed,corrections);$finish;
 end
endmodule
