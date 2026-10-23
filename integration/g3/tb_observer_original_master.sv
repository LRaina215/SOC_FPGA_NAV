`timescale 1ns/1ps
module tb_observer_original_master;
 tb_bmi088 source();
 wire[127:0]acks,nacks,reads;wire[31:0]ids;wire[3:0]seen;
 i2c_id_observer observer(source.clk,source.rst,source.scl,source.sda,acks,nacks,reads,ids,seen);
 initial begin
  wait(source.valid);@(negedge source.clk);#1;
  if(acks[63:32]<2 || acks[127:96]<2 || nacks[31:0]<1 || nacks[95:64]<1 || reads[63:32]<1 || reads[127:96]<1 || ids[15:8]!=8'h1e || ids[31:24]!=8'h0f || seen!=4'b1010)
   $fatal(1,"original master observer mismatch a=%h n=%h r=%h ids=%h seen=%h",acks,nacks,reads,ids,seen);
  $display("ORIGINAL_MASTER_OBSERVER_PASS a=%h n=%h reads=%h ids=%h",acks,nacks,reads,ids);$finish;
 end
endmodule
