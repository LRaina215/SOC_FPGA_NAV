`timescale 1ns/1ps
module tb_count_feedback;
 reg clk=0,reset_n=0,sample_valid=0,quality=0;
 reg signed[31:0] left_delta=0,right_delta=0;
 wire valid,window_valid,slow,fault,lost;
 wire signed[15:0] vx;wire[10:0] travel;
 always #10 clk=~clk;
 nav_count_feedback #(.CLK_HZ(100000)) dut
  (clk,reset_n,sample_valid,quality,left_delta,right_delta,valid,vx,window_valid,slow,travel,fault,lost);
 integer fd,rc,ql,exp_valid,exp_v,exp_w,exp_s,exp_t,seen=0,frames=0,emissions=0;
 integer left_,right_,violations=0;
 reg checking=0;
 always@(negedge clk)begin
  if(checking && valid)begin
   seen++;emissions++;
   if(!exp_valid || vx!==exp_v[15:0] || window_valid!==exp_w[0] || slow!==exp_s[0] || travel!==exp_t[10:0])
    $fatal(1,"count frame=%0d expected valid/v/window/slow/travel=%0d/%0d/%0d/%0d/%0d got=%0d/%0d/%0d/%0d/%0d",frames,exp_valid,exp_v,exp_w,exp_s,exp_t,valid,vx,window_valid,slow,travel);
   if(fault)$fatal(1,"valid published with fault");
  end
 end
 initial begin
  fd=$fopen("frames.txt","r");if(!fd)$fatal(1,"missing frame oracle");
  repeat(5)@(negedge clk);#1;reset_n=1;
  @(negedge clk);
  while(!$feof(fd))begin
   rc=$fscanf(fd,"%d %d %d %d %d %d %d %d\n",left_,right_,ql,exp_valid,exp_v,exp_w,exp_s,exp_t);
   if(rc!=8)$fatal(1,"bad oracle line");
   #1;seen=0;frames++;checking=1;
   left_delta=left_;right_delta=right_;quality=ql;sample_valid=1;
   @(negedge clk);#1;sample_valid=0;
   repeat(999)@(negedge clk);
   if(seen!=exp_valid)$fatal(1,"frame %0d emission count expected %0d got %0d",frames,exp_valid,seen);
  end
  #1;checking=0;
  // Missing input invalidates window; no stale interval can remain 'stopped'.
  repeat(3)@(negedge clk);
  if(window_valid || slow || !fault || !lost)$fatal(1,"missing-sample fault not closed");violations++;
  // First fresh sample then a deliberately overlapping one while dividing.
  #1;quality=1;sample_valid=1;left_delta=1;right_delta=-1;
  @(negedge clk);#1;sample_valid=0;
  repeat(10)@(negedge clk);#1;sample_valid=1;
  @(negedge clk);#1;sample_valid=0;
  repeat(100)@(negedge clk);
  if(valid || window_valid || slow || !fault || !lost)$fatal(1,"overlap did not invalidate observer");violations++;
  // New sample accepts a new epoch, but sticky pose discontinuity is retained.
  #1;sample_valid=1;left_delta=0;right_delta=0;
  @(negedge clk);#1;sample_valid=0;
  repeat(100)@(negedge clk);
  if(!lost || window_valid || slow || fault)$fatal(1,"recovery quality/continuity mismatch");
  #1;reset_n=0;
  repeat(3)@(negedge clk);
  if(lost || valid || window_valid || slow)$fatal(1,"reset did not clear epoch");
  $display("TEST PASSED frames=%0d emissions=%0d explicit_fault_cases=%0d",frames,emissions,violations);
  $finish;
 end
 initial begin #50000000;$fatal(1,"timeout");end
endmodule
