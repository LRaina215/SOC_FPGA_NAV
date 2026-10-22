`timescale 1ns/1ps
module action_map_pair #(parameter LH=1500,LK=1500,RH=1500,RK=1500)
 (input clk,rst_n,run,valid,input [143:0] actions,input signed[11:0] ff_lh,ff_lk,ff_rh,ff_rk);
 wire[272:0] old_out,new_out;
 action_map_original #(.LH_ZERO_US(LH),.LK_ZERO_US(LK),.RH_ZERO_US(RH),.RK_ZERO_US(RK)) original
  (clk,rst_n,run,valid,actions,ff_lh,ff_lk,ff_rh,ff_rk,old_out[31:0],old_out[63:32],old_out[79:64],old_out[95:80],old_out[111:96],old_out[127:112],old_out[271:128],old_out[272]);
 policy_action_map #(.LH_ZERO_US(LH),.LK_ZERO_US(LK),.RH_ZERO_US(RH),.RK_ZERO_US(RK)) candidate
  (clk,rst_n,run,valid,actions,ff_lh,ff_lk,ff_rh,ff_rk,new_out[31:0],new_out[63:32],new_out[79:64],new_out[95:80],new_out[111:96],new_out[127:112],new_out[271:128],new_out[272]);
 always @(negedge clk)if($time>20 && old_out!==new_out)$fatal(1,"action map cycle mismatch LH=%0d",LH);
endmodule
module tb_action_map_equivalence;
 reg clk=0,rst_n=0,run=0,valid=0;
 reg[143:0] actions=0;reg signed[11:0] ff_lh=0,ff_lk=0,ff_rh=0,ff_rk=0;
 integer frames=0,invalid_cycles=0,stops=0,resets=0;
 reg[31:0] rng=32'h450088;
 function[31:0] next(input[31:0] x);
  reg[31:0] y;begin y=x^(x<<13);y=y^(y>>17);next=y^(y<<5);end
 endfunction
 always #10 clk=~clk;
 action_map_pair p0(clk,rst_n,run,valid,actions,ff_lh,ff_lk,ff_rh,ff_rk);
 action_map_pair #(.LH(500),.LK(2500),.RH(1470),.RK(1530)) p1(clk,rst_n,run,valid,actions,ff_lh,ff_lk,ff_rh,ff_rk);
 task automatic step(input integer a,input integer n);
  @(negedge clk);#1;
  for(integer k=0;k<6;k++)begin
   rng=next(rng);
   // Every gain sees each clip-domain value. Out-of-domain vectors below
   // separately exercise positive/negative saturation and all 24 input bits.
   actions[k*24+:24]=a;
   if(n>=8193)begin
    case(n%8)
     0:actions[k*24+:24]=-8388608;
     1:actions[k*24+:24]=8388607;
     2:actions[k*24+:24]=-4097;
     3:actions[k*24+:24]=4097;
     default:actions[k*24+:24]=rng[23:0];
    endcase
   end
  end
  rng=next(rng);ff_lh=rng[11:0];ff_lk=rng[23:12];rng=next(rng);ff_rh=rng[11:0];ff_rk=rng[23:12];
  if(n%4==0)begin ff_lh=-2048;ff_lk=2047;ff_rh=2047;ff_rk=-2048;end
  rst_n=1;run=1;valid=1;frames=frames+1;
 endtask
 initial begin
  repeat(4)@(negedge clk);
  for(integer a=-4096;a<=4096;a++)step(a,a+4096);
  for(integer n=8193;n<8705;n++)step(0,n);
  for(integer n=0;n<64;n++)begin
   step(n*100-3000,9000+n);
   @(negedge clk);#1;valid=0;actions=~actions;invalid_cycles++;
   @(negedge clk);#1;run=0;valid=1;stops++;
  end
  @(negedge clk);#1;rst_n=0;run=1;valid=1;resets++;
  repeat(3)@(negedge clk);
  step(-1,0);repeat(4)@(negedge clk);
  $display("TEST PASSED frames=%0d clip_domain_values=8193 invalid_cycles=%0d stops=%0d resets=%0d parameter_sets=2",frames,invalid_cycles,stops,resets);
  $finish;
 end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule
