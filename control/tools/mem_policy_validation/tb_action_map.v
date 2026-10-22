`timescale 1ns/1ps
module tb_action_map;
reg clk=0;always #10 clk=~clk;
reg rst_n=0,run=0,valid=0;reg[143:0]actions=0;
wire signed[31:0]left,right;wire[15:0]lh,lk,rh,rk;wire[143:0]last;wire have;
policy_action_map dut(.clk(clk),.rst_n(rst_n),.run(run),.action_valid(valid),.actions(actions),
 .ff_lh(12'sd100),.ff_lk(-12'sd100),.ff_rh(12'sd200),.ff_rk(-12'sd200),
 .left_mmps(left),.right_mmps(right),.lh_us(lh),.lk_us(lk),.rh_us(rh),.rk_us(rk),.last_actions(last),.have_action(have));
initial begin
 repeat(2)@(negedge clk);rst_n=1;run=1;valid=1;
 actions={24'sd10000,-24'sd10000,24'sd4096,-24'sd10000,24'sd10000,-24'sd4096};
 @(negedge clk);valid=0;
 if(left!=600 || right!=600 || lh!=1581 || lk!=1381 || rh!=1719 || rk!=1281 || !have)
  $fatal(1,"clipping/sign mapping");
 if(last!=={24'sd4096,-24'sd4096,24'sd4096,-24'sd4096,24'sd4096,-24'sd4096})$fatal(1,"last action clamp");
 run=0;@(negedge clk);
 if(left || right || last || have || lh!=1581 || lk!=1381 || rh!=1719 || rk!=1281)$fatal(1,"stop/hold");
 $display("PASS actuator normalized clamp, calibrated servo directions, full-scale +/-0.6m/s, stop/hold");$finish;
end
endmodule
