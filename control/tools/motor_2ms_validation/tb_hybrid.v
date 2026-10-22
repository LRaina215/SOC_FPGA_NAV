`timescale 1ns/1ps
module tb_hybrid;
reg clk=0;always #5 clk=~clk;reg rst=0;reg signed[31:0]pos=0;wire signed[31:0]speed;wire signed[15:0]cms;wire valid;
motor520_hybrid_speed #(.CLK_HZ(1000000),.SAMPLE_CYCLES(2000),.STOP_CYCLES(50000)) dut(.clk(clk),.reset_n(rst),.position(pos),.speed_mmps(speed),.speed_cms(cms),.sample_valid(valid));
wire signed[31:0] inv_speed;
motor520_hybrid_speed #(.CLK_HZ(1000000),.SAMPLE_CYCLES(2000),.STOP_CYCLES(50000),.ENCODER_SIGN(-1)) inv(.clk(clk),.reset_n(rst),.position(pos),.speed_mmps(inv_speed),.speed_cms(),.sample_valid());
integer i,last_cycle=0,cycle=0,checks=0;always @(posedge clk)cycle=cycle+1;
always @(negedge clk)if(valid)begin
 if(inv_speed!==-speed)$fatal(1,"right encoder sign inversion");
 if(last_cycle && cycle-last_cycle!=2000)$fatal(1,"cadence %0d",cycle-last_cycle);
 last_cycle=cycle;checks=checks+1;
end
task edge_step;input integer interval,sign;begin repeat(interval)@(negedge clk);pos=pos+sign;end endtask
initial begin
repeat(5)@(negedge clk);rst=1;
//10mm/s: pulse every15.47ms. Count-only2ms would jump0/77mm/s.
for(i=0;i<7;i=i+1)begin edge_step(15470,1);repeat(80)@(negedge clk);if(i>=2 && (speed<9||speed>11))$fatal(1,"low speed %0d",speed);end
repeat(53000)@(negedge clk);if(speed!=0)$fatal(1,"stale speed did not decay to zero");
// Reverse requires two new edges; old direction must not leak into period estimate.
edge_step(1000,-1);repeat(2200)@(negedge clk);if(speed>0)$fatal(1,"reversal retained old positive speed");
for(i=0;i<8;i=i+1)edge_step(7735,-1);
repeat(80)@(negedge clk);if(speed> -18||speed< -22)$fatal(1,"negative period speed %0d",speed);
// Count mode ~0.309m/s,4counts/2ms.
for(i=0;i<100;i=i+1)edge_step(500,1);
repeat(80)@(negedge clk);if(speed<300||speed>315)$fatal(1,"count mode %0d",speed);
if(checks<50)$fatal(1,"too few samples");
$display("PASS hybrid2ms: cadence, sparse pulses, stop timeout, reversal, count mode");$finish;
end
initial begin #10000000;$fatal(1,"timeout");end
endmodule
