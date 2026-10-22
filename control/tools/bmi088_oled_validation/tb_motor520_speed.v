`timescale 1ns/1ps
module tb_speed;
// 1MHz test clock, 10000 cycles/window =10ms, same100Hz gate as the board.
reg clk=0;always #500 clk=~clk;reg rst=0,a=0,b=0;
wire signed[15:0]speed,inverted,saturated;wire valid;
wire signed[31:0]position,delta;wire[31:0]illegal;
motor520_speed #(.CLK_HZ(1000000),.SPEED_GATE_CYCLES(10000)) dut(
.clk(clk),.reset_n(rst),.enc_a(a),.enc_b(b),.position(position),.speed_delta(delta),
.speed_cms(speed),.sample_valid(valid),.invalid_transitions(illegal));
motor520_speed #(.CLK_HZ(1000000),.SPEED_GATE_CYCLES(10000),.ENCODER_SIGN(-1)) inv(
.clk(clk),.reset_n(rst),.enc_a(a),.enc_b(b),.position(),.speed_delta(),
.speed_cms(inverted),.sample_valid(),.invalid_transitions());
motor520_speed #(.CLK_HZ(1000000),.SPEED_GATE_CYCLES(10000),.ENCODER_CPR(1)) sat(
.clk(clk),.reset_n(rst),.enc_a(a),.enc_b(b),.position(),.speed_delta(),
.speed_cms(saturated),.sample_valid(),.invalid_transitions());
integer j,phase;
task reset_encoder;
input[1:0]init;
begin
@(negedge clk);rst=0;{a,b}=init;repeat(3)@(negedge clk);rst=1;
wait(dut.warmup==5);@(negedge clk);
end endtask
task count_case;
input integer count;input integer expected;
begin
reset_encoder(0);phase=0;
for(j=0;j<(count<0?-count:count);j=j+1)begin
phase=(phase+(count<0?3:1))&3;
case(phase)0:{a,b}=0;1:{a,b}=1;2:{a,b}=3;3:{a,b}=2;endcase
repeat(8)@(negedge clk);
end
wait(valid);wait(sat.sample_valid);@(negedge clk);
if(delta!==count || position!==count || speed!==expected || inverted!==-expected || illegal!=0)
$fatal(1,"count=%0d delta=%0d pos=%0d cm/s=%0d expected=%0d inv=%0d",count,delta,position,speed,expected,inverted);
if(count==500 && saturated!==32767)$fatal(1,"positive saturation");
if(count==-500 && saturated!==-32767)$fatal(1,"negative saturation");
wait(!valid);wait(valid);wait(sat.sample_valid);@(negedge clk);
if(delta!=0 || speed!=0)$fatal(1,"stop must return zero");
end endtask
initial begin
count_case(0,0);count_case(1,2);count_case(-1,-2);count_case(4,6);count_case(-4,-6);
count_case(40,62);count_case(-40,-62);count_case(500,774);count_case(-500,-774);count_case(1000,1547);
reset_encoder(3);wait(valid);@(negedge clk);if(position!=0 || delta!=0)$fatal(1,"initial AB level phantom step");
reset_encoder(0);{a,b}=3;repeat(8)@(negedge clk);wait(valid);wait(sat.sample_valid);@(negedge clk);
if(delta!=0 || illegal!=1)$fatal(1,"invalid two-bit transition");
reset_encoder(0);wait(dut.gate_counter==9997);@(negedge clk);b=1;
wait(valid);@(negedge clk);if(delta!=1)$fatal(1,"gate boundary edge");
wait(!valid);wait(valid);@(negedge clk);if(delta!=0)$fatal(1,"gate boundary double-count");
$display("PASS520 AB: fourfold decode, forward/reverse, right-sign inversion, exact m/s conversion/half rounding, zero speed, initial-level warmup, invalid transitions, window boundary and saturation");$finish;
end
initial begin #1000000000;$fatal(1,"timeout");end
endmodule
