`timescale 1ns/1ps
module tb_pwm;
reg clk=0;always #10 clk=~clk;reg rst=0,enable=1;reg signed[15:0]duty=500;
wire pwm,d1,d2,ipwm,i1,i2;
motor520_pwm dut(.clk(clk),.reset_n(rst),.enable(enable),.duty_ticks(duty),.pwm(pwm),.dir1(d1),.dir2(d2));
motor520_pwm #(.DRIVE_SIGN(-1)) inv(.clk(clk),.reset_n(rst),.enable(enable),.duty_ticks(duty),.pwm(ipwm),.dir1(i1),.dir2(i2));
integer n,high_ticks;
task period_check;
input integer expected;input reverse;
begin
wait(dut.counter==0 && dut.active_magnitude==expected && dut.active_reverse==reverse);@(negedge clk);
high_ticks=0;
for(n=0;n<2500;n=n+1)begin
if(pwm)high_ticks=high_ticks+1;
if(expected!=0 && (d1!==!reverse || d2!==reverse))$fatal(1,"direction");
if(d1 && d2)$fatal(1,"conflicting direction pins");
@(negedge clk);
end
if(high_ticks!=expected)$fatal(1,"PWM count actual=%0d expected=%0d",high_ticks,expected);
end endtask
initial begin
repeat(3)@(negedge clk);rst=1;period_check(500,0);
if(i1 || !i2)$fatal(1,"drive sign inversion");
duty=-750;wait(dut.counter==0 && dut.active_magnitude==0);@(negedge clk);
for(n=0;n<2500;n=n+1)begin
if(pwm || d1 || d2)$fatal(1,"reversal must coast for full carrier");
@(negedge clk);
end
period_check(750,1);
duty=3000;period_check(2500,0);
enable=0;#1;if(pwm || d1 || d2)$fatal(1,"immediate disable");
repeat(3)@(negedge clk);enable=1;duty=0;repeat(2600)@(negedge clk);
if(pwm || d1 || d2)$fatal(1,"zero duty");
rst=0;#1;if(pwm || d1 || d2)$fatal(1,"reset output");
$display("PASS20kHz PWM: exact duty pulse widths, positive/negative direction, sign mapping, full-period reversal blank, clamp, disable/reset");$finish;
end
initial begin #1000000;$fatal(1,"PWM timeout");end
endmodule
