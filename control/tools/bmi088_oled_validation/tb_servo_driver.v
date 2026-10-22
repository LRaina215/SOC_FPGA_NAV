`timescale 1ns/1ps
module tb_servo;
reg clk=0;always #500 clk=~clk;reg rst=0,enable=1;
reg[15:0]lh=1600,lk=1600,rh=1300,rk=1700;
wire s1,s2,s3,s4,tick,d1,d2,d3,d4,dtick;
wire[15:0]a1,a2,a3,a4;
servo_driver #(.CLK_HZ(1000000)) dut(.clk(clk),.reset_n(rst),.enable(enable),
.pulse_lh_us(lh),.pulse_lk_us(lk),.pulse_rh_us(rh),.pulse_rk_us(rk),
.SERVO1(s1),.SERVO2(s2),.SERVO3(s3),.SERVO4(s4),.frame_tick(tick),
.active_lh_us(a1),.active_lk_us(a2),.active_rh_us(a3),.active_rk_us(a4));
servo_driver #(.CLK_HZ(1000000),.APPLY_CALIBRATED_DIRECTION(0)) direct(
.clk(clk),.reset_n(rst),.enable(enable),.pulse_lh_us(lh),.pulse_lk_us(lk),.pulse_rh_us(rh),.pulse_rk_us(rk),
.SERVO1(d1),.SERVO2(d2),.SERVO3(d3),.SERVO4(d4),.frame_tick(dtick),
.active_lh_us(),.active_lk_us(),.active_rh_us(),.active_rk_us());
integer j,c1,c2,c3,c4,dc2;
task check_frame;
input integer e1,e2,e3,e4,ed2,update;
begin
while(!tick)@(negedge clk);
if(a1!=e1 || a2!=e2 || a3!=e3 || a4!=e4)$fatal(1,"latched widths");
c1=0;c2=0;c3=0;c4=0;dc2=0;
for(j=0;j<20000;j=j+1)begin
c1=c1+s1;c2=c2+s2;c3=c3+s3;c4=c4+s4;dc2=dc2+d2;
if(j==1000 && update==1)begin lh=100;lk=100;rh=3000;rk=65535;end
if(j==1000 && update==2)begin lh=2500;lk=2500;rh=2500;rk=2500;end
@(negedge clk);
end
if(c1!=e1 || c2!=e2 || c3!=e3 || c4!=e4 || dc2!=ed2)
$fatal(1,"pulse counts %0d %0d %0d %0d directLK%0d",c1,c2,c3,c4,dc2);
end endtask
initial begin
repeat(3)@(negedge clk);if(s1 || s2 || s3 || s4)$fatal(1,"reset outputs");rst=1;
check_frame(1600,1400,1300,1700,1600,1);
check_frame(500,2500,2500,2500,500,2);
check_frame(2500,500,2500,2500,2500,0);
repeat(1000)@(negedge clk);enable=0;@(negedge clk);
if(s1 || s2 || s3 || s4 || tick)$fatal(1,"disable");
lh=1500;lk=1500;rh=1500;rk=1500;enable=1;
check_frame(1500,1500,1500,1500,1500,0);
rst=0;@(negedge clk);if(s1 || s2 || s3 || s4)$fatal(1,"reset high pulse");
$display("PASS4 servo PWM:20ms frames, independent pulse widths, calibrated + - + + signs, direct mode, wide-input clamps, atomic frame updates, no mid-pulse changes, disable/reset/restart");$finish;
end
initial begin #150000000;$fatal(1,"timeout");end
endmodule
