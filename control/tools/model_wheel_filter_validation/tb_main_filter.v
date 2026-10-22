`timescale 1ns/1ps
module tb_main_filter;
reg clk=0;always #10 clk=~clk;reg rst=0;tri1 a,b,c,d;
main #(.MODEL_WHEEL_FILTER_ENABLE(1'b1)) dut(.clk(clk),.reset_n(rst),.IRIN(1'b1),.UART_RXD(1'b1),.sck(a),.sda(b),.BMI088_SCL(c),.BMI088_SDA(d),.M1_ENCA(1'b0),.M1_ENCB(1'b0),.M2_ENCA(1'b0),.M2_ENCB(1'b0));
defparam dut.model_wheel_filter.UPDATE_CYCLES=16;
tri1 e,f,g,h;
main #(.MODEL_ENABLE(0),.IR_ENABLE(0),.TARGET_LEFT_MPS(0.2),.TARGET_RIGHT_MPS(-0.3)) manual(.clk(clk),.reset_n(rst),.IRIN(1'b1),.UART_RXD(1'b1),.sck(e),.sda(f),.BMI088_SCL(g),.BMI088_SDA(h),.M1_ENCA(1'b0),.M1_ENCB(1'b0),.M2_ENCA(1'b0),.M2_ENCB(1'b0));
integer i;
initial begin
force dut.model_active=0;force dut.model_left_mmps=600;force dut.model_right_mmps=-600;
repeat(4)@(negedge clk);rst=1;repeat(270)@(negedge clk);
if(manual.wheel_control.target_left_mmps!=200||manual.wheel_control.target_right_mmps!=-300)$fatal(1,"manual target altered");
force dut.model_active=1;
for(i=0;i<6;i=i+1)begin
 wait(dut.model_wheel_filter.counter==15);@(posedge clk);repeat(6)@(negedge clk);
 if(i<5 && (dut.wheel_control.target_left_mmps!=0||dut.wheel_control.target_right_mmps!=0))$fatal(1,"PID bypassed delay");
end
if(dut.wheel_control.target_left_mmps!=26||dut.wheel_control.target_right_mmps!=-26)$fatal(1,"filter not connected to PID");
if(dut.wheel_left_target_mmps!=600||dut.wheel_right_target_mmps!=-600)$fatal(1,"raw target diagnostics lost");
force dut.ir_emergency_stop=1;#1;
if(dut.wheel_control.enable||dut.M1_PWM||dut.M2_PWM||dut.wheel_left_pid_target_mmps!=0)$fatal(1,"stop delayed by filter");
repeat(3)@(negedge clk);if(dut.model_wheel_filter.filter_left!=0)$fatal(1,"stop did not flush state");
$display("PASS main filter: model path reaches PID after delay,raw diagnostics kept,manual path unchanged,immediate emergency PWM gate and state flush");$finish;
end
initial begin #1000000;$fatal(1,"timeout");end
endmodule
