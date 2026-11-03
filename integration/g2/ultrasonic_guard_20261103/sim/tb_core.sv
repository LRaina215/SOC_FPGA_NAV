`timescale 1ns/1ps
module tb_core;
reg clk=0;always #10 clk=~clk;
reg rst=0,ir=1;
reg signed [23:0] nav_v=300,nav_y=-200;
wire hold_forward;
tri1 sck,sda,bmi_scl,bmi_sda;
integer checks=0;
control4500_core dut(.clk(clk),.reset_n(rst),.IRIN(ir),
 .nav_forward_q12(nav_v),.nav_yaw_q12(nav_y),.UART_RXD(1'b1),
 .sck(sck),.sda(sda),.BMI088_SCL(bmi_scl),.BMI088_SDA(bmi_sda),
 .M1_ENCA(1'b0),.M1_ENCB(1'b0),.M2_ENCA(1'b0),.M2_ENCB(1'b0),
 .range_distance_mm(16'd200),.range_distance_valid(1'b1),
 .range_sample_strobe(1'b1),.range_forward_ready(hold_forward),.range_hold_forward(hold_forward));
// NEC and guard counters are compressed together by 50 for this wiring test only.
// Production prescaler/PWM tests and the real4500 test are separate.
defparam dut.remote_thread.CLOCK_FREQ_HZ=1000000;
defparam dut.u_ultrasonic_guard.CLOCK_HZ=1000000;
task level(input bit value,input integer ticks);
 begin ir=value;repeat(ticks)@(negedge clk);end
endtask
task frame(input [7:0] code);
 reg [31:0] bits;integer i;
 begin
  bits={~code,code,16'hff00};level(1,20000);level(0,9000);level(1,4500);
  for(i=0;i<32;i++)begin level(0,560);level(1,bits[i]?1690:560);end
  level(0,560);level(1,20);
 end
endtask
task check(input bit ok,input string why);
 begin checks++;if(!ok)$fatal(1,"%s",why);end
endtask
initial begin
 repeat(4)@(negedge clk);rst=1;repeat(300)@(negedge clk);
 check(!hold_forward,"production core mode defaults off");
 check(dut.command_forward_q12==nav_v && dut.command_yaw_q12==nav_y,"original NAV command wiring");
 frame(8'h16);check(hold_forward,"mode reaches range hold output");
 check(dut.command_forward_q12==nav_v && dut.command_yaw_q12==nav_y,"NAV bypasses near obstacle");
 frame(8'h18);check(dut.ir_drive && dut.requested_forward_q12==1229,"real IR forward owns source");
 check(dut.command_forward_q12==0 && dut.command_yaw_q12==0,"guard enters actual policy command input");
 check(dut.control_run && dut.MODEL_ENABLE,"sonar never disables original balance enable");
 nav_v=888;nav_y=999;repeat(5)@(negedge clk);
 check(dut.command_forward_q12==0,"NAV cannot leak into active remote command");
 frame(8'h52);check(dut.command_forward_q12==-1229,"reverse route through actual core");
 level(1,181000);check(!dut.ir_drive && dut.command_forward_q12==nav_v && dut.command_yaw_q12==nav_y,"release returns both axes to NAV");
 frame(8'h1c);check(!dut.control_run && dut.command_forward_q12==0 && dut.command_yaw_q12==0,"existing IR5 stop priority");
 $display("TEST PASSED CORE checks=%0d",checks);$finish;
end
initial begin #(50ms);$fatal(1,"core wiring watchdog");end
endmodule
