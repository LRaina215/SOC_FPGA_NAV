`timescale 1ns/1ps
module tb_derivative;
reg clk=0;always #10 clk=~clk;reg rst=0,enable=1,valid=0;
reg signed[31:0]target=1000;reg signed[15:0]feedback=0;
wire signed[15:0]duty;wire[2:0]fault;
motor520_velocity_pid #(.KP_Q12(0),.KI_Q12(0),.KFF_Q12(0),.KD_Q12(8192),
.DUTY_MAX(2400),.RAMP_STEP_MMPS(1000),.FEEDBACK_TIMEOUT_CYCLES(400),
.STALL_SAMPLES(0),.WRONG_DIRECTION_SAMPLES(0)) dut(
.clk(clk),.reset_n(rst),.enable(enable),.target_mmps(target),.feedback_cms(feedback),
.sample_valid(valid),.duty_ticks(duty),.ramped_target_mmps(),.error_mmps(),.control_valid(),.fault_code(fault));
task sample;
begin @(negedge clk);valid=1;@(negedge clk);valid=0;repeat(18)@(negedge clk);end endtask
initial begin
repeat(3)@(negedge clk);rst=1;sample;
if(duty!=0)$fatal(1,"first sample derivative kick");
feedback=10;sample;if(duty!=-50)$fatal(1,"filtered derivative magnitude expected-50 actual%0d",duty);
target=2000;sample;if(duty!=-38)$fatal(1,"setpoint must not create derivative kick actual%0d",duty);
feedback=-32768;sample;if(duty!=250)$fatal(1,"D positive clamp");
feedback=32767;sample;if(duty!=-250)$fatal(1,"D negative clamp");
target=0;@(negedge clk);if(duty || dut.derivative_q8 || dut.measurement_seen)$fatal(1,"stop clears derivative history");
target=1000;feedback=100;sample;if(duty)$fatal(1,"restart with nonzero measured velocity must not kick");
$display("PASS PID D: filtered measurement derivative, no target kick, both-sign clamp, stop/history reset, warm start");$finish;
end
initial begin #100000;$fatal(1,"timeout");end
endmodule
