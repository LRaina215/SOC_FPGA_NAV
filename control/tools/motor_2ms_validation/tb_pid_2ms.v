`timescale 1ns/1ps
module tb_pid_2ms;
reg clk=0;always #5 clk=~clk;reg rst=0,en=0,valid=0;reg signed[31:0] target=0,feedback=0;wire signed[15:0] duty;wire[2:0]fault;wire signed[31:0]reference,error;
motor520_velocity_pid #(.KI_Q12(287),.KD_Q12(40960),.RAMP_STEP_MMPS(20),.USE_MMPS_FEEDBACK(1),.DERIVATIVE_ALPHA_Q16(0),.DERIVATIVE_FILTER_SHIFT(4),.STALL_SAMPLES(500),.WRONG_DIRECTION_SAMPLES(50)) dut(.clk(clk),.reset_n(rst),.enable(en),.target_mmps(target),.feedback_cms(feedback/10),.feedback_precise_mmps(feedback),.sample_valid(valid),.duty_ticks(duty),.fault_code(fault),.ramped_target_mmps(reference),.error_mmps(error),.control_valid());
task tick;begin @(negedge clk);valid=1;@(negedge clk);valid=0;repeat(12)@(negedge clk);end endtask
integer i;
initial begin
repeat(4)@(negedge clk);rst=1;en=1;target=300;feedback=1;tick;
if(reference!=20||error!=19)$fatal(1,"precise feedback/ramp %0d %0d",reference,error);
for(i=0;i<14;i=i+1)tick;if(reference!=300)$fatal(1,"30ms ramp");
// 100ms opposite-motion protection:49 samples do not trip,50th trips.
feedback=-60;for(i=0;i<49;i=i+1)tick;if(fault!=0)$fatal(1,"opposite early");tick;if(fault!=3)$fatal(1,"opposite not tripped");
en=0;tick;en=1;feedback=0;target=600;
// Count consecutive stall-qualified samples, independent of ramp warm-up.
wait(dut.fault_code==0);while(!(dut.next_reference>=200 && duty>=500))tick;
for(i=0;i<499;i=i+1)begin tick;if(fault!=0)$fatal(1,"stall early %0d",i);end
tick;if(fault!=2)$fatal(1,"stall not tripped");
$display("PASS2ms PID: mm/s precision, scaled ramp,100ms direction guard,1s stall guard");$finish;
end
initial begin #1000000;$fatal(1,"timeout");end
endmodule
