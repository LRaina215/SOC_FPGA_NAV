`timescale 1ns/1ps
module tb_response;
reg clk=0;always #10 clk=~clk;reg rst=0,valid=0;
reg signed[15:0]old_feedback=0,new_feedback=0;
wire signed[15:0]old_duty,new_duty;wire[2:0]old_fault,new_fault;
motor520_velocity_pid #(.KP_Q12(10240),.KI_Q12(1024),.KD_Q12(0),.DUTY_MAX(1250),
.RAMP_STEP_MMPS(20),.FEEDBACK_TIMEOUT_CYCLES(400)) old_loop(
.clk(clk),.reset_n(rst),.enable(1'b1),.target_mmps(32'sd1000),.feedback_cms(old_feedback),
.sample_valid(valid),.duty_ticks(old_duty),.ramped_target_mmps(),.error_mmps(),.control_valid(),.fault_code(old_fault));
motor520_velocity_pid #(.FEEDBACK_TIMEOUT_CYCLES(400)) new_loop(
.clk(clk),.reset_n(rst),.enable(1'b1),.target_mmps(32'sd1000),.feedback_cms(new_feedback),
.sample_valid(valid),.duty_ticks(new_duty),.ramped_target_mmps(),.error_mmps(),.control_valid(),.fault_code(new_fault));
real old_v=0,new_v=0;integer n,old_count=0,new_count=0,old_settle=-1,new_settle=-1;
initial begin
repeat(3)@(negedge clk);rst=1;
for(n=0;n<200;n=n+1)begin
old_v=old_v+0.1*(1.12*old_duty-old_v);new_v=new_v+0.1*(1.12*new_duty-new_v);
old_feedback=$rtoi(old_v/10);new_feedback=$rtoi(new_v/10);
@(negedge clk);valid=1;@(negedge clk);valid=0;repeat(18)@(negedge clk);
if(old_fault || new_fault)$fatal(1,"model fault");
if(old_v>=970 && old_v<=1030)old_count=old_count+1;else old_count=0;
if(new_v>=970 && new_v<=1030)new_count=new_count+1;else new_count=0;
if(old_count==10 && old_settle<0)old_settle=n-9;
if(new_count==10 && new_settle<0)new_settle=n-9;
end
if(old_settle<0 || new_settle<0 || new_settle>=old_settle)$fatal(1,"model response not improved old=%d new=%d",old_settle,new_settle);
$display("PASS response comparison in same first-order model: old settle=%0dms, new=%0dms (not measured motor response)",old_settle*10,new_settle*10);$finish;
end
initial begin #1000000;$fatal(1,"timeout");end
endmodule
