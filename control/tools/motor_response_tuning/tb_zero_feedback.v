`timescale 1ns/1ps
module tb_zero_feedback;
reg clk=0;always #10 clk=~clk;
reg rst=0,enable=1,valid=0;
reg signed [31:0] target_l=0,target_r=0;
reg signed [15:0] speed_l=20,speed_r=-20;
wire signed [15:0] duty_l,duty_r;
wire [2:0] fault_l,fault_r;
wire pl,l1,l2,pr,r1,r2,cv_l,cv_r;
wire signed [31:0] ref_l,ref_r,err_l,err_r;
integer updates=0;
always @(posedge clk) if(cv_l)updates=updates+1;
motor520_closed_loop #(.LEFT_KP_Q12(16384),.RIGHT_KP_Q12(16384),
 .LEFT_KI_Q12(0),.RIGHT_KI_Q12(0),.LEFT_KD_Q12(0),.RIGHT_KD_Q12(0),
 .LEFT_KFF_Q12(14336),.RIGHT_KFF_Q12(14336),.RAMP_STEP_MMPS(600),
 .CLOSED_LOOP_AT_ZERO(1),.FEEDBACK_TIMEOUT_CYCLES(400),.PWM_PERIOD_CYCLES(16)) dut(
 .clk(clk),.reset_n(rst),.enable(enable),.target_left_mmps(target_l),.target_right_mmps(target_r),
 .speed_left_cms(speed_l),.speed_right_cms(speed_r),.valid_left(valid),.valid_right(valid),
 .pwm_left(pl),.dir_left1(l1),.dir_left2(l2),.pwm_right(pr),.dir_right1(r1),.dir_right2(r2),
 .duty_left(duty_l),.duty_right(duty_r),.fault_left(fault_l),.fault_right(fault_r),
 .reference_left_mmps(ref_l),.reference_right_mmps(ref_r),.error_left_mmps(err_l),.error_right_mmps(err_r),
 .control_valid_left(cv_l),.control_valid_right(cv_r));
task sample;
begin @(negedge clk);valid=1;@(negedge clk);valid=0;repeat(30)@(negedge clk);end endtask
initial begin
 repeat(3)@(negedge clk);rst=1;sample;
 if(duty_l!=-800 || duty_r!=800 || err_l!=-200 || err_r!=200 || ref_l || ref_r)
  $fatal(1,"zero speed must retain signed feedback braking %d %d",duty_l,duty_r);
 if(!l2 || !r1)$fatal(1,"zero target incorrectly gated off driver direction");
 speed_l=0;speed_r=0;sample;
 if(duty_l || duty_r || pl || pr)$fatal(1,"no command or motion must not drive");
 target_l=300;target_r=-300;sample;
 if(ref_l!=300 || ref_r!=-300 || duty_l<=0 || duty_r>=0)$fatal(1,"target signs");
 target_l=0;target_r=0;speed_l=20;speed_r=-20;sample;
 if(duty_l>=0 || duty_r<=0 || !dut.left_control.measurement_seen || updates<4)
  $fatal(1,"crossing zero lost control/history");
 enable=0;#1;if(pl || pr || l1 || l2 || r1 || r2)$fatal(1,"disable must shut down immediately");
 @(negedge clk);enable=1;sample;valid=0;repeat(450)@(negedge clk);
 if(fault_l!=1 || fault_r!=1 || duty_l || duty_r || pl || pr)$fatal(1,"missing feedback at zero must still fault");
 enable=0;@(negedge clk);if(fault_l || fault_r)$fatal(1,"disable must clear faults");
 $display("PASS zero feedback: opposite braking, rest, sign crossing, diagnostic reference/error, immediate disable, timeout and clear");$finish;
end
initial begin #1000000;$fatal(1,"timeout");end
endmodule
