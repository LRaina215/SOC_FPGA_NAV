`timescale 1ns/1ps
module tb_control;
reg clk=0;always #10 clk=~clk;reg rst=0,enable=1,valid=0;
reg signed[31:0]target_l=1000,target_r=1000;
reg signed[15:0]feedback_l=0,feedback_r=0;
wire signed[15:0]duty_l,duty_r;wire[2:0]fault_l,fault_r;
wire pl,l1,l2,pr,r1,r2;
motor520_closed_loop #(.FEEDBACK_TIMEOUT_CYCLES(400)) dut(
.clk(clk),.reset_n(rst),.enable(enable),.target_left_mmps(target_l),.target_right_mmps(target_r),
.speed_left_cms(feedback_l),.speed_right_cms(feedback_r),.valid_left(valid),.valid_right(valid),
.pwm_left(pl),.dir_left1(l1),.dir_left2(l2),.pwm_right(pr),.dir_right1(r1),.dir_right2(r2),
.duty_left(duty_l),.duty_right(duty_r),.fault_left(fault_l),.fault_right(fault_r));
real velocity_l=0,velocity_r=0,load_l=0,load_r=0;
integer n,prior_l,jitter_l;
task tick;
begin
 // One model step represents10ms; HDL clock cadence compressed for simulation.
 velocity_l=velocity_l+0.1*(1.12*duty_l-load_l-velocity_l);
 velocity_r=velocity_r+0.08*(0.96*duty_r-load_r-velocity_r);
 feedback_l=$rtoi(velocity_l/10.0);feedback_r=$rtoi(velocity_r/10.0);
 @(negedge clk);valid=1;@(negedge clk);valid=0;repeat(18)@(negedge clk);
 if(duty_l>2400 || duty_l< -2400 || duty_r>2400 || duty_r< -2400)$fatal(1,"duty clamp");
end endtask
task reset_loop;
begin rst=0;repeat(3)@(negedge clk);rst=1;end endtask
initial begin
 reset_loop;
 for(n=0;n<200;n=n+1)begin tick;if(fault_l || fault_r)$fatal(1,"plant fault l=%0d r=%0d at%0d",fault_l,fault_r,n);end
 if(velocity_l<970 || velocity_l>1030 || velocity_r<970 || velocity_r>1030)
 $fatal(1,"1m/s convergence left=%f right=%f",velocity_l,velocity_r);
 // Apply load to the left wheel; integral must restore the commanded speed.
 load_l=150;for(n=0;n<150;n=n+1)tick;
 if(velocity_l<970 || velocity_l>1030 || fault_l)$fatal(1,"load recovery %f fault=%d",velocity_l,fault_l);
 // Independent targets.
 target_l=600;target_r=800;for(n=0;n<150;n=n+1)tick;
 if(velocity_l<570 || velocity_l>630 || velocity_r<770 || velocity_r>830)$fatal(1,"independent setpoints");
 // Zero target clears integral and both driver lines immediately.
 target_l=0;target_r=0;@(negedge clk);
 if(duty_l || duty_r || pl || pr || l1 || l2 || r1 || r2 || dut.left_control.integral_q)$fatal(1,"zero command stop");
 load_l=0;velocity_l=0;velocity_r=0;target_l=-1000;target_r=-1000;
 for(n=0;n<200;n=n+1)begin tick;if(fault_l || fault_r)$fatal(1,"reverse fault");end
 if(velocity_l> -970 || velocity_l< -1030 || velocity_r> -970 || velocity_r< -1030)$fatal(1,"reverse convergence");
 // Missing samples must stop both loops and latch fault.
 valid=0;repeat(450)@(negedge clk);
 if(fault_l!=1 || fault_r!=1 || duty_l || duty_r || pl || pr)$fatal(1,"feedback timeout");
 enable=0;@(negedge clk);enable=1;feedback_l=0;feedback_r=0;target_l=1000;target_r=1000;
 // Stalled encoder: zero samples continue, but output cannot stay powered indefinitely.
 for(n=0;n<180;n=n+1)begin
 @(negedge clk);valid=1;@(negedge clk);valid=0;repeat(18)@(negedge clk);
 end
 if(fault_l!=2 || fault_r!=2 || duty_l || duty_r)$fatal(1,"stall fault l=%d r=%d",fault_l,fault_r);
 enable=0;@(negedge clk);enable=1;feedback_l=-20;feedback_r=-20;
 for(n=0;n<50;n=n+1)begin
 @(negedge clk);valid=1;@(negedge clk);valid=0;repeat(18)@(negedge clk);
 end
 if(fault_l!=3 || fault_r!=3)$fatal(1,"wrong direction fault");
 // Disable/reset clears fault and all actuation.
 enable=0;@(negedge clk);
 if(fault_l || fault_r || pl || pr || l1 || l2 || r1 || r2)$fatal(1,"disable clear");
 $display("PASS velocity PID:1m/s model convergence, load rejection, independent targets, reverse, zero/disable, PWM limits, timeout/stall/direction faults");$finish;
end
initial begin #10000000;$fatal(1,"timeout");end
endmodule
