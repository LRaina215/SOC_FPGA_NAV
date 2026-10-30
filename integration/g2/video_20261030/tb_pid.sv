`timescale 1ns/1ps
module tb_pid;
 reg clk=0,rst=0,en=0,sv=0;reg signed[31:0] target=0,feedback=0;
 wire signed [15:0] d0,d1;wire signed[31:0] r0,r1,e0,e1;
 wire v0,v1;wire[2:0] f0,f1;
 reg [79:0] expected[0:255];integer head=0,tail=0,checks=0;
 always #10 clk=~clk;
 motor520_velocity_pid_ref #(.CLOSED_LOOP_AT_ZERO(1),.USE_MMPS_FEEDBACK(1),
  .FEEDBACK_TIMEOUT_CYCLES(500),.STALL_SAMPLES(0),.WRONG_DIRECTION_SAMPLES(0)) reference(
  .clk(clk),.reset_n(rst),.enable(en),.target_mmps(target),.feedback_cms(16'sd0),
  .feedback_precise_mmps(feedback),.sample_valid(sv),.duty_ticks(d0),
  .ramped_target_mmps(r0),.error_mmps(e0),.control_valid(v0),.fault_code(f0));
 motor520_velocity_pid #(.CLOSED_LOOP_AT_ZERO(1),.USE_MMPS_FEEDBACK(1),
  .FEEDBACK_TIMEOUT_CYCLES(500),.STALL_SAMPLES(0),.WRONG_DIRECTION_SAMPLES(0)) dut(
  .clk(clk),.reset_n(rst),.enable(en),.target_mmps(target),.feedback_cms(16'sd0),
  .feedback_precise_mmps(feedback),.sample_valid(sv),.duty_ticks(d1),
  .ramped_target_mmps(r1),.error_mmps(e1),.control_valid(v1),.fault_code(f1));
 always @(posedge clk)begin
  #1;
  if(rst && v0)begin expected[tail]={d0,r0,e0};tail=tail+1;end
  if(rst && v1)begin
   if(head>=tail || expected[head]!=={d1,r1,e1})$fatal(1,"PID sample mismatch head=%0d tail=%0d",head,tail);
   head=head+1;checks=checks+1;
  end
 end
 task sample(input integer t,input integer fb);
  begin
   @(negedge clk);target=t;feedback=fb;sv=1;
   @(negedge clk);sv=0;repeat(15)@(negedge clk);
  end
 endtask
 initial begin
  #45;rst=1;
  // Coincident arming/sample must not be lost by the newly registered clear.
  @(negedge clk);en=1;target=600;feedback=0;sv=1;
  @(negedge clk);sv=0;repeat(15)@(negedge clk);
  if(checks!=1)$fatal(1,"first arming sample lost");
  for(integer k=0;k<80;k=k+1)sample(k<20 ? 600 : k<40 ? -600 : k<60 ? 0 : 2400,(k%11-5)*47);
  if(head!=tail || checks!=81)$fatal(1,"PID sample count");
  repeat(510)@(negedge clk);
  if(f0!==1 || f1!==1 || d0!==0 || d1!==0)$fatal(1,"feedback timeout stop");
  en=0;repeat(4)@(negedge clk);
  if(f0!==0 || f1!==0 || d1!==0)$fatal(1,"disable did not clear fault");
  en=1;sample(-500,0);if(head!=tail)$fatal(1,"rearm mismatch");
  rst=0;repeat(3)@(negedge clk);
  if(d1!==0 || f1!==0)$fatal(1,"reset");
  $display("PID_PASS checks=%0d arming=1 timeout=1 rearm=1",checks);$finish;
 end
 initial begin #1000000;$fatal(1,"watchdog");end
endmodule
