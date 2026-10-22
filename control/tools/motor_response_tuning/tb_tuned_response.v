`timescale 1ns/1ps
module tb_tuned_response;
reg clk=0;always #10 clk=~clk;reg rst=0,valid=0;
reg signed [31:0] target=0;
reg signed [15:0] old_speed=0,new_speed=0;
wire signed [15:0] old_duty,new_duty;wire [2:0] old_fault,new_fault;
motor520_velocity_pid #(.FEEDBACK_TIMEOUT_CYCLES(500)) old_loop(.clk(clk),.reset_n(rst),.enable(1'b1),
 .target_mmps(target),.feedback_cms(old_speed),.sample_valid(valid),.duty_ticks(old_duty),.fault_code(old_fault));
motor520_velocity_pid #(.KP_Q12(16384),.KI_Q12(1434),.KD_Q12(2048),.KFF_Q12(14336),
 .CLOSED_LOOP_AT_ZERO(1),.RAMP_STEP_MMPS(600),.FEEDBACK_TIMEOUT_CYCLES(500)) candidate(
 .clk(clk),.reset_n(rst),.enable(1'b1),.target_mmps(target),.feedback_cms(new_speed),.sample_valid(valid),.duty_ticks(new_duty),.fault_code(new_fault));
real ov=0,nv=0,op=0,np=0,ideal=0,old_command=0,old_error=0,new_error=0;
integer n,oc=0,nc=0,oc_prev=0,nc_prev=0,od,nd;
initial begin
 repeat(3)@(negedge clk);rst=1;
 for(n=0;n<400;n=n+1)begin
  ov=ov+(1.0-$exp(-0.01/0.12106245))*(0.47042279*old_duty/2500.0-ov);
  nv=nv+(1.0-$exp(-0.01/0.12106245))*(0.47042279*new_duty/2500.0-nv);
  op=op+ov*0.01;np=np+nv*0.01;
  oc=$rtoi($floor(op/0.0001546996));nc=$rtoi($floor(np/0.0001546996));od=oc-oc_prev;nd=nc-nc_prev;
  oc_prev=oc;nc_prev=nc;
  old_speed=$rtoi(od*1.547+(od<0 ? -0.5:0.5));new_speed=$rtoi(nd*1.547+(nd<0 ? -0.5:0.5));
  ideal=ideal+(1.0-$exp(-0.01/0.045))*(old_command-ideal);
  target=(n<30 || (n>=130 && n<190)) ? 0 : ((n<130 || n>=290) ? 300 : -300);
  old_command=target/1000.0;
  if(n>=30)begin old_error=old_error+(ov-ideal)*(ov-ideal)*0.01;new_error=new_error+(nv-ideal)*(nv-ideal)*0.01;end
  @(negedge clk);valid=1;@(negedge clk);valid=0;repeat(20)@(negedge clk);
  if(old_fault || new_fault)$fatal(1,"unexpected nominal-plant motor fault old=%d new=%d n=%d",old_fault,new_fault,n);
  if(new_duty>2400 || new_duty<-2400)$fatal(1,"duty limit");
 end
 if(new_error>=old_error*0.8)$fatal(1,"candidate did not improve modeled response old=%f new=%f",old_error,new_error);
 $display("PASS actual RTL candidate on reduced telemetry-fit plant: tracking squared integrals old=%f new=%f; NOT physical validation",old_error,new_error);$finish;
end
initial begin #1000000;$fatal(1,"timeout");end
endmodule
