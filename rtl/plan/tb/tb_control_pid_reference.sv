// Read-only control snapshot module; no training or control RTL modifications.
`timescale 1ns/1ps
module tb_control_pid_reference;
 reg clk=0,reset_n=0,valid=0;always #5 clk=~clk;
 reg signed [31:0] target=0;reg signed [15:0] feedback=0;
 wire signed [15:0] duty;wire signed [31:0] reference,error;
 wire [2:0] fault;wire done;
 reg [31:0] vectors[0:29999];integer n,fd,errs=0;
 motor520_velocity_pid #(.KP_Q12(12288),.KI_Q12(1434),.KD_Q12(8192),.KFF_Q12(3277)) dut(
   .clk(clk),.reset_n(reset_n),.enable(1'b1),.target_mmps(target),.feedback_cms(feedback),
   .feedback_precise_mmps(32'd0),.sample_valid(valid),.duty_ticks(duty),
   .ramped_target_mmps(reference),.error_mmps(error),.control_valid(done),.fault_code(fault));
 initial begin
   fd=$fopen("counts.txt","r");void'($fscanf(fd,"%d",n));$fclose(fd);
   $readmemh("vectors.hex",vectors);repeat(5)@(negedge clk);reset_n=1;
   for(integer i=0;i<n;i=i+1)begin
     @(negedge clk);target=vectors[i*6];feedback=vectors[i*6+1];valid=1;
     @(negedge clk);valid=0;repeat(12)@(negedge clk);
     if(duty!==vectors[i*6+2][15:0] || reference!==vectors[i*6+3] ||
        error!==vectors[i*6+4] || fault!==vectors[i*6+5][2:0])begin
       errs=errs+1;if(errs<8)$display("PID sample %0d target%0d fb%0d duty expect%0d got%0d fault%0d",i,
          target,feedback,$signed(vectors[i*6+2]),duty,fault);
     end
   end
   if(errs==0)$display("TEST PASSED: %0d PID samples",n);else $display("TEST FAILED errs=%0d",errs);
   $finish;
 end
endmodule
