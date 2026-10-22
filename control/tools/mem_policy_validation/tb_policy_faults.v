`timescale 1ns/1ps
module tb_policy_faults;
reg clk=0;always #10 clk=~clk;
reg rst_n=0,sample=0;integer c=0;
always @(negedge clk)begin
 c=c+1;sample=(c%1000==0);
end
wire bad_active,bad_suspect,late_active;
wire[2:0]late_fault;
wire signed[31:0]bl,br,ll,lr;
mem_policy #(.BIAS_SAMPLES(4),.PERIOD_CLKS(20000)) bad(
 .clk(clk),.rst_n(rst_n),.enable(1'b1),.sensor_ready(1'b1),.sensor_error(1'b0),.sample_valid(sample),.external_fault(1'b0),
 .accel_x(16'sd0),.accel_y(16'sd0),.accel_z(16'sd10920),.gyro_x(16'sd500),.gyro_y(16'sd0),.gyro_z(16'sd0),
 .left_encoder_delta(32'sd0),.right_encoder_delta(32'sd0),
 .command_forward_q12(24'sd0),.command_yaw_q12(24'sd0),.command_height_q12(24'sd535),
 .target_left_mmps(bl),.target_right_mmps(br),.active(bad_active),.bias_suspect(bad_suspect));
mem_policy #(.BIAS_SAMPLES(4),.PERIOD_CLKS(20000)) late(
 .clk(clk),.rst_n(rst_n),.enable(1'b1),.sensor_ready(1'b1),.sensor_error(1'b0),.sample_valid(sample),.external_fault(1'b0),
 .accel_x(16'sd0),.accel_y(16'sd0),.accel_z(16'sd10920),.gyro_x(16'sd0),.gyro_y(16'sd0),.gyro_z(16'sd0),
 .left_encoder_delta(32'sd0),.right_encoder_delta(32'sd0),
 .command_forward_q12(24'sd0),.command_yaw_q12(24'sd0),.command_height_q12(24'sd535),
 .target_left_mmps(ll),.target_right_mmps(lr),.active(late_active),.policy_fault(late_fault));
always @(posedge clk)if(rst_n && (bad_active || late_active))$fatal(1,"faulted model armed");
initial begin
 repeat(2)@(negedge clk);rst_n=1;
 repeat(60000)@(negedge clk);
 if(!bad_suspect || bl || br || ll || lr || !late_fault[1] || !late.stop_latched)
  $fatal(1,"bad bias/deadline not handled flags=%b",late_fault);
 $display("PASS moving-at-start bias rejection and missed-policy-deadline latch, no actuator arming");$finish;
end
endmodule
