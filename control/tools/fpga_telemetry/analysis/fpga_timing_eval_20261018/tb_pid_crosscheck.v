`timescale 1ns/1ps
module tb_pid_crosscheck;
reg clk=0;always #10 clk=~clk;
reg reset_n=0,enable=1,valid=0;reg signed [31:0] target;reg signed [15:0] speed;
wire signed [15:0] duty;wire [2:0] fault;
reg [51:0] input_samples[0:416];integer i;
motor520_velocity_pid dut(.clk(clk),.reset_n(reset_n),.enable(enable),.target_mmps(target),.feedback_cms(speed),.sample_valid(valid),.duty_ticks(duty),.fault_code(fault));
initial begin
$readmemh("/home/cyd/fpga/tools/fpga_telemetry/analysis/fpga_timing_eval_20261018/pid_stim.hex",input_samples);target=0;speed=0;#100;reset_n=1;
for(i=0;i<417;i=i+1)begin
@(negedge clk);{target,speed,enable}={input_samples[i][51:20],input_samples[i][19:4],input_samples[i][0]};
// Leave time for zero/disable handling before publishing the sample.
repeat(2)@(negedge clk);valid=1;@(negedge clk);valid=0;
repeat(20)@(negedge clk);$display("ROW %d %d %d",i,$signed(duty),fault);
end
$finish;end
endmodule
