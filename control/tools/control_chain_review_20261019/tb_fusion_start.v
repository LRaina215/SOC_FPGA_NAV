`timescale 1ns/1ps
module tb_fusion_start;
reg clk=0;always #5000 clk=~clk;reg rst=0,sample=0;wire valid,calib;wire signed[23:0]gx,gy,gz;
imu_fusion #(.CLK_HZ(100000),.BIAS_SAMPLES(4)) dut(.clk(clk),.rst_n(rst),.sample_valid(sample),.accel_x(16'sd0),.accel_y(-16'sd3735),.accel_z(16'sd10261),.gyro_x(16'sd0),.gyro_y(16'sd0),.gyro_z(16'sd0),.ang_vel_x(),.ang_vel_y(),.ang_vel_z(),.gravity_x(gx),.gravity_y(gy),.gravity_z(gz),.valid(valid),.calibrating(calib),.bias_suspect(),.accel_rejected());
integer ticks=0,seen=0;
always @(negedge clk)begin
 sample=0;
 if(rst)begin ticks=ticks+1;if(ticks%1000==0)sample=1;end
 if(valid)begin
  seen=seen+1;
  if(seen==1)begin
   $display("FIRST_POSTCAL_STATIC20DEG gravity_q12=%0d,%0d,%0d; expected approximately0,1401,-3849",gx,gy,gz);
   if(gy>100)$fatal(1,"startup behavior changed; review test expectation");
  end
  if(seen==50)begin $display("AFTER50_SAMPLES gravity_q12=%0d,%0d,%0d",gx,gy,gz);$finish;end
 end
end
initial begin repeat(3)@(negedge clk);rst=1;end
initial begin #1000000000;$fatal(1,"timeout");end
endmodule
