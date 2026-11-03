`timescale 1ns/1ps
module tb_policy;
reg clk=0;always #10 clk=~clk;
reg rst=0,key_event=0,key_active=0,manual=0;
reg [7:0] key=0;
reg signed [23:0] request_v=0,request_y=0;
reg [15:0] mm=1200;
reg range_strobe=0;
wire enabled,healthy,obstacle_stop;
wire signed [23:0] v,y;
ultrasonic_drive_guard guard(.clk(clk),.reset_n(rst),.remote_key(key),
 .remote_key_event(key_event),.remote_key_active(key_active),.manual_owner(manual),.hard_stop(1'b0),
 .requested_forward_q12(request_v),.requested_yaw_q12(request_y),
 .sample_strobe(range_strobe),.sample_valid(1'b1),.forward_ready(enabled),.distance_mm(mm),
 .enabled(enabled),.forward_q12(v),.yaw_q12(y),.sample_healthy(healthy),.stop_latched(obstacle_stop),
 .braking(),.sample_age_ms(),.range_target_q12());
reg [31:0] sample_count=0,range_count=0;
wire sample_valid=sample_count==499999;
always @(posedge clk)begin
 if(!rst)begin sample_count<=0;range_count<=0;range_strobe<=0;end
 else begin
  sample_count<=sample_valid ? 0 : sample_count+1;
  range_count<=range_count==2499999 ? 0 : range_count+1;
  range_strobe<=range_count==2499999; // 50ms valid forward samples, after explicit initial pair
 end
end
wire ready,active,action_valid;
wire [2:0] fault;
wire [31:0] frames,cycles;
wire [143:0] actions;
wire signed [31:0] left,right;
wire [15:0] lh,lk,rh,rk;
mem_policy #(.BIAS_SAMPLES(4)) policy(
 .clk(clk),.rst_n(rst),.enable(1'b1),.sensor_ready(1'b1),.sensor_error(1'b0),
 .sample_valid(sample_valid),.external_fault(1'b0),
 .accel_x(16'sd0),.accel_y(16'sd0),.accel_z(16'sd10920),
 .gyro_x(16'sd0),.gyro_y(16'sd0),.gyro_z(16'sd0),
 .left_encoder_delta(32'sd0),.right_encoder_delta(32'sd0),
 .command_forward_q12(v),.command_yaw_q12(y),.command_height_q12(24'sd535),
 .target_left_mmps(left),.target_right_mmps(right),
 .servo_lh_us(lh),.servo_lk_us(lk),.servo_rh_us(rh),.servo_rk_us(rk),
 .ready(ready),.active(active),.action_valid(action_valid),.policy_fault(fault),
 .completed_frames(frames),.inference_cycles(cycles),.executed_actions(actions),
 .calibrating(),.bias_suspect(),.accel_rejected(),.status_flags(),
 .debug_fusion_valid(),.debug_wx(),.debug_wy(),.debug_wz(),.debug_gx(),.debug_gy(),.debug_gz());
integer checks=0,publications=0,phase=0,positive_frames=0,zero_frames=0,csv;
reg signed [23:0] captured_v,captured_y;
reg [31:0] first_stop_frames;
reg signed [23:0] loaded_v,loaded_y;
always @(posedge clk)if(rst)begin
 if(policy.capture)begin captured_v<=v;captured_y<=y;end
 // The actual inference input RAM, not a flat output disabled by ring storage.
 if(policy.policy.state==3)begin
  loaded_v=policy.policy.core.input_mem[6];loaded_y=policy.policy.core.input_mem[7];
  if(loaded_v!==captured_v*2)$fatal(1,"guard command did not reach real4500 input6: %0d != %0d",loaded_v,captured_v*2);
  if(loaded_y!==captured_y*5325/4096)$fatal(1,"yaw input mismatch");
  checks++;
 end
 if(action_valid)begin
  publications++;
  if($isunknown({actions,left,right,lh,lk,rh,rk}))$fatal(1,"unknown real4500 result");
  if(fault!=0 || !ready)$fatal(1,"zero setpoint must not fault/disable policy");
  if(phase==1 && captured_v>0)positive_frames++;
  if(phase==2 && captured_v==0)zero_frames++;
  $fdisplay(csv,"%0t,%0d,%0d,%0d,%0d,%0d,%0d,%0d",$time,phase,captured_v,frames,active,cycles,left,right);
 end
end
initial begin
 csv=$fopen("real4500_publications.csv","w");$fdisplay(csv,"time_ns,phase,captured_forward_q12,frames,active,inference_cycles,left_target_mmps,right_target_mmps");
 repeat(5)@(negedge clk);rst=1;
 wait(ready);wait(publications>=2);
 @(negedge clk);key=8'h16;key_active=1;key_event=1;
 @(negedge clk);key_event=0;
 wait(healthy);
 @(negedge clk);manual=1;request_v=1229;phase=1;
 wait(positive_frames>=3);
 @(negedge clk);mm=200; // next real sample requests zero, while forward remains held
 wait(obstacle_stop);#1000;
 if(v!==24'sd0 || !ready)$fatal(1,"near obstacle did not give zero without disabling policy");
 first_stop_frames=frames;phase=2;
 wait(zero_frames>=3);repeat(10)@(negedge clk);
 if(!active || fault!=0 || frames<=first_stop_frames || cycles!=83534 || checks<publications)
   $fatal(1,"balance inference did not continue at original timing");
 checks++;
 $fclose(csv);$display("TEST PASSED POLICY checks=%0d publications=%0d positive_frames=%0d stopped_frames=%0d cycles=%0d",checks,publications,positive_frames,zero_frames,cycles);$finish;
end
initial begin #(400ms);$fatal(1,"real4500 watchdog frames=%0d ready=%0d fault=%0d",frames,ready,fault);end
endmodule
