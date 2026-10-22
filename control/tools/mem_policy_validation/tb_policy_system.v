`timescale 1ns/1ps
module tb_policy_system;
reg clk=0;always #10 clk=~clk;reg rst=0;
tri1 scl,sda;reg slave_low=0;assign sda=slave_low?1'b0:1'bz;
tri1 oscl,osda;reg oack=0;assign osda=oack?1'b0:1'bz;
main #(.IR_ENABLE(0),.IMU_BIAS_SAMPLES(4),.COMMAND_FORWARD_MPS(0.1),.COMMAND_YAW_RADPS(0.2)) dut(.clk(clk),.reset_n(rst),.IRIN(1'b1),.sck(oscl),.sda(osda),.BMI088_SCL(scl),.BMI088_SDA(sda),.M1_ENCA(1'b0),.M1_ENCB(1'b0),.M2_ENCA(1'b0),.M2_ENCB(1'b0));
defparam dut.sensor.STARTUP_CYCLES=4,dut.sensor.WRITE_WAIT_CYCLES=4,
 dut.sensor.ACCEL_CONF_WAIT_CYCLES=4,dut.sensor.ACCEL_POWER_WAIT_CYCLES=4,
 dut.sensor.GYRO_WAIT_CYCLES=4,dut.sensor.POLL_CYCLES=10000,
 dut.sensor.QUARTER_CYCLES=4,dut.sensor.TIMEOUT_CYCLES=200,
 dut.sensor.RETRY_CYCLES=2000,dut.display.STARTUP_CYCLES=4,
 dut.display.IIC_QUARTER_CYCLES=4,dut.display.IIC_TIMEOUT_CYCLES=200;
defparam dut.model_thread.PERIOD_CLKS=150000,dut.model_thread.SENSOR_TIMEOUT_CLKS=150000;
wire init_done=dut.sensor_ready,init_error=dut.sensor_error,valid=dut.sample_valid;
wire[3:0]code=dut.sensor_error_code;wire[5:0]operation=dut.sensor.operation;
wire signed[15:0]ax=dut.accel_x,ay=dut.accel_y,az=dut.accel_z,gx=dut.gyro_x,gy=dut.gyro_y,gz=dut.gyro_z;
wire[6:0]aa=dut.sensor.accel_addr,ga=dut.sensor.gyro_addr;
reg [7:0] acc[0:255],gyro[0:255];
reg active=0,selected=0,isgyro=0,txmode=0,pending_read=0,master_ack=0;
reg[7:0]shift=0,sendbyte=0,aptr=0,gptr=0;
integer bits=0,byte_index=0,sentbits=0,i,starts=0;
reg inject=0,injected=0,corrupt_verify=0;reg release_scl=1;
assign scl=release_scl?1'bz:1'b0;
function[7:0] regvalue;
 input g;input[7:0]addr;
 begin
  regvalue=g?gyro[addr]:acc[addr];
  if(g && addr==8'h10)regvalue=regvalue|8'h80; // Gyro BW bit7 is read-only.
  if(corrupt_verify && !g && addr==8'h41)regvalue=8'h03;
 end
endfunction
always @(negedge sda)if(scl && rst)begin
 active=1;selected=0;bits=0;byte_index=0;shift=0;txmode=0;pending_read=0;starts=starts+1;
end
always @(posedge sda)if(scl && rst)begin active=0;slave_low=0;end
always @(posedge scl)if(active)begin
 if(!txmode)begin
  if(bits<8)begin shift={shift[6:0],sda};bits=bits+1;end else bits=9;
 end else begin
  if(sentbits<8)sentbits=sentbits+1;
  else begin master_ack=(sda==0);sentbits=9;end
 end
end
always @(negedge scl)if(active)begin
 if(!txmode)begin
  if(bits==8)begin
   if(byte_index==0)begin
    selected=(shift[7:1]==7'h19 || shift[7:1]==7'h69);
    isgyro=(shift[7:1]==7'h69);pending_read=shift[0];
   end
   if(byte_index==1 && selected)begin
    if(isgyro)gptr=shift;else aptr=shift;
   end
   if(byte_index==2 && selected)begin
    if(isgyro)gyro[gptr]=shift;else acc[aptr]=shift;
   end
   slave_low=selected;
   if(byte_index==1 && selected && isgyro && shift==2 && inject)begin
    slave_low=0;inject=0;injected=1;
   end
  end else if(bits==9)begin
   slave_low=0;bits=0;byte_index=byte_index+1;shift=0;
   if(pending_read && selected)begin
    txmode=1;sentbits=0;sendbyte=regvalue(isgyro,isgyro?gptr:aptr);slave_low=~sendbyte[7];
   end
  end
 end else begin
  if(sentbits<8)slave_low=~sendbyte[7-sentbits];
  else if(sentbits==8)slave_low=0;
  else if(sentbits==9)begin
   if(master_ack)begin
    if(isgyro)gptr=gptr+1;else aptr=aptr+1;
    sendbyte=regvalue(isgyro,isgyro?gptr:aptr);sentbits=0;slave_low=~sendbyte[7];
   end else begin slave_low=0;txmode=0;bits=0;byte_index=0;end
  end
 end
end
task sample_values;
 input[15:0] a,b,c,d,e,f;
 begin
 {acc[8'h13],acc[8'h12]}=a;{acc[8'h15],acc[8'h14]}=b;{acc[8'h17],acc[8'h16]}=c;
 {gyro[8'h03],gyro[8'h02]}=d;{gyro[8'h05],gyro[8'h04]}=e;{gyro[8'h07],gyro[8'h06]}=f;
 end
endtask

integer input_file,layer_file,result_file,frames=0,j;
reg saw_motor_pwm=0,saw_servo_pwm=0;
always @(posedge clk) begin
 if(dut.M1_PWM || dut.M2_PWM) saw_motor_pwm<=1;
 if(dut.SERVO1 && dut.SERVO2 && dut.SERVO3 && dut.SERVO4) saw_servo_pwm<=1;
end
reg [15:0] held_lh,held_lk,held_rh,held_rk;
always @(posedge clk) begin
 if(dut.model_thread.frame_valid) begin
  for(j=0;j<28;j=j+1) $fwrite(input_file,"%06x\n",dut.model_thread.current_flat[j*24+:24]);
  for(j=0;j<140;j=j+1) $fwrite(input_file,"%06x\n",dut.model_thread.history_flat[j*24+:24]);
 end
 if(dut.model_thread.policy.core.dbg_valid)
  $fwrite(layer_file,"%06x\n",dut.model_thread.policy.core.dbg_data);
end
initial begin
 input_file=$fopen("/tmp/policy_inputs.mem","w");
 layer_file=$fopen("/tmp/policy_layers.mem","w");
 result_file=$fopen("/tmp/policy_outputs.txt","w");
 for(i=0;i<256;i=i+1)begin acc[i]=0;gyro[i]=0;end
 acc[0]=8'h1e;gyro[0]=8'h0f;
 sample_values(0,0,10920,0,0,0);
 repeat(3)@(negedge clk);rst=1;
 while(frames<4) begin
  @(posedge dut.model_action_valid);repeat(3)@(negedge clk);
  if(!dut.model_active || dut.model_fault || dut.imu_bias_suspect) $fatal(1,"not active/fault");
  for(j=0;j<6;j=j+1) $fwrite(result_file,"%0d ",$signed(dut.model_executed_actions[j*24+:24]));
  $fwrite(result_file,"%0d %0d %0d %0d %0d %0d %0d %0d\n",
   dut.model_left_mmps,dut.model_right_mmps,dut.model_lh_us,dut.model_lk_us,dut.model_rh_us,dut.model_rk_us,
   dut.model_thread.captured_height,dut.model_inference_cycles);
  frames=frames+1;
  if(frames==1) begin
   force dut.command_height_q12=24'sd615;
   sample_values(1000,-2000,10600,80,-40,20);
  end
 end
 if(!saw_motor_pwm || !saw_servo_pwm) $fatal(1,"physical PWM outputs never activated");
 if(dut.model_completed_frames!=4) $fatal(1,"completion count");
 held_lh=dut.model_lh_us;held_lk=dut.model_lk_us;held_rh=dut.model_rh_us;held_rk=dut.model_rk_us;
 force dut.command_emergency_stop=1'b1;
 repeat(5)@(negedge clk);
 if(dut.model_active || dut.M1_PWM || dut.M2_PWM || dut.model_left_mmps || dut.model_right_mmps)
  $fatal(1,"emergency stop did not suppress targets/PWM");
 if({held_lh,held_lk,held_rh,held_rk}!=={dut.model_lh_us,dut.model_lk_us,dut.model_rh_us,dut.model_rk_us})
  $fatal(1,"leg pose changed on fault");
 release dut.command_emergency_stop;
 repeat(10)@(negedge clk);
 if(dut.model_active || !dut.model_thread.stop_latched) $fatal(1,"unexpected auto restart");
 force dut.command_run=1'b0;repeat(5)@(negedge clk);
 release dut.command_run;
 wait(dut.model_active);
 // Exercise stale-IMU detection, with actuator control previously rearmed.
 force dut.sample_valid=1'b0;
 repeat(150010)@(negedge clk);
 if(dut.model_active || !dut.model_thread.stop_latched || dut.M1_PWM || dut.M2_PWM)
  $fatal(1,"stale sample did not stop");
 $fclose(input_file);$fclose(layer_file);$fclose(result_file);
 $display("PASS full main: real BMI I2C -> fusion -> history -> ROM inference -> IK/servo + PID; height update, emergency latch/rearm and stale IMU stop");
 $finish;
end
initial begin #50_000_000;$fatal(1,"policy integrated timeout flags=%h",dut.model_status_flags);end
endmodule
