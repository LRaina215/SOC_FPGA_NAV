`timescale 1ns/1ps
module tb_ir_system;
reg clk=0;always #10 clk=~clk;reg rst=0;reg ir_input=1;
tri1 scl,sda;reg slave_low=0;assign sda=slave_low?1'b0:1'bz;
tri1 oscl,osda;reg oack=0;assign osda=oack?1'b0:1'bz;
main #(.IR_ENABLE(1),.IMU_BIAS_SAMPLES(4),.COMMAND_FORWARD_MPS(0.1),.COMMAND_YAW_RADPS(0.2)) dut(.clk(clk),.reset_n(rst),.IRIN(ir_input),.sck(oscl),.sda(osda),.BMI088_SCL(scl),.BMI088_SDA(sda),.M1_ENCA(1'b0),.M1_ENCB(1'b0),.M2_ENCA(1'b0),.M2_ENCB(1'b0));
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


defparam dut.remote_thread.CLOCK_FREQ_HZ=1000000;
reg signed[23:0] captured_v,captured_y,captured_h;
reg saw_forward=0,saw_backward=0,saw_left=0,saw_right=0,saw_zero=0;
integer observations=0;
always @(posedge clk)begin
 if(rst && dut.model_active) begin
  if(dut.model_left_mmps>600 || dut.model_left_mmps< -600 || dut.model_right_mmps>600 || dut.model_right_mmps< -600)
   $fatal(1,"wheel target exceeded retained600mm/s limit");
  if(dut.left_duty>2400 || dut.left_duty< -2400 || dut.right_duty>2400 || dut.right_duty< -2400)
   $fatal(1,"PWM exceeded retained2400-tick limit");
 end
 if(dut.model_thread.capture)begin captured_v<=dut.command_forward_q12;captured_y<=dut.command_yaw_q12;captured_h<=dut.command_height_q12;end
 if(dut.model_thread.frame_valid)begin
  observations=observations+1;
  if($signed(dut.model_thread.current_flat[6*24+:24])!=$signed(captured_v)*2)
   $fatal(1,"remote forward did not reach policy observation");
  if($signed(dut.model_thread.current_flat[8*24+:24])!=$signed(captured_h)*5)
   $fatal(1,"remote height did not reach policy observation");
  if(captured_y==4096 && $signed(dut.model_thread.current_flat[7*24+:24])!=5325)$fatal(1,"left yaw scale");
  if(captured_y==-4096 && $signed(dut.model_thread.current_flat[7*24+:24])!=-5325)$fatal(1,"right yaw scale");
  if(captured_v==1229)saw_forward=1;
  if(captured_v==-1229)saw_backward=1;
  if(captured_y==4096)saw_left=1;
  if(captured_y==-4096)saw_right=1;
  if(captured_y==0 && captured_v==0)saw_zero=1;
 end
end
task level;
 input value;input integer ticks;
 begin ir_input=value;repeat(ticks)@(negedge clk);end
endtask
task send_frame;
 input[7:0]cmd;
 reg[31:0]f;integer b;
 begin
  f={~cmd,cmd,16'hff00};level(1,20000);level(0,9000);level(1,4500);
  for(b=0;b<32;b=b+1)begin level(0,560);level(1,f[b]?1690:560);end
  level(0,560);level(1,10);
 end
endtask
reg [63:0] stopped_servos;
integer raised_height;
initial begin
 for(i=0;i<256;i=i+1)begin acc[i]=0;gyro[i]=0;end
 acc[0]=8'h1e;gyro[0]=8'h0f;sample_values(0,0,10920,0,0,0);
 repeat(3)@(negedge clk);rst=1;
 wait(dut.model_active);
 if(dut.command_forward_q12 || dut.command_yaw_q12)$fatal(1,"idle remote not parking");
 send_frame(8'h18);wait(saw_forward);
 send_frame(8'h52);wait(saw_backward);
 send_frame(8'h08);wait(saw_left);
 send_frame(8'h5a);wait(saw_right);
 // Release: motion request zero, model keeps balancing, no stop-latch.
 level(1,180010);repeat(160000)@(negedge clk);
 if(dut.command_forward_q12 || dut.command_yaw_q12 || !dut.model_active || dut.model_fault)
  $fatal(1,"release disabled balance or retained movement");
 if(!saw_zero || dut.model_completed_frames<5)$fatal(1,"no continued inference");
 send_frame(8'h15);level(1,60000);
 if(dut.command_height_q12<=535)$fatal(1,"height not raised in main");
 level(1,180010);raised_height=dut.command_height_q12;
 level(1,20000);
 if(dut.command_height_q12!=raised_height)$fatal(1,"height not held in main");
 send_frame(8'h07);level(1,60000);
 if(dut.command_height_q12>=raised_height)$fatal(1,"height not lowered in main");
 send_frame(8'h1c);repeat(10)@(negedge clk);
 if(!dut.command_emergency_stop || dut.control_run || dut.wheel_control.enable || dut.model_active)
  $fatal(1,"stop did not gate model/wheel control");
 if(dut.model_left_mmps || dut.model_right_mmps || dut.M1_PWM || dut.M2_PWM)
  $fatal(1,"stop left motor output active");
 stopped_servos={dut.model_lh_us,dut.model_lk_us,dut.model_rh_us,dut.model_rk_us};
 send_frame(8'h18);send_frame(8'h15);level(1,60000);
 if(!dut.command_emergency_stop || dut.command_forward_q12 || dut.command_yaw_q12 || dut.model_active)
  $fatal(1,"motion key released emergency stop");
 if({dut.model_lh_us,dut.model_lk_us,dut.model_rh_us,dut.model_rk_us}!==stopped_servos)
  $fatal(1,"servos did not hold stopped pose");
 $display("PASS main IR pulses -> directional and height policy observations; release keeps balancing; latched stop gates wheels/model and holds servos");$finish;
end
initial begin #100000000;$fatal(1,"IR integrated timeout observations=%0d",observations);end
endmodule
