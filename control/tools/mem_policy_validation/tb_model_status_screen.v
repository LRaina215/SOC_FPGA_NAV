`timescale 1ns/1ps
module tb_model_status_screen;
reg clk=0;always #10 clk=~clk;reg rst=0;
tri1 scl,sda;reg ack_low=0;assign sda=ack_low?1'b0:1'bz;
reg signed[15:0]ax=-32768,ay=32767,az=1234,gx=-1,gy=0,gz=-23456;
reg sample_valid=0,sensor_ready=1,sensor_error=0;reg[3:0]sensor_code=0;
wire ready,error;
reg[35:0] debug_status=36'h001234081;
reg signed[15:0]sl=98,sr=-102;
oled #(.SIX_AXIS_MODE(1),.WHEEL_HEADER_MODE(1),.MODEL_DEBUG_MODE(1),.STARTUP_CYCLES(4),.IIC_QUARTER_CYCLES(4),.IIC_TIMEOUT_CYCLES(200)) dut(
.clk(clk),.reset_n(rst),.LED_NUM(32'd0),.VAL0(ax),.VAL1(ay),.VAL2(az),.VAL3(gx),.VAL4(gy),.VAL5(gz),
.SPEED_L(sl),.SPEED_R(sr),.sample_valid(sample_valid),.sensor_ready(sensor_ready),.sensor_error(sensor_error),.sensor_error_code(sensor_code),.DEBUG_STATUS(debug_status),
.IIC_SCL(scl),.IIC_SDA(sda),.ready(ready),.error(error));
reg active=0,selected=0;reg[7:0]shift=0,control=0;
integer bits=0,byte_index=0,data_count=0,i;
reg[7:0]received[0:1023],expected[0:1023];
always @(negedge sda)if(scl && rst)begin active=1;bits=0;byte_index=0;shift=0;end
always @(posedge sda)if(scl && rst)begin active=0;ack_low=0;end
always @(posedge scl)if(active)begin
 if(bits<8)begin shift={shift[6:0],sda};bits=bits+1;end else bits=9;
end
always @(negedge scl)if(active)begin
 if(bits==8)begin
  if(byte_index==0)selected=(shift==8'h7a);
  if(byte_index==1)control=shift;
  if(byte_index==2 && selected && control==8'h40)begin received[data_count%1024]=shift;data_count=data_count+1;end
  ack_low=selected;
 end else if(bits==9)begin ack_low=0;bits=0;byte_index=byte_index+1;shift=0;end
end
task compare_frame;
 begin
 for(i=0;i<1024;i=i+1)if(received[i]!==expected[i])$fatal(1,"pixel byte %0d actual=%h expected=%h",i,received[i],expected[i]);
 end
endtask
initial begin
 repeat(3)@(negedge clk);rst=1;sample_valid=1;@(negedge clk);sample_valid=0;
 wait(ready);
 wait(dut.axes_display.thread.frame_seen && dut.axes_display.thread.state==3 && dut.axes_display.thread.segment==0);
 @(negedge clk);ax=0;ay=0;az=0;gx=0;gy=0;gz=0;sample_valid=1;
 @(negedge clk);sample_valid=0;sensor_error=1;sensor_code=7;debug_status=36'h201234151;
 wait(data_count==2048);$readmemh("/tmp/axes_model_status.mem",expected);compare_frame;
 if(error)$fatal(1,"OLED error");
 wait(data_count==3072);$readmemh("/tmp/axes_wheel_error.mem",expected);compare_frame;
 @(negedge clk);sensor_error=0;debug_status=36'h002345081;sl=-123;sr=45;ax=1;ay=-2;az=300;gx=-400;gy=5000;gz=-6000;sample_valid=1;
 @(negedge clk);sample_valid=0;
 wait(data_count==4096);$readmemh("/tmp/axes_model_status_new.mem",expected);compare_frame;
 $display("PASS OLED model status header + six-axis: 12px two columns, all 1024 frame bytes, physical units, signed extrema, per-frame snapshot, error placeholders and recovery");$finish;
end
initial begin #100_000_000;$fatal(1,"timeout");end
endmodule
