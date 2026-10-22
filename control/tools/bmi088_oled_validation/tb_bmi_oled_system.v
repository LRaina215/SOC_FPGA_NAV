`timescale 1ns/1ps
module tb_system;
reg clk=0;always #10 clk=~clk;reg rst=0;
tri1 scl,sda;reg slave_low=0;assign sda=slave_low?1'b0:1'bz;
tri1 oscl,osda;reg oack=0;assign osda=oack?1'b0:1'bz;
main #(.MOTOR_ENABLE(0),.OLED_MODEL_DEBUG(0)) dut(.clk(clk),.reset_n(rst),.sck(oscl),.sda(osda),.BMI088_SCL(scl),.BMI088_SDA(sda),.M1_ENCA(1'b0),.M1_ENCB(1'b0),.M2_ENCA(1'b0),.M2_ENCB(1'b0));
defparam dut.sensor.STARTUP_CYCLES=4,dut.sensor.WRITE_WAIT_CYCLES=4,
 dut.sensor.ACCEL_CONF_WAIT_CYCLES=4,dut.sensor.ACCEL_POWER_WAIT_CYCLES=4,
 dut.sensor.GYRO_WAIT_CYCLES=4,dut.sensor.POLL_CYCLES=10000,
 dut.sensor.QUARTER_CYCLES=4,dut.sensor.TIMEOUT_CYCLES=200,
 dut.sensor.RETRY_CYCLES=2000,dut.display.STARTUP_CYCLES=4,
 dut.display.IIC_QUARTER_CYCLES=4,dut.display.IIC_TIMEOUT_CYCLES=200;
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
reg oa=0,oselected=0;reg[7:0]oshift=0,ocontrol=0;
integer obits=0,obyte=0,odata=0;
reg[7:0]oframe[0:1023],golden[0:1023];
always @(negedge osda)if(oscl && rst)begin oa=1;obits=0;obyte=0;oshift=0;end
always @(posedge osda)if(oscl && rst)begin oa=0;oack=0;end
always @(posedge oscl)if(oa)begin
 if(obits<8)begin oshift={oshift[6:0],osda};obits=obits+1;end else obits=9;
end
always @(negedge oscl)if(oa)begin
 if(obits==8)begin
  if(obyte==0)oselected=(oshift==8'h7a);
  if(obyte==1)ocontrol=oshift;
  if(obyte==2 && oselected && ocontrol==8'h40)begin oframe[odata%1024]=oshift;odata=odata+1;end
  oack=oselected;
 end else if(obits==9)begin oack=0;obits=0;obyte=obyte+1;oshift=0;end
end
initial begin
 for(i=0;i<256;i=i+1)begin acc[i]=0;gyro[i]=0;end
 acc[0]=8'h1e;gyro[0]=8'h0f;
 sample_values(16'h8000,16'h7fff,16'd1234,16'hffff,16'd0,-16'sd23456);
 repeat(3)@(negedge clk);rst=1;
 wait(odata==2048);$readmemh("/tmp/axes_system_wheel.mem",golden);
 for(i=0;i<1024;i=i+1)if(oframe[i]!==golden[i])$fatal(1,"integrated screen byte %0d actual=%h golden=%h",i,oframe[i],golden[i]);
 if(!init_done || init_error || aa!=7'h19 || ga!=7'h69)$fatal(1,"sensor state");
 $display("PASS end-to-end main with zero-speed 520 header: real BMI088 I2C register model -> all six physical values -> real OLED I2C -> all 1024 golden screen bytes");$finish;
end
initial begin #100_000_000;$fatal(1,"integrated timeout");end
endmodule
