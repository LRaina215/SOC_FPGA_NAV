`timescale 1ns/1ps
module tb_bmi088;
reg clk=0;always #10 clk=~clk;reg rst=0;
tri1 scl,sda;reg slave_low=0;assign sda=slave_low?1'b0:1'bz;
wire init_done,init_error,valid;wire[3:0]code;wire[5:0]operation;
wire signed[15:0] ax,ay,az,gx,gy,gz;wire[6:0]aa,ga;
bmi088 #(.STARTUP_CYCLES(4),.WRITE_WAIT_CYCLES(4),.ACCEL_CONF_WAIT_CYCLES(4),
.ACCEL_POWER_WAIT_CYCLES(4),.GYRO_WAIT_CYCLES(4),.POLL_CYCLES(10000),
.QUARTER_CYCLES(4),.TIMEOUT_CYCLES(200),.RETRY_CYCLES(2000)) dut(
.clk(clk),.reset_n(rst),.IIC_SCL(scl),.IIC_SDA(sda),
.init_done(init_done),.init_error(init_error),.data_valid(valid),.error_code(code),.operation(operation),
.accel_x(ax),.accel_y(ay),.accel_z(az),.gyro_x(gx),.gyro_y(gy),.gyro_z(gz),
.accel_addr(aa),.gyro_addr(ga),.accel_id(),.gyro_id(),.last_read());
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
initial begin
 for(i=0;i<256;i=i+1)begin acc[i]=0;gyro[i]=0;end
 acc[0]=8'h1e;gyro[0]=8'h0f;
 sample_values(16'h8000,16'h7fff,16'd1234,16'hffff,16'd0,-16'sd23456);
 repeat(3)@(negedge clk);rst=1;
 wait(valid);@(negedge clk);
 if(!init_done || init_error || aa!=7'h19 || ga!=7'h69)$fatal(1,"ID/address/config failure code=%0d op=%0d",code,operation);
 if(ax!==-16'sd32768 || ay!==16'sd32767 || az!==16'sd1234 || gx!==-16'sd1 || gy!==0 || gz!==-16'sd23456)
 $fatal(1,"burst endian failure: %0d %0d %0d %0d %0d %0d",ax,ay,az,gx,gy,gz);
 sample_values(100,200,300,-100,-200,-300);inject=1;
 wait(init_error);@(negedge clk);
 if(!injected || code!=2 || valid || ax!==-16'sd32768 || gx!==-16'sd1)$fatal(1,"failed gyro must not publish half-frame");
 wait(valid);@(negedge clk);
 if(init_error || ax!==100 || ay!==200 || az!==300 || gx!==-100 || gy!==-200 || gz!==-300)$fatal(1,"automatic retry");
 corrupt_verify=1;@(negedge clk);rst=0;repeat(3)@(negedge clk);rst=1;
 wait(init_error);@(negedge clk);if(code!=8 || init_done || valid)$fatal(1,"readback mismatch");
 corrupt_verify=0;wait(valid);@(negedge clk);if(!init_done || init_error)$fatal(1,"readback recovery");
 release_scl=0;wait(init_error);@(negedge clk);if(code!=4)$fatal(1,"SCL timeout");
 release_scl=1;wait(valid);@(negedge clk);
 $display("PASS BMI088: address fallback, IDs, config readback/mask, six-byte I2C burst, signed endian, atomic publication, NACK/readback/timeout recovery");$finish;
end
initial begin #100_000_000;$fatal(1,"timeout op=%0d code=%0d",operation,code);end
endmodule
