`timescale 1ns/1ps
module tb_bmi088;
reg clk=0, reset_n=0; always #10 clk=~clk;
wire sclk,mosi,c1,c2,ready,error,valid,swapped;
reg miso=1;
reg reverse_cs=0,disconnected=0,bad_config=0;
wire [7:0] aid,gid,p1,p2;wire pc;
bmi088 #(.STARTUP_CYCLES(5),.WRITE_WAIT_CYCLES(5),.GYRO_WAIT_CYCLES(5),.POLL_CYCLES(4000),.SPI_HALF_CYCLES(10)) dut(
.clk(clk),.reset_n(reset_n),.SPI_MISO(miso),.SPI_SCLK(sclk),.SPI_MOSI(mosi),.SPI_CSB1(c1),.SPI_CSB2(c2),
.init_done(ready),.init_error(error),.data_valid(valid),.cs_swapped(swapped),.accel_id(aid),.gyro_id(gid),.probe_cs1(p1),.probe_cs2(p2),.probe_complete(pc));
reg [7:0] acc[0:127],gyr[0:127];
integer bitn=0,byten=0,index=0,i; reg [7:0] incoming=0,address=0,reply=0;
reg isgyro=0,reading=0; wire active=!c1 || !c2;
always @(negedge active) begin bitn=0;byten=0;incoming=0;miso=1;end
always @(posedge active) begin
bitn=0;byten=0;incoming=0;reading=0;miso=1;isgyro=(!c2)^reverse_cs;
end
// Mode0: latch MOSI on rising edges; advance MISO on falling edges.
always @(posedge sclk) if(active) begin
 incoming={incoming[6:0],mosi};bitn=bitn+1;
 if(bitn==8) begin
  if(byten==0) begin address=incoming;reading=incoming[7];end
  else if(!reading) begin
   index=(address[6:0]+byten-1)&127;
   if(isgyro)gyr[index]=incoming;else acc[index]=incoming;
  end
  incoming=0;byten=byten+1;bitn=0;
 end
end
always @(negedge sclk) if(active) begin
 if(byten==0 || (!isgyro && byten==1)) reply=0;
 else begin
  index=(address[6:0]+byten-(isgyro?1:2))&127;
  reply=isgyro?gyr[index]:acc[index];
  if(bad_config && reading && index==64 && !isgyro)reply=0;
 end
 miso=disconnected?1:reply[7-bitn];
end
task initialize;
 begin
 reset_n=0;repeat(10)@(posedge clk);
 for(i=0;i<128;i=i+1)begin acc[i]=0;gyr[i]=0;end
 acc[0]=8'h1e;gyr[0]=8'h0f;acc[18]=8'h34;acc[19]=8'h12;
 @(negedge clk);reset_n=1;
 end
endtask
task await_result;
 input expect_error;input expect_swap;
 integer n;
 begin
 n=0;while(!error && !valid && n<100000)begin @(posedge clk);n=n+1;end
 if(n==100000 || error!==expect_error || swapped!==expect_swap)$fatal(1,"BMI fail n=%d error=%b swap=%b",n,error,swapped);
 if(expect_error)begin
 n=0;while(!pc && n<10000)begin @(posedge clk);n=n+1;end
 if(!pc || ready || valid)$fatal(1,"diagnostic probe must complete with actuators disarmed");
 if(disconnected && (p1!==8'hff || p2!==8'hff))$fatal(1,"disconnected probe IDs");
 if(bad_config && p2!==8'h0f)$fatal(1,"CS2 gyro probe ID");
 end
 if(!expect_error && (aid!==8'h1e || gid!==8'h0f || dut.accel_x!==16'h1234))$fatal(1,"ID/data mismatch");
 end
endtask
initial begin
 initialize;await_result(0,0);
 reverse_cs=1;initialize;await_result(0,1);
 reverse_cs=0;disconnected=1;initialize;await_result(1,1);
 disconnected=0;bad_config=1;initialize;await_result(1,0);
 $display("PASS BMI088 normal/swapped IDs, disconnected and config mismatch fail closed");$finish;
end
endmodule
