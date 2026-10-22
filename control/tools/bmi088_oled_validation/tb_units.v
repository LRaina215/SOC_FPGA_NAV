`timescale 1ns/1ps
module oled_driver #(parameter ADDRESS=0,AUTO_ADDRESS=0,STARTUP_CYCLES=0,IIC_QUARTER_CYCLES=1,IIC_TIMEOUT_CYCLES=100)(
input wire clk,reset_n,inout wire OLED_SCL,OLED_SDA,input wire start,input wire[2:0]op,
input wire[7:0]x,y,data,input wire[4:0]font_size,input wire mode,
output wire busy,output reg ready,done,error,output wire[3:0]iic_error_code);
assign busy=0;assign iic_error_code=0;
always @(posedge clk)begin done<=0;if(!reset_n)begin ready<=0;error<=0;end
else begin ready<=1;if(start)done<=1;end end
endmodule
module tb_units;
reg clk=0;always #10 clk=~clk;reg rst=0;reg signed[15:0]val=0;reg sample_valid=0;
wire ready,error;tri1 scl,sda;
oled_axes_thread dut(.clk(clk),.reset_n(rst),.VAL0(val),.VAL1(val),.VAL2(val),.VAL3(val),.VAL4(val),.VAL5(val),
.sample_valid(sample_valid),.sensor_ready(1'b1),.sensor_error(1'b0),.sensor_error_code(4'd0),
.IIC_SCL(scl),.IIC_SDA(sda),.ready(ready),.error(error));
reg[7:0] chars[0:59];integer frames=0,i,slot,column;
always @(posedge clk)if(rst && dut.start)begin
if(dut.state==6)frames=frames+1;
else if(dut.char_y!=0)begin
slot=(dut.char_y/16)-1+(dut.char_x>=64 ? 3 : 0);
column=(dut.char_x>=64 ? dut.char_x-64 : dut.char_x)/6;
chars[slot*10+column]=dut.ascii;
if(dut.driver.font_size!=12)$fatal(1,"font size");
end end
task check;
input signed[15:0] raw;input[55:0] accel_field,gyro_field;
reg[7:0]expected;integer prior,index;
begin
@(negedge clk);val=raw;sample_valid=1;@(negedge clk);sample_valid=0;
wait(dut.frame_seen && dut.frame_values=={6{raw}});prior=frames;
wait(frames>prior);@(negedge clk);
for(index=0;index<6;index=index+1)begin
for(i=0;i<7;i=i+1)begin
expected=(index<3) ? accel_field[55-i*8 -: 8] : gyro_field[55-i*8 -: 8];
if(chars[index*10+3+i]!==expected)$fatal(1,"raw=%0d axis=%0d char=%0d actual=%h expected=%h",raw,index,i,chars[index*10+3+i],expected);
end end
end endtask
initial begin repeat(3)@(negedge clk);rst=1;
check(16'sd0,"+0.00g ","+0000.0");
check(16'sd1,"+0.00g ","+0000.1");
check(-16'sd1,"+0.00g ","-0000.1");
check(16'sd54,"+0.00g ","+0003.3");
check(16'sd55,"+0.01g ","+0003.4");
check(-16'sd54,"+0.00g ","-0003.3");
check(-16'sd55,"-0.01g ","-0003.4");
check(16'sd10647,"+0.98g ","+0649.8");
check(16'sd10648,"+0.98g ","+0649.9");
check(16'sd10702,"+0.98g ","+0653.2");
check(-16'sd10702,"-0.98g ","-0653.2");
check(16'sd10920,"+1.00g ","+0666.5");
check(-16'sd10920,"-1.00g ","-0666.5");
check(16'sd5460,"+0.50g ","+0333.3");
check(-16'sd5460,"-0.50g ","-0333.3");
check(16'sd10865,"+0.99g ","+0663.1");
check(16'sd10866,"+1.00g ","+0663.2");
check(16'sd16384,"+1.50g ","+1000.0");
check(-16'sd16384,"-1.50g ","-1000.0");
check(16'sd8192,"+0.75g ","+0500.0");
check(-16'sd8192,"-0.75g ","-0500.0");
check(16'sd1234,"+0.11g ","+0075.3");
check(-16'sd23456,"-2.15g ","-1431.6");
check(16'sd32767,"+3.00g ","+1999.9");
check(-16'sd32768,"-3.00g ","-2000.0");
$display("PASS 25 signed unit/rounding cases across all six axes: 0.98g, 1g, near-zero, carry, full scale");$finish;end
initial begin #100000000;$fatal(1,"timeout");end
endmodule
