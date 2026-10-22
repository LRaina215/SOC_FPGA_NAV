`timescale 1ns/1ps
module tb_multitext;
reg clk=0;always #10 clk=~clk;
reg reset_n=0;reg[31:0] LED_NUM=1234;
tri1 scl,sda;reg ack_low=0;assign sda=ack_low?1'b0:1'bz;
wire ready,error;
oled #(.TEXT("Ab1:"),.TEXT_LEN(4),.TEXT_X(4),.TEXT_Y(16),.TEXT_FONT_SIZE(8),
.TEXT2("Z!"),.TEXT2_LEN(2),.TEXT2_X(10),.TEXT2_Y(32),.TEXT2_FONT_SIZE(8),
.TEXT4("09"),.TEXT4_LEN(2),.TEXT4_X(20),.TEXT4_Y(48),.TEXT4_FONT_SIZE(8),
.DIGITS(10),.FONT_SIZE(8),.STARTUP_CYCLES(4),.IIC_QUARTER_CYCLES(4),.IIC_TIMEOUT_CYCLES(100)) dut(
.clk(clk),.reset_n(reset_n),.LED_NUM(LED_NUM),.IIC_SCL(scl),.IIC_SDA(sda),.ready(ready),.error(error));
reg active=0,selected=0;reg [7:0] shift=0,control=0;
integer bits=0,byte_index=0,data_count=0,i;
reg[7:0]received[0:1023];reg[7:0]expected[0:23];
always @(negedge sda) if(scl && reset_n) begin active=1;bits=0;byte_index=0;shift=0;end
always @(posedge sda) if(scl && reset_n) begin active=0;ack_low=0;end
always @(posedge scl) if(active) begin
 if(bits<8) begin shift={shift[6:0],sda};bits=bits+1;end else bits=9;
end
always @(negedge scl) if(active) begin
 if(bits==8) begin
  if(byte_index==0)selected=(shift==8'h7a);
  if(byte_index==1)control=shift;
  if(byte_index==2 && selected && control==8'h40)begin
   received[data_count%1024]=shift;data_count=data_count+1;
  end
  ack_low=selected;
 end else if(bits==9)begin ack_low=0;bits=0;byte_index=byte_index+1;shift=0;end
end
initial begin
expected[0]=0;expected[1]='h7c;expected[2]='h12;expected[3]='h11;expected[4]='h12;expected[5]='h7c;
expected[6]=0;expected[7]='h7f;expected[8]='h48;expected[9]='h44;expected[10]='h44;expected[11]='h38;
expected[12]=0;expected[13]=0;expected[14]='h42;expected[15]='h7f;expected[16]='h40;expected[17]=0;
expected[18]=0;expected[19]=0;expected[20]='h36;expected[21]='h36;expected[22]=0;expected[23]=0;
repeat(3)@(negedge clk);reset_n=1;
wait(ready);@(negedge clk);if(error || dut.text_display.thread.driver.oled_addr!=7'h3d)$fatal(1,"address fallback");
wait(dut.text_display.thread.state==2);@(negedge clk);
if(dut.text_display.thread.bcd!==40'h0000001234)$fatal(1,"1234 BCD");
for(i=0;i<24;i=i+1)if(dut.text_display.thread.driver.gram[260+i]!==expected[i])$fatal(1,"text byte %0d",i);
if(dut.text_display.thread.driver.gram[522]!==8'h00) $fatal(1,"extra text byte 522");
if(dut.text_display.thread.driver.gram[523]!==8'h61) $fatal(1,"extra text byte 523");
if(dut.text_display.thread.driver.gram[524]!==8'h51) $fatal(1,"extra text byte 524");
if(dut.text_display.thread.driver.gram[525]!==8'h49) $fatal(1,"extra text byte 525");
if(dut.text_display.thread.driver.gram[526]!==8'h45) $fatal(1,"extra text byte 526");
if(dut.text_display.thread.driver.gram[527]!==8'h43) $fatal(1,"extra text byte 527");
if(dut.text_display.thread.driver.gram[528]!==8'h00) $fatal(1,"extra text byte 528");
if(dut.text_display.thread.driver.gram[529]!==8'h00) $fatal(1,"extra text byte 529");
if(dut.text_display.thread.driver.gram[530]!==8'h00) $fatal(1,"extra text byte 530");
if(dut.text_display.thread.driver.gram[531]!==8'h2f) $fatal(1,"extra text byte 531");
if(dut.text_display.thread.driver.gram[532]!==8'h00) $fatal(1,"extra text byte 532");
if(dut.text_display.thread.driver.gram[533]!==8'h00) $fatal(1,"extra text byte 533");
if(dut.text_display.thread.driver.gram[788]!==8'h00) $fatal(1,"extra text byte 788");
if(dut.text_display.thread.driver.gram[789]!==8'h3e) $fatal(1,"extra text byte 789");
if(dut.text_display.thread.driver.gram[790]!==8'h51) $fatal(1,"extra text byte 790");
if(dut.text_display.thread.driver.gram[791]!==8'h49) $fatal(1,"extra text byte 791");
if(dut.text_display.thread.driver.gram[792]!==8'h45) $fatal(1,"extra text byte 792");
if(dut.text_display.thread.driver.gram[793]!==8'h3e) $fatal(1,"extra text byte 793");
if(dut.text_display.thread.driver.gram[794]!==8'h00) $fatal(1,"extra text byte 794");
if(dut.text_display.thread.driver.gram[795]!==8'h06) $fatal(1,"extra text byte 795");
if(dut.text_display.thread.driver.gram[796]!==8'h49) $fatal(1,"extra text byte 796");
if(dut.text_display.thread.driver.gram[797]!==8'h49) $fatal(1,"extra text byte 797");
if(dut.text_display.thread.driver.gram[798]!==8'h29) $fatal(1,"extra text byte 798");
if(dut.text_display.thread.driver.gram[799]!==8'h1e) $fatal(1,"extra text byte 799");
LED_NUM=32'hffffffff;
wait(data_count==2048);
for(i=0;i<1024;i=i+1)if(received[i]!==dut.text_display.thread.driver.gram[i])$fatal(1,"refresh %0d",i);
wait(dut.text_display.thread.state==2);@(negedge clk);if(dut.text_display.thread.bcd!==40'h4294967295)$fatal(1,"max BCD");
LED_NUM=0;wait(data_count==3072);wait(dut.text_display.thread.state==2);@(negedge clk);
if(dut.text_display.thread.bcd!==0)$fatal(1,"zero BCD");
$display("PASS three text sections, skipped TEXT3, text order, ASCII font bytes, coordinates, whole frame, numeric snapshot, max/zero, address fallback");$finish;
end
initial begin #100_000_000;$fatal(1,"timeout");end
endmodule
