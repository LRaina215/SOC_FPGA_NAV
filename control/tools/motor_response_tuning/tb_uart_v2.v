`timescale 1ns/1ps
module tb_uart_v2;
localparam CPB=217;
reg clk=0,reset_n=0;always #10 clk=~clk;
reg [959:0] payload;wire tx;
uart_telemetry dut(.clk(clk),.reset_n(reset_n),.payload(payload),.tx(tx));
integer i,j,count,fd;reg [7:0] value;
initial begin
 for(i=0;i<120;i=i+1)payload[i*8+:8]=i;
 payload[800+:32]=-300;payload[832+:32]=600;payload[864+:32]=-50;payload[896+:32]=25;
 payload[928+:16]=12;payload[944+:16]=34;
 #50 reset_n=1;@(posedge dut.active);repeat(100)@(posedge clk);payload={960{1'b1}};
end
initial begin
 fd=$fopen("uart_v2_bytes.hex","w");count=0;wait(reset_n);
 while(count<378)begin
  @(negedge tx);#(CPB*20+CPB*10);
  for(j=0;j<8;j=j+1)begin value[j]=tx;#(CPB*20);end
  if(tx!==1'b1)$fatal(1,"stop bit");
  $fdisplay(fd,"%02x",value);count=count+1;
 end
 $fclose(fd);$display("PASS V2 UART:378 bytes at production100Hz/230400,8N1 stop bits valid");$finish;
end
initial begin #50000000;$fatal(1,"UART timeout");end
endmodule
