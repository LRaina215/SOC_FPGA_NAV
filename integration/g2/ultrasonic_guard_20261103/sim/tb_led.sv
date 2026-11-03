`timescale 1ns/1ps
module tb_led;
reg clk=0;always #500000 clk=~clk;
reg rst=0,enabled=0,healthy=0;
reg [3:0] cpu;
wire [3:0] board;
integer i,j,checks=0;
ultrasonic_mode_led #(.CLOCK_HZ(1000)) dut(.clk(clk),.reset_n(rst),.enabled(enabled),.sample_healthy(healthy),
 .cpu_leds(cpu),.board_cpu_leds(board));
initial begin
 repeat(3)@(negedge clk);rst=1;
 for(j=0;j<16;j++)begin
  cpu=j;@(negedge clk);
  if(board!=={1'b0,cpu[2:0]})$fatal(1,"disabled/CPU low3 LED outputs");checks++;
 end
 enabled=1;
 for(i=0;i<249;i++)begin @(negedge clk);if(board[3]!==1)$fatal(1,"first on period too short");checks++;end
 @(negedge clk);if(board[3]!==0)$fatal(1,"250ms off edge");checks++;
 repeat(249)@(negedge clk);if(board[3]!==0)$fatal(1,"off period too short");checks++;
 @(negedge clk);if(board[3]!==1)$fatal(1,"500ms on edge");checks++;
 healthy=1;
 for(i=0;i<550;i++)begin
  cpu=i%16;@(negedge clk);
  if(board!=={1'b1,cpu[2:0]})$fatal(1,"healthy must stay solid and retain CPU LEDs");checks++;
 end
 healthy=0;enabled=0;#1;if(board[3]!==0)$fatal(1,"disable must extinguish immediately");checks++;
 enabled=1;rst=0;#1;if(board[3]!==0)$fatal(1,"reset must extinguish");checks++;
 $display("TEST PASSED LED checks=%0d",checks);$finish;
end
endmodule
