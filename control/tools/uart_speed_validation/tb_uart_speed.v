`timescale 1ns/1ps
module tb_uart_speed;
reg clk=0;always #10 clk=~clk;
reg rst=0,rx=1;
main #(.IR_ENABLE(0),.UART_CLKS_PER_BIT(16),.COMMAND_FORWARD_MPS(0.1)) dut(
 .clk(clk),.reset_n(rst),.UART_RXD(rx),.IRIN(1'b1),
 .M1_ENCA(1'b0),.M1_ENCB(1'b0),.M2_ENCA(1'b0),.M2_ENCB(1'b0));
defparam dut.speed_command_rx.FRAME_TIMEOUT_CLKS=4000;
function [15:0] crc_byte;
 input [15:0] previous;input [7:0] value;reg [15:0] c;integer i;
 begin c=previous^{value,8'b0};for(i=0;i<8;i=i+1)c=c[15]?(c<<1)^16'h1021:(c<<1);crc_byte=c;end
endfunction
task byte_send;input [7:0] data;integer i;
 begin @(negedge clk);rx=0;repeat(16)@(negedge clk);
 for(i=0;i<8;i=i+1)begin rx=data[i];repeat(16)@(negedge clk);end
 rx=1;repeat(16)@(negedge clk);end
endtask
task packet;input signed [15:0] v;input corrupt;reg[15:0] c;
 begin c=crc_byte(crc_byte(crc_byte(crc_byte(16'hffff,8'h10),8'h02),v[7:0]),v[15:8]);
 byte_send(8'ha5);byte_send(8'h5a);byte_send(8'h10);byte_send(8'h02);
 byte_send(v[7:0]);byte_send(v[15:8]);byte_send(c[7:0]^{7'b0,corrupt});byte_send(c[15:8]);repeat(30)@(negedge clk);end
endtask
task motion_packet;
 input signed [15:0] v,yaw;input corrupt;reg [15:0] c;
 begin
 c=crc_byte(crc_byte(16'hffff,8'h10),8'h04);
 c=crc_byte(crc_byte(crc_byte(crc_byte(c,v[7:0]),v[15:8]),yaw[7:0]),yaw[15:8]);
 byte_send(8'ha5);byte_send(8'h5a);byte_send(8'h10);byte_send(8'h04);
 byte_send(v[7:0]);byte_send(v[15:8]);byte_send(yaw[7:0]);byte_send(yaw[15:8]);
 byte_send(c[7:0]^{7'b0,corrupt});byte_send(c[15:8]);repeat(30)@(negedge clk);
 end
endtask
task check_yaw;input signed [23:0] expected;
 begin if(dut.command_yaw_q12!==expected)$fatal(1,"yaw expected %0d got %0d",expected,dut.command_yaw_q12);end
endtask
task check;input signed[23:0] expected;
 begin if(dut.command_forward_q12!==expected) $fatal(1,"command expected %0d got %0d",expected,dut.command_forward_q12);end
endtask
initial begin
 repeat(10)@(negedge clk);rst=1;repeat(300)@(negedge clk);check(410);
 packet(300,0);check(1229);if(!dut.uart_speed_seen)$fatal(1,"not selected");
 packet(-300,1);check(1229); // CRC error must not change live command.
 packet(601,0);check(1229); // Out-of-range input rejected.
 packet(-300,0);check(-1229);
 packet(0,0);check(0);
 byte_send(8'h77);byte_send(8'ha5);byte_send(8'ha5); // garbage/repeated sync
 packet(600,0);check(2458);
 byte_send(8'ha5);byte_send(8'h5a);byte_send(8'h10);
 repeat(5000)@(negedge clk);packet(300,0);check(1229);
 motion_packet(0,1000,0);check(0);check_yaw(4096);
 motion_packet(300,-1000,1);check(0);check_yaw(4096); // atomic CRC rejection
 motion_packet(300,2901,0);check(0);check_yaw(4096); // bad yaw rejects both
 motion_packet(601,-1000,0);check(0);check_yaw(4096); // bad linear rejects both
 motion_packet(300,-1000,0);check(1229);check_yaw(-4096);
 motion_packet(-600,2900,0);check(-2458);check_yaw(11878);
 motion_packet(600,-2900,0);check(2458);check_yaw(-11878);
 packet(300,0);check(1229);check_yaw(0); // legacy frame clears yaw
 motion_packet(300,1000,0);check(1229);check_yaw(4096);
 force dut.command_emergency_stop=1'b1;repeat(2)@(negedge clk);check(0);check_yaw(0);
 release dut.command_emergency_stop;repeat(2)@(negedge clk);check(1229);check_yaw(4096);
 rst=0;repeat(10)@(negedge clk);rst=1;repeat(300)@(negedge clk);check(410);check_yaw(0);
 $display("PASS UART linear+yaw atomic frames, legacy yaw0, CRC/range/parser, signed Q12, top command mux, emergency/reset");$finish;
end
initial begin #10000000;$fatal(1,"timeout");end
endmodule
