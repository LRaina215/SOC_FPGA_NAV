`timescale 1ns/1ps
module tb_ir_remote;
reg clk=0;always #10 clk=~clk;
reg rst=0,ir=1;wire signed [23:0]v,y;wire[7:0]key;wire active,event_key,event_repeat;
wire signed [23:0]height;wire estop;
integer keys=0,repeats=0;
ir_remote_control #(.CLOCK_FREQ_HZ(1000000)) dut(.clk(clk),.reset_n(rst),.ir_in(ir),
 .forward_q12(v),.yaw_q12(y),.height_q12(height),.emergency_stop(estop),
 .key_code(key),.key_active(active),.key_event(event_key),.repeat_event(event_repeat));
always @(posedge clk)begin if(event_key)keys=keys+1;if(event_repeat)repeats=repeats+1;end
task level;
 input value;input integer us;
 begin ir=value;repeat(us)@(negedge clk);end
endtask
task send_frame;
 input[7:0]cmd;input bad;
 reg[31:0]frame;integer i;
 begin
  frame={~cmd,cmd,16'hff00};if(bad)frame[31]=~frame[31];
  level(1,20000);level(0,9000);level(1,4500);
  for(i=0;i<32;i=i+1)begin level(0,560);level(1,frame[i]?1690:560);end
  level(0,560);level(1,10);
 end
endtask
task send_repeat;
 begin level(1,50000);level(0,9000);level(1,2250);level(0,560);level(1,10);end
endtask
task expect_command;
 input integer ev,ey;
 begin if(v!==ev || y!==ey)$fatal(1,"command v=%0d y=%0d expected=%0d,%0d",v,y,ev,ey);end
endtask
integer previous_keys,previous_repeats,held_height;
initial begin
 repeat(3)@(negedge clk);rst=1;
 send_repeat;expect_command(0,0);if(repeats)$fatal(1,"orphan repeat accepted");
 send_frame(8'h18,0);expect_command(1229,0);if(key!=8'h18 || !active)$fatal(1,"forward key");
 repeat(4)begin send_repeat;expect_command(1229,0);end
 if(repeats!=4)$fatal(1,"repeat hold");
 level(1,180010);expect_command(0,0);if(active)$fatal(1,"release watchdog");
 send_repeat;expect_command(0,0);
 if(height!==535 || estop)$fatal(1,"height/stop reset");
 send_frame(8'h15,0);level(1,50000);
 if(height<=535)$fatal(1,"plus did not raise");
 repeat(45)send_repeat;
 if(height!==778)$fatal(1,"upper height clamp %0d",height);
 level(1,180010);held_height=height;level(1,30000);
 if(height!==held_height)$fatal(1,"height changed after release");
 send_frame(8'h07,0);level(1,50000);
 if(height>=778)$fatal(1,"minus did not lower");
 repeat(75)send_repeat;
 if(height!==348)$fatal(1,"lower height clamp %0d",height);
 send_frame(8'h1c,1);if(estop)$fatal(1,"bad stop frame accepted");
 send_frame(8'h1c,0);if(!estop)$fatal(1,"stop not latched");
 held_height=height;
 send_frame(8'h18,0);expect_command(0,0);
 send_frame(8'h5a,0);expect_command(0,0);
 send_frame(8'h15,0);repeat(2)send_repeat;
 if(height!==held_height || !estop)$fatal(1,"stop released or height moved");
 rst=0;repeat(4)@(negedge clk);rst=1;
 if(estop || height!==535)$fatal(1,"reset did not restore");
 send_frame(8'h18,0);expect_command(1229,0);
 $display("PASS +/- height direction, long hold, both clamps, release hold, invalid stop rejection, latched stop and reset recovery");
 send_frame(8'h52,0);expect_command(-1229,0);
 send_frame(8'h08,0);expect_command(0,4096);
 send_frame(8'h5a,0);expect_command(0,-4096);
 previous_keys=keys;send_frame(8'h18,1);expect_command(0,-4096);
 if(keys!=previous_keys)$fatal(1,"bad inverse accepted");
 send_frame(8'h45,0);expect_command(0,0); // Any other valid key requests parking.
 send_frame(8'h18,0);expect_command(1229,0);
 level(0,50);level(1,75);expect_command(1229,0); // Noise does not change command.
 rst=0;repeat(4)@(negedge clk);expect_command(0,0);rst=1;
 send_repeat;expect_command(0,0);
 $display("PASS NEC reference keys, bounded commands, long-press repeats, release timeout, orphan repeat rejection, invalid inverse/noise, other-key stop and reset");$finish;
end
initial begin #1000000000;$fatal(1,"timeout");end
endmodule
