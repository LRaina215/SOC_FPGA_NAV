`timescale 1ns/1ps
module tb_nec;
reg clk=0;always #500 clk=~clk;
reg rst=0,ir=1;
wire signed [23:0] rv,ry,rh,v,y;
wire active,key_event,repeat_event,estop,enabled,healthy,stop_latched;
wire [7:0] key;
integer checks=0,repeat_count=0;
wire manual=rv!=0 || ry!=0;
ir_remote_control #(.CLOCK_FREQ_HZ(1000000)) remote(.clk(clk),.reset_n(rst),.ir_in(ir),
 .forward_q12(rv),.yaw_q12(ry),.height_q12(rh),.emergency_stop(estop),
 .key_code(key),.key_active(active),.key_event(key_event),.repeat_event(repeat_event));
ultrasonic_drive_guard #(.CLOCK_HZ(1000000)) guard(.clk(clk),.reset_n(rst),
 .remote_key(key),.remote_key_event(key_event),.remote_key_active(active),.manual_owner(manual),
 .hard_stop(estop),.requested_forward_q12(rv),.requested_yaw_q12(ry),
 .sample_strobe(1'b0),.sample_valid(1'b0),.forward_ready(1'b0),.distance_mm(16'hffff),
 .enabled(enabled),.forward_q12(v),.yaw_q12(y),.sample_healthy(healthy),
 .braking(),.stop_latched(stop_latched),.sample_age_ms(),.range_target_q12());
always @(posedge clk)if(repeat_event)repeat_count++;
task level(input bit b,input integer us);
 begin ir=b;repeat(us)@(negedge clk);end
endtask
task frame(input [7:0] code,input bit bad);
 reg [31:0] bits;integer i;
 begin
  bits={~code,code,16'hff00};if(bad)bits[31]=~bits[31];
  level(1,20000);level(0,9000);level(1,4500);
  for(i=0;i<32;i++)begin level(0,560);level(1,bits[i]?1690:560);end
  level(0,560);level(1,20);
 end
endtask
task rep;
 begin level(1,50000);level(0,9000);level(1,2250);level(0,560);level(1,20);end
endtask
task check(input bit ok,input string why);
 begin checks++;if(!ok)$fatal(1,"%s key=%h en=%0d v=%0d y=%0d stop=%0d",why,key,enabled,v,y,estop);end
endtask
initial begin
 repeat(3)@(negedge clk);rst=1;
 rep;check(!enabled,"orphan repeat");frame(8'h16,1);check(!enabled,"bad inverse cannot enable");
 frame(8'h16,0);check(enabled,"valid 0 enables");
 repeat(3)begin rep;check(enabled,"short repeat cannot retoggle");end
 frame(8'h16,0);check(enabled,"held full frame cannot retoggle");
 level(1,181000);frame(8'h16,0);check(!enabled,"released second 0 disables");
 frame(8'h18,0);check(v==1229 && y==0,"off original forward");
 frame(8'h16,0);check(enabled,"different key then 0 is new press");
 frame(8'h18,0);check(v==0 && manual,"unqualified range inhibits forward");
 frame(8'h52,0);check(v==-1229,"backwards escape");
 frame(8'h08,0);check(y==4096,"turn remains available");
 frame(8'h15,0);level(1,30000);check(rh>535,"height original behavior");
 frame(8'h1c,0);check(estop && v==0 && y==0,"5 still latches hard stop");
 frame(8'h16,0);check(!enabled && estop,"mode toggle cannot release stop5");
 frame(8'h52,0);check(v==0 && y==0,"hard stop dominates reverse");
 rst=0;repeat(4)@(negedge clk);rst=1;repeat(4)@(negedge clk);
 check(!enabled && !estop && rh==535,"hardware reset defaults");
 $display("TEST PASSED NEC checks=%0d repeats=%0d",checks,repeat_count);$finish;
end
initial begin #(2500ms);$fatal(1,"NEC watchdog");end
endmodule
