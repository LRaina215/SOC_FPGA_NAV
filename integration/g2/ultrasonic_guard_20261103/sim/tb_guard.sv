`timescale 1ns/1ps
module tb_guard;
reg clk=0;always #500 clk=~clk; // 1MHz parameterized command logic, physical ms retained
reg rst=0,key_active=0,key_event=0,manual=0,hard_stop=0;
reg [7:0] key=0;
reg signed [23:0] req=0,yaw=0;
reg strobe=0,valid=1,center=0;
reg [15:0] mm=1000;
wire enabled,healthy,braking,stopped;
wire signed [23:0] out_v,out_y;
wire [15:0] age;
wire [23:0] cap;
integer checks=0,phase=0,ms=0,csv,previous,i;
ultrasonic_drive_guard #(.CLOCK_HZ(1000000)) dut(
 .clk(clk),.reset_n(rst),.remote_key(key),.remote_key_event(key_event),.remote_key_active(key_active),
 .manual_owner(manual),.hard_stop(hard_stop),.requested_forward_q12(req),.requested_yaw_q12(yaw),
 .sample_strobe(strobe),.sample_valid(valid),.forward_ready(center),.distance_mm(mm),
 .enabled(enabled),.forward_q12(out_v),.yaw_q12(out_y),.sample_healthy(healthy),
 .braking(braking),.stop_latched(stopped),.sample_age_ms(age),.range_target_q12(cap));
task check(input bit ok,input string what);
 begin checks++;if(!ok)$fatal(1,"%s phase=%0d v=%0d cap=%0d age=%0d latch=%0d",what,phase,out_v,cap,age,stopped);end
endtask
task ping(input bit v,input [15:0] d);
 begin @(negedge clk);valid=v;mm=d;strobe=1;@(negedge clk);strobe=0;repeat(2)@(negedge clk);end
endtask
task keypress(input [7:0] k);
 begin @(negedge clk);key=k;key_active=1;key_event=1;@(negedge clk);key_event=0;repeat(3)@(negedge clk);end
endtask
task wait_ms(input integer n,input bit refresh);
 integer j;
 begin
  for(j=0;j<n;j++) begin
   repeat(1000)@(negedge clk);ms++;
   if(refresh && (j%50)==0)ping(1,mm);
   $fdisplay(csv,"%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",ms,phase,mm,req,out_v,healthy,stopped,manual);
   if(phase==2)begin check(out_v>=previous,"acceleration not monotone");check(out_v-previous<=4,"acceleration slew");end
   if(phase==3)begin check(out_v<=previous,"deceleration not monotone");check(previous-out_v<=8,"deceleration slew");end
   previous=out_v;
  end
 end
endtask
initial begin
 csv=$fopen("guard_profile.csv","w");$fdisplay(csv,"ms,phase,distance_mm,requested_q12,forward_q12,healthy,stop_latched,manual_owner");
 repeat(4)@(negedge clk);rst=1;repeat(3)@(negedge clk);
 check(!enabled,"default must be disabled");
 for(i=-10;i<=10;i++)begin req=i*111;yaw=i*217;manual=i%2;@(negedge clk);check(out_v===req && out_y===yaw,"disabled exact bypass");end
 phase=1;req=1229;yaw=0;manual=1;keypress(8'h16);check(enabled,"0 enables");
 repeat(3)begin keypress(8'h16);check(enabled,"repeated full frames must not toggle");end
 ping(1,1000);wait_ms(5,0);check(out_v==0 && !healthy,"old side sample must not arm");
 center=1;ping(1,1000);check(!healthy,"first forward sample insufficient");ping(1,1000);check(healthy,"second forward sample arms");
 phase=2;previous=out_v;wait_ms(650,1);check(out_v==1229,"far steady speed");
 phase=3;ping(1,500);previous=out_v;wait_ms(120,1);check(out_v==877,"500mm expected linear target");
 ping(1,400);previous=out_v;wait_ms(100,1);check(out_v==526,"400mm expected linear target");
 ping(1,300);previous=out_v;wait_ms(120,1);check(out_v==175,"300mm expected linear target");
 phase=4;ping(1,250);check(out_v==0 && stopped,"stop boundary");
 ping(1,1000);ping(1,1000);wait_ms(10,1);check(out_v==0 && stopped,"held forward must not restart");
 req=0;manual=0;repeat(5)@(negedge clk);check(!stopped,"clear distance and release rearm");
 manual=1;req=1229;wait_ms(10,1);check(out_v>0 && out_v<30,"bounded restart ramp");
 phase=5;ping(0,16'hffff);check(out_v==0 && !healthy && stopped,"invalid echo fail stop");
 req=-1229;yaw=0;repeat(3)@(negedge clk);check(out_v==-1229,"reverse escape preserved");
 req=0;yaw=4096;repeat(3)@(negedge clk);check(out_v==0 && out_y==4096,"pure turn preserved (no side coverage claimed)");
 phase=6;manual=0;req=765;yaw=-987;check(!healthy,"fault still present");@(negedge clk);check(out_v==req && out_y==yaw,"NAV bypass during sonar fault");
 req=-765;yaw=987;@(negedge clk);check(out_v==req && out_y==yaw,"NAV negative bypass");
 ping(1,1000);ping(1,1000);repeat(4)@(negedge clk);manual=1;req=1229;yaw=0;wait_ms(20,1);
 phase=7;wait_ms(151,0);check(!healthy && out_v==0 && stopped,"150ms missing-update watchdog");
 ping(1,1000);ping(1,1000);wait_ms(3,0);check(out_v==0 && stopped,"fault recovery cannot restart held key");
 manual=0;req=0;repeat(4)@(negedge clk);manual=1;req=1229;wait_ms(10,1);check(out_v>0,"fresh command after valid recovery");
 center=0;repeat(4)@(negedge clk);check(out_v==0 && !healthy,"lost forward qualification");
 phase=8;manual=0;req=4096;yaw=2048;repeat(3)@(negedge clk);check(out_v==req && out_y==yaw,"NAV remains unchanged while mode on");
 hard_stop=1;repeat(3)@(negedge clk);check(out_v==0 && out_y==0,"hard stop highest priority");
 key_active=0;repeat(3)@(negedge clk);keypress(8'h16);check(!enabled,"second press disables");check(out_v==0,"mode key cannot bypass hard stop");
 hard_stop=0;repeat(3)@(negedge clk);check(out_v==req && out_y==yaw,"off restores original source");
 phase=9;key_active=0;repeat(3)@(negedge clk);keypress(8'h16);center=1;
 valid=1;mm=16'hxxxx;strobe=1;@(negedge clk);strobe=0;repeat(3)@(negedge clk);manual=1;req=1229;
 repeat(3)@(negedge clk);check(out_v===24'd0 && !healthy,"X payload cannot qualify clear space");
 rst=0;repeat(3)@(negedge clk);rst=1;repeat(3)@(negedge clk);check(!enabled && !stopped,"reset returns to disabled");
 $fclose(csv);$display("TEST PASSED GUARD checks=%0d",checks);$finish;
end
initial begin #2000000000;$fatal(1,"guard watchdog");end
endmodule
