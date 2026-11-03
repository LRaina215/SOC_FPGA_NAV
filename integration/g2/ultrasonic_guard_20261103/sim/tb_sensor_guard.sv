`timescale 1ns/1ps
module tb_sensor_guard;
reg clk=0;always #500 clk=~clk;
reg rst=0,key_event=0,key_active=0,manual=1,echo=0;
reg [7:0] key=0;
reg signed [23:0] req=1229,yaw=0;
integer obstacle_mm=1200,response=1,checks=0,samples=0,csv;
wire enabled,trig,servo_pwm,valid,timeout,strobe,ready,healthy,stop_latched;
wire [15:0] mm;wire [1:0] position;
wire signed [23:0] v,y;
g2_range_servo #(.CLOCKS_PER_US(1)) sensor(.clk(clk),.reset_n(rst),.echo(echo),.hold_forward(enabled),
 .trig(trig),.servo_pwm(servo_pwm),.distance_mm(mm),.distance_valid(valid),
 .distance_timeout(timeout),.servo_position(position),.sample_strobe(strobe),.forward_ready(ready));
ultrasonic_drive_guard #(.CLOCK_HZ(1000000)) guard(.clk(clk),.reset_n(rst),
 .remote_key(key),.remote_key_event(key_event),.remote_key_active(key_active),
 .manual_owner(manual),.hard_stop(1'b0),.requested_forward_q12(req),.requested_yaw_q12(yaw),
 .sample_strobe(strobe),.sample_valid(valid),.forward_ready(ready),.distance_mm(mm),
 .enabled(enabled),.forward_q12(v),.yaw_q12(y),.sample_healthy(healthy),
 .braking(),.stop_latched(stop_latched),.sample_age_ms(),.range_target_q12());
always @(negedge trig)if(rst && response)begin
 #200000;echo=1;#(obstacle_mm*5800);echo=0;
end
always @(negedge clk)if(strobe)samples++;
task check(input bit ok,input string why);
 begin checks++;if(!ok)$fatal(1,"%s v=%0d measured=%0d",why,v,mm);end
endtask
initial begin
 csv=$fopen("sensor_guard_profile.csv","w");$fdisplay(csv,"time_us,obstacle_mm,measured_mm,forward_q12,healthy,stop_latched,manual_owner");
 forever begin #(1ms);$fdisplay(csv,"%0d,%0d,%0d,%0d,%0d,%0d,%0d",$time/1000,obstacle_mm,mm,v,healthy,stop_latched,manual);end
end
initial begin
 repeat(4)@(negedge clk);rst=1;repeat(5)@(negedge clk);
 check(!enabled && v==1229,"disabled original manual path");
 key=8'h16;key_active=1;key_event=1;@(negedge clk);key_event=0;
 repeat(5)@(negedge clk);check(enabled && v==0,"arming stops positive setpoint");
 #(299ms);check(!healthy && v==0,"boot settling cannot start forward");
 wait(healthy);check(position==1 && ready,"front stream qualified");
 #(650ms);check(v==1229,"full-speed forward after fresh readings and ramp");
 obstacle_mm=500;#(200ms);check(v>=874 && v<=878,"500mm limits speed near 0.214m/s");
 obstacle_mm=400;#(200ms);check(v>=523 && v<=527,"400mm limits speed near 0.128m/s");
 obstacle_mm=300;#(200ms);check(v>=172 && v<=176,"300mm limits speed near 0.043m/s");
 obstacle_mm=250;wait(stop_latched);repeat(4)@(negedge clk);check(v==0,"actual Echo chain stops at 250mm");
 obstacle_mm=1200;#(180ms);check(v==0 && stop_latched,"clear echoes cannot relaunch held command");
 manual=0;req=777;yaw=-456;repeat(6)@(negedge clk);
 check(v==777 && y==-456,"NAV exact bypass through shared chain");
 manual=1;req=1229;yaw=0;#(10ms);check(v>0 && v<30,"rearmed manual slow restart");
 response=0;wait(!healthy);repeat(4)@(negedge clk);check(v==0 && stop_latched,"missing Echo locks forward");
 req=-1229;repeat(4)@(negedge clk);check(v==-1229,"reverse escape with lost Echo");
 $fclose(csv);$display("TEST PASSED SENSOR_GUARD checks=%0d samples=%0d",checks,samples);$finish;
end
initial begin #(3s);$fatal(1,"sensor guard watchdog");end
endmodule
