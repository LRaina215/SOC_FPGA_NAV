`timescale 1ns/1ps
module tb_servo50;
reg clk=0;always #10 clk=~clk;reg rst=0;wire s1,s2,s3,s4;
servo_driver dut(.clk(clk),.reset_n(rst),.enable(1'b1),
.pulse_lh_us(16'd1500),.pulse_lk_us(16'd1750),.pulse_rh_us(16'd1600),.pulse_rk_us(16'd1700),
.SERVO1(s1),.SERVO2(s2),.SERVO3(s3),.SERVO4(s4),.frame_tick(),
.active_lh_us(),.active_lk_us(),.active_rh_us(),.active_rk_us());
time start1,start2,start3,start4;
initial begin repeat(3)@(negedge clk);rst=1;
@(posedge s1);start1=$time;@(negedge s1);if($time-start1!=1500000)$fatal(1,"50MHz1500us");
@(posedge s1);if($time-start1!=20000000)$fatal(1,"50MHz20ms");
$display("PASS servo default50MHz: exact1500us pulse,20ms frame,50Hz output");$finish;end
initial begin @(posedge s2);start2=$time;@(negedge s2);if($time-start2!=1250000)$fatal(1,"LK1250us");end
initial begin @(posedge s3);start3=$time;@(negedge s3);if($time-start3!=1600000)$fatal(1,"RH1600us");end
initial begin @(posedge s4);start4=$time;@(negedge s4);if($time-start4!=1700000)$fatal(1,"RK1700us");end
initial begin #25000000;$fatal(1,"timeout");end
endmodule
