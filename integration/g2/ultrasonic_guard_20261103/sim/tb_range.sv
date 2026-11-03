`timescale 1ns/1ps
module tb_range;
reg clk=0;always #500 clk=~clk;
reg rst=0,echo=0,hold_forward=0;
wire trig,pwm,valid,timeout,strobe,ready;
wire [15:0] mm;wire [1:0] pos;
wire old_trig,old_pwm,old_valid,old_timeout;wire [15:0] old_mm;wire [1:0] old_pos;
integer checks=0,samples=0,response=0,expected_mm=500,high_us=0;
bit compare_old=1;
time enabled_at,ready_at,pwm_start;
g2_range_servo #(.CLOCKS_PER_US(1)) dut(.clk(clk),.reset_n(rst),.echo(echo),.hold_forward(hold_forward),
 .trig(trig),.servo_pwm(pwm),.distance_mm(mm),.distance_valid(valid),.distance_timeout(timeout),
 .servo_position(pos),.sample_strobe(strobe),.forward_ready(ready));
g2_range_servo_reference #(.CLOCKS_PER_US(1)) old(.clk(clk),.reset_n(rst),.echo(echo),
 .trig(old_trig),.servo_pwm(old_pwm),.distance_mm(old_mm),.distance_valid(old_valid),
 .distance_timeout(old_timeout),.servo_position(old_pos));
always @(negedge clk) if(rst)begin
 if(compare_old)begin
  if({trig,pwm,valid,timeout,mm,pos}!=={old_trig,old_pwm,old_valid,old_timeout,old_mm,old_pos})
   $fatal(1,"disabled range/scan differs from original");checks++;
 end
 if(strobe)samples++;
end
// Echo wire stimulus only: no force or internal state writes.
always @(negedge trig) if(rst)begin
 if(response==0)begin #200000;echo=1;#(expected_mm*5800);echo=0;end
 else if(response==2)begin #200000;echo=1;#2000;echo=0;end
 else if(response==3)begin #200000;echo=1;#33000000;echo=0;end
end
task next_sample;
 begin @(posedge strobe);@(negedge clk);end
endtask
task check(input bit ok,input string why);
 begin checks++;if(!ok)$fatal(1,"%s mm=%0d valid=%0d timeout=%0d",why,mm,valid,timeout);end
endtask
initial begin
 repeat(4)@(negedge clk);rst=1;
 #1030000000;check(pos==1,"original automatic scan reaches center");compare_old=0;
 // Enter in flight; old scan result must not make the forward stream valid.
 @(posedge trig);#1000;hold_forward=1;enabled_at=$time;
 repeat(8)@(negedge clk);check(!valid && !ready,"enable invalidates scan sample");
 wait(ready);ready_at=$time;check(ready_at-enabled_at>=300000000,"mechanical settle budget shortened");
 check(pos==1,"forward mode must hold center");next_sample;
 check(valid && mm>=498 && mm<=501,"forward calibrated 500mm echo");
 @(posedge pwm);pwm_start=$time;@(negedge pwm);check($time-pwm_start==1500000,"center PWM width");
 next_sample;check(valid && !timeout,"second good sample");
 response=1;echo=0;next_sample;check(!valid && timeout,"no Echo must time out");
 response=2;next_sample;check(!valid && !timeout,"too-short Echo must be invalid");
 response=3;next_sample;check(!valid && timeout,"stuck high Echo must time out");
 response=1;echo=1;next_sample;check(!valid && timeout,"pre-trigger high must be rejected");
 echo=0;response=0;expected_mm=250;next_sample;check(valid && mm>=248 && mm<=251,"near range conversion");
 // Leaving protection immediately removes forward qualification and aborts in-flight data.
 @(posedge trig);#1000;hold_forward=0;repeat(8)@(negedge clk);
 check(!ready && !valid,"disable invalidates in-flight forward transaction");
 next_sample;check(valid,"display ranging resumes");
 rst=0;repeat(3)@(negedge clk);check(!ready && !strobe && !valid && pos==0,"reset");
 $display("TEST PASSED RANGE checks=%0d samples=%0d settle_us=%0d",checks,samples,(ready_at-enabled_at)/1000);$finish;
end
initial begin #(2500ms);$fatal(1,"range watchdog");end
endmodule
