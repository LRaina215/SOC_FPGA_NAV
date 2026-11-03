`timescale 1ns/1ps
module tb_range;
reg clk=0;always #500 clk=~clk;
reg rst=0,echo=0,hold_forward=0;
wire trig,pwm,valid,timeout,strobe,ready;
wire [15:0] mm;wire [1:0] pos;
integer checks=0,samples=0,response=0,expected_mm=500,pwm_frames=0,saved_samples;
time reset_at,ready_at,pwm_start,last_trig=0,trig_start;
g2_range_servo #(.CLOCKS_PER_US(1)) dut(.clk(clk),.reset_n(rst),.echo(echo),.hold_forward(hold_forward),
 .trig(trig),.servo_pwm(pwm),.distance_mm(mm),.distance_valid(valid),.distance_timeout(timeout),
 .servo_position(pos),.sample_strobe(strobe),.forward_ready(ready));
always @(negedge clk)if(rst)begin
 if(pos!==2'd1)$fatal(1,"SG90 must never scan, regardless of protection mode");
 if(strobe)samples++;
end
always @(posedge pwm)begin
 pwm_start=$time;@(negedge pwm);
 if(rst)begin
  if($time-pwm_start!=1500000)$fatal(1,"only1500us forward PWM allowed");
  pwm_frames++;checks++;
 end
end
always @(posedge trig)if(rst)begin
 trig_start=$time;
 if(last_trig!=0 && $time-last_trig<65000000)$fatal(1,"mode change shortened65ms trigger interval");
 last_trig=$time;checks++;
end
// Echo wire stimulus only: no force or internal state writes.
always @(negedge trig) if(rst)begin
 if($time-trig_start!=12000)$fatal(1,"mode change shortened12us trigger pulse");
 if(response==0)begin #200000;echo=1;#(expected_mm*5800);echo=0;end
 else if(response==2)begin #200000;echo=1;#2000;echo=0;end
 else if(response==3)begin #200000;echo=1;#33000000;echo=0;end
end
task next_sample;
 begin @(posedge strobe);@(negedge clk);#1;end
endtask
task check(input bit ok,input string why);
 begin checks++;if(!ok)$fatal(1,"%s mm=%0d valid=%0d timeout=%0d",why,mm,valid,timeout);end
endtask
initial begin
 repeat(4)@(negedge clk);rst=1;reset_at=$time;
 #(299ms);check(!ready && !trig && !valid,"power-on settling blocks measurements even with guard off");
 wait(ready);ready_at=$time;check(ready_at-reset_at>=300000000,"power-on settle shortened");
 next_sample;check(valid && mm>=498 && mm<=501,"forward500mm measurement while protection off");
 saved_samples=samples;
 @(posedge trig);#1000;hold_forward=1;
 repeat(8)@(negedge clk);check(!valid && ready && trig,"mode toggle keeps center ready and original pulse");
 #(10ms);check(samples==saved_samples,"in-flight old measurement was not discarded");
 next_sample;check(valid && !timeout,"new forward sample after enable");
 response=1;echo=0;next_sample;check(!valid && timeout,"no Echo timeout");
 response=2;next_sample;check(!valid && !timeout,"too-short Echo invalid");
 response=3;next_sample;check(!valid && timeout,"stuck high Echo timeout");
 response=1;echo=1;next_sample;check(!valid && timeout,"pre-trigger high rejected");
 echo=0;response=0;expected_mm=250;next_sample;check(valid && mm>=248 && mm<=251,"near range conversion");
 @(posedge trig);#1000;hold_forward=0;repeat(8)@(negedge clk);
 check(ready && !valid && pos==1,"disable must not move servo or lose center readiness");
 next_sample;check(valid,"forward display ranging resumes after invalidating old transaction");
 // Stay disabled long enough to cross the old1s scan interval: still forward.
 #(1050ms);check(pos==1 && ready && pwm_frames>50,"no scanning after old hold interval");
 rst=0;repeat(3)@(negedge clk);check(!ready && !strobe && !valid && pos==1 && !pwm,"reset remains centered");
 $display("TEST PASSED RANGE checks=%0d samples=%0d settle_us=%0d pwm_frames=%0d",checks,samples,(ready_at-reset_at)/1000,pwm_frames);$finish;
end
initial begin #(2500ms);$fatal(1,"range watchdog");end
endmodule
