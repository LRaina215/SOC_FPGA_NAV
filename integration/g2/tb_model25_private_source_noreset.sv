`timescale 1ns/1ps
module event_pair #(parameter W=128)(input sc,dc,srn,drn,snn,dnn,monitor);
 reg valid=0;reg[W-1:0]data=0;wire rr,rn,orr,orn,vr,vn;wire[W-1:0]qr,qn;
 integer checks=0,commits=0,overflow_samples=0;
 g2_model25_mailbox_ref #(.WIDTH(W)) ref_dut(sc,srn,valid,data,rr,orr,dc,drn,vr,qr);
 g2_model25_mailbox_candidate #(.WIDTH(W)) new_dut(sc,snn,valid,data,rn,orn,dc,dnn,vn,qn);
 initial begin #1;if(^new_dut.holding!==1'bx)$fatal(1,"candidate private holding must start naturalX W%0d",W);end
 always @(posedge sc or posedge dc or negedge srn or negedge drn or negedge snn or negedge dnn)begin
  #1;if(monitor)begin
   if({rr,orr,vr,qr}!=={rn,orn,vn,qn})$fatal(1,"event public miter W%0d t%0t",W,$time);
   if(^{rr,orr,vr,qr,rn,orn,vn,qn}===1'bx)$fatal(1,"unknown public event W%0d",W);
   checks=checks+1;if(orr)overflow_samples=overflow_samples+1;
  end
 end
 always @(posedge dc)begin #1;if(vr)commits=commits+1;end
 function[W-1:0]word(input integer k);integer j;begin for(j=0;j<W;j=j+1)word[j]=((k>> (j%31))^(j*13)^(k*7+j/3))&1;end endfunction
 task launch_wait(input integer phase,input integer k);
  begin
   @(negedge sc);while(!rr)@(negedge sc);data=word(k);valid=1;
   @(posedge sc);#1;valid=0;
   case(phase)
    0:begin if(ref_dut.request===ref_dut.ack2)$fatal(1,"request phase not busy");end
    1:begin wait(ref_dut.req1!==ref_dut.acknowledge);#1;end
    2:begin wait(ref_dut.req2!==ref_dut.acknowledge);#1;end
    3:begin wait(vr===1'b1);#1;end
    4:begin wait(ref_dut.ack1===ref_dut.request && ref_dut.ack2!==ref_dut.request);#1;end
   endcase
  end
 endtask
 task create_overflow;
  begin @(negedge sc);data=word(999);valid=1;repeat(3)@(posedge sc);#1;valid=0;end
 endtask
endmodule
module latest_pair #(parameter W=140)(input sc,dc,srn,drn,snn,dnn,monitor);
 reg[W-1:0]data=0;wire vr,vn;wire[W-1:0]qr,qn;integer checks=0,commits=0,updates=0;
 g2_model25_latest_ref #(.WIDTH(W))ref_dut(sc,srn,data,dc,drn,vr,qr);
 g2_model25_latest_candidate #(.WIDTH(W))new_dut(sc,snn,data,dc,dnn,vn,qn);
 initial begin #1;if(^new_dut.last_sent!==1'bx||^new_dut.transfer.holding!==1'bx)$fatal(1,"candidate latest private words must start naturalX W%0d",W);end
 always @(posedge sc or posedge dc or negedge srn or negedge drn or negedge snn or negedge dnn)begin
  #1;if(monitor)begin
   if({ref_dut.ready,ref_dut.unused_overflow,vr,qr}!=={new_dut.ready,new_dut.unused_overflow,vn,qn})$fatal(1,"latest public miter W%0d t%0t",W,$time);
   if(^{ref_dut.ready,ref_dut.unused_overflow,vr,qr,new_dut.ready,new_dut.unused_overflow,vn,qn}===1'bx)$fatal(1,"unknown public latest W%0d",W);
   checks=checks+1;
  end
 end
 always @(posedge dc)begin #1;if(vr)commits=commits+1;end
 function[W-1:0]word(input integer k);integer j;begin for(j=0;j<W;j=j+1)word[j]=((k>> (j%31))^(j*7)^(k*3+j/5))&1;end endfunction
 task busy_updates(input integer n);
  begin for(integer k=1;k<=n;k=k+1)begin @(negedge sc);data=word(k);updates=updates+1;end end
 endtask
 task final_word_check;
  begin if(qr!==data||qn!==data)$fatal(1,"latest final word not delivered W%0d",W);end
 endtask
endmodule
module tb_private_noreset;
 reg clk50=0,clk25=0,run25=1,reset_n=0,lock=0,monitor=0;
 always #10 clk50=~clk50;
 always #20 if(run25)clk25=~clk25;
 wire ior,modr,fr,ion,modn,fn;
 wire overflow_r=bmi.orr|action.orr|fusion.orr;
 wire overflow_n=bmi.orn|action.orn|fusion.orn;
 g2_model25_guard_ref gr(clk50,clk25,reset_n,lock,overflow_r,ior,modr,fr);
 g2_model25_guard_candidate gn(clk50,clk25,reset_n,lock,overflow_n,ion,modn,fn);
 event_pair #(128)bmi(clk50,clk25,ior,modr,ion,modn,monitor);
 event_pair #(336)action(clk25,clk50,modr,ior,modn,ion,monitor);
 event_pair #(145)fusion(clk25,clk50,modr,ior,modn,ion,monitor);
 latest_pair #(140)configuration(clk50,clk25,ior,modr,ion,modn,monitor);
 latest_pair #(15)status(clk25,clk50,modr,ior,modn,ion,monitor);
 integer resets=0,phase_resets=0,clock_stops=0,guard_checks=0,rearms=0;
 always @(posedge clk50 or posedge clk25 or negedge reset_n or negedge lock)begin
  #2;if(monitor)begin if({ior,modr,fr}!=={ion,modn,fn})$fatal(1,"guard miter");guard_checks=guard_checks+1;end
 end
 task rearm;
  begin
   bmi.valid=0;action.valid=0;fusion.valid=0;reset_n=0;lock=0;run25=1;
   #83;if(ior!==0||modr!==0||ion!==0||modn!==0)$fatal(1,"common asynchronous reset did not reach publics");
   reset_n=1;lock=1;repeat(12)@(negedge clk50);#2;
   if(fr||fn||!ior||!modr||!ion||!modn)$fatal(1,"explicit rearm failed");
   resets=resets+1;rearms=rearms+1;
  end
 endtask
 task no_old_event;
  begin repeat(16)begin @(negedge clk50);#2;if(bmi.vr||action.vr||fusion.vr||bmi.vn||action.vn||fusion.vn)$fatal(1,"old event replay after reset");end end
 endtask
 task halted_fault_and_resume;
  begin
   run25=0;repeat(24)@(negedge clk50);#2;
   if(!fr||!fn||ior||modr||ion||modn)$fatal(1,"25 clock halt not latched/stopped");
   run25=1;repeat(12)@(negedge clk50);#2;
   if(!fr||!fn||ior||modr)$fatal(1,"clock recovery auto-rearmed");
   clock_stops=clock_stops+1;rearm();no_old_event();
  end
 endtask
 initial begin
  #3;monitor=1;rearm();
  for(integer k=0;k<5;k=k+1)begin
   bmi.launch_wait(k,100+k);#3;rearm();phase_resets=phase_resets+1;no_old_event();
   action.launch_wait(k,200+k);#3;rearm();phase_resets=phase_resets+1;no_old_event();
  end
  // All five original logical widths are compared, including complete145bit fusion.
  fusion.launch_wait(4,321);repeat(20)@(negedge clk50);
  if(fusion.commits==0)$fatal(1,"fusion word path not exercised");
  fork configuration.busy_updates(200);status.busy_updates(200);join
  repeat(40)@(negedge clk25);#2;configuration.final_word_check();status.final_word_check();
  for(integer k=0;k<5;k=k+1)begin
   bmi.launch_wait(k,400+k);#3;halted_fault_and_resume();
   action.launch_wait(k,500+k);#3;halted_fault_and_resume();
  end
  // Common reset while25 is halted must not release its public state without edges.
  action.launch_wait(0,700);#3;run25=0;reset_n=0;lock=0;#83;
  reset_n=1;lock=1;repeat(4)@(negedge clk50);#2;
  if(modr!==0||modn!==0)$fatal(1,"stopped domain released without25 edges");
  run25=1;repeat(20)@(negedge clk50);#2;
  if(fr||fn||!ior||!modr)$fatal(1,"stopped common reset recovery failed");
  no_old_event();resets=resets+1;rearms=rearms+1;
  // Explicit overflow plus latched guard fault, followed only by physical reset rearm.
  bmi.create_overflow();repeat(10)@(negedge clk50);#2;
  if(!fr||!fn||bmi.overflow_samples==0)$fatal(1,"overflow/guard coverage missing");
  rearm();no_old_event();
  // Lock loss shorter than one50MHz period remains asynchronous and sticky.
  @(negedge clk50);#3;lock=0;#2;lock=1;#1;
  if(!fr||!fn)$fatal(1,"short lock loss not latched");
  repeat(12)@(negedge clk50);if(!fr||!fn)$fatal(1,"PLL relock auto-rearmed");
  rearm();no_old_event();
  if(phase_resets!=10||clock_stops!=10)$fatal(1,"coverage counters");
  $display("PRIVATE_NORESET_PASS event_checks=%0d latest_checks=%0d guard_checks=%0d phase_resets=%0d stops25=%0d explicit_rearms=%0d event_commits=%0d latest_commits=%0d naturalX_private_words=7",bmi.checks+action.checks+fusion.checks,configuration.checks+status.checks,guard_checks,phase_resets,clock_stops,rearms,bmi.commits+action.commits+fusion.commits,configuration.commits+status.commits);
  $finish;
 end
 initial begin #1000000;$fatal(1,"finite unit fixture watchdog");end
endmodule
