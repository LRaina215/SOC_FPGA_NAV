`timescale 1ns/1ps
// New deadline-focused regression; does not replay the old post-ramp suite.
// Logical millisecond divider is4 for finite contract testing. There is no
// force or hierarchical state write: actual guard states follow public inputs.
module tb_lease_deadline_compare;
 reg clk=0;always #10 clk=~clk;
 reg reset_n=0,valid=0,source_fault=0,estop=0,feedback_valid=1;
 reg[1:0]owner=1,source_id=1;reg[31:0]session=1,packet_session=1;
 reg[15:0]seq=0;reg signed[23:0]v=0,w=0;reg signed[15:0]mv=0,mw=0;
 wire signed[23:0]gv,gw,cv,cw;wire ga,ge,gs,gb,ca,ce,cs,cb;
 wire[31:0]gage,gac,grc,cage,cac,crc;
 original_guard #(.CLK_HZ(4000),.STOP_DWELL_MS(3)) gold(
  clk,reset_n,owner,source_id,session,packet_session,seq,valid,source_fault,estop,v,w,
  feedback_valid,mv,mw,gv,gw,ga,ge,gs,gage,gac,grc,gb);
 command_lease_guard #(.CLK_HZ(4000),.STOP_DWELL_MS(3)) candidate(
  clk,reset_n,owner,source_id,session,packet_session,seq,valid,source_fault,estop,v,w,
  feedback_valid,mv,mw,cv,cw,ca,ce,cs,cage,cac,crc,cb);
 integer cycles=0,checks=0,math_checks=0;
 reg math_done=0;
 always @(negedge clk)begin
  #1;cycles=cycles+1;
  if(reset_n)begin
   if($isunknown({gv,gw,ga,ge,gs,gage,gac,grc,gb,cv,cw,ca,ce,cs,cage,cac,crc,cb}))$fatal(1,"UNKNOWN public guard result");
   if({gv,gw,ga,ge,gs,gage,gac,grc,gb}!=={cv,cw,ca,ce,cs,cage,cac,crc,cb})$fatal(1,"public output cycle mismatch %0d",cycles);
   if({gold.divider,gold.owner,gold.session,gold.last_seq,gold.have_seq,gold.have_lease,gold.target_v,gold.target_w,gold.feedback_age,gold.still_ms,gold.slow_feedback}!==
      {candidate.divider,candidate.owner,candidate.session,candidate.last_seq,candidate.have_seq,candidate.have_lease,candidate.target_v,candidate.target_w,candidate.feedback_age,candidate.still_ms,candidate.slow_feedback})$fatal(1,"registered internal state cycle mismatch");
   if(gold.deadline!==candidate.deadline||gold.stop_mode!==candidate.stop_mode)$fatal(1,"deadline/stop selection mismatch");
  end
 end
 task clocks(input integer n);begin repeat(n)@(negedge clk);#2;end endtask
 task check(input bit ok,input string label);begin if(!ok)$fatal(1,"%s",label);checks=checks+1;$display("PASS %s",label);end endtask
 task packet(input[15:0]number,input signed[23:0]fv,fw);
  begin @(negedge clk);seq=number;v=fv;w=fw;valid=1;clocks(1);valid=0;clocks(1);end
 endtask
 task establish_motion;
  begin
   wait(gs);packet(seq+1,0,0);check(ga&&!ge,"fresh stopped zero arms");
   packet(seq+1,410,-820);clocks(48);check(gv==410&&gw==-820,"baseline reaches motion target");
  end
 endtask
 reg[31:0]old_ac,old_rc;reg[15:0]old_seq;reg signed[23:0]old_v;
 initial begin
  clocks(4);reset_n=1;clocks(20);
  establish_motion();
  wait(gage==198&&gold.tick);@(negedge clk);#2;
  if(gage!=198||!gold.tick)$fatal(1,"fixture missed pre-deadline phase");
  old_ac=gac;seq=seq+1;v=820;w=410;valid=1;clocks(1);valid=0;
  check(ga&&!ge&&gage==0&&gac==old_ac+1,"age198 tick accepts renewal and age reset overrides increment");
  wait(gage==199&&!gold.tick);@(negedge clk);#2;
  if(gage!=199||gold.tick)$fatal(1,"fixture missed non-tick boundary");
  old_ac=gac;seq=seq+1;v=410;w=-820;valid=1;clocks(1);valid=0;
  check(ga&&!ge&&gage==0&&gac==old_ac+1,"age199 non-tick accepts renewal");
  wait(gage==199&&gold.tick);@(negedge clk);#2;
  if(gage!=199||!gold.tick)$fatal(1,"fixture missed deadline phase");
  old_ac=gac;old_rc=grc;old_seq=gold.last_seq;old_v=gv;
  seq=seq+1;v=820;w=410;valid=1;clocks(1);valid=0;
  check(!ga&&ge&&!gold.have_lease&&gage==200&&gac==old_ac&&grc==old_rc+1&&gold.last_seq==old_seq&&gold.target_v==0&&gold.target_w==0,
   "deadline beats same-edge valid command; rejects without sequence renewal");
  check(gv==old_v-41,"deadline output begins original same-tick controlled ramp");
  packet(seq+1,410,410);check(!ga,"nonzero command cannot automatically recover");
  establish_motion();
  source_fault=1;valid=1;old_rc=grc;clocks(1);valid=0;source_fault=0;
  check(!ga&&ge&&grc==old_rc+1,"source fault beats valid command");
  establish_motion();
  estop=1;valid=1;clocks(1);valid=0;estop=0;
  check(!ga&&ge&&gv==0&&gw==0,"estop retains immediate zero priority");
  establish_motion();
  wait(gold.tick);@(negedge clk);#2;
  old_v=gv;owner=2;source_id=2;session=2;packet_session=2;estop=1;valid=1;old_rc=grc;clocks(1);valid=0;estop=0;
  check(!ga&&ge&&gold.owner==2&&gold.session==2&&!gold.have_seq&&grc==old_rc+1&&gv==old_v-41,
   "owner/session change retains original priority over coincident estop and controlled output update");
  establish_motion();
  feedback_valid=0;wait(gold.feedback_age==50);@(negedge clk);#2;
  valid=1;feedback_valid=1;old_rc=grc;clocks(1);valid=0;
  check(!ga&&ge&&gold.feedback_age==0&&grc==old_rc+1,"stale prestate feedback revokes before same-edge fresh feedback/command");
  establish_motion();
  reset_n=0;#1;
  check(gv==0&&gw==0&&cv==0&&cw==0&&!ga&&!ca,"common asynchronous reset clears both exact output/state chains");
  clocks(3);reset_n=1;clocks(20);packet(seq+1,410,410);check(!ga&&!ca,"reset requires fresh zero handshake");
  wait(math_done);
  $display("LEASE_DEADLINE_COMPARE_PASS contracts=%0d compared_cycles=%0d math_parameter_XZ_cases=%0d",checks,cycles,math_checks);$finish;
 end

 reg[31:0]age_test=0;wire[4:0]old_math,new_math;
 deadline_math_pair #(.TIMEOUT_MS(200)) m0(age_test,old_math[0],new_math[0]);
 deadline_math_pair #(.TIMEOUT_MS(1)) m1(age_test,old_math[1],new_math[1]);
 deadline_math_pair #(.TIMEOUT_MS(0)) m2(age_test,old_math[2],new_math[2]);
 deadline_math_pair #(.TIMEOUT_MS(-1)) m3(age_test,old_math[3],new_math[3]);
 deadline_math_pair #(.TIMEOUT_MS(32'sh80000000)) m4(age_test,old_math[4],new_math[4]);
 task math_case(input[31:0]value);begin age_test=value;#1;if(old_math!==new_math)$fatal(1,"math/4state mismatch age=%h old=%b candidate=%b",age_test,old_math,new_math);math_checks=math_checks+5;end endtask
 initial begin
  math_case(0);math_case(1);math_case(198);math_case(199);math_case(200);math_case(32'h7fffffff);
  math_case(32'h80000000);math_case(32'hfffffffe);math_case(32'hffffffff);
  math_case(32'hxxxxxxxx);math_case(32'hzzzzzzzz);math_case(32'hfffffffx);math_case(32'h0000000x);
  for(integer i=0;i<32;i=i+1)begin age_test=32'hffffffff;age_test[i]=1'bx;#1;if(old_math!==new_math)$fatal(1,"partialX math differs bit%0d",i);math_checks=math_checks+5;end
  math_done=1;
 end
 initial begin clocks(100000);$fatal(1,"finite contract stimulus watchdog");end
endmodule

module deadline_math_pair #(parameter integer TIMEOUT_MS=200)(input[31:0]age_ms,output original,output candidate);
 assign original=age_ms+1>=TIMEOUT_MS;
 wire tick=1'b1,have_lease=1'b1;
 // The runner replaces this marker with the exact candidate source expression.
 `include "deadline_math_candidate.vh"
 assign candidate=deadline;
endmodule
