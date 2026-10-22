`timescale 1ns/1ps
module observation_pair #(parameter LS=-1,RS=-1)(input clk,rst_n,capture,
 input[71:0] gyro,gravity,input signed[15:0] ld,rd,
 input signed[23:0] linear,yaw,height,input[143:0] actions,output wire valid);
 wire v0,v1,i0,i1;wire[671:0] c0,c1;wire[3359:0] h0,h1;
 obs_original #(.LEFT_ENCODER_SIGN(LS),.RIGHT_ENCODER_SIGN(RS)) oldobs(
  clk,rst_n,capture,gyro,gravity,ld,rd,linear,yaw,height,actions,v0,i0,c0,h0);
 rl_observation_v5 #(.LEFT_ENCODER_SIGN(LS),.RIGHT_ENCODER_SIGN(RS)) newobs(
  clk,rst_n,capture,gyro,gravity,ld,rd,linear,yaw,height,actions,v1,i1,c1,h1);
 assign valid=v0;
 always @(negedge clk)if({v0,i0,c0,h0}!=={v1,i1,c1,h1})$fatal(1,"observation cycle/data mismatch signs %0d,%0d",LS,RS);
endmodule
module tb_observation_equivalence;
 reg clk=0;always #10 clk=~clk;
 reg rst_n=0,capture=0;
 reg[71:0] gyro=0,gravity=0;reg signed[15:0] ld=0,rd=0;
 reg signed[23:0] linear=0,yaw=0,height=535;reg[143:0] actions=0;
 wire valid;
 observation_pair p0(clk,rst_n,capture,gyro,gravity,ld,rd,linear,yaw,height,actions,valid);
 observation_pair #(.LS(1),.RS(-1)) p1(clk,rst_n,capture,gyro,gravity,ld,rd,linear,yaw,height,actions,);
 observation_pair #(.LS(-1),.RS(1)) p2(clk,rst_n,capture,gyro,gravity,ld,rd,linear,yaw,height,actions,);
 observation_pair #(.LS(1),.RS(1)) p3(clk,rst_n,capture,gyro,gravity,ld,rd,linear,yaw,height,actions,);
 reg[31:0] rng=32'd4500;integer cycle=0,frames=0;
 function[31:0] next(input[31:0] x);next=x^(x<<13)^(x>>17)^(x<<5);endfunction
 always @(negedge clk)begin
  cycle=cycle+1;
  rng=next(rng);ld=rng[15:0];rd=rng[31:16];
  if(cycle%1700==1)case((cycle/1700)%4)
   0:begin ld=-32768;rd=32767;end
   1:begin ld=32767;rd=-32768;end
   2:begin ld=0;rd=-1;end
   3:begin end // retain random captured deltas
  endcase
  if(cycle%1700==2)begin ld=32767;rd=-32768;end
  for(integer j=0;j<3;j++)begin rng=next(rng);gyro[j*24+:24]=rng[23:0];rng=next(rng);gravity[j*24+:24]=rng[23:0];end
  for(integer j=0;j<6;j++)begin rng=next(rng);actions[j*24+:24]=rng[23:0];end
  rng=next(rng);linear=rng[23:0];rng=next(rng);yaw=rng[23:0];rng=next(rng);height=rng[23:0];
  capture=cycle%1700==1 || cycle%1700==2 || cycle%1700==27; // busy captures ignored on both sides
  rst_n=cycle>=5 && !(cycle>=10300 && cycle<10305); // reset in a busy observation
  if(valid)frames++;
  if(cycle==56000)begin
   if(frames<25)$fatal(1,"insufficient complete frames");
   $display("TEST PASSED %0d complete frames,4 sign combinations,cycle-exact outputs,reset and busy capture",frames);$finish;
  end
 end
endmodule
