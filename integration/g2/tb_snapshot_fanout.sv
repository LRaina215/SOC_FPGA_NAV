`timescale 1ns/1ps
// Both controllers see changing inputs while loading, faults and resets.
// Compare all inputs to the unchanged core for these exercised traces.
// This test double is not full RL inference or physical routing acceptance.
module fc_v4_top #(
 parameter WEIGHT_BITS=16,WEIGHTS_FILE="",BIASES_FILE="",ELU_FILE=""
)(input clk,rst_n,wr_en,wr_history,input[7:0]wr_addr,input signed[23:0]wr_data,
 input start,input[31:0]frame_id,output reg busy,done,output[4:0]error_flags,
 output saturated,output[31:0]result_frame_id,output reg[31:0]cycles,
 output signed[23:0]action0,action1,action2,action3,action4,action5,
 output dbg_valid,output[2:0]dbg_layer,output[8:0]dbg_index,output[23:0]dbg_data);
 reg[31:0]sum,tag;reg[10:0]remaining;
 assign error_flags=tb_snapshot_fanout.mode==1?5'b1:0;
 assign saturated=tb_snapshot_fanout.mode==2;
 assign result_frame_id=tag+(tb_snapshot_fanout.mode==3);
 assign {action5,action4,action3,action2,action1,action0}={sum[23:0],sum[23:0]^24'hffaaff,sum[23:0]+24'd1,sum[23:0]-24'd1,~sum[23:0],sum[23:0]};
 assign dbg_valid=0;assign dbg_layer=0;assign dbg_index=0;assign dbg_data=0;
 always@(posedge clk or negedge rst_n)begin
  if(!rst_n)begin sum<=0;tag<=0;remaining<=0;busy<=0;done<=0;cycles<=0;end
  else begin
   done<=0;
   if(wr_en)sum<={sum[30:0],sum[31]}^{{8{wr_data[23]}},wr_data}^{23'd0,wr_history,wr_addr};
   if(start)begin tag<=frame_id;busy<=1;remaining<=tb_snapshot_fanout.mode==4?700:30;cycles<=0;end
   else if(busy)begin
    cycles<=cycles+1;
    if(remaining==0)begin busy<=0;done<=1;end else remaining<=remaining-1'b1;
   end
  end
 end
endmodule

module tb_snapshot_fanout;
 reg clk=0;always #10 clk=~clk;
 reg rst_n=0,enable=1,sensor_ok=1,frame_valid=0;
 reg[671:0]current_flat=0;reg[3359:0]history_flat=0;
 integer mode=0,cycle=0,publications=0,writes=0,resets=0;
 reg[31:0]random=32'h78130bad;
 wire capture_a,capture_b,action_a,action_b;
 wire[143:0]actions_a,actions_b;
 wire[31:0]complete_a,complete_b,ic_a,ic_b,fc_a,fc_b,rc_a,rc_b,ei_a,ei_b,ri_a,ri_b;
 wire[2:0]fault_a,fault_b,last_a,last_b;wire[6:0]detail_a,detail_b;
 snapshot_original #(.PERIOD_CLKS(512),.RETRY_ON_FAULT(1)) a(
  clk,rst_n,enable,sensor_ok,frame_valid,current_flat,history_flat,capture_a,action_a,actions_a,
  complete_a,ic_a,fault_a,last_a,detail_a,fc_a,rc_a,ei_a,ri_a);
 rl_policy_tick_v5 #(.PERIOD_CLKS(512),.RETRY_ON_FAULT(1)) b(
  clk,rst_n,enable,sensor_ok,frame_valid,current_flat,history_flat,capture_b,action_b,actions_b,
  complete_b,ic_b,fault_b,last_b,detail_b,fc_b,rc_b,ei_b,ri_b);
 integer k;
 always@(negedge clk)begin
  cycle=cycle+1;
  for(k=0;k<126;k=k+1)begin
   random={random[30:0],random[31]^random[21]^random[1]^random[0]};
   if(k<21)current_flat[k*32+:32]=random;else history_flat[(k-21)*32+:32]=random;
  end
  frame_valid=(random[2:0]!=0);
  if(cycle==4)rst_n=1;
  if(cycle>=5000 && cycle<10000)mode=1;
  else if(cycle>=10000 && cycle<15000)mode=2;
  else if(cycle>=15000 && cycle<20000)mode=3;
  else if(cycle>=20000 && cycle<25000)mode=4;
  else mode=0;
  if(cycle>26000 && cycle<30000)begin
   sensor_ok=cycle%139!=0;enable=cycle%163!=0;
   if(cycle%877==0)begin rst_n=0;resets=resets+1;end
   else rst_n=1;
  end else begin sensor_ok=1;enable=1;end
 end
 always@(posedge clk)begin
  if(rst_n)begin
   if({a.core_rst_n,a.core.wr_en,a.core.wr_history,a.core.wr_addr,a.core.wr_data,a.core.start,a.core.frame_id} !==
      {b.core_rst_n,b.core.wr_en,b.core.wr_history,b.core.wr_addr,b.core.wr_data,b.core.start,b.core.frame_id})
      $fatal(1,"core input mismatch cycle %0d",cycle);
   if(a.core.wr_en)writes=writes+1;
  end
  #1;
  if(cycle>4)begin
   if({capture_a,action_a,actions_a,complete_a,ic_a,fault_a,last_a,detail_a,fc_a,rc_a,ei_a,ri_a} !==
      {capture_b,action_b,actions_b,complete_b,ic_b,fault_b,last_b,detail_b,fc_b,rc_b,ei_b,ri_b} || a.snapshot !== b.snapshot)
       $fatal(1,"state/output/snapshot mismatch cycle %0d",cycle);
   if(action_a)publications=publications+1;
  end
  if(cycle==42000)begin
   if(publications<20 || writes<4000 || resets<4)$fatal(1,"insufficient coverage");
   $display("SNAPSHOT_FANOUT_PASS cycles=%0d publications=%0d writes=%0d resets=%0d",cycle,publications,writes,resets);$finish;
  end
 end
endmodule
