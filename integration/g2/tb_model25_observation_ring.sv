`timescale 1ns/1ps
// Explicit inference substitute: proves only observation/load/scheduler protocol.
module fc_v4_top #(parameter WEIGHT_BITS=16,WEIGHTS_FILE="",BIASES_FILE="",ELU_FILE="")(
 input clk,rst_n,wr_en,wr_history,input[7:0]wr_addr,input signed[23:0]wr_data,
 input start,input[31:0]frame_id,output reg busy,done,output reg[4:0]error_flags,
 output reg saturated,output reg[31:0]result_frame_id,cycles,
 output reg signed[23:0]action0,action1,action2,action3,action4,action5,
 output dbg_valid,output[2:0]dbg_layer,output[7:0]dbg_index,output[23:0]dbg_data);
 reg signed[23:0]input_mem[0:167];reg[167:0]input_valid;integer delay_count;
 integer test_mode=0; // 1 deadline/no done,2 saturation,3 wrong ID,4 errors.
 wire[7:0]address=wr_history?wr_addr+8'd28:wr_addr;
 assign dbg_valid=0;assign dbg_layer=0;assign dbg_index=0;assign dbg_data=0;
 always @(posedge clk)if(wr_en&&!busy&&rst_n)input_mem[address]<=wr_data;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin busy<=0;done<=0;error_flags<=0;saturated<=0;result_frame_id<=0;cycles<=0;input_valid<=0;delay_count<=0;action0<=0;action1<=0;action2<=0;action3<=0;action4<=0;action5<=0;end
  else begin
   done<=0;
   if(wr_en)begin if(busy)$fatal(1,"stub busy write");input_valid[address]<=1;end
   if(start)begin
    if(wr_en||busy||!(&input_valid))$fatal(1,"stub incomplete168 input or same-edge start");
    busy<=1;input_valid<=0;cycles<=0;delay_count<=0;result_frame_id<=frame_id;
   end else if(busy)begin
    delay_count<=delay_count+1;cycles<=cycles+1;
    if(delay_count==8&&test_mode!=1)begin
     busy<=0;done<=1;saturated<=test_mode==2;error_flags<=test_mode==4?5'b00100:0;
     if(test_mode==3)result_frame_id<=result_frame_id+1;
     action0<=1;action1<=-2;action2<=3;action3<=-4;action4<=5;action5<=-6;
    end
   end
  end
 end
endmodule

module ring_pair #(parameter ORIGINAL=0,RING=0)(
 input clk,rst_n,enable,sensor_ok,stop_latched,external_fault,
 input[71:0]angular,gravity,input signed[15:0]left_delta,right_delta,
 input signed[23:0]linear,yaw,height,input[143:0]last_actions,
 output capture,valid,initialized,action_valid,output[143:0]actions,
 output[31:0]completed,cycles,output[2:0]fault,
 output[671:0]current_flat,output[3359:0]history_flat,
 output wr_en,wr_history,start,output[7:0]wr_addr,output[23:0]wr_data,output[2:0]state);
 wire freeze;wire[7:0]index;wire signed[23:0]data;
 wire obs_rst_n=rst_n&&enable&&!stop_latched;
 if(ORIGINAL)begin:g_ref
  rl_observation_v5_ref u_obs(.clk(clk),.rst_n(obs_rst_n),.capture(capture),.angular_velocity_q12(angular),.gravity_q12(gravity),.left_delta(left_delta),.right_delta(right_delta),.linear_command_q12(linear),.yaw_command_q12(yaw),.height_command_q12(height),.last_executed_actions(last_actions),.valid(valid),.initialized(initialized),.current_flat(current_flat),.history_flat(history_flat));
  rl_policy_tick_v5_ref #(.PERIOD_CLKS(2500))u_policy(.clk(clk),.rst_n(rst_n),.enable(enable),.sensor_ok(sensor_ok&&!external_fault&&!stop_latched),.frame_valid(valid),.current_flat(current_flat),.history_flat(history_flat),.capture(capture),.action_valid(action_valid),.actions(actions),.completed_frames(completed),.inference_cycles(cycles),.fault(fault),.diag_last_fault(),.diag_core_detail(),.diag_fault_count(),.diag_retry_count(),.diag_expected_id(),.diag_result_id());
  assign wr_en=u_policy.core.wr_en;assign wr_history=u_policy.core.wr_history;assign wr_addr=u_policy.core.wr_addr;assign wr_data=u_policy.core.wr_data;assign start=u_policy.core.start;assign state=u_policy.state;
 end else begin:g_new
  rl_observation_v5_candidate #(.OBS_RING_STORAGE(RING))u_obs(.clk(clk),.rst_n(obs_rst_n),.capture(capture),.frame_freeze(freeze),.frame_read_index(index),.frame_read_data(data),.angular_velocity_q12(angular),.gravity_q12(gravity),.left_delta(left_delta),.right_delta(right_delta),.linear_command_q12(linear),.yaw_command_q12(yaw),.height_command_q12(height),.last_executed_actions(last_actions),.valid(valid),.initialized(initialized),.current_flat(current_flat),.history_flat(history_flat));
  rl_policy_tick_v5_candidate #(.OBS_RING_STORAGE(RING),.PERIOD_CLKS(2500))u_policy(.clk(clk),.rst_n(rst_n),.enable(enable),.sensor_ok(sensor_ok&&!external_fault&&!stop_latched),.frame_valid(valid),.current_flat(current_flat),.history_flat(history_flat),.frame_freeze(freeze),.frame_read_index(index),.frame_read_data(data),.capture(capture),.action_valid(action_valid),.actions(actions),.completed_frames(completed),.inference_cycles(cycles),.fault(fault),.diag_last_fault(),.diag_core_detail(),.diag_fault_count(),.diag_retry_count(),.diag_expected_id(),.diag_result_id());
  assign wr_en=u_policy.core.wr_en;assign wr_history=u_policy.core.wr_history;assign wr_addr=u_policy.core.wr_addr;assign wr_data=u_policy.core.wr_data;assign start=u_policy.core.start;assign state=u_policy.state;
 end
endmodule

module tb_observation_ring;
 reg clk=0,clock_run=1,rst_n=0,enable=0,sensor_ok=1,external_fault=0,stop_latched=0;
 always #20 if(clock_run)clk=~clk;
 reg[31:0]rng=32'h31415927;
 reg[71:0]angular,gravity;reg signed[15:0]left_delta,right_delta;
 reg signed[23:0]linear,yaw,height;reg[143:0]last_actions;
 wire[2:0]cap,v,init,av,wren,wh,start;wire[2:0][143:0]actions;
 wire[2:0][31:0]completed,cycles;wire[2:0][2:0]fault,state;wire[2:0][7:0]addr;wire[2:0][23:0]data;
 wire[2:0][671:0]current;wire[2:0][3359:0]history;
 ring_pair #(.ORIGINAL(1))p_ref(clk,rst_n,enable,sensor_ok,stop_latched,external_fault,angular,gravity,left_delta,right_delta,linear,yaw,height,last_actions,cap[0],v[0],init[0],av[0],actions[0],completed[0],cycles[0],fault[0],current[0],history[0],wren[0],wh[0],start[0],addr[0],data[0],state[0]);
 ring_pair #(.RING(1))p_ring(clk,rst_n,enable,sensor_ok,stop_latched,external_fault,angular,gravity,left_delta,right_delta,linear,yaw,height,last_actions,cap[1],v[1],init[1],av[1],actions[1],completed[1],cycles[1],fault[1],current[1],history[1],wren[1],wh[1],start[1],addr[1],data[1],state[1]);
 ring_pair #(.RING(0))p_legacy(clk,rst_n,enable,sensor_ok,stop_latched,external_fault,angular,gravity,left_delta,right_delta,linear,yaw,height,last_actions,cap[2],v[2],init[2],av[2],actions[2],completed[2],cycles[2],fault[2],current[2],history[2],wren[2],wh[2],start[2],addr[2],data[2],state[2]);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)stop_latched<=0;
  else if(!enable)stop_latched<=0;
  else if(!sensor_ok||external_fault||fault[0]!=0)stop_latched<=1;
 end
 always @(negedge clk)begin
  rng={rng[30:0],rng[31]^rng[21]^rng[1]^rng[0]};
  angular={rng[23:0],~rng[23:0],24'h7fffff};gravity={24'h800000,rng[23:0],~rng[23:0]};
  left_delta=rng[15:0];right_delta=~rng[15:0];linear=rng[23:0];yaw=~rng[23:0];height=24'd535;
  last_actions={rng[23:0],24'h800000,24'h7fffff,~rng[23:0],24'sd4096,-24'sd4096};
 end
 integer checked=0,writes=0,starts=0,wraps=0,aborts=0,cycle_no=0,capture_edge=0,j,k;
 integer clipped_positive=0,clipped_negative=0,first_repeated_frames=0,complete_slot_commits=0;
 reg[27:0]written_lanes=0;
 always @(posedge clk or negedge p_ring.g_new.u_obs.rst_n)begin
  if(!p_ring.g_new.u_obs.rst_n)written_lanes<=0;
  else begin
   if(p_ring.g_new.u_obs.state==0&&cap[1])written_lanes<=0;
   if(p_ring.g_new.u_obs.g_ring.lane_write)written_lanes[p_ring.g_new.u_obs.lane]<=1;
   if(p_ring.g_new.u_obs.state==4)begin
    if(!(&written_lanes))$fatal(1,"partial28word slot committed");
    complete_slot_commits=complete_slot_commits+1;
   end
  end
 end
 reg prior_valid=0;reg[14:0]frozen_saved;reg was_load=0;reg[2:0]last_work=0;
 function signed[23:0]clip(input signed[23:0]x);begin clip=x>409600?24'sd409600:x< -409600?-24'sd409600:x;end endfunction
 always @(negedge clk)if(rst_n)begin
  cycle_no=cycle_no+1;checked=checked+1;
  if({cap[0],v[0],init[0],av[0],actions[0],completed[0],cycles[0],fault[0],state[0]}!=={cap[1],v[1],init[1],av[1],actions[1],completed[1],cycles[1],fault[1],state[1]})$fatal(1,"ring public/cycle mismatch cycle%0d state%0d/%0d fault%0d/%0d",cycle_no,state[0],state[1],fault[0],fault[1]);
  if({cap[0],v[0],init[0],av[0],actions[0],completed[0],cycles[0],fault[0],state[0],current[0],history[0]}!=={cap[2],v[2],init[2],av[2],actions[2],completed[2],cycles[2],fault[2],state[2],current[2],history[2]})$fatal(1,"default legacy branch changed");
  if({wren[0],wh[0],start[0],addr[0]}!=={wren[1],wh[1],start[1],addr[1]}||{wren[0],wh[0],start[0],addr[0]}!=={wren[2],wh[2],start[2],addr[2]})$fatal(1,"load protocol cycle changed");
  if(wren[0])begin
   if(data[0]!==data[1]||data[0]!==data[2])$fatal(1,"word mismatch addr%0d history%0d ref%h ring%h",addr[0],wh[0],data[0],data[1]);
   if($isunknown(data[1]))$fatal(1,"uninitialized RAM word escaped");writes=writes+1;
  end
  if(cap[0])capture_edge=cycle_no+1; // observer accepts capture on next posedge.
  if(v[0]&&!prior_valid)begin
   if(cycle_no-capture_edge!=679)$fatal(1,"observation valid moved %0d",cycle_no-capture_edge);
  end
  prior_valid=v[0];
  if(start[0])begin
   starts=starts+1;
   for(j=0;j<28;j=j+1)begin
    if(p_ring.g_new.u_policy.core.input_mem[j]!==clip(p_ring.g_new.u_policy.core.input_mem[140+j]))$fatal(1,"current not clipped newest history");
    if($signed(p_ring.g_new.u_policy.core.input_mem[140+j])>409600)clipped_positive=clipped_positive+1;
    if($signed(p_ring.g_new.u_policy.core.input_mem[140+j])< -409600)clipped_negative=clipped_negative+1;
    if(p_ring.g_new.u_obs.g_ring.live0==p_ring.g_new.u_obs.g_ring.live4)
     for(k=0;k<5;k=k+1)if(p_ring.g_new.u_policy.core.input_mem[28+k*28+j]!==p_ring.g_new.u_policy.core.input_mem[140+j])$fatal(1,"first history not five copies");
   end
   if(p_ring.g_new.u_obs.g_ring.live0==p_ring.g_new.u_obs.g_ring.live4)first_repeated_frames=first_repeated_frames+1;
  end
  if(init[1])begin
   if(p_ring.g_new.u_obs.g_ring.work_slot>5)$fatal(1,"invalid work slot");
   if(p_ring.g_new.u_obs.g_ring.work_slot==p_ring.g_new.u_obs.g_ring.live0||p_ring.g_new.u_obs.g_ring.work_slot==p_ring.g_new.u_obs.g_ring.live1||p_ring.g_new.u_obs.g_ring.work_slot==p_ring.g_new.u_obs.g_ring.live2||p_ring.g_new.u_obs.g_ring.work_slot==p_ring.g_new.u_obs.g_ring.live3||p_ring.g_new.u_obs.g_ring.work_slot==p_ring.g_new.u_obs.g_ring.live4)$fatal(1,"work slot overwrites committed history");
   if(last_work==5&&p_ring.g_new.u_obs.g_ring.work_slot==0)wraps=wraps+1;
  end
  last_work=p_ring.g_new.u_obs.g_ring.work_slot;
  if(state[1]==2)begin
   if(p_ring.g_new.u_obs.g_ring.lane_write)$fatal(1,"writer overlaps frozen LOAD");
   if(was_load&&frozen_saved!=={p_ring.g_new.u_obs.g_ring.frozen0,p_ring.g_new.u_obs.g_ring.frozen1,p_ring.g_new.u_obs.g_ring.frozen2,p_ring.g_new.u_obs.g_ring.frozen3,p_ring.g_new.u_obs.g_ring.frozen4})$fatal(1,"frozen map changed during LOAD");
   frozen_saved={p_ring.g_new.u_obs.g_ring.frozen0,p_ring.g_new.u_obs.g_ring.frozen1,p_ring.g_new.u_obs.g_ring.frozen2,p_ring.g_new.u_obs.g_ring.frozen3,p_ring.g_new.u_obs.g_ring.frozen4};
  end
  was_load=state[1]==2;
 end
 task modes(input integer m);begin p_ref.g_ref.u_policy.core.test_mode=m;p_ring.g_new.u_policy.core.test_mode=m;p_legacy.g_new.u_policy.core.test_mode=m;end endtask
 task rearm;begin @(negedge clk);enable=0;repeat(4)@(negedge clk);sensor_ok=1;external_fault=0;modes(0);enable=1;was_load=0;prior_valid=0;end endtask
 task abort_load(input integer index);begin
  wait(wren[0]&&((wh[0]?addr[0]+28:addr[0])==index));@(negedge clk);enable=0;aborts=aborts+1;
  repeat(4)@(negedge clk);enable=1;wait(completed[0]>=1);
 end endtask
 initial begin
  angular=0;gravity=0;left_delta=0;right_delta=0;linear=0;yaw=0;height=535;last_actions=0;
  #2;if(!$isunknown(p_ring.g_new.u_obs.g_ring.frame_mem[0]))$fatal(1,"RAM initial value assumed known");
  repeat(4)@(negedge clk);rst_n=1;enable=1;
  wait(completed[0]>=18);if(wraps<2)$fatal(1,"less than two full ring wraps");
  abort_load(0);abort_load(27);abort_load(28);abort_load(139);abort_load(140);abort_load(167);
  wait(cap[0]);repeat(30)@(negedge clk);enable=0;aborts=aborts+1;rearm();wait(completed[0]>=1);
  wait(wren[0]&&wh[0]&&addr[0]==45);@(negedge clk);sensor_ok=0;aborts=aborts+1;repeat(5)@(negedge clk);rearm();wait(completed[0]>=1);
  modes(1);wait(fault[0][1]);aborts=aborts+1;rearm();wait(completed[0]>=1);
  modes(2);wait(fault[0][2]);aborts=aborts+1;rearm();wait(completed[0]>=1);
  modes(3);wait(fault[0][2]);aborts=aborts+1;rearm();wait(completed[0]>=1);
  wait(wren[0]&&addr[0]==12);@(negedge clk);clock_run=0;#5;rst_n=0;#200;rst_n=1;clock_run=1;aborts=aborts+1;repeat(4)@(negedge clk);rearm();wait(completed[0]>=3);
  if(clipped_positive==0||clipped_negative==0||first_repeated_frames<5)$fatal(1,"missing clip/first-session coverage");
  $display("OBSERVATION_RING_PROTOCOL_PASS cycles=%0d words=%0d starts=%0d wraps=%0d aborts=%0d complete_slots=%0d positive_clips=%0d negative_clips=%0d first_repeats=%0d naturalX=1 defaults=legacy",checked,writes,starts,wraps,aborts,complete_slot_commits,clipped_positive,clipped_negative,first_repeated_frames);$finish;
 end
 initial begin #20000000;$fatal(1,"finite protocol watchdog");end
endmodule
