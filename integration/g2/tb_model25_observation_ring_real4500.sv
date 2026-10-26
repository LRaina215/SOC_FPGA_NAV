`timescale 1ns/1ps
// Real original observers/schedulers and two unmodified4500 cores.  No fusion
// calibration or wholeG2 is simulated. Both model clocks are physical25MHz;
// both periods are200000 cycles/8ms. No forced state or inference substitute.
module tb_observation_ring_real4500;
 reg clk=0,rst_n=0,enable=0,sensor_ok=1,stop_latched=0;
 always #20 clk=~clk;
 reg signed[23:0]forward=409,yaw=-205,height=535;
 reg[143:0]last_actions=0;
 wire[671:0]r_cur,c_cur;wire[3359:0]r_hist,c_hist;
 wire r_capture,c_capture,r_valid,c_valid,r_init,c_init,r_av,c_av;
 wire[143:0]r_actions,c_actions;wire[31:0]r_frames,c_frames,r_cycles,c_cycles;
 wire[2:0]r_fault,c_fault;wire freeze;wire[7:0]read_index;wire signed[23:0]read_data;
 wire obs_rst=rst_n&&enable&&!stop_latched;
 rl_observation_v5_ref r_obs(.clk(clk),.rst_n(obs_rst),.capture(r_capture),
  .angular_velocity_q12({24'sd0,24'sd0,24'sd0}),.gravity_q12({-24'sd4096,24'sd0,24'sd0}),
  .left_delta(16'sd0),.right_delta(16'sd0),.linear_command_q12(forward),.yaw_command_q12(yaw),.height_command_q12(height),
  .last_executed_actions(last_actions),.valid(r_valid),.initialized(r_init),.current_flat(r_cur),.history_flat(r_hist));
 rl_observation_v5_candidate #(.OBS_RING_STORAGE(1))c_obs(.clk(clk),.rst_n(obs_rst),.capture(c_capture),
  .frame_freeze(freeze),.frame_read_index(read_index),.frame_read_data(read_data),
  .angular_velocity_q12({24'sd0,24'sd0,24'sd0}),.gravity_q12({-24'sd4096,24'sd0,24'sd0}),
  .left_delta(16'sd0),.right_delta(16'sd0),.linear_command_q12(forward),.yaw_command_q12(yaw),.height_command_q12(height),
  .last_executed_actions(last_actions),.valid(c_valid),.initialized(c_init),.current_flat(c_cur),.history_flat(c_hist));
 rl_policy_tick_v5_ref #(.PERIOD_CLKS(200000))r_policy(.clk(clk),.rst_n(rst_n),.enable(enable),.sensor_ok(sensor_ok&&!stop_latched),
  .frame_valid(r_valid),.current_flat(r_cur),.history_flat(r_hist),.capture(r_capture),.action_valid(r_av),.actions(r_actions),
  .completed_frames(r_frames),.inference_cycles(r_cycles),.fault(r_fault),.diag_last_fault(),.diag_core_detail(),.diag_fault_count(),.diag_retry_count(),.diag_expected_id(),.diag_result_id());
 rl_policy_tick_v5_candidate #(.OBS_RING_STORAGE(1),.PERIOD_CLKS(200000))c_policy(.clk(clk),.rst_n(rst_n),.enable(enable),.sensor_ok(sensor_ok&&!stop_latched),
  .frame_valid(c_valid),.current_flat(c_cur),.history_flat(c_hist),.frame_freeze(freeze),.frame_read_index(read_index),.frame_read_data(read_data),
  .capture(c_capture),.action_valid(c_av),.actions(c_actions),.completed_frames(c_frames),.inference_cycles(c_cycles),.fault(c_fault),
  .diag_last_fault(),.diag_core_detail(),.diag_fault_count(),.diag_retry_count(),.diag_expected_id(),.diag_result_id());
 // This is common external applied-action stimulus from an already compared
 // published result, not a replacement/initialization of either4500 core.
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin stop_latched<=0;last_actions<=0;end
  else begin
   if(!enable)begin stop_latched<=0;last_actions<=0;end
   else begin
    if(!sensor_ok||r_fault!=0)stop_latched<=1;
    if(stop_latched)last_actions<=0;
    else if(r_av)last_actions<=r_actions;
   end
  end
 end
 `define RC r_policy.core
 `define CC c_policy.core
 integer edges=0,checked_edges=0,words=0,traces=0,core_results=0,published=0,wraps=0,aborts=0,first_repeats=0;
 integer load_count=0,trace_count=0,epoch=0,j,k;reg[2:0]last_work=0;
 real capture_us=0,previous_capture_us=0,previous_publish_us=0,max_latency_us=0;
 reg[23:0]loaded[0:167];
 reg[27:0]written_lanes=0;
 function signed[23:0]clip(input signed[23:0]x);begin clip=x>409600?24'sd409600:x< -409600?-24'sd409600:x;end endfunction
 always @(posedge clk or negedge obs_rst)begin
  if(!obs_rst)written_lanes<=0;
  else begin
   if(c_obs.state==0&&c_capture)written_lanes<=0;
   if(c_obs.g_ring.lane_write)written_lanes[c_obs.lane]<=1;
   if(c_obs.state==4&&!(&written_lanes))$fatal(1,"real4500 partial slot commit");
  end
 end
 always @(posedge clk)begin
  edges=edges+1;
  if(!rst_n||!enable)begin previous_capture_us=0;previous_publish_us=0;end
  if(rst_n&&enable)begin
   if(r_capture!==c_capture||r_valid!==c_valid||r_init!==c_init)$fatal(1,"real4500 capture/commit edge differs");
   if(r_capture)begin
    if($isunknown({r_policy.accepted_id,c_policy.accepted_id})||r_policy.accepted_id!==c_policy.accepted_id)$fatal(1,"capture frame ID");
    capture_us=$realtime/1000.0;load_count=0;
    if(previous_capture_us>0&&($realtime/1000.0-previous_capture_us<7999.999||$realtime/1000.0-previous_capture_us>8000.001))$fatal(1,"capture not125Hz");
    previous_capture_us=$realtime/1000.0;
   end
   if({`RC.wr_en,`RC.wr_history,`RC.wr_addr,`RC.start,`RC.frame_id}!=={`CC.wr_en,`CC.wr_history,`CC.wr_addr,`CC.start,`CC.frame_id})$fatal(1,"core protocol/START moved");
   if(`RC.wr_en)begin
    if($isunknown({`RC.wr_data,`CC.wr_data})||`RC.wr_data!==`CC.wr_data)$fatal(1,"168word real input mismatch index%0d",load_count);
    if(load_count>=168)$fatal(1,"too many real core input writes");
    loaded[load_count]=`CC.wr_data;load_count=load_count+1;words=words+1;
   end
   if(`RC.start)begin
    if(load_count!=168)$fatal(1,"real core START incomplete frame");trace_count=0;
    for(j=0;j<28;j=j+1)begin
     if(loaded[j]!==clip(loaded[140+j]))$fatal(1,"current/history newest mismatch");
     if(c_obs.g_ring.live0==c_obs.g_ring.live4)
      for(k=0;k<5;k=k+1)if(loaded[28+k*28+j]!==loaded[140+j])$fatal(1,"first history not repeated");
    end
    if(c_obs.g_ring.live0==c_obs.g_ring.live4)first_repeats=first_repeats+1;
   end
  end
 end
 always @(negedge clk)if(rst_n)begin
  checked_edges=checked_edges+1;
  if({r_policy.state,r_fault,r_av,r_actions,r_frames,r_cycles}!=={c_policy.state,c_fault,c_av,c_actions,c_frames,c_cycles})$fatal(1,"scheduler/published value-cycle differs");
  if($isunknown({`RC.dbg_valid,`CC.dbg_valid,`RC.done,`CC.done,r_av,c_av}))$fatal(1,"unknown real trace/result control");
  if(`RC.dbg_valid!==`CC.dbg_valid)$fatal(1,"425trace edge moved");
  if(`RC.dbg_valid)begin
   if($isunknown({`RC.pending_frame_id,`RC.dbg_layer,`RC.dbg_index,`RC.dbg_data,`CC.pending_frame_id,`CC.dbg_layer,`CC.dbg_index,`CC.dbg_data}))$fatal(1,"unknown actual425trace");
   if({`RC.pending_frame_id,`RC.dbg_layer,`RC.dbg_index,`RC.dbg_data}!=={`CC.pending_frame_id,`CC.dbg_layer,`CC.dbg_index,`CC.dbg_data})$fatal(1,"425 intermediate value/index/ID mismatch ordinal%0d",trace_count);
   trace_count=trace_count+1;traces=traces+1;
  end
  if(`RC.done!==`CC.done)$fatal(1,"real core done moved");
  if(`RC.done)begin
   if(trace_count!=425)$fatal(1,"not425 actual intermediate results");
   if($isunknown({`RC.action0,`RC.action1,`RC.action2,`RC.action3,`RC.action4,`RC.action5,`CC.action0,`CC.action1,`CC.action2,`CC.action3,`CC.action4,`CC.action5}))$fatal(1,"unknown core six actions");
   if({`RC.result_frame_id,`RC.cycles,`RC.error_flags,`RC.saturated,`RC.action0,`RC.action1,`RC.action2,`RC.action3,`RC.action4,`RC.action5}!=={`CC.result_frame_id,`CC.cycles,`CC.error_flags,`CC.saturated,`CC.action0,`CC.action1,`CC.action2,`CC.action3,`CC.action4,`CC.action5})$fatal(1,"actual core result/action mismatch");
   core_results=core_results+1;
  end
  if(r_av)begin
   if(load_count!=168||trace_count!=425||`CC.result_frame_id!==c_policy.accepted_id||c_frames!==r_frames)$fatal(1,"publication count/identity");
   if($realtime/1000.0-capture_us>8000.0)$fatal(1,"actual8ms missed");
   if(max_latency_us<$realtime/1000.0-capture_us)max_latency_us=$realtime/1000.0-capture_us;
   if(previous_publish_us>0&&($realtime/1000.0-previous_publish_us<7999.999||$realtime/1000.0-previous_publish_us>8000.001))$fatal(1,"publish not125Hz");
   previous_publish_us=$realtime/1000.0;published=published+1;
   $display("RING_REAL4500_FRAME_MATCH epoch=%0d id=%0d us=%0.3f words=168 trace=425 actions=6 core_cycles=%0d latency_us=%0.3f",epoch,c_frames,$realtime/1000.0,c_cycles,$realtime/1000.0-capture_us);
  end
  if(c_init&&last_work==5&&c_obs.g_ring.work_slot==0)wraps=wraps+1;
  last_work=c_obs.g_ring.work_slot;
  if(c_policy.state==2&&c_obs.g_ring.lane_write)$fatal(1,"write overlaps frozen LOAD");
 end
 task rearm;begin
  @(negedge clk);enable=0;repeat(10)@(negedge clk);sensor_ok=1;epoch=epoch+1;enable=1;
 end endtask
 initial begin
  #2;if(!$isunknown(c_obs.g_ring.frame_mem[0]))$fatal(1,"ring RAM not naturalX");
  repeat(10)@(negedge clk);rst_n=1;enable=1;
  wait(published>=6);@(negedge clk);forward=819;yaw=410;
  wait(published>=10);@(negedge clk);forward=0;yaw=0;
  wait(published>=14);if(wraps<2)$fatal(1,"less than two complete wraps");
  wait(`RC.wr_en&&`RC.wr_history&&`RC.wr_addr==40);rearm();aborts=aborts+1;wait(c_frames>=2);
  wait(`RC.dbg_valid&&`RC.dbg_layer==0&&`RC.dbg_index==20);rearm();aborts=aborts+1;wait(c_frames>=2);
  wait(`RC.busy);@(negedge clk);sensor_ok=0;repeat(10)@(negedge clk);
  if(r_av||c_av||r_fault[0]!==1||c_fault[0]!==1)$fatal(1,"sensor fault did not cancel/lock");
  rearm();aborts=aborts+1;wait(c_frames>=3);
  if(first_repeats<4)$fatal(1,"fresh epoch history not verified");
  $display("OBSERVATION_RING_REAL4500_PASS cycles=%0d words=%0d trace=%0d results=%0d published=%0d wraps=%0d aborts=%0d first_repeats=%0d max_latency_us=%0.3f clock_MHz=25 period_us=8000",checked_edges,words,traces,core_results,published,wraps,aborts,first_repeats,max_latency_us);$finish;
 end
 initial begin #400000000;$fatal(1,"finite400ms real-time stimulus watchdog");end
endmodule
