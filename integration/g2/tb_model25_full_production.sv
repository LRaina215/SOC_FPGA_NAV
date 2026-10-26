`timescale 1ns/1ps
// ProductionBIAS200/100Hz and actual4500 cores. Only external stimulus and
// read-only hierarchical observations are used. No force, CPU/core substitute,
// shortened calibration, period, or accelerated hardware time.
module tb_model25_full_production;
 reg clk50=0,reset_n=0,run_model_clock=1;
 always #10 clk50=~clk50;
 GSR GSR(.GSRI(1'b1));
 wire pll25,pll_lock;
 Gowin_Model25_PLL pll(.clkin(clk50),.reset(!reset_n),.lock(pll_lock),.enclk0(1'b1),.clkout0(pll25));
 // Fixture-only externally stopped input to model/guard; hardwarePLL source is
 // frozen and has no such gate. The protected/internal PLL state is untouched.
 wire clk25=run_model_clock?pll25:1'b0;
 wire transport_n,model_n,domain_fault,transport_fault;
 g2_model25_guard guard(.clk50(clk50),.clk25(clk25),.reset_n(reset_n),.pll_lock(pll_lock),
  .transport_fault(transport_fault),.io_rst_n(transport_n),.model_rst_n(model_n),.fault(domain_fault));
 reg enable=1,external_fault=0;
 reg signed[23:0]forward=409,yaw=-205,height=535;
 reg sample_valid=0;
 reg[31:0]sensor_ticks=0,sample_count=0;
 always @(posedge clk50 or negedge reset_n)begin
  if(!reset_n)begin sensor_ticks<=0;sample_valid<=0;sample_count<=0;end
  else begin
   sample_valid<=0;
   if(sensor_ticks==499999)begin sensor_ticks<=0;sample_valid<=1;sample_count<=sample_count+1'b1;end
   else sensor_ticks<=sensor_ticks+1'b1;
  end
 end
 wire signed[31:0]rl,rr,cl,cr;
 wire[15:0]rlh,rlk,rrh,rrk,clh,clknee,crh,crk;
 wire rcal,ccal,rbias,cbias,rreject,creject,rready,cready,ractive,cactive,rvalid,cvalid;
 wire[7:0]rstatus,cstatus;wire[2:0]rfault,cfault;
 wire[31:0]rframes,cframes,rcycles,ccycles;
 wire[143:0]ractions,cactions;
 wire rfv,cfv;wire[23:0]rwx,rwy,rwz,rgx,rgy,rgz,cwx,cwy,cwz,cgx,cgy,cgz;
 mem_policy_reference #(.BIAS_SAMPLES(200),.PERIOD_CLKS(400000),.SENSOR_TIMEOUT_CLKS(2500000)) reference(
 .clk(clk50),.rst_n(reset_n),.enable(enable),.sensor_ready(1'b1),.sensor_error(1'b0),.sample_valid(sample_valid),.external_fault(external_fault),
 .accel_x(16'sd0),.accel_y(16'sd0),.accel_z(16'sd10920),.gyro_x(16'sd0),.gyro_y(16'sd0),.gyro_z(16'sd0),
 .left_encoder_delta(32'sd0),.right_encoder_delta(32'sd0),.command_forward_q12(forward),.command_yaw_q12(yaw),.command_height_q12(height),
 .target_left_mmps(rl),.target_right_mmps(rr),.servo_lh_us(rlh),.servo_lk_us(rlk),.servo_rh_us(rrh),.servo_rk_us(rrk),
 .calibrating(rcal),.bias_suspect(rbias),.accel_rejected(rreject),.ready(rready),.active(ractive),.action_valid(rvalid),
 .status_flags(rstatus),.policy_fault(rfault),.completed_frames(rframes),.inference_cycles(rcycles),.executed_actions(ractions),
 .debug_fusion_valid(rfv),.debug_wx(rwx),.debug_wy(rwy),.debug_wz(rwz),.debug_gx(rgx),.debug_gy(rgy),.debug_gz(rgz));
 mem_policy_related25 #(.BIAS_SAMPLES(200),.PERIOD_CLKS(200000),.SENSOR_TIMEOUT_CLKS(1250000)) dut(
 .clk(clk50),.clk_model(clk25),.rst_n(reset_n),.transport_rst_n(transport_n),.model_rst_n(model_n),.domain_fault(domain_fault),.transport_fault(transport_fault),
 .enable(enable),.sensor_ready(1'b1),.sensor_error(1'b0),.sample_valid(sample_valid),.external_fault(external_fault),
 .accel_x(16'sd0),.accel_y(16'sd0),.accel_z(16'sd10920),.gyro_x(16'sd0),.gyro_y(16'sd0),.gyro_z(16'sd0),
 .left_encoder_delta(32'sd0),.right_encoder_delta(32'sd0),.command_forward_q12(forward),.command_yaw_q12(yaw),.command_height_q12(height),
 .target_left_mmps(cl),.target_right_mmps(cr),.servo_lh_us(clh),.servo_lk_us(clknee),.servo_rh_us(crh),.servo_rk_us(crk),
 .calibrating(ccal),.bias_suspect(cbias),.accel_rejected(creject),.ready(cready),.active(cactive),.action_valid(cvalid),
 .status_flags(cstatus),.policy_fault(cfault),.completed_frames(cframes),.inference_cycles(ccycles),.executed_actions(cactions),
 .debug_fusion_valid(cfv),.debug_wx(cwx),.debug_wy(cwy),.debug_wz(cwz),.debug_gx(cgx),.debug_gy(cgy),.debug_gz(cgz));

 `define RC reference.policy.core
 `define CC dut.model.policy.core
 localparam MAXKEY=128;
 integer epoch=0,numeric_frames=0,epochs_compared[0:3],command_checks=0,known_value_checks=0;
 integer rloads[0:MAXKEY-1],cloads[0:MAXKEY-1],rtraces[0:MAXKEY-1],ctraces[0:MAXKEY-1];
 reg[23:0]rinputs[0:MAXKEY-1][0:167],cinputs[0:MAXKEY-1][0:167];
 reg[34:0]rtrace[0:MAXKEY-1][0:424],ctrace[0:MAXKEY-1][0:424];
 reg[335:0]routputs[0:MAXKEY-1];reg rpublished[0:MAXKEY-1];
 real rcapture_us[0:MAXKEY-1],ccapture_us[0:MAXKEY-1],cpublish_us[0:MAXKEY-1];
 real previous_capture_us[0:3],previous_publish_us[0:3],max_latency_us=0;
 integer reference_epochs_frames=0;
 reg rpublish_delay=0;
 always @(posedge clk50 or negedge reset_n)
  if(!reset_n)rpublish_delay<=0;else rpublish_delay<=rvalid;

 always @(posedge clk50)begin:reference_capture
  integer key,address;
  if(reset_n&&epoch>=0)begin
   if($isunknown({reference.capture,`RC.wr_en}))$fatal(1,"UNKNOWN reference capture/write control");
   if(reference.capture)begin
    if($isunknown(reference.policy.accepted_id))$fatal(1,"UNKNOWN reference accepted frame ID");
    key=epoch*32+reference.policy.accepted_id;
    if(key<0||key>=MAXKEY)$fatal(1,"reference capture key overflow");
    rcapture_us[key]=$realtime/1000.0;known_value_checks=known_value_checks+1;
   end
   if(`RC.wr_en)begin
    if($isunknown({`RC.frame_id,`RC.wr_history,`RC.wr_addr,`RC.wr_data}))$fatal(1,"UNKNOWN reference actual4500 input word or identity");
    key=epoch*32+`RC.frame_id;
    if(key<0||key>=MAXKEY)$fatal(1,"reference input key overflow");
    address=`RC.wr_history?28+`RC.wr_addr:`RC.wr_addr;
    if(address<0||address>=168)$fatal(1,"reference input address");
    rinputs[key][address]=`RC.wr_data;rloads[key]=rloads[key]+1;known_value_checks=known_value_checks+1;
   end
  end
 end
 always @(posedge clk25)begin:candidate_capture
  integer key,address;
  if(model_n&&epoch>=0)begin
   if($isunknown({dut.model.capture,`CC.wr_en}))$fatal(1,"UNKNOWN candidate capture/write control");
   if(dut.model.capture)begin
    if($isunknown(dut.model.policy.accepted_id))$fatal(1,"UNKNOWN candidate accepted frame ID");
    key=epoch*32+dut.model.policy.accepted_id;
    if(key<0||key>=MAXKEY)$fatal(1,"candidate capture key overflow");
    ccapture_us[key]=$realtime/1000.0;known_value_checks=known_value_checks+1;
    if(previous_capture_us[epoch]>0&&(($realtime/1000.0-previous_capture_us[epoch])<7999.999||($realtime/1000.0-previous_capture_us[epoch])>8000.001))
     $fatal(1,"production capture cadence changed delta_us=%f",$realtime/1000.0-previous_capture_us[epoch]);
    previous_capture_us[epoch]=$realtime/1000.0;
   end
   if(`CC.wr_en)begin
    if($isunknown({`CC.frame_id,`CC.wr_history,`CC.wr_addr,`CC.wr_data}))$fatal(1,"UNKNOWN candidate actual4500 input word or identity");
    key=epoch*32+`CC.frame_id;
    if(key<0||key>=MAXKEY)$fatal(1,"candidate input key overflow");
    address=`CC.wr_history?28+`CC.wr_addr:`CC.wr_addr;
    if(address<0||address>=168)$fatal(1,"candidate input address");
    cinputs[key][address]=`CC.wr_data;cloads[key]=cloads[key]+1;known_value_checks=known_value_checks+1;
   end
  end
 end
 always @(negedge clk50)begin:reference_results
  integer key;
  if(reset_n&&epoch>=0)begin
   if($isunknown({`RC.dbg_valid,rpublish_delay}))$fatal(1,"UNKNOWN reference trace/publication control");
   // While computing, pending_frame_id owns the layer trace.
   if(`RC.dbg_valid)begin
    if($isunknown({`RC.pending_frame_id,`RC.dbg_layer,`RC.dbg_index,`RC.dbg_data}))$fatal(1,"UNKNOWN reference actual4500 layer result");
    key=epoch*32+`RC.pending_frame_id;
    if(key<0||key>=MAXKEY)$fatal(1,"reference trace key overflow");
    if(rtraces[key]>=425)$fatal(1,"reference trace overflow");
    rtrace[key][rtraces[key]]={`RC.dbg_layer,`RC.dbg_index,`RC.dbg_data};rtraces[key]=rtraces[key]+1;known_value_checks=known_value_checks+1;
   end
   if(rpublish_delay)begin
    if($isunknown({`RC.result_frame_id,rl,rr,rlh,rlk,rrh,rrk,ractions,rframes,rcycles}))$fatal(1,"UNKNOWN reference actuator/publication bundle");
    key=epoch*32+rframes;
    if(key<0||key>=MAXKEY)$fatal(1,"reference publication key overflow");
    if(`RC.result_frame_id!==rframes)$fatal(1,"reference frame identity");
    routputs[key]={rl,rr,rlh,rlk,rrh,rrk,ractions,rframes,rcycles};rpublished[key]=1;known_value_checks=known_value_checks+1;
    $display("REF_PUBLICATION epoch=%0d id=%0d us=%0.3f core_cycles=%0d capture_us=%0.3f",epoch,rframes,$realtime/1000.0,rcycles,rcapture_us[key]);
   end
  end
 end
 always @(negedge clk25)begin:candidate_trace
  integer key;
  if(model_n&&epoch>=0)begin
   if($isunknown(`CC.dbg_valid))$fatal(1,"UNKNOWN candidate trace control");
   if(`CC.dbg_valid)begin
    if($isunknown({`CC.pending_frame_id,`CC.dbg_layer,`CC.dbg_index,`CC.dbg_data}))$fatal(1,"UNKNOWN candidate actual4500 layer result");
    key=epoch*32+`CC.pending_frame_id;
    if(key<0||key>=MAXKEY)$fatal(1,"candidate trace key overflow");
    if(ctraces[key]>=425)$fatal(1,"candidate trace overflow");
    ctrace[key][ctraces[key]]={`CC.dbg_layer,`CC.dbg_index,`CC.dbg_data};ctraces[key]=ctraces[key]+1;known_value_checks=known_value_checks+1;
   end
   if($isunknown({dut.model.command_forward_q12,dut.model.command_yaw_q12,dut.model.command_height_q12}))$fatal(1,"UNKNOWN atomic command bundle");
   if(!((dut.model.command_forward_q12==409&&dut.model.command_yaw_q12==-205)||
       (dut.model.command_forward_q12==819&&dut.model.command_yaw_q12==410)||
       (dut.model.command_forward_q12==0&&dut.model.command_yaw_q12==0)))
    $fatal(1,"partial command pair crossed into model");
   command_checks=command_checks+1;
  end
 end
 always @(negedge clk50)begin:candidate_publication
  integer key;real latency,interval;
  if(reset_n&&transport_n&&$isunknown({cvalid,cactive,domain_fault,transport_fault,cstatus,cfault}))
   $fatal(1,"UNKNOWN candidate bridge/health/stop control");
  if(reset_n&&epoch>=0&&cvalid)begin
   if($isunknown({`CC.result_frame_id,cl,cr,clh,clknee,crh,crk,cactions,cframes,ccycles}))$fatal(1,"UNKNOWN candidate actuator/publication bundle");
   key=epoch*32+cframes;
   if(key<0||key>=MAXKEY)$fatal(1,"candidate publication key overflow");
   if(`CC.result_frame_id!==cframes||!rpublished[key])$fatal(1,"candidate/reference actual frame identity missing");
   if(rloads[key]!=168||cloads[key]!=168)$fatal(1,"not a complete actual168word frame");
   for(integer i=0;i<168;i=i+1)begin
    if($isunknown({rinputs[key][i],cinputs[key][i]}))$fatal(1,"UNKNOWN saved captured input word epoch%0d id%0d word%0d",epoch,cframes,i);
    if(rinputs[key][i]!==cinputs[key][i])$fatal(1,"CAPTURE_INPUT_MISMATCH epoch=%0d id=%0d word=%0d ref=%h candidate=%h",epoch,cframes,i,rinputs[key][i],cinputs[key][i]);
   end
   if(rtraces[key]!=425||ctraces[key]!=425)$fatal(1,"not complete425 layer outputs ref%0d cand%0d",rtraces[key],ctraces[key]);
   for(integer i=0;i<425;i=i+1)begin
    if($isunknown({rtrace[key][i],ctrace[key][i]}))$fatal(1,"UNKNOWN saved actual4500 layer result epoch%0d id%0d index%0d",epoch,cframes,i);
    if(rtrace[key][i]!==ctrace[key][i])$fatal(1,"real layer trace mismatch epoch%0d id%0d index%0d",epoch,cframes,i);
   end
   if($isunknown(routputs[key]))$fatal(1,"UNKNOWN saved reference publication bundle");
   if({cl,cr,clh,clknee,crh,crk,cactions,cframes,ccycles}!==routputs[key])$fatal(1,"actuator/publication values mismatch epoch%0d id%0d",epoch,cframes);
   latency=$realtime/1000.0-ccapture_us[key];
   if(latency<=0||latency>8000.0)$fatal(1,"end_to_end8ms missed age_us=%f",latency);
   if(max_latency_us<latency)max_latency_us=latency;
   if(previous_publish_us[epoch]>0)begin
    interval=$realtime/1000.0-previous_publish_us[epoch];
    if(interval<7999.999||interval>8000.001)$fatal(1,"external125Hz publish cadence=%f",interval);
   end
   previous_publish_us[epoch]=$realtime/1000.0;cpublish_us[key]=$realtime/1000.0;
   numeric_frames=numeric_frames+1;epochs_compared[epoch]=epochs_compared[epoch]+1;known_value_checks=known_value_checks+1;
   $display("FRAME_NUMERIC_MATCH epoch=%0d id=%0d actual_input_words=168 trace_words=425 core_actions=6 actuator_bits=336 capture_us=%0.3f publish_us=%0.3f latency_us=%0.3f core_cycles25=%0d core_ms25=%0.6f",epoch,cframes,ccapture_us[key],$realtime/1000.0,latency,ccycles,ccycles/25000.0);
  end
 end

 reg[63:0]held_ref_pose,held_candidate_pose;
 integer stop_cases=0,recovery_cases=0,reset_cases=0,clock_halt_cases=0;
 real stop_start_us,halt_start_us;
 initial begin:stimulus
  for(integer k=0;k<MAXKEY;k=k+1)begin
   rloads[k]=0;cloads[k]=0;rtraces[k]=0;ctraces[k]=0;rpublished[k]=0;
   rcapture_us[k]=0;ccapture_us[k]=0;cpublish_us[k]=0;
  end
  for(integer k=0;k<4;k=k+1)begin epochs_compared[k]=0;previous_capture_us[k]=0;previous_publish_us[k]=0;end
  #1000;reset_n=1;
  wait(epochs_compared[0]>=3);
  @(negedge clk50);forward=819;yaw=410;
  wait(epochs_compared[0]>=5);
  @(negedge clk50);forward=0;yaw=0;
  wait(epochs_compared[0]>=7);
  if(!cactive||!ractive||cfault||rfault)$fatal(1,"controlled zero disabled balancing");
  $display("CONTROLLED_ZERO_BALANCING_PASS us=%0.3f model_active=1",$realtime/1000.0);stop_cases=stop_cases+1;
  // Stop while a complete source action is pending transfer. Each side must
  // retain its own last delivered pose; their publication latencies differ.
  wait(dut.action_bridge.request!=dut.action_bridge.ack2);
  epoch=-1;@(negedge clk50);
  held_ref_pose={rlh,rlk,rrh,rrk};held_candidate_pose={clh,clknee,crh,crk};
  external_fault=1;stop_start_us=$realtime/1000.0;#1;
  if(cactive)$fatal(1,"independent50MHz externalfault did not gate active");
  repeat(4)@(negedge clk50);
  if(cl!==0||cr!==0||cactions!==0)$fatal(1,"stop did not clear published wheel/action outputs");
  repeat(100000)begin
   @(negedge clk50);
   if({clh,clknee,crh,crk}!==held_candidate_pose||{rlh,rlk,rrh,rrk}!==held_ref_pose)$fatal(1,"stop changed last delivered leg pose");
   if(cvalid||cactive)$fatal(1,"new candidate action escaped stopped bridge");
  end
  external_fault=0;repeat(800000)@(negedge clk50);
  if(cactive||ractive)$fatal(1,"fault removal automatically resumed");
  stop_cases=stop_cases+1;$display("FAULT_STOP_POSE_HOLD_PASS no_auto_resume=1 us=%0.3f",$realtime/1000.0);
  enable=0;repeat(100000)@(negedge clk50);
  epoch=1;enable=1;
  wait(epochs_compared[1]>=3);recovery_cases=recovery_cases+1;
  $display("EXPLICIT_REARM_PASS us=%0.3f",$realtime/1000.0);
  // Common physical reset during a pending action transaction. Never alter
  // internal request/state/RAM. PLL reset itself stops the model clock.
  wait(dut.action_bridge.request!=dut.action_bridge.ack2);epoch=-1;
  @(negedge clk50);reset_n=0;
  #1;if(model_n||transport_n)$fatal(1,"reset did not asynchronously assert both local resets");
  #99999;reset_n=1;epoch=2;
  repeat(100000)begin
   @(negedge clk50);if(cvalid||cactive||cframes!=0)$fatal(1,"pre-reset action replayed before production recalibration");
  end
  wait(epochs_compared[2]>=2);reset_cases=reset_cases+1;
  $display("PENDING_TRANSFER_RESET_RECALIBRATION_PASS us=%0.3f",$realtime/1000.0);
  wait(dut.action_bridge.request!=dut.action_bridge.ack2);epoch=-1;
  @(negedge clk25);run_model_clock=0;halt_start_us=$realtime/1000.0;
  wait(domain_fault);#1;held_candidate_pose={clh,clknee,crh,crk};
  if(($realtime/1000.0-halt_start_us)>0.5)$fatal(1,"25MHz halt exceeded500ns guard envelope");
  repeat(4)@(negedge clk50);
  if(cactive||cl!==0||cr!==0)$fatal(1,"model halt did not stop wheel command");
  repeat(1000)begin @(negedge clk50);if({clh,clknee,crh,crk}!==held_candidate_pose||cvalid)$fatal(1,"halt altered held pose or published action");end
  @(negedge pll25);run_model_clock=1;repeat(1000)@(negedge clk50);
  if(!domain_fault||cactive||cl!==0||cr!==0)$fatal(1,"model clock restoration auto-rearmed");
  clock_halt_cases=clock_halt_cases+1;
  $display("MODEL25_FULL_PRODUCTION_PASS numeric_frames=%0d command_checks=%0d known_value_checks=%0d BIAS_SAMPLES=200 SENSOR_HZ=100 POLICY_HZ=125 max_capture_to_publication_us=%0.3f controlled_and_fault_stops=%0d explicit_rearms=%0d pending_reset_cases=%0d clock_halt_cases=%0d",numeric_frames,command_checks,known_value_checks,max_latency_us,stop_cases,recovery_cases,reset_cases,clock_halt_cases);
  $finish;
 end
 always begin #100000000;$display("PRODUCTION_PROGRESS us=%0.3f samples=%0d refcal=%b candcal=%b ref_frames=%0d cand_frames=%0d flags=%h fault=%b epoch=%0d",$realtime/1000.0,sample_count,rcal,ccal,rframes,cframes,cstatus,domain_fault,epoch);end
 initial begin #(64'd5000000000);$fatal(1,"finite5second stimulus watchdog");end
endmodule
