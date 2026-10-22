// Transport-independent motion lease. Inputs are complete, CRC/range checked
// packets; parser activity alone must not drive valid. Balance stays enabled.
// owner/session are trusted supervisor selections, not packet-controlled.
module command_lease_guard #(
 parameter integer CLK_HZ=50000000,
 parameter integer TIMEOUT_MS=200,FEEDBACK_TIMEOUT_MS=50,STOP_DWELL_MS=500,
 parameter integer STEP_V_Q12=41,STEP_W_Q12=410,
 parameter integer LIMIT_V_Q12=2458,LIMIT_W_Q12=11878
)(input wire clk,reset_n,
 input wire [1:0] owner_select,source_id,
 input wire [31:0] session_select,packet_session,
 input wire [15:0] packet_seq,
 input wire valid,source_fault,estop,
 input wire signed [23:0] forward_q12,yaw_q12,
 input wire feedback_valid,
 input wire signed [15:0] measured_vx_mmps,measured_wz_mradps,
 output reg signed [23:0] out_forward_q12,out_yaw_q12,
 output reg armed,expired,stop_confirmed,
 output reg [31:0] age_ms,accepted_count,rejected_count,
 output wire balance_enable);
 localparam integer DIV=CLK_HZ/1000;
 reg [$clog2(DIV+1)-1:0] divider;
 wire tick=divider==DIV-1;
 reg [1:0] owner;reg [31:0] session;
 reg [15:0] last_seq;reg have_seq,have_lease;
 reg signed [23:0] target_v,target_w;
 reg [31:0] feedback_age,still_ms;
 reg slow_feedback;
 wire changing=owner_select!=owner || session_select!=session;
 wire [15:0] seq_delta=packet_seq-last_seq;
 wire ordered=!have_seq || (seq_delta!=0 && !seq_delta[15]);
 wire legal=source_id==owner && owner!=0 && packet_session==session && ordered &&
  forward_q12>=-LIMIT_V_Q12 && forward_q12<=LIMIT_V_Q12 &&
  yaw_q12>=-LIMIT_W_Q12 && yaw_q12<=LIMIT_W_Q12;
 wire zero_packet=forward_q12==0 && yaw_q12==0;
 wire fresh_feedback=feedback_age<FEEDBACK_TIMEOUT_MS;
 wire slow_now=measured_vx_mmps>=-5 && measured_vx_mmps<=5 &&
  measured_wz_mradps>=-20 && measured_wz_mradps<=20;
 wire deadline=tick && have_lease && age_ms+1>=TIMEOUT_MS;
 wire stop_mode=!armed || expired || source_fault || !fresh_feedback || changing || deadline;
 assign balance_enable=1'b1; // A communication stop never disables the policy.
 function signed [23:0] approach;
  input signed [23:0] current,target;input integer step;
  reg signed [24:0] difference;
  begin
   difference=$signed(target)-$signed(current);
   approach=difference>step?current+step:difference < -step?current-step:target;
  end
 endfunction
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin
   divider<=0;owner<=0;session<=0;last_seq<=0;have_seq<=0;have_lease<=0;
   target_v<=0;target_w<=0;out_forward_q12<=0;out_yaw_q12<=0;
   armed<=0;expired<=1;age_ms<=0;feedback_age<=FEEDBACK_TIMEOUT_MS;
   still_ms<=0;slow_feedback<=0;stop_confirmed<=0;
   accepted_count<=0;rejected_count<=0;
  end else begin
   divider<=tick?0:divider+1'b1;
   if(tick)begin
    if(age_ms!=32'hffffffff)age_ms<=age_ms+1'b1;
    if(feedback_age!=32'hffffffff)feedback_age<=feedback_age+1'b1;
    if(!fresh_feedback || !slow_feedback || out_forward_q12!=0 || out_yaw_q12!=0)begin
     still_ms<=0;stop_confirmed<=0;
    end else if(still_ms<STOP_DWELL_MS)begin
     still_ms<=still_ms+1'b1;
     if(still_ms+1>=STOP_DWELL_MS)stop_confirmed<=1;
    end
    out_forward_q12<=approach(out_forward_q12,stop_mode?24'sd0:target_v,STEP_V_Q12);
    out_yaw_q12<=approach(out_yaw_q12,stop_mode?24'sd0:target_w,STEP_W_Q12);
   end
   if(feedback_valid)begin
    feedback_age<=0;slow_feedback<=slow_now;
    if(!slow_now)begin still_ms<=0;stop_confirmed<=0;end
   end
   // Sequence history is retained on timeout. Session/owner switch clears it,
   // flushes the old target and requires another fresh zero/stop handshake.
   if(changing)begin
    owner<=owner_select;session<=session_select;have_seq<=0;have_lease<=0;
    armed<=0;expired<=1;age_ms<=0;target_v<=0;target_w<=0;
    still_ms<=0;stop_confirmed<=0;
    if(valid)rejected_count<=rejected_count+1'b1;
   end else if(estop)begin
    out_forward_q12<=0;out_yaw_q12<=0;target_v<=0;target_w<=0;
    armed<=0;expired<=1;have_lease<=0;still_ms<=0;stop_confirmed<=0;
    if(valid)rejected_count<=rejected_count+1'b1;
   end else if(source_fault || deadline ||
               (armed && !fresh_feedback))begin
    armed<=0;expired<=1;have_lease<=0;target_v<=0;target_w<=0;
    if(valid)rejected_count<=rejected_count+1'b1;
   end else if(valid)begin
    if(legal && (armed || (zero_packet && stop_confirmed && fresh_feedback &&
                         !(feedback_valid && !slow_now))))begin
     target_v<=forward_q12;target_w<=yaw_q12;
     last_seq<=packet_seq;have_seq<=1;have_lease<=1;age_ms<=0;
     armed<=1;expired<=0;accepted_count<=accepted_count+1'b1;
    end else rejected_count<=rejected_count+1'b1;
   end
  end
 end
endmodule
