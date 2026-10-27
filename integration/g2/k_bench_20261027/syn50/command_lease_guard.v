// Transport-independent motion lease. Inputs are complete, CRC/range checked
// packets; parser activity alone must not drive valid. Balance stays enabled.
// owner/session are trusted supervisor selections, not packet-controlled.
module command_lease_guard #(
 parameter integer CLK_HZ=50000000,
 parameter integer TIMEOUT_MS=200,FEEDBACK_TIMEOUT_MS=50,
 // Stop is judged on the mean of each STILL_SAMPLES feedback block (100 = 1 s at 100 Hz):
 // a balancing chassis never holds instantaneous zero speed.
 parameter integer STILL_SAMPLES=100,STILL_V_MMPS=50,STILL_W_MRADPS=100,
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
 reg [31:0] feedback_age;
 reg fb_valid_q;reg signed [15:0] fb_vx_q,fb_wz_q;
 reg [7:0] still_n;
 reg signed [24:0] still_vx,still_wz,eval_vx,eval_wz;
 reg still_quiet,still_eval,eval_quiet;
 localparam [7:0] STILL_LAST=STILL_SAMPLES-1;
 localparam signed [24:0] STILL_V_SUM=STILL_V_MMPS*STILL_SAMPLES,STILL_W_SUM=STILL_W_MRADPS*STILL_SAMPLES;
 wire signed [24:0] still_vx_next=still_vx+fb_vx_q,still_wz_next=still_wz+fb_wz_q;
 wire still_mean_ok=eval_vx>=-STILL_V_SUM && eval_vx<=STILL_V_SUM &&
  eval_wz>=-STILL_W_SUM && eval_wz<=STILL_W_SUM;
 wire changing=owner_select!=owner || session_select!=session;
 wire [15:0] seq_delta=packet_seq-last_seq;
 wire ordered=!have_seq || (seq_delta!=0 && !seq_delta[15]);
 wire legal=source_id==owner && owner!=0 && packet_session==session && ordered &&
  forward_q12>=-LIMIT_V_Q12 && forward_q12<=LIMIT_V_Q12 &&
  yaw_q12>=-LIMIT_W_Q12 && yaw_q12<=LIMIT_W_Q12;
 wire zero_packet=forward_q12==0 && yaw_q12==0;
 wire fresh_feedback=feedback_age<FEEDBACK_TIMEOUT_MS;
 localparam [31:0] LEASE_TIMEOUT_U32=TIMEOUT_MS;
 wire lease_age_due;
 generate if(LEASE_TIMEOUT_U32==32'd0)begin:g_deadline_zero
  assign lease_age_due=(age_ms+32'd1)>=LEASE_TIMEOUT_U32;
 end else begin:g_deadline_nonzero
  wire [7:0] max4 /* synthesis syn_keep=1 */;
  wire [1:0] max16 /* synthesis syn_keep=1 */;
  wire nonmax32 /* synthesis syn_keep=1 */;
  genvar group;
  for(group=0;group<8;group=group+1)begin:g_max4
   LUT4 u_and(.F(max4[group]),.I0(age_ms[group*4]),.I1(age_ms[group*4+1]),
              .I2(age_ms[group*4+2]),.I3(age_ms[group*4+3]));
   defparam u_and.INIT=16'h8000;
  end
  LUT4 u_low16(.F(max16[0]),.I0(max4[0]),.I1(max4[1]),.I2(max4[2]),.I3(max4[3]));
  defparam u_low16.INIT=16'h8000;
  LUT4 u_high16(.F(max16[1]),.I0(max4[4]),.I1(max4[5]),.I2(max4[6]),.I3(max4[7]));
  defparam u_high16.INIT=16'h8000;
  LUT2 u_nonmax32(.F(nonmax32),.I0(max16[0]),.I1(max16[1]));
  defparam u_nonmax32.INIT=4'h7;
  assign lease_age_due=(age_ms>=LEASE_TIMEOUT_U32-32'd1) && nonmax32;
 end endgenerate
 wire deadline=tick && have_lease && lease_age_due;
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
   fb_valid_q<=0;fb_vx_q<=0;fb_wz_q<=0;
   still_n<=0;still_vx<=0;still_wz<=0;eval_vx<=0;eval_wz<=0;
   still_quiet<=0;still_eval<=0;eval_quiet<=0;stop_confirmed<=0;
   accepted_count<=0;rejected_count<=0;
  end else begin
   divider<=tick?0:divider+1'b1;
   if(tick)begin
    if(age_ms!=32'hffffffff)age_ms<=age_ms+1'b1;
    if(feedback_age!=32'hffffffff)feedback_age<=feedback_age+1'b1;
    out_forward_q12<=approach(out_forward_q12,stop_mode?24'sd0:target_v,STEP_V_Q12);
    out_yaw_q12<=approach(out_yaw_q12,stop_mode?24'sd0:target_w,STEP_W_Q12);
   end
   if(feedback_valid)feedback_age<=0;
   fb_valid_q<=feedback_valid;fb_vx_q<=measured_vx_mmps;fb_wz_q<=measured_wz_mradps;
   // A block counts only if feedback stayed fresh and the lease output stayed zero throughout.
   still_eval<=0;
   if(fb_valid_q)begin
    if(still_n==STILL_LAST)begin
     eval_vx<=still_vx_next;eval_wz<=still_wz_next;eval_quiet<=still_quiet;still_eval<=1;
     still_n<=0;still_vx<=0;still_wz<=0;still_quiet<=1;
    end else begin
     still_n<=still_n+1'b1;still_vx<=still_vx_next;still_wz<=still_wz_next;
    end
   end
   if(still_eval)stop_confirmed<=eval_quiet && still_mean_ok;
   if(!fresh_feedback || out_forward_q12!=0 || out_yaw_q12!=0)begin
    still_quiet<=0;stop_confirmed<=0;
   end
   // Sequence history is retained on timeout. Session/owner switch clears it,
   // flushes the old target and requires another fresh zero/stop handshake.
   if(changing)begin
    owner<=owner_select;session<=session_select;have_seq<=0;have_lease<=0;
    armed<=0;expired<=1;age_ms<=0;target_v<=0;target_w<=0;
    still_quiet<=0;stop_confirmed<=0;
    if(valid)rejected_count<=rejected_count+1'b1;
   end else if(estop)begin
    out_forward_q12<=0;out_yaw_q12<=0;target_v<=0;target_w<=0;
    armed<=0;expired<=1;have_lease<=0;still_quiet<=0;stop_confirmed<=0;
    if(valid)rejected_count<=rejected_count+1'b1;
   end else if(source_fault || deadline ||
               (armed && !fresh_feedback))begin
    armed<=0;expired<=1;have_lease<=0;target_v<=0;target_w<=0;
    if(valid)rejected_count<=rejected_count+1'b1;
   end else if(valid)begin
    if(legal && (armed || (zero_packet && stop_confirmed && fresh_feedback)))begin
     target_v<=forward_q12;target_w<=yaw_q12;
     last_seq<=packet_seq;have_seq<=1;have_lease<=1;age_ms<=0;
     armed<=1;expired<=0;accepted_count<=accepted_count+1'b1;
    end else rejected_count<=rejected_count+1'b1;
   end
  end
 end
endmodule
