// Isolated NAV/stop observer for original4500 nominal 65mm/1320CPR,10ms counts.
// Input deltas are raw AB x4: left forward positive, right forward negative.
// Original PID/RL inputs remain untouched. Not connected to g2_top by default.
module nav_count_feedback #(
 parameter integer CLK_HZ=50000000
)(input wire clk,reset_n,sample_valid,quality,
 input wire signed[31:0] left_delta,right_delta,
 output reg valid,output reg signed[15:0] vx_mmps,
 output reg stop_window_valid,stop_translation_slow,
 output reg[10:0] window_capped_travel,
 output reg fault,continuity_lost);
 localparam integer GATE_CYCLES=CLK_HZ/100;
 localparam CONFIG_VALID=(CLK_HZ%100==0 && GATE_CYCLES>=64);
 // vx = (left_delta-right_delta)*1547/200 mm/s, exactly for nominal geometry.
 // Carry the signed remainder: 200*sum(vx)+remainder=1547*sum(count difference).
 // Truncate toward zero; nearest-away rounding with error feedback would
 // oscillate at half a unit even after the vehicle stopped.
 localparam IDLE=0,MULTIPLY=1,ACCUMULATE=2,ABSOLUTE=3,DIVIDE=4,PUBLISH=5;
 reg[2:0] state;
 reg signed[31:0] left_q,right_q;
 reg signed[32:0] count_sum;
 reg signed[43:0] product,numerator;
 reg signed[8:0] residual;
 reg negative;
 reg[43:0] dividend,quotient;
 reg[7:0] remainder;
 reg[5:0] bit_index;
 wire[8:0] trial={remainder,dividend[43]};
 wire take=trial>=9'd200;
 wire[8:0] reduced=take ? trial-9'd200 : trial;
 wire[43:0] next_quotient={quotient[42:0],take};
 reg[$clog2(GATE_CYCLES+2)-1:0] sample_age;
 reg have_sample,have_published;

 // Saturating each entry at31 is enough for the <=30 decision. If a slot
 // saturates, slow is necessarily false until that slot leaves the window.
 (* syn_ramstyle="distributed_ram" *) reg[4:0] travel[0:49];
 reg[5:0] head,filled;
 reg[10:0] travel_sum;
 reg pending_window_valid,pending_slow;
 reg[10:0] pending_travel;
 wire[31:0] left_abs=left_q[31] ? (~left_q+32'd1) : left_q;
 wire[31:0] right_abs=right_q[31] ? (~right_q+32'd1) : right_q;
 wire[32:0] raw_travel={1'b0,left_abs}+{1'b0,right_abs};
 wire[4:0] new_travel=raw_travel>31 ? 5'd31 : raw_travel[4:0];
 wire[4:0] old_travel=filled<50 ? 5'd0 : travel[head];
 wire[10:0] next_travel=travel_sum+new_travel-old_travel;
 // 50x10ms window, 5mm/s limit. Add two count quanta for endpoint phase:
 // (N+2)*1547/20000 mm <= 2.5mm iff N<=30. This is encoder-based
 // supplementary sampled-net-count evidence. Within-gate reversal, sub-count
 // motion and slip are unobservable here. It cannot alone confirm a physical
 // stop or replace velocity/gyro/quality and an independently tested envelope.
 wire cadence_bad=have_sample && sample_age!=GATE_CYCLES-1;
 wire missing_sample=have_sample && sample_age>=GATE_CYCLES;

 always@(posedge clk or negedge reset_n)begin
  if(!reset_n)begin
   state<=IDLE;left_q<=0;right_q<=0;count_sum<=0;product<=0;numerator<=0;
   residual<=0;negative<=0;dividend<=0;quotient<=0;remainder<=0;bit_index<=0;
   sample_age<=0;have_sample<=0;have_published<=0;
   head<=0;filled<=0;travel_sum<=0;pending_window_valid<=0;pending_slow<=0;pending_travel<=0;
   valid<=0;vx_mmps<=0;stop_window_valid<=0;stop_translation_slow<=0;window_capped_travel<=0;
   fault<=0;continuity_lost<=0;
  end else begin
   valid<=0;
   if(sample_age<GATE_CYCLES+1)sample_age<=sample_age+1'b1;
   if(!quality || !CONFIG_VALID || (sample_valid && (state!=IDLE || cadence_bad)) || (!sample_valid && missing_sample))begin
    if(have_sample || have_published)continuity_lost<=1;
    state<=IDLE;residual<=0;have_sample<=0;sample_age<=0;fault<=1;
    head<=0;filled<=0;travel_sum<=0;
    stop_window_valid<=0;stop_translation_slow<=0;window_capped_travel<=0;
   end else if(sample_valid)begin
    left_q<=left_delta;right_q<=right_delta;
    count_sum<=$signed({left_delta[31],left_delta})-$signed({right_delta[31],right_delta});
    have_sample<=1;sample_age<=0;state<=MULTIPLY;
   end else case(state)
    MULTIPLY:begin
     product<=$signed(count_sum)*44'sd1547;
     travel[head]<=new_travel;travel_sum<=next_travel;
     head<=head==49 ? 0 : head+1'b1;
     if(filled<50)filled<=filled+1'b1;
     pending_window_valid<=filled>=49;
     pending_slow<=filled>=49 && next_travel<=30;
     pending_travel<=next_travel;
     state<=ACCUMULATE;
    end
    ACCUMULATE:begin numerator<=$signed(product)+$signed(residual);state<=ABSOLUTE;end
    ABSOLUTE:begin
     negative<=numerator[43];dividend<=numerator[43] ? -numerator : numerator;
     quotient<=0;remainder<=0;bit_index<=0;state<=DIVIDE;
    end
    DIVIDE:begin
     dividend<={dividend[42:0],1'b0};quotient<=next_quotient;remainder<=reduced[7:0];
     if(bit_index==43)begin
      if(next_quotient>32767)begin
       // Do not wrap/saturate an out-of-range value into a false stop sample.
       state<=IDLE;fault<=1;continuity_lost<=1;residual<=0;have_sample<=0;sample_age<=0;
       head<=0;filled<=0;travel_sum<=0;
       stop_window_valid<=0;stop_translation_slow<=0;window_capped_travel<=0;
      end else state<=PUBLISH;
     end else bit_index<=bit_index+1'b1;
    end
    PUBLISH:begin
     vx_mmps<=negative ? -$signed(quotient[15:0]) : $signed(quotient[15:0]);
     residual<=negative ? -$signed({1'b0,remainder}) : $signed({1'b0,remainder});
     valid<=1;fault<=0;have_published<=1;
     stop_window_valid<=pending_window_valid;stop_translation_slow<=pending_slow;
     window_capped_travel<=pending_travel;state<=IDLE;
    end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
