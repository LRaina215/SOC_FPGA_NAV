// Optional MANUAL-command guard. Never scales wheel PWM or disables balance.
// Distances are measured from the sensor face; physical clearance needs calibration.
module ultrasonic_drive_guard #(
    parameter integer CLOCK_HZ=50000000,
    parameter [7:0] MODE_KEY=8'h16, // candidate NEC code for remote 0
    parameter integer STOP_MM=250,
    parameter integer RELEASE_MM=350,
    parameter integer SLOW_MM=600,
    parameter integer MAX_FORWARD_Q12=1229,
    parameter integer STALE_MS=150,
    parameter integer ACCEL_STEP_Q12=2, // per ms: approximately 0.488 m/s^2
    parameter integer DECEL_STEP_Q12=4  // per ms: approximately 0.977 m/s^2
)(
    input wire clk,reset_n,
    input wire [7:0] remote_key,
    input wire remote_key_event,remote_key_active,
    input wire manual_owner,hard_stop,
    input wire signed [23:0] requested_forward_q12,requested_yaw_q12,
    input wire sample_strobe,sample_valid,forward_ready,
    input wire [15:0] distance_mm,
    input wire feedback_valid,
    input wire signed [15:0] feedback_forward_mmps,
    output reg enabled,
    output wire signed [23:0] forward_q12,yaw_q12,
    output wire sample_healthy,braking,
    output reg stop_latched,
    output reg [15:0] sample_age_ms,
    output reg [23:0] range_target_q12
);
    localparam integer MS_CYCLES=CLOCK_HZ/1000;
    reg [31:0] ms_count;
    wire ms_tick=ms_count==MS_CYCLES-1;
    reg mode_key_held;
    reg [1:0] good_count,clear_count;
    reg seen_healthy;
    reg signed [23:0] ramp_q12;
    reg signed [23:0] parking_q12;
    reg signed [14:0] parking_sum;
    reg signed [31:0] parking_error;
    reg [15:0] feedback_age_ms;
    reg parking_seen,parking_negative;
    reg [1:0] parking_phase;
    reg [5:0] parking_bit;
    reg [35:0] parking_dividend,parking_quotient;
    reg [16:0] parking_remainder;
    wire positive_manual=manual_owner && requested_forward_q12>0;
    assign sample_healthy=enabled && forward_ready && good_count>=2 && sample_age_ms<STALE_MS;
    wire inhibit=!sample_healthy || stop_latched;
    wire signed [23:0] desired_q12=requested_forward_q12<$signed(range_target_q12) ?
        requested_forward_q12 : $signed(range_target_q12);
    wire protected_forward=enabled && positive_manual;
    wire parking_active=protected_forward && stop_latched;
    wire signed [23:0] parking_fresh=parking_seen && feedback_age_ms<50 ? parking_q12 : 24'sd0;
    // No positive correction into unknown space after a sonar fault.
    wire signed [23:0] parking_command=sample_healthy || parking_fresh<0 ? parking_fresh : 24'sd0;
    // Ownership changes and upstream zero/negative commands take effect without a ramp delay.
    assign forward_q12=hard_stop ? 24'sd0 : !protected_forward ? requested_forward_q12 :
        stop_latched ? parking_command : inhibit ? 24'sd0 :
        requested_forward_q12<ramp_q12 ? requested_forward_q12 : ramp_q12;
    assign yaw_q12=hard_stop ? 24'sd0 : protected_forward && inhibit ? 24'sd0 : requested_yaw_q12;
    assign braking=protected_forward && forward_q12<requested_forward_q12;
    function [23:0] distance_target;
        input [15:0] mm;
        reg [31:0] scaled;
        begin
            if(mm<=STOP_MM) distance_target=0;
            else if(mm>=SLOW_MM) distance_target=MAX_FORWARD_Q12;
            else begin
                scaled=(mm-STOP_MM)*MAX_FORWARD_Q12;
                distance_target=scaled/(SLOW_MM-STOP_MM);
            end
        end
    endfunction
    //100Hz encoder parking PI from the MuJoCo candidate, ONLY in obstacle-stop state.
    //Kp0.5, Ki0.5/s: round(-(100*feedback_mmps+sum)*1024/50000), clip +/-205Q12.
    //Shared36-step divider avoids introducing a wide combinational divider path.
    wire signed [31:0] feedback_extended={{16{feedback_forward_mmps[15]}},feedback_forward_mmps};
    wire signed [31:0] sum_extended={{17{parking_sum[14]}},parking_sum};
    wire signed [31:0] sum_trial=sum_extended+feedback_extended;
    wire signed [31:0] sum_limited=sum_trial>10000 ? 32'sd10000 :
        sum_trial < -10000 ? -32'sd10000 : sum_trial;
    wire [16:0] parking_trial={parking_remainder[15:0],parking_dividend[35]};
    wire parking_take=parking_trial>=17'd50000;
    wire [35:0] parking_next_q={parking_quotient[34:0],parking_take};
    wire [31:0] parking_magnitude=parking_error<0 ? -parking_error : parking_error;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            parking_q12<=0;parking_sum<=0;parking_error<=0;feedback_age_ms<=16'hffff;
            parking_seen<=0;parking_negative<=0;parking_phase<=0;parking_bit<=0;
            parking_dividend<=0;parking_quotient<=0;parking_remainder<=0;
        end else if(!parking_active || hard_stop) begin
            parking_q12<=0;parking_sum<=0;parking_error<=0;feedback_age_ms<=16'hffff;
            parking_seen<=0;parking_negative<=0;parking_phase<=0;parking_bit<=0;
            parking_dividend<=0;parking_quotient<=0;parking_remainder<=0;
        end else begin
            if(ms_tick && feedback_age_ms!=16'hffff) feedback_age_ms<=feedback_age_ms+1'b1;
            if(parking_seen && feedback_age_ms>=50) begin
                parking_q12<=0;parking_sum<=0;parking_seen<=0;parking_phase<=0;
            end else case(parking_phase)
                0: if(feedback_valid) begin
                    feedback_age_ms<=0;parking_sum<=sum_limited[14:0];
                    parking_error<=-(feedback_extended*32'sd100+sum_limited);
                    parking_phase<=1;
                end
                1: begin
                    parking_negative<=parking_error<0;
                    parking_dividend<=({4'd0,parking_magnitude}<<10)+36'd25000;
                    parking_remainder<=0;parking_quotient<=0;parking_bit<=35;parking_phase<=2;
                end
                2: begin
                    parking_dividend<={parking_dividend[34:0],1'b0};
                    parking_remainder<=parking_take ? parking_trial-17'd50000 : parking_trial;
                    parking_quotient<=parking_next_q;
                    if(parking_bit==0) begin
                        parking_q12<=parking_next_q>=205 ? (parking_negative ? -24'sd205 : 24'sd205) :
                            (parking_negative ? -$signed(parking_next_q[23:0]) : $signed(parking_next_q[23:0]));
                        parking_seen<=1;parking_phase<=0;
                    end else parking_bit<=parking_bit-1'b1;
                end
                default:parking_phase<=0;
            endcase
        end
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin enabled<=0;mode_key_held<=0;end
        else begin
            // Covers both short NEC repeats (no key_event) and repeated full frames.
            if(!remote_key_active || remote_key!=MODE_KEY) mode_key_held<=0;
            if(remote_key_event && remote_key==MODE_KEY && !mode_key_held) begin
                enabled<=!enabled;mode_key_held<=1;
            end
        end
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            ms_count<=0;sample_age_ms<=16'hffff;good_count<=0;clear_count<=0;
            seen_healthy<=0;range_target_q12<=0;stop_latched<=0;ramp_q12<=0;
        end else begin
            ms_count<=ms_tick ? 0 : ms_count+1'b1;
            if(ms_tick && sample_age_ms!=16'hffff) sample_age_ms<=sample_age_ms+1'b1;
            if(!enabled) begin
                sample_age_ms<=16'hffff;good_count<=0;clear_count<=0;
                seen_healthy<=0;range_target_q12<=0;stop_latched<=0;
            end else if(!forward_ready) begin
                sample_age_ms<=16'hffff;good_count<=0;clear_count<=0;range_target_q12<=0;
            end else if(sample_strobe) begin
                // An unknown/invalid/out-of-range sample is never treated as free space.
                if(sample_valid && distance_mm>=20 && distance_mm<=4000) begin
                    sample_age_ms<=0;
                    if(good_count<2) good_count<=good_count+1'b1;
                    range_target_q12<=distance_target(distance_mm);
                    if(distance_mm>=RELEASE_MM) begin
                        if(clear_count<2) clear_count<=clear_count+1'b1;
                    end else clear_count<=0;
                    if(distance_mm<=STOP_MM) stop_latched<=1;
                end else begin
                    good_count<=0;clear_count<=0;range_target_q12<=0;
                    if(seen_healthy && positive_manual) stop_latched<=1;
                end
            end
            if(sample_healthy) seen_healthy<=1;
            if(enabled && seen_healthy && positive_manual &&
               (!forward_ready || sample_age_ms>=STALE_MS)) stop_latched<=1;
            // A held forward key cannot restart after an obstacle or sensor fault.
            if(enabled && !sample_strobe && !positive_manual && sample_healthy && clear_count>=2)
                stop_latched<=0;
            // 1ms setpoint slew. Near obstacle, invalid sample and hard stop bypass it to zero.
            if(!enabled || !positive_manual || hard_stop || inhibit) ramp_q12<=0;
            else if(ms_tick) begin
                if(ramp_q12<desired_q12)
                    ramp_q12<=desired_q12-ramp_q12<=ACCEL_STEP_Q12 ? desired_q12 : ramp_q12+ACCEL_STEP_Q12;
                else if(ramp_q12>desired_q12)
                    ramp_q12<=ramp_q12-desired_q12<=DECEL_STEP_Q12 ? desired_q12 : ramp_q12-DECEL_STEP_Q12;
            end
        end
    end
endmodule
