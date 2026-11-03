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
    wire positive_manual=manual_owner && requested_forward_q12>0;
    assign sample_healthy=enabled && forward_ready && good_count>=2 && sample_age_ms<STALE_MS;
    wire inhibit=!sample_healthy || stop_latched;
    wire signed [23:0] desired_q12=requested_forward_q12<$signed(range_target_q12) ?
        requested_forward_q12 : $signed(range_target_q12);
    wire protected_forward=enabled && positive_manual;
    // Ownership changes and upstream zero/negative commands take effect without a ramp delay.
    assign forward_q12=hard_stop ? 24'sd0 : !protected_forward ? requested_forward_q12 :
        inhibit ? 24'sd0 : requested_forward_q12<ramp_q12 ? requested_forward_q12 : ramp_q12;
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
