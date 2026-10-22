`timescale 1ns/1ps
// Sensor-to-policy adapter for the actuator-disabled shadow loop.
//
// This block owns only observation formatting and encoder differentiation. It
// never emits actuator signals. The acceleration mapping below is the explicit
// calibrated body-axis mapping used by the current integration (identity axis
// basis, 1 g ~= 10920 LSB); gravity-estimator replacement must be validated
// against runtime/observations.py before any armable design is considered.
module observation_builder #(
    parameter integer CLK_HZ=50000000,
    parameter integer ENCODER_TICKS_PER_REV=16384
)(
    input wire clk,
    input wire rst_n,
    input wire capture,
    input wire [31:0] now_ticks,
    input wire sensor_valid,
    input wire [31:0] sample_sequence,
    input wire [31:0] sample_ticks,
    input wire signed [15:0] accel_x,
    input wire signed [15:0] accel_y,
    input wire signed [15:0] accel_z,
    input wire signed [15:0] gyro_x,
    input wire signed [15:0] gyro_y,
    input wire signed [15:0] gyro_z,
    input wire [13:0] left_angle,
    input wire [13:0] right_angle,
    input wire [31:0] left_ticks,
    input wire [31:0] right_ticks,
    output reg frame_valid,
    output reg [31:0] frame_sequence,
    output reg [31:0] frame_age_ticks,
    output reg signed [27*24-1:0] current_flat,
    output reg signed [135*24-1:0] history_flat,
    output reg signed [23:0] left_velocity_q12,
    output reg signed [23:0] right_velocity_q12
);
    integer i;
    reg have_angle;
    reg [13:0] prev_left_angle, prev_right_angle;
    reg [31:0] prev_left_ticks, prev_right_ticks;
    reg [31:0] prev_sequence;
    reg signed [23:0] next_words [0:26];
    reg signed [31:0] left_delta, right_delta;
    reg signed [31:0] left_dt, right_dt;
    reg signed [63:0] left_velocity_calc, right_velocity_calc;
    reg signed [63:0] gyro_calc;
    reg signed [63:0] gravity_calc;

    function signed [23:0] clamp24;
        input signed [63:0] value;
        begin
            if (value > 64'sd8388607) clamp24 = 24'sh7fffff;
            else if (value < -64'sd8388608) clamp24 = -24'sh800000;
            else clamp24 = value[23:0];
        end
    endfunction

    function signed [23:0] clip_observation;
        input signed [23:0] value;
        begin
            if (value > 24'sd409600) clip_observation = 24'sd409600;
            else if (value < -24'sd409600) clip_observation = -24'sd409600;
            else clip_observation = value;
        end
    endfunction

    function signed [31:0] unwrap14;
        input [13:0] now_angle;
        input [13:0] old_angle;
        reg signed [15:0] difference;
        begin
            difference = $signed({1'b0, now_angle}) - $signed({1'b0, old_angle});
            if (difference > 16'sd8191) unwrap14 = difference - 16'sd16384;
            else if (difference < -16'sd8192) unwrap14 = difference + 16'sd16384;
            else unwrap14 = difference;
        end
    endfunction

    always @* begin
        left_delta = unwrap14(left_angle, prev_left_angle);
        right_delta = unwrap14(right_angle, prev_right_angle);
        left_dt = $signed(left_ticks - prev_left_ticks);
        right_dt = $signed(right_ticks - prev_right_ticks);
        left_velocity_calc = 64'sd0;
        right_velocity_calc = 64'sd0;
        // q12 wheel velocity = (delta angle / 16384) * 2pi / dt * 0.15 * 4096.
        // dt is in 50 MHz clock ticks; 2pi*0.15*4096*50e6 ~= 193019000000.
        if (have_angle && left_dt > 0 && left_dt <= CLK_HZ/10)
            left_velocity_calc = ($signed(left_delta) * 64'sd193019000000) /
                                 ($signed(ENCODER_TICKS_PER_REV) * left_dt);
        if (have_angle && right_dt > 0 && right_dt <= CLK_HZ/10)
            right_velocity_calc = ($signed(right_delta) * 64'sd193019000000) /
                                  ($signed(ENCODER_TICKS_PER_REV) * right_dt);

        // Gyro: raw LSB -> rad/s, then policy scale 1.3 and F12.
        next_words[0] = clamp24(($signed(gyro_x) * 64'sd5672) / 64'sd1000);
        next_words[1] = clamp24(($signed(gyro_y) * 64'sd5672) / 64'sd1000);
        next_words[2] = clamp24(($signed(gyro_z) * 64'sd5672) / 64'sd1000);
        // Gravity direction: -accel normalized to the calibrated 1 g scale.
        next_words[3] = clamp24(-($signed(accel_x) * 64'sd12288) / 64'sd10920);
        next_words[4] = clamp24(-($signed(accel_y) * 64'sd12288) / 64'sd10920);
        next_words[5] = clamp24(-($signed(accel_z) * 64'sd12288) / 64'sd10920);
        // Commands are zero in the disarmed shadow loop.
        next_words[6] = 0;
        next_words[7] = 0;
        next_words[8] = 0;
        // Previous executed action is zero while all actuator outputs are hard-disabled.
        for (i=9; i<15; i=i+1) next_words[i] = 0;
        next_words[15] = 0;
        next_words[16] = 0;
        next_words[17] = clamp24(left_velocity_calc);
        next_words[18] = 0;
        next_words[19] = 0;
        next_words[20] = clamp24(right_velocity_calc);
        for (i=21; i<27; i=i+1) next_words[i] = 0;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_valid <= 1'b0;
            frame_sequence <= 32'd0;
            frame_age_ticks <= 32'd0;
            have_angle <= 1'b0;
            prev_left_angle <= 14'd0;
            prev_right_angle <= 14'd0;
            prev_left_ticks <= 32'd0;
            prev_right_ticks <= 32'd0;
            prev_sequence <= 32'd0;
            left_velocity_q12 <= 24'sd0;
            right_velocity_q12 <= 24'sd0;
            current_flat <= {(27*24){1'b0}};
            history_flat <= {(135*24){1'b0}};
        end else begin
            frame_valid <= 1'b0;
            if (capture && sensor_valid && (sample_sequence != prev_sequence)) begin
                frame_valid <= 1'b1;
                frame_sequence <= sample_sequence;
                frame_age_ticks <= now_ticks - sample_ticks;
                prev_sequence <= sample_sequence;
                left_velocity_q12 <= clamp24(left_velocity_calc);
                right_velocity_q12 <= clamp24(right_velocity_calc);
                if (!have_angle) have_angle <= 1'b1;
                prev_left_angle <= left_angle;
                prev_right_angle <= right_angle;
                prev_left_ticks <= left_ticks;
                prev_right_ticks <= right_ticks;
                for (i=0; i<27; i=i+1) begin
                    current_flat[i*24 +: 24] <= clip_observation(next_words[i]);
                end
                for (i=0; i<108; i=i+1) begin
                    history_flat[i*24 +: 24] <= history_flat[(i+27)*24 +: 24];
                end
                for (i=0; i<27; i=i+1) begin
                    history_flat[(108+i)*24 +: 24] <= next_words[i];
                end
            end
        end
    end
endmodule
