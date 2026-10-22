`timescale 1ns/1ps
// Atomic sensor publication boundary for the shadow loop.
// A publication pulse is consumed together with its payload and timestamp;
// sequence counters never describe a different payload than the corresponding
// registered sample. No actuator signal is present in this module.
module sensor_snapshot #(
    parameter integer CLK_HZ=50000000,
    parameter integer MAX_IMU_AGE_TICKS=600000,
    parameter integer MAX_ENCODER_AGE_TICKS=100000
)(
    input wire clk,
    input wire rst_n,
    input wire [31:0] ticks,
    input wire imu_valid,
    input wire imu_ready,
    input wire imu_error,
    input wire signed [15:0] accel_x,
    input wire signed [15:0] accel_y,
    input wire signed [15:0] accel_z,
    input wire signed [15:0] gyro_x,
    input wire signed [15:0] gyro_y,
    input wire signed [15:0] gyro_z,
    input wire left_valid,
    input wire left_ready,
    input wire left_error,
    input wire [13:0] left_angle,
    input wire [3:0] left_status,
    input wire [3:0] left_error_code,
    input wire right_valid,
    input wire right_ready,
    input wire right_error,
    input wire [13:0] right_angle,
    input wire [3:0] right_status,
    input wire [3:0] right_error_code,
    output reg [31:0] imu_sequence,
    output reg [31:0] imu_ticks,
    output reg [31:0] left_sequence,
    output reg [31:0] left_ticks,
    output reg [31:0] right_sequence,
    output reg [31:0] right_ticks,
    output reg signed [15:0] accel_x_q,
    output reg signed [15:0] accel_y_q,
    output reg signed [15:0] accel_z_q,
    output reg signed [15:0] gyro_x_q,
    output reg signed [15:0] gyro_y_q,
    output reg signed [15:0] gyro_z_q,
    output reg [13:0] left_angle_q,
    output reg [13:0] right_angle_q,
    output reg [3:0] left_status_q,
    output reg [3:0] right_status_q,
    output reg [3:0] left_error_code_q,
    output reg [3:0] right_error_code_q,
    output reg imu_seen,
    output reg left_seen,
    output reg right_seen,
    output wire snapshot_event,
    output wire snapshot_usable,
    output wire [31:0] imu_age_ticks,
    output wire [31:0] left_age_ticks,
    output wire [31:0] right_age_ticks
);
    wire accept_imu = imu_valid && imu_ready && !imu_error;
    wire accept_left = left_valid && left_ready && !left_error;
    wire accept_right = right_valid && right_ready && !right_error;
    assign snapshot_event = accept_imu || accept_left || accept_right;
    assign imu_age_ticks = ticks - imu_ticks;
    assign left_age_ticks = ticks - left_ticks;
    assign right_age_ticks = ticks - right_ticks;
    assign snapshot_usable = imu_seen && left_seen && right_seen &&
        imu_ready && !imu_error && left_ready && !left_error &&
        right_ready && !right_error &&
        (imu_age_ticks <= MAX_IMU_AGE_TICKS) &&
        (left_age_ticks <= MAX_ENCODER_AGE_TICKS) &&
        (right_age_ticks <= MAX_ENCODER_AGE_TICKS);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            imu_sequence <= 0; imu_ticks <= 0; left_sequence <= 0; left_ticks <= 0;
            right_sequence <= 0; right_ticks <= 0;
            accel_x_q <= 0; accel_y_q <= 0; accel_z_q <= 0;
            gyro_x_q <= 0; gyro_y_q <= 0; gyro_z_q <= 0;
            left_angle_q <= 0; right_angle_q <= 0;
            left_status_q <= 0; right_status_q <= 0;
            left_error_code_q <= 0; right_error_code_q <= 0;
            imu_seen <= 0; left_seen <= 0; right_seen <= 0;
        end else begin
            if (accept_imu) begin
                accel_x_q <= accel_x; accel_y_q <= accel_y; accel_z_q <= accel_z;
                gyro_x_q <= gyro_x; gyro_y_q <= gyro_y; gyro_z_q <= gyro_z;
                imu_ticks <= ticks; imu_sequence <= imu_sequence + 1'b1; imu_seen <= 1'b1;
            end
            if (accept_left) begin
                left_angle_q <= left_angle; left_status_q <= left_status;
                left_error_code_q <= left_error_code; left_ticks <= ticks;
                left_sequence <= left_sequence + 1'b1; left_seen <= 1'b1;
            end
            if (accept_right) begin
                right_angle_q <= right_angle; right_status_q <= right_status;
                right_error_code_q <= right_error_code; right_ticks <= ticks;
                right_sequence <= right_sequence + 1'b1; right_seen <= 1'b1;
            end
        end
    end
endmodule
