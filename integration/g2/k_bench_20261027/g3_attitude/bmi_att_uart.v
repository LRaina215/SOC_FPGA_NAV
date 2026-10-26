`timescale 1ns/1ps
// BMI088 fusion result -> navigation IMU input, as an internal 8N1 byte stream.
// Each fusion publication becomes two 11-byte frames in the WIT-style framing
// that fw/nav/imu_parse.c already understands (0x55, type, 4 x int16 LE, sum):
//   0x5B: gravity x/y/z (control body axes, Q12), sample sequence
//   0x5C: angular velocity x/y/z (control body axes, Q12 >>> 3 = Q9 rad/s), flags
// Axis mapping to the navigation frame is left to firmware so it can be fixed
// without a rebuild. A publication arriving while frames are still being sent
// is dropped; at 115200 baud the pair takes about 1.9 ms of each 10 ms period.
module bmi_att_uart #(
    parameter integer DIV = 434   // clk / baud; must equal the nav IMU.BAUD_DIV register
) (
    input  wire clk,
    input  wire rst,
    input  wire valid,
    input  wire signed [23:0] gx, gy, gz,
    input  wire signed [23:0] wx, wy, wz,
    output reg  txd = 1'b1,
    output reg  [15:0] seq = 16'd0,
    output reg  [15:0] dropped = 16'd0
);
    function [15:0] sat16;
        input signed [23:0] v;
        begin
            if (v > 24'sd32767) sat16 = 16'h7FFF;
            else if (v < -24'sd32768) sat16 = 16'h8000;
            else sat16 = v[15:0];
        end
    endfunction

    reg [7:0] frame [0:21];
    reg [4:0] idx = 5'd0;
    reg [3:0] bit_i = 4'd0;
    reg [15:0] bc = 16'd0;
    reg [9:0] sh = 10'h3FF;
    reg busy = 1'b0;
    reg load = 1'b0;
    reg [15:0] lg_x, lg_y, lg_z, lw_x, lw_y, lw_z, lseq;

    always @(posedge clk) begin
        if (rst) begin
            busy <= 1'b0; load <= 1'b0; txd <= 1'b1; idx <= 5'd0; bit_i <= 4'd0; bc <= 16'd0;
            seq <= 16'd0; dropped <= 16'd0;
        end else begin
            load <= 1'b0;
            if (valid) begin
                if (busy || load) dropped <= dropped + 1'b1;
                else begin
                    lg_x <= sat16(gx); lg_y <= sat16(gy); lg_z <= sat16(gz);
                    lw_x <= sat16(wx >>> 3); lw_y <= sat16(wy >>> 3); lw_z <= sat16(wz >>> 3);
                    lseq <= seq; seq <= seq + 1'b1;
                    load <= 1'b1;
                end
            end
            if (load) begin
                frame[0] <= 8'h55; frame[1] <= 8'h5B;
                frame[2] <= lg_x[7:0]; frame[3] <= lg_x[15:8];
                frame[4] <= lg_y[7:0]; frame[5] <= lg_y[15:8];
                frame[6] <= lg_z[7:0]; frame[7] <= lg_z[15:8];
                frame[8] <= lseq[7:0]; frame[9] <= lseq[15:8];
                frame[10] <= 8'h55 + 8'h5B + lg_x[7:0] + lg_x[15:8] + lg_y[7:0] + lg_y[15:8]
                           + lg_z[7:0] + lg_z[15:8] + lseq[7:0] + lseq[15:8];
                frame[11] <= 8'h55; frame[12] <= 8'h5C;
                frame[13] <= lw_x[7:0]; frame[14] <= lw_x[15:8];
                frame[15] <= lw_y[7:0]; frame[16] <= lw_y[15:8];
                frame[17] <= lw_z[7:0]; frame[18] <= lw_z[15:8];
                frame[19] <= 8'h00; frame[20] <= 8'h00;
                frame[21] <= 8'h55 + 8'h5C + lw_x[7:0] + lw_x[15:8] + lw_y[7:0] + lw_y[15:8]
                           + lw_z[7:0] + lw_z[15:8];
                busy <= 1'b1; idx <= 5'd0; bit_i <= 4'd0; bc <= 16'd0;
            end else if (busy) begin
                if (bc != 0) bc <= bc - 1'b1;
                else begin
                    bc <= DIV - 1;
                    if (bit_i == 4'd0) begin
                        sh <= {1'b1, frame[idx], 1'b0};
                        txd <= 1'b0;
                        bit_i <= 4'd1;
                    end else if (bit_i < 4'd10) begin
                        txd <= sh[bit_i];
                        bit_i <= bit_i + 1'b1;
                    end else begin
                        bit_i <= 4'd0;
                        if (idx == 5'd21) begin busy <= 1'b0; txd <= 1'b1; end
                        else idx <= idx + 1'b1;
                    end
                end
            end
        end
    end
endmodule
