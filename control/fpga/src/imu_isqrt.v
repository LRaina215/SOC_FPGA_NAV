`ifndef IMU_ISQRT_V_INCLUDED
`define IMU_ISQRT_V_INCLUDED
`timescale 1ns/1ps

// Sequential integer square root, one quotient bit per clock, 32 iterations.
//
// Sequential rather than unrolled because a 32-deep combinational 64-bit
// compare/subtract chain does not close timing at 50 MHz, and the sample period
// is 500000 clocks so 32 clocks here costs nothing.
//
// Hold start for one clock; busy rises, then done pulses once with root valid.
// Matches floor(sqrt(value)) exactly, which is what makes it checkable against
// a Python model.
//
// The walking variable is named bitv, not bit: "bit" is a SystemVerilog keyword
// and -g2012 rejects it as a plain identifier.
module imu_isqrt (
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [63:0] value,
    output reg [31:0] root = 0,
    output reg done = 0,
    output reg busy = 0
);
    reg [63:0] rem, bitv, acc;
    reg [5:0] cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 0; done <= 0; cnt <= 0;
            rem <= 0; bitv <= 0; acc <= 0; root <= 0;
        end else begin
            done <= 0;
            if (!busy) begin
                if (start) begin
                    busy <= 1'b1;
                    cnt <= 6'd0;
                    acc <= 64'd0;
                    rem <= value;
                    bitv <= 64'd1 << 62;
                end
            end else if (cnt == 6'd32) begin
                // 32 iterations done; bitv has walked 2^62 down to 2^0.
                busy <= 1'b0;
                done <= 1'b1;
                root <= acc[31:0];
            end else begin
                if (rem >= (acc + bitv)) begin
                    rem <= rem - (acc + bitv);
                    acc <= (acc >> 1) + bitv;
                end else begin
                    acc <= acc >> 1;
                end
                bitv <= bitv >> 2;
                cnt <= cnt + 6'd1;
            end
        end
    end
endmodule

`endif
