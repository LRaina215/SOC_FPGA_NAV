`ifndef IMU_RECIP_V_INCLUDED
`define IMU_RECIP_V_INCLUDED
`timescale 1ns/1ps

// Bit-serial reciprocal: q = 2^28 / d, one quotient bit per clock.
//
// Restoring long division, 29 iterations. Exact (truncating) and trivially
// checkable against Python, which matters because a wrong reciprocal silently
// produces a wrong attitude with no error flag anywhere.
//
//   d = |accel| ~ 10920  ->  q = 24576, and accel*q>>16 gives the F12 unit
//   d = |g|     ~ 4096   ->  q = 65536, and g*q>>16 returns g unchanged
//
// Scale rationale: the fusion module normalizes signed F12 3-vectors, where
// |v| is a small integer (~4096) and v*q>>16 must land in F12. That fixes
// q = 4096*2^16/d = 2^28/d. For d >= 2^12 the quotient fits in 17 bits.
//
// The sample period is 500000 clocks, so 29 clocks here costs nothing; the
// sequential form also keeps the combinational path short enough for 50 MHz.
//
// q IS DECLARED SIGNED ON PURPOSE. The quotient 2^28/d is always positive and
// fits in 31 bits, so signedness costs nothing here -- but it matters at every
// call site. Verilog promotes a binary operation to UNSIGNED if either operand
// is unsigned, so a caller computing (signed_vector * q) would silently do an
// unsigned multiply whenever q was unsigned. For a negative vector component
// that turns -3 into 65533 and yields a wildly wrong product. Declaring q signed
// keeps every call site's arithmetic signed, which is what those expressions
// assume.
module imu_recip (
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [31:0] d,
    output reg signed [31:0] q = 0,
    output reg done = 0,
    output reg busy = 0
);
    reg [31:0] rem;
    reg [5:0] i;
    reg [31:0] next_rem;
    reg [4:0] bitpos;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 0; done <= 0; q <= 0; rem <= 0; i <= 0;
        end else begin
            done <= 0;
            if (!busy) begin
                if (start) begin
                    busy <= 1'b1;
                    q <= 32'sd0;
                    rem <= 32'd0;
                    i <= 6'd0;
                end
            end else if (i == 6'd29) begin
                busy <= 1'b0;
                done <= 1'b1;
            end else begin
                // Numerator is 2^28: its only set bit is position 28, which is
                // injected on the first iteration. Remaining iterations inject 0.
                bitpos = 5'd28 - i[4:0];
                next_rem = {rem[30:0], (i == 6'd0)};
                if (next_rem >= d) begin
                    rem <= next_rem - d;
                    q <= q | (32'sd1 << bitpos);
                end else begin
                    rem <= next_rem;
                end
                i <= i + 6'd1;
            end
        end
    end
endmodule

`endif
