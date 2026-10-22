`timescale 1ns/1ps

// F24 accumulator to F12, nearest with exact half cases away from zero.
module fc_v4_round_sat (
    input wire signed [47:0] acc,
    output reg signed [23:0] value,
    output reg saturated
);
    reg signed [48:0] extended_acc;
    reg signed [48:0] rounded;
    always @* begin
        extended_acc = {acc[47], acc};
        if (extended_acc < 0)
            rounded = -(((-extended_acc) + 49'sd2048) >>> 12);
        else
            rounded = (extended_acc + 49'sd2048) >>> 12;
        saturated = 1'b0;
        if (rounded > 49'sd8388607) begin
            value = 24'sd8388607;
            saturated = 1'b1;
        end else if (rounded < -49'sd8388608) begin
            value = -24'sd8388608;
            saturated = 1'b1;
        end else begin
            value = rounded[23:0];
        end
    end
endmodule
