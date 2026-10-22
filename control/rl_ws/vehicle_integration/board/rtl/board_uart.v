`timescale 1ns/1ps
// 8N1 receiver. Only this module samples the asynchronous pin.
module board_uart_rx #(
    parameter integer CLKS_PER_BIT = 434
)(
    input wire clk, input wire rst_n, input wire rxd,
    output reg [7:0] data, output reg valid, output reg framing_error
);
    reg sync1, sync2;
    reg [2:0] state;
    reg [31:0] count;
    reg [2:0] bit_index;
    reg [7:0] shift;
    localparam IDLE=0, START=1, DATA=2, STOP=3, RECOVER=4;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin sync1<=1; sync2<=1; end
        else begin sync1<=rxd; sync2<=sync1; end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=IDLE; count<=0; bit_index<=0; shift<=0;
            data<=0; valid<=0; framing_error<=0;
        end else begin
            valid<=0; framing_error<=0;
            case (state)
                IDLE: if (!sync2) begin state<=START; count<=(CLKS_PER_BIT/2)-1; end
                START: if (count!=0) count<=count-1;
                    else if (sync2) state<=IDLE; // Reject a false start.
                    else begin state<=DATA; count<=CLKS_PER_BIT-1; bit_index<=0; end
                DATA: if (count!=0) count<=count-1;
                    else begin
                        shift[bit_index]<=sync2; count<=CLKS_PER_BIT-1;
                        if (bit_index==7) state<=STOP;
                        else bit_index<=bit_index+1'b1;
                    end
                STOP: if (count!=0) count<=count-1;
                    else if (sync2) begin data<=shift; valid<=1; state<=IDLE; end
                    else begin framing_error<=1; state<=RECOVER; count<=CLKS_PER_BIT-1; end
                // Wait for an entire high bit after a break/bad stop bit.
                RECOVER: if (!sync2) count<=CLKS_PER_BIT-1;
                    else if (count!=0) count<=count-1;
                    else state<=IDLE;
                default: state<=IDLE;
            endcase
        end
    end
endmodule

// A byte is accepted on valid && ready. Busy includes the complete stop bit.
module board_uart_tx #(
    parameter integer CLKS_PER_BIT = 434
)(
    input wire clk, input wire rst_n, input wire [7:0] data,
    input wire valid, output wire ready, output reg txd, output reg busy
);
    reg [9:0] shift;
    reg [3:0] bit_index;
    reg [31:0] count;
    assign ready=!busy;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin txd<=1; busy<=0; count<=0; shift<=10'h3ff; bit_index<=0; end
        else if (!busy) begin
            txd<=1;
            if (valid) begin
                shift<={1'b1,data,1'b0}; txd<=0; busy<=1;
                bit_index<=0; count<=CLKS_PER_BIT-1;
            end
        end else if (count!=0) count<=count-1;
        else if (bit_index==9) begin busy<=0; txd<=1; end
        else begin
            shift<={1'b1,shift[9:1]}; txd<=shift[1];
            bit_index<=bit_index+1'b1; count<=CLKS_PER_BIT-1;
        end
    end
endmodule
