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
