// 流水线寄存器片（skid buffer）：切断 valid/data 与 ready 的组合路径，满吞吐。
module nav_skid #(
    parameter integer DW = 8
) (
    input  wire          clk,
    input  wire          rst,
    input  wire          s_valid,
    input  wire [DW-1:0] s_data,
    output wire          s_ready,
    output reg           m_valid,
    output reg  [DW-1:0] m_data,
    input  wire          m_ready
);
    reg          sk_v;
    reg [DW-1:0] sk_d;
    assign s_ready = ~sk_v;

    always @(posedge clk) begin
        if (rst) begin
            m_valid <= 1'b0;
            sk_v    <= 1'b0;
        end else begin
            if (!m_valid || m_ready) begin
                if (sk_v) begin
                    m_valid <= 1'b1;
                    m_data  <= sk_d;
                    sk_v    <= 1'b0;
                end else begin
                    m_valid <= s_valid;
                    m_data  <= s_data;
                end
            end else if (s_valid && s_ready) begin
                sk_v <= 1'b1;
                sk_d <= s_data;
            end
        end
    end
endmodule
