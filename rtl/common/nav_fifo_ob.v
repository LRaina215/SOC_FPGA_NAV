// FIFO 读侧的公共部分：把"同步读 RAM（延迟 1 拍）"变成首字直通（FWFT）的 valid/ready 流。
// 输出端带 2 项缓冲，满吞吐（每拍 1 字）。
//   ram_avail : RAM 中尚有未读出的字
//   ram_rd    : 本拍向 RAM 发读（调用方据此推进读指针）
//   ram_q     : RAM 读数据（ram_rd 的下一拍有效）
module nav_fifo_ob #(
    parameter integer DW = 8
) (
    input  wire          clk,
    input  wire          rst,
    input  wire          ram_avail,
    output wire          ram_rd,
    input  wire [DW-1:0] ram_q,
    output wire          m_valid,
    output wire [DW-1:0] m_data,
    input  wire          m_ready,
    output wire [1:0]    held      // 输出缓冲与在途读的字数之和（0..2）
);
    reg          pend;
    reg [DW-1:0] b0, b1;
    reg [1:0]    cnt;       // 缓冲中的字数

    wire pop  = m_valid & m_ready;
    // 在途 + 已缓冲 - 本拍弹出 < 2 时可以再发一个读
    wire [2:0] occ = {1'b0, cnt} + {2'b0, pend} - {2'b0, pop};
    assign ram_rd  = ram_avail & (occ < 3'd2);
    assign m_valid = (cnt != 2'd0);
    assign m_data  = b0;
    assign held    = cnt + {1'b0, pend};

    always @(posedge clk) begin
        if (rst) begin
            pend <= 1'b0;
            cnt  <= 2'd0;
        end else begin
            pend <= ram_rd;
            case ({pend, pop})
                2'b10: begin                      // 只有到达
                    if (cnt == 2'd0) b0 <= ram_q; else b1 <= ram_q;
                    cnt <= cnt + 2'd1;
                end
                2'b01: begin                      // 只有弹出
                    b0  <= b1;
                    cnt <= cnt - 2'd1;
                end
                2'b11: begin                      // 同时到达与弹出
                    if (cnt == 2'd1) b0 <= ram_q;
                    else begin b0 <= b1; b1 <= ram_q; end
                end
                default: ;
            endcase
        end
    end
endmodule
