// 同步 FIFO，valid/ready 接口，首字直通。存储推断为 BSRAM。
// 深度 = 2^AW（RAM 部分）+ 最多 2 项输出缓冲。
module nav_fifo #(
    parameter integer DW = 8,
    parameter integer AW = 9
) (
    input  wire          clk,
    input  wire          rst,
    input  wire          s_valid,
    input  wire [DW-1:0] s_data,
    output wire          s_ready,
    output wire          m_valid,
    output wire [DW-1:0] m_data,
    input  wire          m_ready,
    output wire [AW+1:0] level,     // 总字数
    output wire [AW:0]   free       // RAM 剩余空间
);
    reg  [AW:0] wp, rp;
    wire [AW:0] used = wp - rp;
    wire        full = used[AW];
    wire        wr   = s_valid & ~full;
    wire        rd;
    wire [DW-1:0] q;
    wire [1:0]  held;

    assign s_ready = ~full;
    assign level   = {1'b0, used} + {{AW{1'b0}}, held};
    assign free    = (1 << AW) - used;

    always @(posedge clk) begin
        if (rst) begin
            wp <= 0;
            rp <= 0;
        end else begin
            if (wr) wp <= wp + 1'b1;
            if (rd) rp <= rp + 1'b1;
        end
    end

    nav_ram_sdp #(.DW(DW), .AW(AW)) u_ram (
        .clk(clk), .we(wr), .waddr(wp[AW-1:0]), .wdata(s_data),
        .re(rd), .raddr(rp[AW-1:0]), .rdata(q)
    );

    nav_fifo_ob #(.DW(DW)) u_ob (
        .clk(clk), .rst(rst), .ram_avail(used != 0), .ram_rd(rd), .ram_q(q),
        .m_valid(m_valid), .m_data(m_data), .m_ready(m_ready), .held(held)
    );
endmodule
