// 异步 FIFO（跨时钟域），格雷码指针 + 两级同步，读侧首字直通。
// 写侧 wfree 为保守估计（读指针同步滞后），可用于整帧准入判断。
module nav_async_fifo #(
    parameter integer DW = 10,
    parameter integer AW = 11
) (
    input  wire          wclk,
    input  wire          wrst,
    input  wire          s_valid,
    input  wire [DW-1:0] s_data,
    output wire          s_ready,
    output wire [AW:0]   wfree,

    input  wire          rclk,
    input  wire          rrst,
    output wire          m_valid,
    output wire [DW-1:0] m_data,
    input  wire          m_ready
);
    function [AW:0] b2g(input [AW:0] b); b2g = b ^ (b >> 1); endfunction
    function [AW:0] g2b(input [AW:0] g);
        integer k;
        begin
            g2b[AW] = g[AW];
            for (k = AW - 1; k >= 0; k = k - 1) g2b[k] = g2b[k+1] ^ g[k];
        end
    endfunction

    reg [AW:0] rp_b, rp_g;
    reg [AW:0] wp_b, wp_g;

    // ---------------- 写侧 ----------------
    (* syn_keep = 1 *) reg [AW:0] rp_g_w1, rp_g_w2;
    reg  [AW:0] rp_b_w;             // 读指针（写域，多滞后一拍，满判断更保守）
    always @(posedge wclk) rp_b_w <= wrst ? {AW+1{1'b0}} : g2b(rp_g_w2);
    wire [AW:0] wused  = wp_b - rp_b_w;
    wire        wfull  = wused[AW];
    wire        wr     = s_valid & ~wfull;
    assign s_ready = ~wfull;
    assign wfree   = (1 << AW) - wused;

    always @(posedge wclk) begin
        if (wrst) begin
            wp_b <= 0; wp_g <= 0; rp_g_w1 <= 0; rp_g_w2 <= 0;
        end else begin
            rp_g_w1 <= rp_g; rp_g_w2 <= rp_g_w1;
            if (wr) begin
                wp_b <= wp_b + 1'b1;
                wp_g <= b2g(wp_b + 1'b1);
            end
        end
    end

    reg [DW-1:0] mem [0:(1<<AW)-1];
    always @(posedge wclk) if (wr) mem[wp_b[AW-1:0]] <= s_data;

    // ---------------- 读侧 ----------------
    (* syn_keep = 1 *) reg [AW:0] wp_g_r1, wp_g_r2;
    reg  [AW:0] wp_b_r;
    always @(posedge rclk) wp_b_r <= rrst ? {AW+1{1'b0}} : g2b(wp_g_r2);
    wire        ravail = (wp_b_r != rp_b);
    wire        rd;
    reg  [DW-1:0] q;

    always @(posedge rclk) begin
        if (rrst) begin
            rp_b <= 0; rp_g <= 0; wp_g_r1 <= 0; wp_g_r2 <= 0;
        end else begin
            wp_g_r1 <= wp_g; wp_g_r2 <= wp_g_r1;
            if (rd) begin
                rp_b <= rp_b + 1'b1;
                rp_g <= b2g(rp_b + 1'b1);
            end
        end
    end
    always @(posedge rclk) if (rd) q <= mem[rp_b[AW-1:0]];

    nav_fifo_ob #(.DW(DW)) u_ob (
        .clk(rclk), .rst(rrst), .ram_avail(ravail), .ram_rd(rd), .ram_q(q),
        .m_valid(m_valid), .m_data(m_data), .m_ready(m_ready), .held()
    );
endmodule
