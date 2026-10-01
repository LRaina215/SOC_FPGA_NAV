// 主机报文邮箱：
//   接收：主机口收到的 UDP 负载按报文存入环形缓冲（坏报文回退、空间不足整包丢弃），
//         软核按报文读取：RX_STAT 看当前报文，RX_DATA 每次取 4 字节（小端），RX_POP 切下一报文。
//   发送：软核逐字写 TX_DATA，写 TX_LEN 后 TX_GO，由 net_port 组帧发出。
//   串口注入：PC 经 uart_bus_bridge 逐字写 INJ_DATA、写 INJ_LEN 后 INJ_GO，报文按与网口相同的方式入队
//         （网口报文不在途时开始；注入期间开始的网口报文整包丢弃并计数）——导航不依赖网口/RK3588。
//   发送回读：发送缓冲的影子副本可按字读回（txb_*），tx_seq 为已发送报文数，PC 经串口取状态/应答用。
// 报文内容（目标点、目标坐标、状态）的格式由固件定义，见 docs/2_interface/icd.md。
module host_mbx #(
    parameter integer RX_AW  = 12,     // 接收缓冲字节地址位宽
    parameter integer DQ_AW  = 4,      // 描述符队列深度 2^DQ_AW
    parameter integer TX_WAW = 9       // 发送缓冲字地址位宽（2^TX_WAW 个 32 位字）
) (
    input  wire        clk,
    input  wire        rst,

    // 来自 net_port 的负载
    input  wire        e_valid,
    input  wire [7:0]  e_data,
    input  wire        e_last,
    input  wire        e_err,
    input  wire [31:0] e_src_ip,
    input  wire [15:0] e_src_port,
    input  wire [15:0] e_len,
    input  wire [31:0] e_ts,

    // 串口注入（软核总线）
    input  wire        inj_we,
    input  wire [31:0] inj_wdata,
    input  wire        inj_go,
    input  wire [15:0] inj_len,
    input  wire [31:0] now_us,
    // 发送回读
    input  wire        txb_re,
    input  wire [TX_WAW-1:0] txb_addr,
    output wire [31:0] txb_q,
    output reg  [31:0] tx_seq,

    // 软核接口（接收）
    output wire        rx_avail,
    output wire [15:0] rx_len,
    output wire [7:0]  rx_pkts,
    output wire [31:0] rx_ts,
    output wire [31:0] rx_src_ip,
    output wire [15:0] rx_src_port,
    input  wire        rx_rd,           // 单拍：取下 4 字节
    output reg         rx_rd_ack,       // 单拍：rx_word 有效
    output reg  [31:0] rx_word,
    input  wire        rx_pop,          // 单拍
    output reg         rx_irq,          // 单拍：新报文入队
    output reg  [31:0] st_drop,

    // 软核接口（发送）
    input  wire        tx_we,
    input  wire [31:0] tx_wdata,
    input  wire        tx_go_in,
    input  wire [15:0] tx_len_in,
    output wire [TX_WAW:0] tx_words,

    // 到 net_port
    output reg         tx_go,
    output reg  [15:0] tx_len,
    input  wire        tx_busy,
    input  wire                pl_re,
    input  wire [TX_WAW+1:0]   pl_addr,
    output wire [7:0]          pl_q
);
    localparam integer RXSZ = 1 << RX_AW;

    // ================================================================ 串口注入：字缓冲 -> 字节流，与网口流合并
    reg  [TX_WAW:0] iwp;                // 注入缓冲写指针（字）
    reg             ig_pend, ia;        // 待注入 / 发读中
    reg  [15:0]     ilen, ioff;
    reg  [31:0]     its;
    reg             e_pkt, e_drop;      // 网口报文在途 / 丢弃中
    reg             i_re;
    reg  [TX_WAW-1:0] i_ra;
    wire [31:0]     iq;
    reg             iv1, iv2, il1, il2; // 读流水：发读 -> RAM 采样 -> 数据
    reg  [1:0]      ib1, ib2;
    nav_ram_sdp #(.DW(32), .AW(TX_WAW)) u_injb (
        .clk(clk), .we(inj_we & ~iwp[TX_WAW]), .waddr(iwp[TX_WAW-1:0]), .wdata(inj_wdata),
        .re(i_re), .raddr(i_ra), .rdata(iq)
    );
    wire ibusy   = ia | iv1 | iv2;
    wire e_first = e_valid & ~e_pkt;
    wire e_pass  = e_valid & ~(e_first ? ibusy : e_drop);
    always @(posedge clk) begin
        i_re <= 1'b0; iv1 <= 1'b0;
        iv2 <= iv1; ib2 <= ib1; il2 <= il1;
        if (rst) begin
            iwp <= 0; ig_pend <= 1'b0; ia <= 1'b0; e_pkt <= 1'b0; e_drop <= 1'b0; iv2 <= 1'b0;
        end else begin
            if (inj_we && !iwp[TX_WAW]) iwp <= iwp + 1'b1;
            if (inj_go) begin ig_pend <= 1'b1; ilen <= inj_len; its <= now_us; end
            if (e_valid) begin
                e_pkt <= ~e_last;
                if (e_first) e_drop <= ibusy;
            end
            // 网口报文不在途时开始注入
            if (ig_pend && !ibusy && !e_pkt && !e_valid) begin
                ig_pend <= 1'b0;
                ioff <= 0;
                if (ilen == 0) iwp <= 0; else ia <= 1'b1;
            end
            if (ia) begin
                i_re <= 1'b1; i_ra <= ioff[TX_WAW+1:2];
                iv1 <= 1'b1; ib1 <= ioff[1:0]; il1 <= (ioff + 1'b1 >= ilen);
                ioff <= ioff + 1'b1;
                if (ioff + 1'b1 >= ilen) begin ia <= 1'b0; iwp <= 0; end
            end
        end
    end
    // 合并后的流寄存一拍（切断 选择器 -> 空间判断 -> 写使能 的长路径）
    reg         u_valid, u_last, u_err;
    reg  [7:0]  u_data;
    reg  [31:0] u_src_ip, u_ts;
    reg  [15:0] u_src_port, u_len;
    always @(posedge clk) begin
        u_valid    <= ~rst & (iv2 | e_pass);
        u_data     <= iv2 ? iq[8*ib2 +: 8] : e_data;
        u_last     <= iv2 ? il2 : e_last;
        u_err      <= iv2 ? 1'b0 : e_err;
        u_src_ip   <= iv2 ? 32'd0 : e_src_ip;
        u_src_port <= iv2 ? 16'd0 : e_src_port;
        u_len      <= iv2 ? ilen : e_len;
        u_ts       <= iv2 ? its : e_ts;
    end

    // ================================================================ 接收写侧
    reg [RX_AW:0] wp_c, wp_t, rp;       // 已提交写指针、临时写指针、读基址
    reg           in_pkt, drop_pkt;
    wire [RX_AW:0] used = wp_c - rp;
    wire [RX_AW:0] free = RXSZ - used;

    wire        dq_s_ready, dq_m_valid;
    wire [95:0] dq_m;
    wire [DQ_AW+1:0] dq_level;
    reg         dq_push;
    reg [95:0]  dq_in;

    wire first = u_valid & ~in_pkt;
    wire fits  = ({1'b0, u_len} <= free) && dq_s_ready;
    wire keep  = first ? fits : ~drop_pkt;
    wire wr    = u_valid & keep;

    always @(posedge clk) begin
        dq_push <= 1'b0; rx_irq <= 1'b0;
        if (rst) begin
            wp_c <= 0; wp_t <= 0; in_pkt <= 1'b0; drop_pkt <= 1'b0; st_drop <= 0;
        end else if (u_valid) begin
            if (first) begin
                drop_pkt <= ~fits;
                if (!fits) st_drop <= st_drop + 1'b1;
            end
            in_pkt <= ~u_last;
            if (wr) begin
                if (u_last) begin
                    if (u_err) wp_t <= wp_c;            // 坏报文：回退
                    else begin
                        wp_t <= wp_t + 1'b1;
                        wp_c <= wp_t + 1'b1;
                        dq_push <= 1'b1;
                        dq_in <= {u_ts, u_src_ip, u_src_port, u_len};
                        rx_irq <= 1'b1;
                    end
                end else wp_t <= wp_t + 1'b1;
            end
        end
    end

    nav_fifo #(.DW(96), .AW(DQ_AW)) u_dq (
        .clk(clk), .rst(rst), .s_valid(dq_push), .s_data(dq_in), .s_ready(dq_s_ready),
        .m_valid(dq_m_valid), .m_data(dq_m), .m_ready(rx_pop & dq_m_valid), .level(dq_level), .free()
    );
    assign rx_avail    = dq_m_valid;
    assign rx_len      = dq_m[15:0];
    assign rx_src_port = dq_m[31:16];
    assign rx_src_ip   = dq_m[63:32];
    assign rx_ts       = dq_m[95:64];
    assign rx_pkts     = dq_level;          // 描述符队列深度 <= 2^DQ_AW + 2，不会超过 8 位

    // ================================================================ 接收读侧
    reg  [15:0] roff;                    // 当前报文内读偏移
    reg  [2:0]  rcnt;                    // 本次取字进度
    reg         rbusy;
    reg         re_q;
    reg  [RX_AW-1:0] ra;
    wire [7:0]  rq;
    reg  [15:0] roff_b;                  // 正在返回的字节在报文内的偏移

    nav_ram_sdp #(.DW(8), .AW(RX_AW)) u_rxb (
        .clk(clk), .we(wr), .waddr(wp_t[RX_AW-1:0]), .wdata(u_data),
        .re(re_q), .raddr(ra), .rdata(rq)
    );

    reg        rv1;
    reg [1:0]  rb1;
    reg        rin1;
    always @(posedge clk) begin
        rx_rd_ack <= 1'b0;
        re_q <= 1'b0;
        rv1 <= 1'b0;
        if (rst) begin
            rp <= 0; roff <= 0; rbusy <= 1'b0; rcnt <= 0;
        end else begin
            if (rx_pop && dq_m_valid) begin
                rp <= rp + dq_m[15:0];
                roff <= 0;
            end
            if (rx_rd && !rbusy) begin
                rbusy <= 1'b1; rcnt <= 0; rx_word <= 32'd0;
            end
            if (rbusy && rcnt < 4) begin
                re_q <= 1'b1;
                ra   <= rp[RX_AW-1:0] + roff[RX_AW-1:0] + rcnt;
                rcnt <= rcnt + 1'b1;
            end
            // 数据返回（发读后第 2 拍）
            rv1 <= re_q; rb1 <= rcnt[1:0] - 2'd1; rin1 <= 1'b1;
            if (rv1) begin
                if ({roff[15:2], rb1} < dq_m[15:0] && dq_m_valid)
                    rx_word[8*rb1 +: 8] <= rq;
                if (rb1 == 2'd3) begin
                    rbusy <= 1'b0; rx_rd_ack <= 1'b1;
                    roff <= roff + 16'd4;
                end
            end
        end
    end

    // ================================================================ 发送
    reg [TX_WAW:0] twp;
    assign tx_words = twp;
    reg  [1:0] pb;
    wire [31:0] tq;
    nav_ram_sdp #(.DW(32), .AW(TX_WAW)) u_txb (
        .clk(clk), .we(tx_we & ~twp[TX_WAW]), .waddr(twp[TX_WAW-1:0]), .wdata(tx_wdata),
        .re(pl_re), .raddr(pl_addr[TX_WAW+1:2]), .rdata(tq)
    );
    always @(posedge clk) if (pl_re) pb <= pl_addr[1:0];
    assign pl_q = tq[8*pb +: 8];
    nav_ram_sdp #(.DW(32), .AW(TX_WAW)) u_txs (          // 发送缓冲影子（总线回读）
        .clk(clk), .we(tx_we & ~twp[TX_WAW]), .waddr(twp[TX_WAW-1:0]), .wdata(tx_wdata),
        .re(txb_re), .raddr(txb_addr), .rdata(txb_q)
    );

    always @(posedge clk) begin
        tx_go <= 1'b0;
        if (rst) begin twp <= 0; tx_seq <= 0; end
        else begin
            if (tx_we && !twp[TX_WAW]) twp <= twp + 1'b1;
            if (tx_go_in && !tx_busy) begin
                tx_go <= 1'b1;
                tx_seq <= tx_seq + 1'b1;
                tx_len <= tx_len_in;
                twp <= 0;
            end
        end
    end
endmodule
