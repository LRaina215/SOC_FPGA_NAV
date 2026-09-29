// 一个网口的完整 UDP 协议栈（MAC 客户端接口 <-> 核心时钟域）。
//   接收：MAC 域整帧准入（缓冲不足整帧丢弃）-> 异步 FIFO -> udp_rx（核心域）
//   发送：udp_tx（核心域）-> 整帧写入异步 FIFO 后才交给 MAC（存储转发，避免 MAC 发送中途欠载）
//   ARP：自动应答；对端地址学习：收到合法 UDP 报文时更新（LEARN_PEER）
// MAC 客户端接口与高云 TSE_MAC / 1G Serial Ethernet(MAC_FUNC) 一致：
//   rx_mac_valid/data/last/error；tx_mac_valid/data/last/ready（valid&ready 时传输一个字节）
module net_port #(
    parameter integer RX_AW = 11,
    parameter integer TX_AW = 11,
    parameter integer PL_AW = 11
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] time_us,

    // MAC 接收域
    input  wire        rx_mac_clk,
    input  wire        rx_mac_rst,
    input  wire        rx_mac_valid,
    input  wire [7:0]  rx_mac_data,
    input  wire        rx_mac_last,
    input  wire        rx_mac_error,
    // MAC 发送域
    input  wire        tx_mac_clk,
    input  wire        tx_mac_rst,
    output wire        tx_mac_valid,
    output wire [7:0]  tx_mac_data,
    output wire        tx_mac_last,
    input  wire        tx_mac_ready,

    // 配置（核心域）
    input  wire [47:0] cfg_mac,
    input  wire [31:0] cfg_ip,
    input  wire [15:0] cfg_port,
    input  wire        cfg_enable,
    input  wire        cfg_any_ip,
    input  wire        cfg_arp_en,
    input  wire [31:0] cfg_mcast_ip,    // 另外接收的组播组（0 = 不收）
    input  wire        cfg_learn,
    // 对端地址（软核可写；学习开启时由收包更新）
    input  wire        peer_we,
    input  wire [47:0] peer_mac_w,
    input  wire [31:0] peer_ip_w,
    input  wire [15:0] peer_port_w,
    output reg  [47:0] peer_mac,
    output reg  [31:0] peer_ip,
    output reg  [15:0] peer_port,

    // UDP 负载输出（核心域，不反压）
    output wire        u_valid,
    output wire [7:0]  u_data,
    output wire        u_last,
    output wire        u_err,
    output wire [47:0] u_src_mac,
    output wire [31:0] u_src_ip,
    output wire [15:0] u_src_port,
    output wire [15:0] u_len,
    output wire [31:0] u_ts,

    // UDP 发送请求（目的地址 = 对端地址）
    input  wire        tx_go,
    input  wire [15:0] tx_len,
    output wire        tx_busy,
    output wire             pl_re,
    output wire [PL_AW-1:0] pl_addr,
    input  wire [7:0]       pl_q,

    output reg  [31:0] st_rx_frames,
    output reg  [31:0] st_rx_udp,
    output reg  [31:0] st_rx_err,
    output reg  [31:0] st_arp,
    output reg  [31:0] st_tx_frames,
    output reg  [31:0] st_rx_drop      // 缓冲不足丢弃的帧（MAC 域计数，仅供调试）
);
    // ------------------------------------------------ 接收：MAC 域整帧准入
    localparam integer MAXF = 1536;
    wire [RX_AW:0] rx_wfree;
    reg  in_frame, dropping;
    wire start_b = rx_mac_valid & ~in_frame;
    wire admit   = start_b ? (rx_wfree > MAXF) : ~dropping;
    always @(posedge rx_mac_clk) begin
        if (rx_mac_rst) begin
            in_frame <= 1'b0; dropping <= 1'b0; st_rx_drop <= 0;
        end else if (rx_mac_valid) begin
            if (start_b) begin
                dropping <= ~(rx_wfree > MAXF);
                if (!(rx_wfree > MAXF)) st_rx_drop <= st_rx_drop + 1'b1;
            end
            in_frame <= ~rx_mac_last;
        end
    end

    wire       rf_v;
    wire [9:0] rf_d;
    nav_async_fifo #(.DW(10), .AW(RX_AW)) u_rxf (
        .wclk(rx_mac_clk), .wrst(rx_mac_rst), .s_valid(rx_mac_valid & admit),
        .s_data({rx_mac_error, rx_mac_last, rx_mac_data}), .s_ready(), .wfree(rx_wfree),
        .rclk(clk), .rrst(rst), .m_valid(rf_v), .m_data(rf_d), .m_ready(1'b1)
    );

    wire arp_req, s_frame, s_udp, s_err;
    wire [47:0] arp_sha;
    wire [31:0] arp_spa;
    udp_rx u_rx (
        .clk(clk), .rst(rst), .time_us(time_us), .my_mac(cfg_mac), .my_ip(cfg_ip), .my_port(cfg_port),
        .cfg_enable(cfg_enable), .cfg_any_ip(cfg_any_ip), .cfg_arp_en(cfg_arp_en), .cfg_mcast_ip(cfg_mcast_ip),
        .s_valid(rf_v), .s_data(rf_d[7:0]), .s_last(rf_d[8]), .s_err(rf_d[9]),
        .m_valid(u_valid), .m_data(u_data), .m_last(u_last), .m_err(u_err),
        .pkt_src_mac(u_src_mac), .pkt_src_ip(u_src_ip), .pkt_src_port(u_src_port), .pkt_len(u_len), .pkt_ts(u_ts),
        .arp_req(arp_req), .arp_sha(arp_sha), .arp_spa(arp_spa),
        .st_frame(s_frame), .st_udp(s_udp), .st_err(s_err)
    );

    // 对端学习
    always @(posedge clk) begin
        if (rst) begin
            peer_mac <= 0; peer_ip <= 0; peer_port <= 0;
        end else if (peer_we) begin
            peer_mac <= peer_mac_w; peer_ip <= peer_ip_w; peer_port <= peer_port_w;
        end else if (cfg_learn && u_valid && u_last && !u_err) begin
            peer_mac <= u_src_mac; peer_ip <= u_src_ip; peer_port <= u_src_port;
        end
    end

    // ------------------------------------------------ 发送
    wire       t_v, t_l, t_r;
    wire [7:0] t_d;
    wire       s_tx;
    udp_tx #(.PL_AW(PL_AW)) u_tx (
        .clk(clk), .rst(rst), .my_mac(cfg_mac), .my_ip(cfg_ip),
        .arp_req(arp_req), .arp_sha(arp_sha), .arp_spa(arp_spa),
        .tx_go(tx_go), .tx_dst_mac(peer_mac), .tx_dst_ip(peer_ip), .tx_dst_port(peer_port),
        .tx_src_port(cfg_port), .tx_len(tx_len), .tx_busy(tx_busy),
        .pl_re(pl_re), .pl_addr(pl_addr), .pl_q(pl_q),
        .m_valid(t_v), .m_data(t_d), .m_last(t_l), .m_ready(t_r), .st_tx(s_tx)
    );

    wire       tf_v, tf_r;
    wire [8:0] tf_d;
    nav_async_fifo #(.DW(9), .AW(TX_AW)) u_txf (
        .wclk(clk), .wrst(rst), .s_valid(t_v), .s_data({t_l, t_d}), .s_ready(t_r), .wfree(),
        .rclk(tx_mac_clk), .rrst(tx_mac_rst), .m_valid(tf_v), .m_data(tf_d), .m_ready(tf_r)
    );
    // 整帧计数：核心域写完一帧（末字节写入）后在 MAC 域加 1
    wire frame_in = t_v & t_r & t_l;
    wire frame_in_m;
    nav_pulse_sync u_fs (.sclk(clk), .srst(rst), .spulse(frame_in), .dclk(tx_mac_clk), .drst(tx_mac_rst), .dpulse(frame_in_m));
    reg [7:0] nfr;
    reg       sending;
    wire      out_last = tx_mac_valid & tx_mac_ready & tx_mac_last;
    always @(posedge tx_mac_clk) begin
        if (tx_mac_rst) begin
            nfr <= 0; sending <= 1'b0;
        end else begin
            nfr <= nfr + frame_in_m - (out_last ? 8'd1 : 8'd0);
            if (!sending && nfr != 0) sending <= 1'b1;
            else if (out_last) sending <= 1'b0;
        end
    end
    assign tx_mac_valid = sending & tf_v;
    assign tx_mac_data  = tf_d[7:0];
    assign tx_mac_last  = tf_d[8];
    assign tf_r         = sending & tx_mac_ready;

    // ------------------------------------------------ 统计
    always @(posedge clk) begin
        if (rst) begin
            st_rx_frames <= 0; st_rx_udp <= 0; st_rx_err <= 0; st_arp <= 0; st_tx_frames <= 0;
        end else begin
            st_rx_frames <= st_rx_frames + s_frame;
            st_rx_udp    <= st_rx_udp + s_udp;
            st_rx_err    <= st_rx_err + s_err;
            st_arp       <= st_arp + arp_req;
            st_tx_frames <= st_tx_frames + s_tx;
        end
    end
endmodule
