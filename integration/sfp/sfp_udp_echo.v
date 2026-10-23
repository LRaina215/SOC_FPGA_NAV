// One outstanding datagram, up to 1472 bytes. Low-rate board bring-up endpoint.
module sfp_udp_echo (
    input wire clk, input wire rst,
    input wire rx_clk, input wire rx_rst, input wire rx_valid,
    input wire [7:0] rx_data, input wire rx_last, input wire rx_error,
    input wire tx_clk, input wire tx_rst, output wire tx_valid,
    output wire [7:0] tx_data, output wire tx_last, input wire tx_ready,
    output wire [31:0] rx_frames, output wire [31:0] rx_udp,
    output wire [31:0] rx_errors, output wire [31:0] arp_requests,
    output wire [31:0] tx_frames,
    output reg [31:0] echoes, output reg [31:0] echo_drops
);
    wire uv, ul, ue, busy, pl_re;
    wire [7:0] ud, pl_q;
    wire [15:0] ulen;
    wire [10:0] pl_addr;
    reg in_packet, accepted, pending, locked, saw_busy, tx_go;
    reg [10:0] index;
    reg [15:0] tx_len;
    wire take = in_packet ? accepted : (!locked && ulen <= 16'd1472);
    wire [10:0] wa = in_packet ? index : 11'd0;
    nav_ram_sdp #(.DW(8), .AW(11)) u_payload (
        .clk(clk), .we(uv && take), .waddr(wa), .wdata(ud),
        .re(pl_re), .raddr(pl_addr), .rdata(pl_q)
    );
    always @(posedge clk) begin
        if (rst) begin
            in_packet <= 0; accepted <= 0; pending <= 0; locked <= 0;
            saw_busy <= 0; tx_go <= 0; index <= 0; tx_len <= 0;
            echoes <= 0; echo_drops <= 0;
        end else begin
            tx_go <= 0;
            if (pending) begin tx_go <= 1; pending <= 0; end
            if (locked && busy) saw_busy <= 1;
            if (locked && saw_busy && !busy) begin locked <= 0; saw_busy <= 0; end
            if (uv) begin
                in_packet <= !ul;
                accepted <= take;
                index <= wa + 1'b1;
                if (ul) begin
                    if (take && !ue) begin
                        pending <= 1; locked <= 1; saw_busy <= 0;
                        tx_len <= ulen; echoes <= echoes + 1'b1;
                    end else echo_drops <= echo_drops + 1'b1;
                end
            end
        end
    end
    net_port #(.RX_AW(12), .TX_AW(11), .PL_AW(11), .RAW(0)) u_net (
        .clk(clk), .rst(rst), .time_us(32'd0),
        .rx_mac_clk(rx_clk), .rx_mac_rst(rx_rst), .rx_mac_valid(rx_valid),
        .rx_mac_data(rx_data), .rx_mac_last(rx_last), .rx_mac_error(rx_error),
        .tx_mac_clk(tx_clk), .tx_mac_rst(tx_rst), .tx_mac_valid(tx_valid),
        .tx_mac_data(tx_data), .tx_mac_last(tx_last), .tx_mac_ready(tx_ready),
        .cfg_mac(48'h024E56000002), .cfg_ip(32'hC0A80264), .cfg_port(16'd9000),
        .cfg_enable(1'b1), .cfg_any_ip(1'b0), .cfg_arp_en(1'b1),
        .cfg_mcast_ip(32'd0), .cfg_learn(1'b1),
        .peer_we(1'b0), .peer_mac_w(48'd0), .peer_ip_w(32'd0), .peer_port_w(16'd0),
        .peer_mac(), .peer_ip(), .peer_port(),
        .u_valid(uv), .u_data(ud), .u_last(ul), .u_err(ue), .u_len(ulen),
        .u_src_mac(), .u_src_ip(), .u_src_port(), .u_ts(),
        .tx_go(tx_go), .tx_len(tx_len), .tx_busy(busy),
        .pl_re(pl_re), .pl_addr(pl_addr), .pl_q(pl_q),
        .st_rx_frames(rx_frames), .st_rx_udp(rx_udp), .st_rx_err(rx_errors),
        .st_arp(arp_requests), .st_tx_frames(tx_frames), .st_rx_drop(),
        .raw_tx_we(1'b0), .raw_tx_waddr(5'd0), .raw_tx_wdata(32'd0),
        .raw_tx_go(1'b0), .raw_tx_len(8'd0), .raw_tx_busy(),
        .raw_rx_en(1'b0), .raw_rx_raddr(5'd0), .raw_rx_rdata(), .raw_rx_pop(1'b0),
        .raw_rx_n(), .raw_rx_len(), .raw_rx_drop()
    );
endmodule
