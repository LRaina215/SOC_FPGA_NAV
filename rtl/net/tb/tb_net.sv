// 网口协议栈 + 主机邮箱测试（125 MHz MAC 域，100 MHz 核心域）。
//   接收：ARP、各种合法/非法 UDP 帧 -> 检查邮箱内容与统计
//   发送：ARP 应答 + 一个 UDP 报文 -> 写 tx_frames.hex，由 gen_net.py post 解析检查
`timescale 1ns/1ps
module tb_net;
    logic clk = 0, mclk = 0, rst = 1, mrst = 1;
    always #5 clk = ~clk;
    always #4 mclk = ~mclk;

    logic [9:0]  rxs [0:65535];
    logic [31:0] mexp [0:65535];
    logic [31:0] txp [0:1023];
    int n_rx, n_pkt, n_mexp, n_txp;

    logic [31:0] time_us = 0;
    always #1000 time_us++;

    logic rxv = 0, rxl = 0, rxe = 0;
    logic [7:0] rxd = 0;
    logic txv, txl, txr;
    logic [7:0] txd;

    logic u_valid, u_last, u_err;
    logic [7:0] u_data;
    logic [47:0] u_src_mac;
    logic [31:0] u_src_ip, u_ts;
    logic [15:0] u_src_port, u_len;
    logic tx_go, tx_busy, pl_re;
    logic [15:0] tx_len;
    logic [10:0] pl_addr;
    logic [7:0] pl_q;
    logic [31:0] st_rxf, st_udp, st_err, st_arp, st_tx;

    net_port dut (
        .clk(clk), .rst(rst), .time_us(time_us),
        .rx_mac_clk(mclk), .rx_mac_rst(mrst), .rx_mac_valid(rxv), .rx_mac_data(rxd), .rx_mac_last(rxl), .rx_mac_error(rxe),
        .tx_mac_clk(mclk), .tx_mac_rst(mrst), .tx_mac_valid(txv), .tx_mac_data(txd), .tx_mac_last(txl), .tx_mac_ready(txr),
        .cfg_mac(48'h024E56000002), .cfg_ip(32'hC0A80264), .cfg_port(16'd9000), .cfg_enable(1'b1),
        .cfg_any_ip(1'b0), .cfg_arp_en(1'b1), .cfg_learn(1'b1), .cfg_mcast_ip(32'd0),
        .peer_we(1'b0), .peer_mac_w(48'd0), .peer_ip_w(32'd0), .peer_port_w(16'd0),
        .peer_mac(), .peer_ip(), .peer_port(),
        .u_valid(u_valid), .u_data(u_data), .u_last(u_last), .u_err(u_err), .u_src_mac(u_src_mac),
        .u_src_ip(u_src_ip), .u_src_port(u_src_port), .u_len(u_len), .u_ts(u_ts),
        .tx_go(tx_go), .tx_len(tx_len), .tx_busy(tx_busy), .pl_re(pl_re), .pl_addr(pl_addr), .pl_q(pl_q),
        .st_rx_frames(st_rxf), .st_rx_udp(st_udp), .st_rx_err(st_err), .st_arp(st_arp), .st_tx_frames(st_tx),
        .st_rx_drop());

    logic rx_avail, rx_rd = 0, rx_ack, rx_pop = 0, tx_we = 0, tx_go_in = 0;
    logic [15:0] rx_len;
    logic [31:0] rx_word, tx_wdata;
    logic [15:0] tx_len_in;
    host_mbx u_mbx (
        .clk(clk), .rst(rst),
        .e_valid(u_valid), .e_data(u_data), .e_last(u_last), .e_err(u_err), .e_src_ip(u_src_ip),
        .e_src_port(u_src_port), .e_len(u_len), .e_ts(u_ts),
        .inj_we(1'b0), .inj_wdata(32'd0), .inj_go(1'b0), .inj_len(16'd0), .now_us(32'd0),
        .txb_re(1'b0), .txb_addr(9'd0), .txb_q(), .tx_seq(),
        .rx_avail(rx_avail), .rx_len(rx_len), .rx_pkts(), .rx_ts(), .rx_src_ip(), .rx_src_port(),
        .rx_rd(rx_rd), .rx_rd_ack(rx_ack), .rx_word(rx_word), .rx_pop(rx_pop), .rx_irq(), .st_drop(),
        .tx_we(tx_we), .tx_wdata(tx_wdata), .tx_go_in(tx_go_in), .tx_len_in(tx_len_in), .tx_words(),
        .tx_go(tx_go), .tx_len(tx_len), .tx_busy(tx_busy), .pl_re(pl_re), .pl_addr(pl_addr), .pl_q(pl_q));

    // MAC 发送侧：随机 ready，记录所有字节
    int fdo, n_txf = 0;
    always @(posedge mclk) begin
        txr <= ($urandom % 4 != 0);
        if (!mrst && txv && txr) begin
            $fdisplay(fdo, "%03x", {txl, txd});
            if (txl) n_txf++;
        end
    end

    int errs = 0;
    initial begin
        int fd, mi, len, gap;
        fdo = $fopen("tx_frames.hex", "w");
        fd = $fopen("counts.txt", "r");
        void'($fscanf(fd, "%d %d %d %d", n_rx, n_pkt, n_mexp, n_txp));
        $fclose(fd);
        $readmemh("rx.hex", rxs);
        $readmemh("mbx_exp.hex", mexp);
        $readmemh("tx_payload.hex", txp);
        repeat (10) @(posedge clk);
        rst = 0; mrst = 0;
        repeat (20) @(posedge mclk);
        // 注入：帧内连续，帧间 12 拍
        for (int i = 0; i < n_rx; i++) begin
            @(posedge mclk);
            {rxe, rxl, rxd} <= rxs[i];
            rxv <= 1;
            if (rxs[i][8]) begin
                @(posedge mclk); rxv <= 0; rxl <= 0; rxe <= 0;
                repeat (12) @(posedge mclk);
            end
        end
        @(posedge mclk); rxv <= 0;
        repeat (3000) @(posedge clk);

        // 读邮箱
        mi = 0;
        for (int p = 0; p < n_pkt; p++) begin
            if (!rx_avail) begin errs++; $display("报文 %0d 缺失", p); break; end
            len = mexp[mi]; mi++;
            if (rx_len != len) begin errs++; $display("报文 %0d 长度 期望 %0d 实得 %0d", p, len, rx_len); end
            for (int w = 0; w < (len + 3) / 4; w++) begin
                @(posedge clk) rx_rd <= 1;
                @(posedge clk) rx_rd <= 0;
                wait (rx_ack); @(posedge clk);
                if (rx_word !== mexp[mi]) begin
                    errs++;
                    if (errs < 10) $display("报文 %0d 字 %0d 期望 %h 实得 %h", p, w, mexp[mi], rx_word);
                end
                mi++;
            end
            @(posedge clk) rx_pop <= 1;
            @(posedge clk) rx_pop <= 0;
            repeat (3) @(posedge clk);
        end
        if (rx_avail) begin errs++; $display("邮箱有多余报文"); end
        $display("统计 rx=%0d udp=%0d err=%0d arp=%0d", st_rxf, st_udp, st_err, st_arp);
        if (st_arp != 2) begin errs++; $display("ARP 应答数错误"); end

        // 发送
        for (int w = 1; w < n_txp; w++) begin
            @(posedge clk); tx_we <= 1; tx_wdata <= txp[w];
        end
        @(posedge clk); tx_we <= 0; tx_len_in <= txp[0]; tx_go_in <= 1;
        @(posedge clk); tx_go_in <= 0;
        fork
            wait (n_txf == 3);
            begin repeat (200000) @(posedge clk); $display("发送超时 n=%0d", n_txf); errs++; end
        join_any
        repeat (100) @(posedge clk);
        $fclose(fdo);
        if (errs == 0) $display("TEST PASSED");
        else $display("TEST FAILED errs=%0d", errs);
        $finish;
    end
endmodule
