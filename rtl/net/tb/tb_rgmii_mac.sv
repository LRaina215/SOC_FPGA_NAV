// rgmii_mac100 测试：PHY 行为模型（MDIO 从机，地址 3；RGMII 百兆收发），核对
//   接收：去前导码 / SFD、FCS 检查、RX_ER、奇数半字节；发送：前导码、补零、FCS、帧间隔；
//   MDIO：扫描到地址 3、自协商寄存器写入值、链路状态轮询；RX 时钟测频。
`timescale 1ns/1ps
module tb_rgmii_mac;
    logic clk = 0, rxc = 0, rst = 1;
    always #10 clk = ~clk;          // 50 MHz
    always #20 rxc = ~rxc;          // 25 MHz（PHY 恢复时钟，与 clk 无相位关系）

    logic [5:0] rx_st  [0:65535];
    logic [9:0] rx_ex  [0:16383];
    logic [8:0] tx_st  [0:4095];
    logic [7:0] tx_ex  [0:4095];
    int n_rx, n_rxe, n_tx, n_txe, errs = 0;

    logic rx_ctl = 0;
    logic [3:0] rxd = 0;
    wire txc, tx_ctl, phy_rst_n, mdc, mdio_o, mdio_oe;
    wire [3:0] txd;
    logic phy_oe = 0, phy_o = 1;
    wire mdio_line = mdio_oe ? mdio_o : (phy_oe ? phy_o : 1'b1);
    wire rx_clk, rx_valid, rx_last, rx_error, tx_ready;
    wire [7:0] rx_data;
    logic tx_valid = 0, tx_last = 0;
    logic [7:0] tx_data = 0;
    wire [31:0] phy_stat;

    rgmii_mac100 #(.CLK_HZ(50000000), .T_RST_US(2), .T_WAIT_US(5), .T_POLL_US(30)) dut (
        .clk(clk), .rst(rst),
        .rgmii_rxc(rxc), .rgmii_rx_ctl(rx_ctl), .rgmii_rxd(rxd),
        .rgmii_txc(txc), .rgmii_tx_ctl(tx_ctl), .rgmii_txd(txd),
        .phy_rst_n(phy_rst_n), .mdc(mdc), .mdio_o(mdio_o), .mdio_oe(mdio_oe), .mdio_i(mdio_line),
        .rx_clk(rx_clk), .rx_valid(rx_valid), .rx_data(rx_data), .rx_last(rx_last), .rx_error(rx_error),
        .tx_valid(tx_valid), .tx_data(tx_data), .tx_last(tx_last), .tx_ready(tx_ready),
        .phy_stat(phy_stat));

    // ---------------- MDIO 从机（PHY 地址 3） ----------------
    localparam logic [4:0] MY_ADDR = 5'd3;
    logic [15:0] preg [0:31];
    initial begin
        for (int i = 0; i < 32; i++) preg[i] = 16'h0000;
        preg[1] = 16'h796D;          // BMSR：链路已建立（bit 2）
        preg[2] = 16'h001C;          // PHYID1（RTL8211F）
        preg[9] = 16'h0300;          // 复位值：通告千兆
    end
    int ones = 0, st = 0, cnt = 0;
    logic [11:0] hdr;
    logic [17:0] wsh;
    logic [15:0] rdat;
    always @(posedge mdc) begin
        case (st)
        0: begin                                  // 找前导码 + ST(01)
            if (mdio_line) ones++;
            else begin st = (ones >= 32) ? 1 : 0; ones = 0; end
        end
        1: begin st = mdio_line ? 2 : 0; cnt = 0; end
        2: begin                                  // OP(2) PHYAD(5) REGAD(5)
            hdr = {hdr[10:0], mdio_line}; cnt++;
            if (cnt == 12) begin
                cnt = 0;
                if (hdr[11:10] == 2'b10 && hdr[9:5] == MY_ADDR) begin st = 3; rdat = preg[hdr[4:0]]; end
                else if (hdr[11:10] == 2'b01) st = 4;
                else st = 5;                      // 别的地址的读：不应答
            end
        end
        3: begin                                  // 读：TA 第 2 位驱动 0，然后 16 位数据
            cnt++;
            if (cnt == 1) begin phy_oe = 1; phy_o = 0; end
            else if (cnt <= 17) begin phy_o = rdat[15]; rdat = {rdat[14:0], 1'b0}; end
            else begin phy_oe = 0; st = 0; end
        end
        4: begin                                  // 写：TA(2) + 16 位数据
            wsh = {wsh[16:0], mdio_line}; cnt++;
            if (cnt == 18) begin
                if (hdr[9:5] == MY_ADDR) preg[hdr[4:0]] = wsh[15:0];
                st = 0;
            end
        end
        default: begin cnt++; if (cnt == 18) st = 0; end
        endcase
    end

    // ---------------- RGMII 接收激励 ----------------
    int ri = 0;
    logic cur_dv = 0, cur_er = 0;
    always @(negedge rxc) begin
        #10;
        if (ri < n_rx && !rst && phy_rst_n) begin
            {cur_er, cur_dv, rxd} = rx_st[ri]; ri++;
        end else begin cur_er = 0; cur_dv = 0; rxd = 0; end
        rx_ctl = cur_dv;
    end
    always @(posedge rxc) begin #10; rx_ctl = cur_dv ^ cur_er; end

    int ei = 0;
    always @(posedge rx_clk) if (rx_valid) begin
        if (ei >= n_rxe) begin errs++; if (errs < 10) $display("多余接收字节 %h", rx_data); end
        else if ({rx_error, rx_last, rx_data} !== rx_ex[ei]) begin
            errs++; if (errs < 10) $display("接收 #%0d 期望 %h 实得 %h", ei, rx_ex[ei], {rx_error, rx_last, rx_data});
        end
        ei++;
    end

    // ---------------- 发送：客户端激励与线上核对 ----------------
    int ti = 0;
    always @(posedge clk) begin
        if (!rst && ti < n_tx) begin
            if (!tx_valid || tx_ready) begin
                if (tx_valid && tx_ready) ti = ti + 1;
                if (ti < n_tx) begin {tx_last, tx_data} <= tx_st[ti]; tx_valid <= 1; end
                else tx_valid <= 0;
            end
        end else if (tx_ready) tx_valid <= 0;
    end
    int oi = 0, half = 0;
    logic [3:0] lo;
    always @(posedge txc) if (tx_ctl) begin
        if (!half) begin lo = txd; half = 1; end
        else begin
            half = 0;
            if (oi >= n_txe) begin errs++; if (errs < 10) $display("多余发送字节 %h", {txd, lo}); end
            else if ({txd, lo} !== tx_ex[oi]) begin
                errs++; if (errs < 10) $display("发送 #%0d 期望 %h 实得 %h", oi, tx_ex[oi], {txd, lo});
            end
            oi++;
        end
    end else half = 0;

    initial begin
        int fd;
        fd = $fopen("counts.txt", "r");
        void'($fscanf(fd, "%d %d %d %d", n_rx, n_rxe, n_tx, n_txe));
        $fclose(fd);
        $readmemh("rx.hex", rx_st);
        $readmemh("rx_exp.hex", rx_ex);
        $readmemh("tx.hex", tx_st);
        $readmemh("tx_exp.hex", tx_ex);
        repeat (5) @(posedge clk);
        rst = 0;
        #2500us;
        if (ei != n_rxe) begin errs++; $display("接收字节 %0d/%0d", ei, n_rxe); end
        if (oi != n_txe) begin errs++; $display("发送字节 %0d/%0d", oi, n_txe); end
        if (phy_stat[2:0] !== 3'b111 || phy_stat[7:3] !== MY_ADDR) begin
            errs++; $display("PHY 状态 %h（应找到地址 3、初始化完成、链路建立）", phy_stat[7:0]);
        end
        if (preg[4] !== 16'h0181 || preg[9] !== 16'h0000 || preg[0] !== 16'h1200) begin
            errs++; $display("MDIO 写入 ANAR=%h GBCR=%h BMCR=%h", preg[4], preg[9], preg[0]);
        end
        if (phy_stat[31:16] < 388 || phy_stat[31:16] > 394) begin
            errs++; $display("RX 时钟测频 %0d（25 MHz 应约 390）", phy_stat[31:16]);
        end
        $display("rx %0d/%0d tx %0d/%0d phy_stat=%h", ei, n_rxe, oi, n_txe, phy_stat);
        if (errs == 0) $display("TEST PASSED");
        else $display("TEST FAILED errs=%0d", errs);
        $finish;
    end
endmodule
