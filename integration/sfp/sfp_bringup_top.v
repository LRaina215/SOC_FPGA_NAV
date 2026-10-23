// Standalone SFP diagnostic image. Not the navigation/control application.
module sfp_bringup_top (
    input wire clk, input wire reset_n,
    input wire uart_rxd, output wire uart_txd,
    input wire sfp1_los, output wire sfp1_tx_dis,
    output wire M1_DIR1, output wire M1_DIR2, output wire M1_PWM,
    output wire M2_DIR1, output wire M2_DIR2, output wire M2_PWM,
    output wire SERVO1, output wire SERVO2, output wire SERVO3, output wire SERVO4
);
    assign {M1_DIR1, M1_DIR2, M1_PWM, M2_DIR1, M2_DIR2, M2_PWM,
            SERVO1, SERVO2, SERVO3, SERVO4} = 10'b0;
    reg [15:0] por = 0;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) por <= 0;
        else if (!(&por)) por <= por + 1'b1;
    end
    wire rst;
    nav_rst_sync u_reset (.clk(clk), .arst(!reset_n || !(&por)), .rst(rst));
    wire req, we;
    wire [15:0] addr;
    wire [31:0] wdata;
    reg [31:0] rdata;
    uart_bus_bridge #(.CLK_HZ(50000000), .BAUD(921600)) u_uart (
        .clk(clk), .rst(rst), .rxd(uart_rxd), .txd(uart_txd),
        .bus_req(req), .bus_we(we), .bus_addr(addr), .bus_wdata(wdata),
        .bus_ack(req), .bus_rdata(rdata)
    );
    // cfg[0]=SGMII, cfg[1]=AN, cfg[2]=honour LOS. Write reloads IP reset.
    reg [2:0] cfg;
    reg [15:0] reset_hold;
    always @(posedge clk) begin
        if (rst) begin cfg <= 3'b010; reset_hold <= 16'hffff; end
        else if (req && we && addr == 16'h0004) begin
            cfg <= wdata[2:0]; reset_hold <= 16'hffff;
        end else if (reset_hold != 0) reset_hold <= reset_hold - 1'b1;
    end
    wire ip_rst = rst || (reset_hold != 0);
    wire rx_clk, tx_clk, rv, rl, re, tv, tl, tr;
    wire [7:0] rd, td;
    wire [10:0] status_raw, status;
    wire [15:0] partner_raw, partner;
    wire [3:0] diagnostic_raw, diagnostic;
    assign diagnostic_raw[3] = sfp1_los;
    sfp_host_mac u_mac (
        .clk(clk), .rst(ip_rst), .cfg_sgmii(cfg[0]), .cfg_an(cfg[1]),
        .cfg_use_los(cfg[2]), .sfp_los(sfp1_los), .sfp_tx_disable(sfp1_tx_dis),
        .rx_clk(rx_clk), .rx_valid(rv), .rx_data(rd), .rx_last(rl), .rx_error(re),
        .tx_clk(tx_clk), .tx_valid(tv), .tx_data(td), .tx_last(tl), .tx_ready(tr),
        .status(status_raw), .partner(partner_raw), .qpll_ok(diagnostic_raw[0]),
        .rx_lock(diagnostic_raw[1]), .aligned(diagnostic_raw[2])
    );
    // These are diagnostic snapshots. Read again after link becomes stable.
    nav_sync2 #(.W(11)) u_status (.clk(clk), .d(status_raw), .q(status));
    nav_sync2 #(.W(16)) u_partner (.clk(clk), .d(partner_raw), .q(partner));
    nav_sync2 #(.W(4)) u_diag (.clk(clk), .d(diagnostic_raw), .q(diagnostic));
    wire link_complete = status[0] && status[1] && (!cfg[1] || status[2]);
    reg [7:0] link_age;
    reg link_old;
    reg [31:0] link_losses;
    wire link_ready = &link_age;
    always @(posedge clk) begin
        if (rst) begin link_age <= 0; link_old <= 0; link_losses <= 0; end
        else begin
            if (ip_rst || !link_complete) link_age <= 0;
            else if (!link_ready) link_age <= link_age + 1'b1;
            link_old <= link_ready;
            if (link_old && !link_ready) link_losses <= link_losses + 1'b1;
        end
    end
    wire net_arst = ip_rst || !link_ready;
    wire net_rst, rx_rst, tx_rst;
    nav_rst_sync u_net_reset (.clk(clk), .arst(net_arst), .rst(net_rst));
    nav_rst_sync u_rx_reset (.clk(rx_clk), .arst(net_arst), .rst(rx_rst));
    nav_rst_sync u_tx_reset (.clk(tx_clk), .arst(net_arst), .rst(tx_rst));
    wire [31:0] rx_frames, rx_udp, rx_errors, arp_requests, tx_frames, echoes, drops;
    sfp_udp_echo u_echo (
        .clk(clk), .rst(net_rst), .rx_clk(rx_clk), .rx_rst(rx_rst),
        .rx_valid(rv), .rx_data(rd), .rx_last(rl), .rx_error(re),
        .tx_clk(tx_clk), .tx_rst(tx_rst), .tx_valid(tv), .tx_data(td),
        .tx_last(tl), .tx_ready(tr), .rx_frames(rx_frames), .rx_udp(rx_udp),
        .rx_errors(rx_errors), .arp_requests(arp_requests), .tx_frames(tx_frames),
        .echoes(echoes), .echo_drops(drops)
    );
    always @(*) begin
        case (addr)
            16'h0000: rdata = 32'h53465031; // SFP1: identify before any config write
            16'h0004: rdata = {29'd0, cfg};
            16'h0008: rdata = {21'd0, status};
            16'h000c: rdata = {16'd0, partner};
            16'h0010: rdata = {26'd0, ip_rst, link_ready, diagnostic};
            16'h0014: rdata = link_losses;
            16'h0020: rdata = rx_frames;
            16'h0024: rdata = rx_udp;
            16'h0028: rdata = rx_errors;
            16'h002c: rdata = arp_requests;
            16'h0030: rdata = tx_frames;
            16'h0034: rdata = echoes;
            16'h0038: rdata = drops;
            16'h0040: rdata = 32'hc0a80264;
            16'h0044: rdata = 32'd9000;
            default: rdata = 32'hdeadbeef;
        endcase
    end
endmodule
