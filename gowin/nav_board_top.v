// ACX750 (GW5AT-LV138PG484A) 板级顶层：导航子系统单独上板联调用。
// 整车工程中请把 nav_subsystem 并入整车顶层，本文件可作连线参考。
//
// 时钟：板载 50 MHz（W19）。定义 NAV_USE_PLL 时经 Gowin_PLL_Nav 倍频到 100 MHz，否则核心直接跑 50 MHz。
// 雷达网口：RJ45 / RTL8211F（RGMII）。定义 NAV_USE_TSE_MAC 时例化 IP "Gowin_TSE_MAC_Lidar"，否则 MAC 口悬空。
// 主机网口：SFP1（收发器第 0 通道，参考时钟 MGT_CLK1 125 MHz）。定义 NAV_USE_SFP 时例化
//           IP "Gowin_1GSERETH_Host"（1000BASE-X，开启 MAC 功能），否则悬空。
// 调试：板载 USB 串口（L21/M21）接 uart_bus_bridge，PC 用 tools/nav_uart.py 读写全部寄存器。
// IP 生成方法见 docs/board_bringup.md。
module nav_board_top (
    input  wire       clk50,          // W19
    input  wire       key_rst_n,      // S0 D21，按下复位
    // 调试串口
    input  wire       uart_rxd,       // L21
    output wire       uart_txd,       // M21
    // IMU 串口（扩展口，引脚在 cst 中按实际接线分配）
    input  wire       imu_rxd,
    output wire       imu_txd,
    // RGMII（雷达口）
    input  wire       rgmii_rxc,      // Y18
    input  wire       rgmii_rx_ctl,   // T20
    input  wire [3:0] rgmii_rxd,      // P20 N15 AA18 AB18
    output wire       rgmii_txc,      // AB21
    output wire       rgmii_tx_ctl,   // AA19
    output wire [3:0] rgmii_txd,      // AB20 Y19 AB22 W20
    output wire       eth_rst_n,      // P14
    // SFP1 控制线（高速差分脚由 SerDes IP 约束）
    output wire       sfp1_tx_dis,    // R17
    input  wire       sfp1_los,       // P15
    // 状态灯
    output wire [3:0] led             // U22 V22 W21 W22
);
`ifdef NAV_USE_PLL
    localparam integer CLK_HZ = 100000000;
    wire clk, pll_lock;
    Gowin_PLL_Nav u_pll (.clkin(clk50), .clkout0(clk), .lock(pll_lock));
`else
    localparam integer CLK_HZ = 50000000;
    wire clk = clk50;
    wire pll_lock = 1'b1;
`endif
    wire rst;
    nav_rst_sync u_rs (.clk(clk), .arst(~key_rst_n | ~pll_lock), .rst(rst));
    assign eth_rst_n   = ~rst;
    assign sfp1_tx_dis = 1'b0;

    // ---------------- 总线：调试串口桥（软核接入后接 m0） ----------------
    wire        u_req, u_we, u_ack;
    wire [15:0] u_addr;
    wire [31:0] u_wdata, u_rdata;
    uart_bus_bridge #(.CLK_HZ(CLK_HZ), .BAUD(921600)) u_ub (
        .clk(clk), .rst(rst), .rxd(uart_rxd), .txd(uart_txd),
        .bus_req(u_req), .bus_we(u_we), .bus_addr(u_addr), .bus_wdata(u_wdata),
        .bus_ack(u_ack), .bus_rdata(u_rdata)
    );
    wire        s_req, s_we, s_ack;
    wire [15:0] s_addr;
    wire [31:0] s_wdata, s_rdata;
    nav_bus_arb u_arb (
        .clk(clk), .rst(rst),
        .m0_req(1'b0), .m0_we(1'b0), .m0_addr(16'd0), .m0_wdata(32'd0), .m0_ack(), .m0_rdata(),
        .m1_req(u_req), .m1_we(u_we), .m1_addr(u_addr), .m1_wdata(u_wdata), .m1_ack(u_ack), .m1_rdata(u_rdata),
        .s_req(s_req), .s_we(s_we), .s_addr(s_addr), .s_wdata(s_wdata), .s_ack(s_ack), .s_rdata(s_rdata)
    );

    // ---------------- 雷达口 MAC ----------------
    wire       l_rx_clk, l_rx_valid, l_rx_last, l_rx_err, l_tx_clk, l_tx_valid, l_tx_last, l_tx_ready;
    wire [7:0] l_rx_data, l_tx_data;
`ifdef NAV_USE_TSE_MAC
    // TSE_MAC：RGMII 接口，速率由 speedis1000/speedis10 选择；XT-M60 为百兆
    Gowin_TSE_MAC_Lidar u_lmac (
        .rgmii_rxc(rgmii_rxc), .rgmii_rx_ctl(rgmii_rx_ctl), .rgmii_rxd(rgmii_rxd),
        .gtx_clk(clk50),            // 百兆模式下 MAC 内部使用 rgmii_rxc 恢复的时钟；gtx 仅千兆需要
        .rgmii_txc(rgmii_txc), .rgmii_tx_ctl(rgmii_tx_ctl), .rgmii_txd(rgmii_txd),
        .speedis1000(1'b0), .speedis10(1'b0), .duplex_status(1'b1),
        .rstn(~rst),
        .rx_mac_clk(l_rx_clk), .rx_mac_valid(l_rx_valid), .rx_mac_data(l_rx_data), .rx_mac_last(l_rx_last),
        .rx_mac_error(l_rx_err), .rx_statistics_valid(), .rx_statistics_vector(),
        .tx_mac_clk(l_tx_clk), .tx_mac_valid(l_tx_valid), .tx_mac_data(l_tx_data), .tx_mac_last(l_tx_last),
        .tx_mac_error(1'b0), .tx_mac_ready(l_tx_ready), .tx_collision(), .tx_retransmit(),
        .tx_statistics_valid(), .tx_statistics_vector(),
        .rx_fcs_fwd_ena(1'b0), .rx_jumbo_ena(1'b0), .rx_pause_req(), .rx_pause_val(),
        .tx_fcs_fwd_ena(1'b0), .tx_ifg_delay_ena(1'b0), .tx_ifg_delay(8'd12),
        .tx_pause_req(1'b0), .tx_pause_val(16'd0), .tx_pause_source_addr(48'd0)
    );
`else
    assign l_rx_clk = clk; assign l_rx_valid = 1'b0; assign l_rx_data = 8'd0; assign l_rx_last = 1'b0;
    assign l_rx_err = 1'b0; assign l_tx_clk = clk; assign l_tx_ready = 1'b1;
    assign rgmii_txc = 1'b0; assign rgmii_tx_ctl = 1'b0; assign rgmii_txd = 4'd0;
`endif
    wire l_rx_rst, l_tx_rst;
    nav_rst_sync u_lrr (.clk(l_rx_clk), .arst(rst), .rst(l_rx_rst));
    nav_rst_sync u_ltr (.clk(l_tx_clk), .arst(rst), .rst(l_tx_rst));

    // ---------------- 主机口 MAC（SFP） ----------------
    wire       h_rx_clk, h_rx_valid, h_rx_last, h_rx_err, h_tx_clk, h_tx_valid, h_tx_last, h_tx_ready;
    wire [7:0] h_rx_data, h_tx_data;
`ifdef NAV_USE_SFP
    // 1G Serial Ethernet（PCS+MAC）与 SerDes 由 IP 生成器一起产生，顶层封装名 Gowin_1GSERETH_Host
    Gowin_1GSERETH_Host u_hmac (
        .rstn(~rst), .signal_detect(~sfp1_los),
        .rx_mac_clk(h_rx_clk), .rx_mac_valid(h_rx_valid), .rx_mac_data(h_rx_data), .rx_mac_last(h_rx_last),
        .rx_mac_error(h_rx_err),
        .tx_mac_clk(h_tx_clk), .tx_mac_valid(h_tx_valid), .tx_mac_data(h_tx_data), .tx_mac_last(h_tx_last),
        .tx_mac_error(1'b0), .tx_mac_ready(h_tx_ready)
    );
`else
    assign h_rx_clk = clk; assign h_rx_valid = 1'b0; assign h_rx_data = 8'd0; assign h_rx_last = 1'b0;
    assign h_rx_err = 1'b0; assign h_tx_clk = clk; assign h_tx_ready = 1'b1;
`endif
    wire h_rx_rst, h_tx_rst;
    nav_rst_sync u_hrr (.clk(h_rx_clk), .arst(rst), .rst(h_rx_rst));
    nav_rst_sync u_htr (.clk(h_tx_clk), .arst(rst), .rst(h_tx_rst));

    // ---------------- 导航子系统 ----------------
    wire irq, ch_valid, ch_stop, ch_wdog;
    nav_subsystem #(.CLK_HZ(CLK_HZ), .LUT_INIT("lut_default.hex"), .SIN_INIT("sin_lut.hex")) u_nav (
        .clk(clk), .rst(rst),
        .bus_req(s_req), .bus_we(s_we), .bus_addr(s_addr), .bus_wdata(s_wdata), .bus_ack(s_ack), .bus_rdata(s_rdata),
        .irq(irq),
        .l_rx_clk(l_rx_clk), .l_rx_rst(l_rx_rst), .l_rx_valid(l_rx_valid), .l_rx_data(l_rx_data),
        .l_rx_last(l_rx_last), .l_rx_error(l_rx_err),
        .l_tx_clk(l_tx_clk), .l_tx_rst(l_tx_rst), .l_tx_valid(l_tx_valid), .l_tx_data(l_tx_data),
        .l_tx_last(l_tx_last), .l_tx_ready(l_tx_ready),
        .h_rx_clk(h_rx_clk), .h_rx_rst(h_rx_rst), .h_rx_valid(h_rx_valid), .h_rx_data(h_rx_data),
        .h_rx_last(h_rx_last), .h_rx_error(h_rx_err),
        .h_tx_clk(h_tx_clk), .h_tx_rst(h_tx_rst), .h_tx_valid(h_tx_valid), .h_tx_data(h_tx_data),
        .h_tx_last(h_tx_last), .h_tx_ready(h_tx_ready),
        .imu_rx(imu_rxd), .imu_tx(imu_txd),
        .ch_vx(), .ch_vy(), .ch_wz(), .ch_mode(), .ch_stop(ch_stop), .ch_wdog(ch_wdog), .ch_valid(ch_valid), .ch_seq(),
        .odo_valid(1'b0), .odo_vx(16'd0), .odo_vy(16'd0), .odo_wz(16'd0),
        .ext_stop(4'd0)
    );

    // 状态灯：心跳 / 中断 / 急停 / 指令超时
    reg [25:0] hb;
    always @(posedge clk) hb <= hb + 1'b1;
    assign led = {ch_wdog, ch_stop, irq, hb[25]};
endmodule
