// ACX750 (GW5AT-LV138PG484A) 导航与原4500控制的联合顶层。
//
// 时钟：板载 50 MHz（W19）。定义 NAV_USE_PLL 时经 Gowin_PLL_Nav 倍频到 100 MHz，否则核心直接跑 50 MHz。
// 雷达网口：RJ45 / RTL8211F（RGMII）。rtl/net/rgmii_mac100.v：百兆 MAC + PHY 复位 + MDIO 把自协商限制到百兆。
// 主机网口：定义 G2_REAL_SFP 时启用 Q0 Lane0 / REFCLK1 125 MHz 的真实 MAC/SerDes，
//           与导航邮箱共用网栈，旁路复制完整原控制UART遥测。未定义时仅保留旧资源诊断配置。
// 调试：板载 USB 串口（L21/M21）接 uart_bus_bridge，PC 用 tools/nav_uart.py 读写全部寄存器；
//       地址 0xE000–0xFFFF 转到软核调试口（装载程序、读控制台）。
// 软核：nav_soc（PicoRV32，片上 RAM 128 KB，上电运行 fw/build/rv/nav 的镜像），经 nav_bus_arb m0 访问导航子系统。
// IP 生成方法见 docs/3_guide/board_bringup.md。
`include "soc_cfg.vh"          // 构建脚本生成：`define NAV_SOC_RAM_KB（片上 RAM 大小）
module g2_top (
    input  wire       clk50,          // W19
    input  wire       key_rst_n,      // S0 D21，按下复位
    // 调试串口
    input  wire       uart_rxd,       // L21
    output wire       uart_txd,       // M21
    // IMU 串口（扩展口，引脚在 cst 中按实际接线分配）
    input  wire       imu_rxd,
    output wire       control_txd,
    input wire IRIN,M1_ENCA,M1_ENCB,M2_ENCA,M2_ENCB,
    inout wire sck,sda,BMI088_SCL,BMI088_SDA,
    output wire M1_PWM,M1_DIR1,M1_DIR2,M2_PWM,M2_DIR1,M2_DIR2,
    output wire SERVO1,SERVO2,SERVO3,SERVO4,
    // RGMII（雷达口）
    input  wire       rgmii_rxc,      // Y18
    input  wire       rgmii_rx_ctl,   // T20
    input  wire [3:0] rgmii_rxd,      // P20 N15 AA18 AB18
    output wire       rgmii_txc,      // AB21
    output wire       rgmii_tx_ctl,   // AA19
    output wire [3:0] rgmii_txd,      // AB20 Y19 AB22 W20
    output wire       eth_rst_n,      // P14
    output wire       eth_mdc,        // R14
    inout  wire       eth_mdio,       // U21
    // SFP1 控制线（高速差分脚由 SerDes IP 约束）
    output wire       sfp1_tx_dis,    // R17
    input  wire       sfp1_los,       // P15
    // 状态灯
    output wire [3:0] led,            // U22 V22 W21 W22
    output wire [3:0] led_cpu,        // Y21 Y22 N13 N17：软核写的 LED
    output wire       led_trap        // P17：软核异常停机
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

    // ---------------- 总线：调试串口桥（软核接入后接 m0） ----------------
    wire        u_req, u_we, u_ack;
    wire [15:0] u_addr;
    wire [31:0] u_wdata, u_rdata;
    uart_bus_bridge #(.CLK_HZ(CLK_HZ), .BAUD(921600)) u_ub (
        .clk(clk), .rst(rst), .rxd(uart_rxd), .txd(uart_txd),
        .bus_req(u_req), .bus_we(u_we), .bus_addr(u_addr), .bus_wdata(u_wdata),
        .bus_ack(u_ack), .bus_rdata(u_rdata)
    );
    // 串口桥地址 0xE000–0xFFFF -> 软核调试口，其余 -> 导航总线
    wire        u_dbg = (u_addr[15:13] == 3'b111);
    wire        ub_ack, d_ack;
    wire [31:0] ub_rdata, d_rdata;
    assign u_ack   = u_dbg ? d_ack : ub_ack;
    assign u_rdata = u_dbg ? d_rdata : ub_rdata;
    wire        c_req, c_we, c_ack;
    wire [15:0] c_addr;
    wire [31:0] c_wdata, c_rdata;
    nav_soc #(.RAM_KB(`NAV_SOC_RAM_KB), .INIT_B0("ram_b0.hex"), .INIT_B1("ram_b1.hex"), .INIT_B2("ram_b2.hex"), .INIT_B3("ram_b3.hex")) u_soc (
        .clk(clk), .rst(rst),
        .m_req(c_req), .m_we(c_we), .m_addr(c_addr), .m_wdata(c_wdata), .m_ack(c_ack), .m_rdata(c_rdata),
        .d_req(u_req & u_dbg), .d_we(u_we), .d_addr(u_addr), .d_wdata(u_wdata), .d_ack(d_ack), .d_rdata(d_rdata),
        .led(led_cpu), .trap(led_trap)
    );
    wire        s_req, s_we, s_ack;
    wire [15:0] s_addr;
    wire [31:0] s_wdata, s_rdata;
    nav_bus_arb u_arb (
        .clk(clk), .rst(rst),
        .m0_req(c_req), .m0_we(c_we), .m0_addr(c_addr), .m0_wdata(c_wdata), .m0_ack(c_ack), .m0_rdata(c_rdata),
        .m1_req(u_req & ~u_dbg), .m1_we(u_we), .m1_addr(u_addr), .m1_wdata(u_wdata), .m1_ack(ub_ack), .m1_rdata(ub_rdata),
        .s_req(s_req), .s_we(s_we), .s_addr(s_addr), .s_wdata(s_wdata), .s_ack(s_ack), .s_rdata(s_rdata)
    );

    // ---------------- 雷达口 MAC（自写百兆 RGMII MAC，不需要生成 IP） ----------------
    wire       l_rx_clk, l_rx_valid, l_rx_last, l_rx_err, l_tx_clk, l_tx_valid, l_tx_last, l_tx_ready;
    wire [7:0] l_rx_data, l_tx_data;
    wire [31:0] l_phy_stat;
    wire       l_rx_neg;
    wire       mdio_o, mdio_oe;
    rgmii_mac100 #(.CLK_HZ(CLK_HZ)) u_lmac (
        .clk(clk), .rst(rst),
        .rgmii_rxc(rgmii_rxc), .rgmii_rx_ctl(rgmii_rx_ctl), .rgmii_rxd(rgmii_rxd),
        .rgmii_txc(rgmii_txc), .rgmii_tx_ctl(rgmii_tx_ctl), .rgmii_txd(rgmii_txd),
        .phy_rst_n(eth_rst_n), .mdc(eth_mdc), .mdio_o(mdio_o), .mdio_oe(mdio_oe), .mdio_i(eth_mdio), .rx_neg(l_rx_neg),
        .rx_clk(l_rx_clk), .rx_valid(l_rx_valid), .rx_data(l_rx_data), .rx_last(l_rx_last), .rx_error(l_rx_err),
        .tx_valid(l_tx_valid), .tx_data(l_tx_data), .tx_last(l_tx_last), .tx_ready(l_tx_ready),
        .phy_stat(l_phy_stat)
    );
    assign eth_mdio = mdio_oe ? mdio_o : 1'bz;
    assign l_tx_clk = clk;
    wire l_rx_rst, l_tx_rst;
    nav_rst_sync u_lrr (.clk(l_rx_clk), .arst(rst), .rst(l_rx_rst));
    nav_rst_sync u_ltr (.clk(l_tx_clk), .arst(rst), .rst(l_tx_rst));

    // ---------------- 主机口 MAC（SFP） ----------------
    wire       h_rx_clk, h_rx_valid, h_rx_last, h_rx_err, h_tx_clk, h_tx_valid, h_tx_last, h_tx_ready;
    wire [7:0] h_rx_data, h_tx_data;
    wire h_rx_rst,h_tx_rst,h_core_rst;
    wire sfp_sel,sfp_ack,nav_ack;
    wire [31:0] sfp_rdata,nav_rdata;
    wire tap_valid,tap_first,tap_last;
    wire [7:0] tap_data;
    wire [31:0] telemetry_accepted,telemetry_dropped,telemetry_sent;
    assign s_ack=sfp_sel ? sfp_ack : nav_ack;
    assign s_rdata=sfp_sel ? sfp_rdata : nav_rdata;
`ifdef G2_REAL_SFP
    assign sfp_sel=s_addr[15:8]==8'h13;
    sfp_management u_sfp(
        .clk(clk),.rst(rst),.req(s_req && sfp_sel),.we(s_we),.addr(s_addr[7:0]),.wdata(s_wdata),
        .ack(sfp_ack),.rdata(sfp_rdata),.telemetry_accepted(telemetry_accepted),
        .telemetry_dropped(telemetry_dropped),.telemetry_sent(telemetry_sent),
        .sfp_los(sfp1_los),.sfp_tx_disable(sfp1_tx_dis),
        .rx_clk(h_rx_clk),.rx_valid(h_rx_valid),.rx_data(h_rx_data),.rx_last(h_rx_last),.rx_error(h_rx_err),
        .tx_clk(h_tx_clk),.tx_valid(h_tx_valid),.tx_data(h_tx_data),.tx_last(h_tx_last),.tx_ready(h_tx_ready),
        .host_rst(h_core_rst),.rx_rst(h_rx_rst),.tx_rst(h_tx_rst));
`else
    assign sfp_sel=0;assign sfp_ack=0;assign sfp_rdata=0;
    assign sfp1_tx_dis=0;
    assign h_rx_clk=clk;assign h_rx_valid=0;assign h_rx_data=0;assign h_rx_last=0;
    assign h_rx_err=0;assign h_tx_clk=clk;assign h_tx_ready=1;
    assign h_core_rst=rst;assign h_rx_rst=rst;assign h_tx_rst=rst;
    assign tap_valid=0;assign tap_first=0;assign tap_last=0;assign tap_data=0;
`endif

    // ---------------- 导航子系统 ----------------
    wire irq, ch_valid, ch_stop, ch_wdog;
    wire signed [15:0] ch_vx,ch_wz,odo_vx,odo_wz;
    wire odo_valid;
    wire [15:0] ch_seq;
    wire signed [47:0] vx_product=$signed(ch_vx)*48'sd4096;
    wire signed [47:0] wz_product=$signed(ch_wz)*48'sd4096;
`ifdef G2_COMMAND_LEASE
    wire signed [23:0] nav_forward_q12,nav_yaw_q12;
    wire nav_guard_valid;wire [15:0] nav_guard_seq;
    wire signed [23:0] vx_command_q12,wz_command_q12;
`ifdef G2_UNITS_PIPELINE
    nav_command_units u_command_units(.clk(clk),.reset_n(~rst),.kill(ch_stop|ch_wdog),
        .valid(ch_valid),.seq(ch_seq),.vx(ch_vx),.wz(ch_wz),
        .out_valid(nav_guard_valid),.out_seq(nav_guard_seq),
        .out_vx_q12(vx_command_q12),.out_wz_q12(wz_command_q12));
`else
    wire signed [47:0] vx_rounded=vx_product<0?vx_product-500:vx_product+500;
    wire signed [47:0] wz_rounded=wz_product<0?wz_product-500:wz_product+500;
    wire signed [47:0] vx_converted=vx_rounded/1000,wz_converted=wz_rounded/1000;
    assign vx_command_q12=vx_converted[23:0];assign wz_command_q12=wz_converted[23:0];
    assign nav_guard_valid=ch_valid;assign nav_guard_seq=ch_seq;
`endif
    // NAV is the sole owner in this G2 image. SAFE's raw command watchdog is
    // authoritative; periodic SAFE output alone must not keep a stale lease.
    command_lease_guard #(.CLK_HZ(CLK_HZ)) u_command_lease(
        .clk(clk),.reset_n(~rst),.owner_select(2'd1),.source_id(2'd1),
        .session_select(32'd1),.packet_session(32'd1),.packet_seq(nav_guard_seq),
        .valid(nav_guard_valid),.source_fault(ch_wdog),.estop(ch_stop),
        .forward_q12(vx_command_q12),.yaw_q12(wz_command_q12),
        .feedback_valid(odo_valid),.measured_vx_mmps(odo_vx),.measured_wz_mradps(odo_wz),
        .out_forward_q12(nav_forward_q12),.out_yaw_q12(nav_yaw_q12),
        .armed(),.expired(),.stop_confirmed(),.age_ms(),.accepted_count(),.rejected_count(),.balance_enable());
`else
    reg signed [23:0] nav_forward_q12,nav_yaw_q12;
    always @(posedge clk) begin
        if(rst || ch_stop || ch_wdog)begin nav_forward_q12<=0;nav_yaw_q12<=0;end
        else if(ch_valid)begin nav_forward_q12<=vx_product/1000;nav_yaw_q12<=wz_product/1000;end
    end
`endif
    control4500_core u_control(
        .clk(clk),.reset_n(~rst),.nav_forward_q12(nav_forward_q12),.nav_yaw_q12(nav_yaw_q12),
        .feedback_valid(odo_valid),.feedback_vx_mmps(odo_vx),.feedback_wz_mradps(odo_wz),
`ifdef G2_REAL_SFP
        .telemetry_valid(tap_valid),.telemetry_first(tap_first),.telemetry_last(tap_last),.telemetry_data(tap_data),
`endif
        .UART_RXD(1'b1),.UART_TXD(control_txd),.IRIN(IRIN),.sck(sck),.sda(sda),
        .BMI088_SCL(BMI088_SCL),.BMI088_SDA(BMI088_SDA),
        .M1_ENCA(M1_ENCA),.M1_ENCB(M1_ENCB),.M2_ENCA(M2_ENCA),.M2_ENCB(M2_ENCB),
        .M1_PWM(M1_PWM),.M1_DIR1(M1_DIR1),.M1_DIR2(M1_DIR2),
        .M2_PWM(M2_PWM),.M2_DIR1(M2_DIR1),.M2_DIR2(M2_DIR2),
        .SERVO1(SERVO1),.SERVO2(SERVO2),.SERVO3(SERVO3),.SERVO4(SERVO4));
    nav_subsystem #(.CLK_HZ(CLK_HZ), .SIN_INIT("sin_lut.hex"),.SUPPORT_COVERAGE_ENABLE(`G2_COVERAGE)
`ifdef G2_REAL_SFP
        ,.HOST_EXTENSION_ENABLE(1)
`endif
    ) u_nav (
        .clk(clk), .rst(rst),
        .bus_req(s_req && !sfp_sel), .bus_we(s_we), .bus_addr(s_addr), .bus_wdata(s_wdata), .bus_ack(nav_ack), .bus_rdata(nav_rdata),
        .irq(irq),
        .l_rx_clk(l_rx_clk), .l_rx_rst(l_rx_rst), .l_rx_valid(l_rx_valid), .l_rx_data(l_rx_data),
        .l_rx_last(l_rx_last), .l_rx_error(l_rx_err),
        .l_tx_clk(l_tx_clk), .l_tx_rst(l_tx_rst), .l_tx_valid(l_tx_valid), .l_tx_data(l_tx_data),
        .l_tx_last(l_tx_last), .l_tx_ready(l_tx_ready), .l_phy_stat(l_phy_stat), .l_rx_neg(l_rx_neg),
        .h_core_rst(h_core_rst),.control_tap_valid(tap_valid),.control_tap_first(tap_first),
        .control_tap_last(tap_last),.control_tap_data(tap_data),
        .control_tx_accepted(telemetry_accepted),.control_tx_dropped(telemetry_dropped),.control_tx_sent(telemetry_sent),
        .h_rx_clk(h_rx_clk), .h_rx_rst(h_rx_rst), .h_rx_valid(h_rx_valid), .h_rx_data(h_rx_data),
        .h_rx_last(h_rx_last), .h_rx_error(h_rx_err),
        .h_tx_clk(h_tx_clk), .h_tx_rst(h_tx_rst), .h_tx_valid(h_tx_valid), .h_tx_data(h_tx_data),
        .h_tx_last(h_tx_last), .h_tx_ready(h_tx_ready),
        .imu_rx(imu_rxd), .imu_tx(),
        .ch_vx(ch_vx), .ch_vy(), .ch_wz(ch_wz), .ch_mode(), .ch_stop(ch_stop), .ch_wdog(ch_wdog), .ch_valid(ch_valid), .ch_seq(ch_seq),
        .odo_valid(odo_valid), .odo_vx(odo_vx), .odo_vy(16'd0), .odo_wz(odo_wz),
        .ext_stop(4'd0)
    );

    // 状态灯：心跳 / 中断 / 急停 / 指令超时
    reg [25:0] hb;
    always @(posedge clk) hb <= hb + 1'b1;
    assign led = {ch_wdog, ch_stop, irq, hb[25]};
endmodule
