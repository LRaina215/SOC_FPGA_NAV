// RGMII 百兆以太网 MAC（只做 100 Mbps 全双工，XT-M60 就是百兆）+ PHY 复位与 MDIO 初始化。
// 替代需要在 IDE 图形界面生成的 TSE_MAC IP；引脚用法参照板卡厂家例程 eth_udp_lookback_rgmii。
//
// 接收（rgmii_rxc 域，PHY 恢复的 25 MHz）：上升沿采 RXD/RX_DV（百兆每拍一个半字节，低半字节在前），
//   下降沿采 RX_CTL（= RX_DV ^ RX_ER）。去掉前导码与 SFD，输出含 FCS 的整帧字节，末字节给出错误标志
//   （FCS 错、RX_ER、半字节数为奇数）。rx_clk = rgmii_rxc。
// 发送（clk 域）：TXC = clk 分频到 25 MHz，数据在 TXC 下降沿更新（上升沿处居中）；加前导码 + SFD，
//   不足 60 字节补零，追加 FCS，帧间隔 12 字节。客户端接口：tx_valid & tx_ready 时传一个字节。
// MDIO：上电拉低 PHY 复位 20 ms，释放后等 100 ms，扫描 PHY 地址（读 PHYID1），把自协商能力限制为
//   100BASE-TX（ANAR = 0x0181、1000BASE-T 不通告），重启自协商；之后每 200 ms 读一次 BMSR 链路状态。
//   这样接雷达或接电脑千兆网卡都会协商到百兆。
module rgmii_mac100 #(
    parameter integer CLK_HZ   = 50000000,     // 须为 50 MHz 的整数倍
    parameter integer T_RST_US = 20000,
    parameter integer T_WAIT_US = 100000,
    parameter integer T_POLL_US = 200000
) (
    input  wire       clk,
    input  wire       rst,
    // RGMII / PHY
    input  wire       rgmii_rxc,
    input  wire       rgmii_rx_ctl,
    input  wire [3:0] rgmii_rxd,
    output reg        rgmii_txc,
    output reg        rgmii_tx_ctl,
    output reg  [3:0] rgmii_txd,
    output reg        phy_rst_n,
    output reg        mdc,
    output reg        mdio_o,
    output reg        mdio_oe,
    input  wire       mdio_i,
    // MAC 客户端：接收（rx_clk 域）
    output wire       rx_clk,
    output reg        rx_valid,
    output reg  [7:0] rx_data,
    output reg        rx_last,
    output reg        rx_error,
    // MAC 客户端：发送（clk 域）
    input  wire       tx_valid,
    input  wire [7:0] tx_data,
    input  wire       tx_last,
    output reg        tx_ready,
    // 状态（clk 域）：[0] 链路 [1] 初始化完成 [2] 找到 PHY [7:3] PHY 地址 [31:16] RX 时钟 / 64 的上升沿数 / ms
    output wire [31:0] phy_stat
);
    function [31:0] crc8(input [31:0] c, input [7:0] d);
        integer k;
        reg [31:0] x;
        begin
            x = c ^ {24'd0, d};
            for (k = 0; k < 8; k = k + 1) x = x[0] ? ((x >> 1) ^ 32'hEDB88320) : (x >> 1);
            crc8 = x;
        end
    endfunction
    localparam [31:0] CRC_RES = 32'hDEBB20E3;      // 数据 + FCS 一起算完后的余式

    // ================================================================ 接收
    assign rx_clk = rgmii_rxc;
    reg [3:0] rxd_p;
    reg       dv_p, ctl_n;
    always @(posedge rgmii_rxc) begin rxd_p <= rgmii_rxd; dv_p <= rgmii_rx_ctl; end
    always @(negedge rgmii_rxc) ctl_n <= rgmii_rx_ctl;
    localparam [1:0] R_IDLE = 2'd0, R_PRE = 2'd1, R_DATA = 2'd2, R_BAD = 2'd3;
    reg [1:0]  rs;
    reg        nib, rerr, hold_v;
    reg [3:0]  lo;
    reg [7:0]  hold_d;
    reg [31:0] rcrc;
    wire       rer = dv_p ^ ctl_n;                  // 同一拍：上升沿 RX_DV，下降沿 RX_DV ^ RX_ER
    always @(posedge rgmii_rxc) begin
        rx_valid <= 1'b0;
        case (rs)
        R_IDLE: if (dv_p) rs <= (rxd_p == 4'h5) ? R_PRE : R_BAD;
        R_PRE: begin
            if (!dv_p) rs <= R_IDLE;
            else if (rxd_p == 4'hD) begin
                rs <= R_DATA; nib <= 1'b0; rerr <= 1'b0; hold_v <= 1'b0; rcrc <= 32'hFFFFFFFF;
            end else if (rxd_p != 4'h5) rs <= R_BAD;
        end
        R_DATA: begin
            if (dv_p) begin
                if (rer) rerr <= 1'b1;
                if (!nib) begin
                    lo <= rxd_p; nib <= 1'b1;
                end else begin
                    nib <= 1'b0;
                    rcrc <= crc8(rcrc, {rxd_p, lo});
                    if (hold_v) begin
                        rx_valid <= 1'b1; rx_data <= hold_d; rx_last <= 1'b0; rx_error <= 1'b0;
                    end
                    hold_v <= 1'b1; hold_d <= {rxd_p, lo};
                end
            end else begin
                rs <= R_IDLE;
                if (hold_v) begin
                    rx_valid <= 1'b1; rx_data <= hold_d; rx_last <= 1'b1;
                    rx_error <= rerr | nib | (rcrc != CRC_RES);
                end
            end
        end
        default: if (!dv_p) rs <= R_IDLE;
        endcase
    end
    initial begin rs = R_IDLE; rx_valid = 1'b0; hold_v = 1'b0; end

    // ================================================================ 发送
    localparam integer HP = (CLK_HZ / 50000000) < 1 ? 1 : (CLK_HZ / 50000000);   // TXC 半周期（clk 拍）
    reg [$clog2(HP + 1):0] hcnt;
    wire fall = (hcnt == HP - 1) && rgmii_txc;      // 本拍 TXC 由高变低：更新数据
    localparam [2:0] T_IDLE = 3'd0, T_PRE = 3'd1, T_DATA = 3'd2, T_PAD = 3'd3, T_FCS = 3'd4, T_IFG = 3'd5;
    reg [2:0]  ts;
    reg [4:0]  tn;          // 前导码 / 帧间隔 半字节计数
    reg        tnib;        // 0 = 发低半字节
    reg [7:0]  tb;
    reg        tb_last;
    reg [6:0]  tlen;        // 已发数据字节数（饱和到 64）
    reg [31:0] tcrc;
    reg [2:0]  fi;          // FCS 半字节序号
    always @(posedge clk) begin
        tx_ready <= 1'b0;
        if (rst) begin
            rgmii_txc <= 1'b0; hcnt <= 0; ts <= T_IDLE; rgmii_tx_ctl <= 1'b0; rgmii_txd <= 4'd0;
        end else begin
            hcnt <= (hcnt == HP - 1) ? 0 : hcnt + 1'b1;
            if (hcnt == HP - 1) rgmii_txc <= ~rgmii_txc;
            if (fall) begin
                case (ts)
                T_IDLE: begin
                    rgmii_tx_ctl <= 1'b0; rgmii_txd <= 4'd0;
                    if (tx_valid) begin ts <= T_PRE; tn <= 0; end
                end
                T_PRE: begin
                    rgmii_tx_ctl <= 1'b1;
                    rgmii_txd <= (tn == 5'd15) ? 4'hD : 4'h5;
                    tn <= tn + 1'b1;
                    if (tn == 5'd15) begin
                        ts <= T_DATA; tnib <= 1'b0; tlen <= 0; tcrc <= 32'hFFFFFFFF;
                        tb <= tx_data; tb_last <= tx_last; tx_ready <= 1'b1;     // 取第一个字节
                    end
                end
                T_DATA: begin
                    if (!tnib) begin
                        rgmii_txd <= tb[3:0]; tnib <= 1'b1;
                    end else begin
                        rgmii_txd <= tb[7:4]; tnib <= 1'b0;
                        tcrc <= crc8(tcrc, tb);
                        if (tlen != 7'd64) tlen <= tlen + 1'b1;
                        if (tb_last) begin
                            ts <= (tlen + 1'b1 < 7'd60) ? T_PAD : T_FCS; fi <= 0;
                        end else begin
                            tb <= tx_data; tb_last <= tx_last; tx_ready <= 1'b1;
                        end
                    end
                end
                T_PAD: begin
                    rgmii_txd <= 4'd0;
                    if (!tnib) tnib <= 1'b1;
                    else begin
                        tnib <= 1'b0;
                        tcrc <= crc8(tcrc, 8'd0);
                        tlen <= tlen + 1'b1;
                        if (tlen + 1'b1 == 7'd60) begin ts <= T_FCS; fi <= 0; end
                    end
                end
                T_FCS: begin
                    rgmii_txd <= ~tcrc[4 * fi +: 4];
                    fi <= fi + 1'b1;
                    if (fi == 3'd7) begin ts <= T_IFG; tn <= 0; end
                end
                default: begin
                    rgmii_tx_ctl <= 1'b0; rgmii_txd <= 4'd0;
                    tn <= tn + 1'b1;
                    if (tn == 5'd23) ts <= T_IDLE;
                end
                endcase
            end
        end
    end

    // ================================================================ MDIO
    localparam integer MDIV = (CLK_HZ / 3000000) < 2 ? 2 : (CLK_HZ / 3000000);   // MDC ≈ 1.5 MHz
    localparam integer US = CLK_HZ / 1000000;
    reg [$clog2(MDIV + 1):0] mcnt;
    reg        m_go, m_rd, m_busy;
    reg [4:0]  m_pa, m_ra;
    reg [15:0] m_wd, m_q;
    reg [6:0]  mbit;
    reg [63:0] msr;
    always @(posedge clk) begin
        if (rst) begin
            mdc <= 1'b0; mcnt <= 0; m_busy <= 1'b0; mdio_oe <= 1'b0; mdio_o <= 1'b1;
        end else begin
            if (m_go && !m_busy) begin
                m_busy <= 1'b1; mbit <= 0; mcnt <= 0; mdc <= 1'b0;
                msr <= {32'hFFFFFFFF, 2'b01, m_rd ? 2'b10 : 2'b01, m_pa, m_ra, m_rd ? 2'b11 : 2'b10, m_rd ? 16'hFFFF : m_wd};
                mdio_oe <= 1'b1; mdio_o <= 1'b1;
            end else if (m_busy) begin
                mcnt <= (mcnt == MDIV - 1) ? 0 : mcnt + 1'b1;
                if (mcnt == MDIV - 1) begin
                    mdc <= ~mdc;
                    if (mdc) begin
                        // 下降沿：送出下一位
                        if (mbit == 7'd64) begin
                            m_busy <= 1'b0; mdio_oe <= 1'b0;
                        end else begin
                            mdio_o  <= msr[63];
                            mdio_oe <= !(m_rd && mbit >= 7'd46);
                            msr <= {msr[62:0], 1'b1};
                            mbit <= mbit + 1'b1;
                        end
                    end else begin
                        // 上升沿：读操作采数据位
                        if (m_rd && mbit >= 7'd49) m_q <= {m_q[14:0], mdio_i};
                    end
                end
            end
        end
    end

    // 初始化与轮询
    localparam [3:0] I_RST = 4'd0, I_WAIT = 4'd1, I_SCAN = 4'd2, I_SCANW = 4'd3, I_W4 = 4'd4, I_W9 = 4'd5,
                     I_W0 = 4'd6, I_POLL = 4'd7, I_POLLW = 4'd8, I_WW = 4'd9;
    reg [3:0]  is, inext;
    reg [31:0] itmr;
    reg        link, found, idone;
    reg [4:0]  paddr;
    always @(posedge clk) begin
        m_go <= 1'b0;
        if (rst) begin
            is <= I_RST; itmr <= 0; phy_rst_n <= 1'b0; link <= 1'b0; found <= 1'b0; idone <= 1'b0; paddr <= 0;
        end else begin
            itmr <= itmr + 1'b1;
            case (is)
            I_RST:  if (itmr >= T_RST_US * US) begin phy_rst_n <= 1'b1; itmr <= 0; is <= I_WAIT; end
            I_WAIT: if (itmr >= T_WAIT_US * US) begin is <= I_SCAN; paddr <= 0; end
            I_SCAN: if (!m_busy) begin m_go <= 1'b1; m_rd <= 1'b1; m_pa <= paddr; m_ra <= 5'd2; is <= I_SCANW; end
            I_SCANW: if (!m_busy && !m_go) begin
                if (m_q != 16'hFFFF && m_q != 16'h0000) begin found <= 1'b1; is <= I_W4; end
                else begin
                    paddr <= paddr + 1'b1;
                    if (paddr == 5'd31) begin is <= I_WAIT; itmr <= 0; end     // 没找到：过一会儿重扫
                    else is <= I_SCAN;
                end
            end
            I_W4: if (!m_busy) begin m_go <= 1'b1; m_rd <= 1'b0; m_pa <= paddr; m_ra <= 5'd4; m_wd <= 16'h0181; is <= I_WW; inext <= I_W9; end
            I_W9: if (!m_busy) begin m_go <= 1'b1; m_rd <= 1'b0; m_pa <= paddr; m_ra <= 5'd9; m_wd <= 16'h0000; is <= I_WW; inext <= I_W0; end
            I_W0: if (!m_busy) begin m_go <= 1'b1; m_rd <= 1'b0; m_pa <= paddr; m_ra <= 5'd0; m_wd <= 16'h1200; is <= I_WW; inext <= I_POLL; end
            I_WW:  if (!m_busy && !m_go) begin is <= inext; itmr <= 0; if (inext == I_POLL) idone <= 1'b1; end
            I_POLL: if (itmr >= T_POLL_US * US && !m_busy) begin m_go <= 1'b1; m_rd <= 1'b1; m_pa <= paddr; m_ra <= 5'd1; is <= I_POLLW; end
            I_POLLW: if (!m_busy && !m_go) begin link <= m_q[2]; itmr <= 0; is <= I_POLL; end
            default: is <= I_RST;
            endcase
        end
    end

    // RX 时钟测频：rxc/64 的上升沿数 / ms
    reg [5:0]  rdiv;
    always @(posedge rgmii_rxc) rdiv <= rdiv + 1'b1;
    initial rdiv = 0;
    reg [2:0]  rsync;
    reg [15:0] fcnt, fq;
    reg [31:0] fwin;
    always @(posedge clk) begin
        rsync <= {rsync[1:0], rdiv[5]};
        if (rst) begin fwin <= 0; fcnt <= 0; fq <= 0; end
        else if (fwin == US * 1000 - 1) begin fwin <= 0; fq <= fcnt; fcnt <= 0; end
        else begin
            fwin <= fwin + 1'b1;
            if (rsync[2:1] == 2'b01) fcnt <= fcnt + 1'b1;
        end
    end
    assign phy_stat = {fq, 8'd0, paddr, found, idone, link};
endmodule
