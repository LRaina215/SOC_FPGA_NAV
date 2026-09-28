// 以太网帧发送：ARP 应答（优先）与 UDP 报文（负载来自邮箱发送缓冲）。
// 输出为完整以太网帧字节流（不含前导码与 FCS，由 MAC 添加），不足 60 字节补零。
// UDP 校验和填 0（IPv4 允许），IP 头置 DF，TTL=64。
module udp_tx #(
    parameter integer PL_AW = 11          // 负载缓冲地址位宽（字节）
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [47:0] my_mac,
    input  wire [31:0] my_ip,

    input  wire        arp_req,           // 单拍：需要应答
    input  wire [47:0] arp_sha,
    input  wire [31:0] arp_spa,

    input  wire        tx_go,             // 单拍：发送一个 UDP 报文
    input  wire [47:0] tx_dst_mac,
    input  wire [31:0] tx_dst_ip,
    input  wire [15:0] tx_dst_port,
    input  wire [15:0] tx_src_port,
    input  wire [15:0] tx_len,            // 负载字节数（<= 1472）
    output wire        tx_busy,

    output wire             pl_re,        // 负载缓冲读口（下一拍出数）
    output wire [PL_AW-1:0] pl_addr,
    input  wire [7:0]       pl_q,

    output wire        m_valid,
    output wire [7:0]  m_data,
    output wire        m_last,
    input  wire        m_ready,
    output reg         st_tx               // 单拍：发出一帧
);
    reg        arp_pend;
    reg [47:0] a_mac;
    reg [31:0] a_ip;
    reg        udp_pend;
    reg [47:0] u_mac;
    reg [31:0] u_ip;
    reg [15:0] u_dport, u_sport, u_len;

    reg        act, is_arp;
    reg [10:0] i, flen;
    reg [15:0] ident;
    reg [15:0] ipcs;

    assign tx_busy = udp_pend | (act & ~is_arp);

    // IP 头校验和
    wire [15:0] tot = 16'd28 + u_len;
    reg  [19:0] cp1, cp2, cp3, s0_r;       // 两级加法树
    wire [16:0] s1 = s0_r[15:0] + s0_r[19:16];
    wire [15:0] s2 = s1[15:0] + s1[16];
    reg  [1:0]  cs_age;             // UDP 参数锁存后经过的拍数（校验和两级流水）

    // 第 1 级：按字节序号生成
    wire [15:0] ulen = 16'd8 + u_len;
    reg  [7:0]  hb;
    reg         isp;
    always @(*) begin
        hb = 8'h00; isp = 1'b0;
        if (is_arp) begin
            case (i)
                0: hb = a_mac[47:40]; 1: hb = a_mac[39:32]; 2: hb = a_mac[31:24];
                3: hb = a_mac[23:16]; 4: hb = a_mac[15:8];  5: hb = a_mac[7:0];
                6: hb = my_mac[47:40]; 7: hb = my_mac[39:32]; 8: hb = my_mac[31:24];
                9: hb = my_mac[23:16]; 10: hb = my_mac[15:8]; 11: hb = my_mac[7:0];
                12: hb = 8'h08; 13: hb = 8'h06; 14: hb = 8'h00; 15: hb = 8'h01;
                16: hb = 8'h08; 17: hb = 8'h00; 18: hb = 8'd6; 19: hb = 8'd4;
                20: hb = 8'h00; 21: hb = 8'h02;
                22: hb = my_mac[47:40]; 23: hb = my_mac[39:32]; 24: hb = my_mac[31:24];
                25: hb = my_mac[23:16]; 26: hb = my_mac[15:8]; 27: hb = my_mac[7:0];
                28: hb = my_ip[31:24]; 29: hb = my_ip[23:16]; 30: hb = my_ip[15:8]; 31: hb = my_ip[7:0];
                32: hb = a_mac[47:40]; 33: hb = a_mac[39:32]; 34: hb = a_mac[31:24];
                35: hb = a_mac[23:16]; 36: hb = a_mac[15:8];  37: hb = a_mac[7:0];
                38: hb = a_ip[31:24]; 39: hb = a_ip[23:16]; 40: hb = a_ip[15:8]; 41: hb = a_ip[7:0];
                default: hb = 8'h00;
            endcase
        end else begin
            case (i)
                0: hb = u_mac[47:40]; 1: hb = u_mac[39:32]; 2: hb = u_mac[31:24];
                3: hb = u_mac[23:16]; 4: hb = u_mac[15:8];  5: hb = u_mac[7:0];
                6: hb = my_mac[47:40]; 7: hb = my_mac[39:32]; 8: hb = my_mac[31:24];
                9: hb = my_mac[23:16]; 10: hb = my_mac[15:8]; 11: hb = my_mac[7:0];
                12: hb = 8'h08; 13: hb = 8'h00; 14: hb = 8'h45; 15: hb = 8'h00;
                16: hb = tot[15:8]; 17: hb = tot[7:0]; 18: hb = ident[15:8]; 19: hb = ident[7:0];
                20: hb = 8'h40; 21: hb = 8'h00; 22: hb = 8'd64; 23: hb = 8'd17;
                24: hb = ipcs[15:8]; 25: hb = ipcs[7:0];
                26: hb = my_ip[31:24]; 27: hb = my_ip[23:16]; 28: hb = my_ip[15:8]; 29: hb = my_ip[7:0];
                30: hb = u_ip[31:24]; 31: hb = u_ip[23:16]; 32: hb = u_ip[15:8]; 33: hb = u_ip[7:0];
                34: hb = u_sport[15:8]; 35: hb = u_sport[7:0]; 36: hb = u_dport[15:8]; 37: hb = u_dport[7:0];
                38: hb = ulen[15:8]; 39: hb = ulen[7:0]; 40: hb = 8'h00; 41: hb = 8'h00;
                default: begin
                    hb  = 8'h00;
                    isp = ({5'd0, i} < 16'd42 + u_len);
                end
            endcase
        end
    end

    // 输出缓冲：生成领先一拍，余量 >= 3 时推进
    wire [5:0] lvl;
    wire [4:0] ffree;
    wire go1 = act && (ffree >= 5'd3);
    assign pl_re   = go1 & isp;
    assign pl_addr = i - 11'd42;

    reg       v2, p2, l2;
    reg [7:0] h2;

    nav_fifo #(.DW(9), .AW(4)) u_ob (
        .clk(clk), .rst(rst), .s_valid(v2), .s_data({l2, p2 ? pl_q : h2}), .s_ready(),
        .m_valid(m_valid), .m_data({m_last, m_data}), .m_ready(m_ready), .level(lvl), .free(ffree)
    );

    always @(posedge clk) begin
        v2 <= 1'b0; st_tx <= 1'b0;
        cp1 <= 20'h4500 + tot + ident;
        cp2 <= 20'h8011 + my_ip[31:16] + my_ip[15:0];
        cp3 <= u_ip[31:16] + u_ip[15:0];
        s0_r <= cp1 + cp2 + cp3;
        if (udp_pend && cs_age != 2'd3) cs_age <= cs_age + 1'b1;
        if (rst) begin
            arp_pend <= 1'b0; udp_pend <= 1'b0; act <= 1'b0; ident <= 16'd0;
        end else begin
            if (arp_req) begin arp_pend <= 1'b1; a_mac <= arp_sha; a_ip <= arp_spa; end
            if (tx_go && !udp_pend && !(act && !is_arp)) begin
                udp_pend <= 1'b1; u_mac <= tx_dst_mac; u_ip <= tx_dst_ip; cs_age <= 2'd0;
                u_dport <= tx_dst_port; u_sport <= tx_src_port; u_len <= tx_len;
            end
            if (!act) begin
                if (arp_pend && !arp_req) begin
                    act <= 1'b1; is_arp <= 1'b1; i <= 0; flen <= 11'd60; arp_pend <= 1'b0;
                end else if (udp_pend && !tx_go && cs_age == 2'd3) begin
                    act <= 1'b1; is_arp <= 1'b0; i <= 0; udp_pend <= 1'b0;
                    flen <= (u_len + 16'd42 < 16'd60) ? 11'd60 : u_len[10:0] + 11'd42;
                    ipcs <= ~s2;
                end
            end else if (go1) begin
                v2 <= 1'b1; p2 <= isp; h2 <= hb; l2 <= (i == flen - 1);
                if (i == flen - 1) begin
                    act <= 1'b0; st_tx <= 1'b1;
                    if (!is_arp) ident <= ident + 1'b1;
                end
                i <= i + 1'b1;
            end
        end
    end
endmodule
