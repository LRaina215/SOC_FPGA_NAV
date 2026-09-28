// 以太网帧接收解析：Ethernet II -> ARP / IPv4 -> UDP。
// 输入为 MAC 客户端字节流（不含前导码，FCS 有无均可：按 IP 总长截取负载，多余字节忽略）。
// 输出：
//   · 目的端口匹配的 UDP 负载字节流；最后一个字节被扣留到帧结束再发出，
//     这样 m_last 拍的 m_err 能反映整帧的 MAC 错误（FCS）。
//   · ARP 请求（目标 IP 为本机）在帧结束且无错时给出 arp_req 脉冲。
// 负载报文的元数据（源 MAC/IP/端口、首字节到达时间）在 m_first 拍之前锁存，报文期间保持。
module udp_rx (
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] time_us,
    input  wire [47:0] my_mac,
    input  wire [31:0] my_ip,
    input  wire [15:0] my_port,
    input  wire        cfg_enable,
    input  wire        cfg_any_ip,      // 不检查目的 IP
    input  wire        cfg_arp_en,

    input  wire        s_valid,
    input  wire [7:0]  s_data,
    input  wire        s_last,
    input  wire        s_err,

    output reg         m_valid,
    output reg  [7:0]  m_data,
    output reg         m_last,
    output reg         m_err,
    output reg  [47:0] pkt_src_mac,
    output reg  [31:0] pkt_src_ip,
    output reg  [15:0] pkt_src_port,
    output reg  [15:0] pkt_len,         // UDP 负载字节数
    output reg  [31:0] pkt_ts,

    output reg         arp_req,         // 单拍
    output reg  [47:0] arp_sha,
    output reg  [31:0] arp_spa,

    output reg         st_frame,        // 单拍统计脉冲
    output reg         st_udp,
    output reg         st_err
);
    reg [10:0] bc;                 // 帧内字节序号（饱和）
    reg [47:0] dmac, smac;
    reg [15:0] etype;
    reg        is_ip, is_arp;
    reg [3:0]  ihl;
    reg [15:0] ip_tot;
    reg [7:0]  proto;
    reg [31:0] sip, dip;
    reg [16:0] csum;               // 头校验和累加（进位回卷）
    reg [7:0]  hi;                 // 16 位字高字节暂存
    reg [15:0] uoff;               // UDP 头起始字节序号 = 14 + ihl*4
    reg [15:0] sport, dport, ulen;
    reg [15:0] pay_end;            // 负载结束序号（不含）
    reg        deliver;            // 本帧负载需要输出
    reg        bad;                // 帧格式/校验错
    reg        hold_v;             // 扣留的字节
    reg [7:0]  hold_d;
    reg [31:0] ts0;
    // ARP 字段
    reg [15:0] arp_op;
    reg [47:0] a_sha;
    reg [31:0] a_spa, a_tpa;

    wire [15:0] b = {5'd0, bc};
    // UDP 头起点及其固定偏移在 IHL 到达后的下一拍寄存（最早使用在第 34 字节）
    reg  [15:0] uoff_w, uo1, uo3, uo5, uo7, uo8;
    wire        pay_byte = deliver && (b >= uo8) && (b < pay_end);
    reg         tail_v;
    reg  [7:0]  tail_d;
    reg         tail_e;
    wire [15:0] w16 = {hi, s_data};
    // 地址比较结果寄存：目的 MAC 在第 6 字节后、ARP 目标 IP 在第 42 字节后使用
    //（以太网最短帧 60 字节，不含 FCS，ARP 帧必有填充，末字节在 b>=59）
    reg  mac_ok, tpa_ok;
    always @(posedge clk) begin
        mac_ok <= (dmac == my_mac) || (dmac == 48'hFFFFFFFFFFFF);
        tpa_ok <= (a_tpa == my_ip);
    end
    // 16 位反码累加
    wire [16:0] cs_add = {1'b0, csum[15:0]} + {1'b0, w16};
    wire [15:0] cs_new = cs_add[15:0] + cs_add[16];

    always @(posedge clk) begin
        m_valid <= 1'b0; arp_req <= 1'b0; st_frame <= 1'b0; st_udp <= 1'b0; st_err <= 1'b0;
        if (rst) begin
            bc <= 0; hold_v <= 1'b0; deliver <= 1'b0; bad <= 1'b0; csum <= 0; tail_v <= 1'b0;
            is_ip <= 1'b0; is_arp <= 1'b0;
        end else begin
          if (tail_v) begin
            // 补发末字节。下一帧前 42 字节不会产生负载输出，所以与本拍的输入处理不冲突
            tail_v <= 1'b0;
            m_valid <= 1'b1; m_data <= tail_d; m_last <= 1'b1; m_err <= tail_e;
          end
          if (s_valid && cfg_enable) begin
            if (bc != 11'h7FF) bc <= bc + 1'b1;
            if (b[0] == 1'b0) hi <= s_data;
            if (bc == 0) ts0 <= time_us;

            // ---- 以太网头 ----
            if (b < 6)  dmac <= {dmac[39:0], s_data};
            else if (b < 12) smac <= {smac[39:0], s_data};
            else if (b == 13) begin etype <= w16; is_ip <= (w16 == 16'h0800); is_arp <= (w16 == 16'h0806); end

            // ---- IPv4 头 ----
            if (is_ip || b < 14) begin
                if (b == 14) begin
                    ihl <= s_data[3:0];
                    if (s_data[7:4] != 4'd4 || s_data[3:0] < 4'd5) bad <= 1'b1;
                    csum <= 0;
                end
                if (b >= 14 && b[0] == 1'b1 && (b < 34 || b < uoff_w)) csum <= {1'b0, cs_new};
                if (b == 17) ip_tot <= w16;
                if (b == 20 && (s_data[5] || s_data[4:0] != 0)) bad <= 1'b1;         // MF 或分片偏移：不支持分片
                if (b == 21 && s_data != 0) bad <= 1'b1;
                if (b == 23) proto <= s_data;
                if (b >= 26 && b < 30) sip <= {sip[23:0], s_data};
                if (b >= 30 && b < 34) dip <= {dip[23:0], s_data};
                // UDP 头
                if (b == 15) begin
                    uoff_w <= 16'd14 + {10'd0, ihl, 2'b00};
                    uo1 <= 16'd15 + {10'd0, ihl, 2'b00};
                    uo3 <= 16'd17 + {10'd0, ihl, 2'b00};
                    uo5 <= 16'd19 + {10'd0, ihl, 2'b00};
                    uo7 <= 16'd21 + {10'd0, ihl, 2'b00};
                    uo8 <= 16'd22 + {10'd0, ihl, 2'b00};
                end
                if (b == uo1) sport <= w16;
                if (b == uo3) dport <= w16;
                if (b == uo5) ulen  <= w16;
                if (b == uo7) begin
                    // 此时 IP 头已全部参与校验
                    deliver <= !bad && mac_ok && is_ip && (proto == 8'd17) &&
                               (cfg_any_ip || dip == my_ip) && (csum[15:0] == 16'hFFFF) &&
                               (dport == my_port) && (ulen >= 16'd8) &&
                               ({1'b0, ulen} + {1'b0, uoff_w} <= {1'b0, ip_tot} + 17'd14) &&
                               (ulen > 16'd8);
                    pay_end <= uoff_w + ulen;
                    pkt_src_mac <= smac; pkt_src_ip <= sip; pkt_src_port <= sport;
                    pkt_len <= ulen - 16'd8; pkt_ts <= ts0;
                end
                // 负载
                if (pay_byte) begin
                    if (hold_v) begin
                        m_valid <= 1'b1; m_data <= hold_d; m_last <= 1'b0; m_err <= 1'b0;
                    end
                    hold_v <= 1'b1; hold_d <= s_data;
                end
            end

            // ---- ARP ----
            if (is_arp) begin
                if (b == 21) arp_op <= w16;
                if (b >= 22 && b < 28) a_sha <= {a_sha[39:0], s_data};
                if (b >= 28 && b < 32) a_spa <= {a_spa[23:0], s_data};
                if (b >= 38 && b < 42) a_tpa <= {a_tpa[23:0], s_data};
            end

            // ---- 帧结束 ----
            if (s_last) begin
                bc <= 0; deliver <= 1'b0; bad <= 1'b0; hold_v <= 1'b0; etype <= 16'h0000;
                is_ip <= 1'b0; is_arp <= 1'b0;
                st_frame <= 1'b1;
                if (pay_byte && hold_v) begin
                    // 本字节就是负载末字节：先发扣留字节，本字节下一拍作为末字节发出
                    m_valid <= 1'b1; m_data <= hold_d; m_last <= 1'b0; m_err <= 1'b0;
                    tail_v <= 1'b1; tail_d <= s_data; tail_e <= s_err || (b + 1 < pay_end);
                    st_udp <= !(s_err || (b + 1 < pay_end));
                    st_err <= s_err || (b + 1 < pay_end);
                end else if (pay_byte || hold_v) begin
                    // 负载被截断（帧比 UDP 长度短）时 b < pay_end，也按错误报文交付
                    m_valid <= 1'b1; m_data <= pay_byte ? s_data : hold_d; m_last <= 1'b1;
                    m_err   <= s_err || (b + 1 < pay_end);
                    st_udp  <= !(s_err || (b + 1 < pay_end));
                    st_err  <= s_err || (b + 1 < pay_end);
                end else if (s_err) st_err <= 1'b1;
                if (is_arp && cfg_arp_en && !s_err && b >= 16'd43 && mac_ok &&
                    arp_op == 16'd1 && tpa_ok) begin
                    arp_req <= 1'b1; arp_sha <= a_sha; arp_spa <= a_spa;
                end
            end
          end
        end
    end

endmodule
