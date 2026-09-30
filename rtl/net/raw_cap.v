// 原始帧接收：把"发给本机的 ARP 应答"和"发给本机 IP 的 TCP 段"整帧（前 128 字节）存进 4 格环形缓冲，交给软核
// （雷达口的最小 TCP 客户端用：软核自己做 ARP 请求与 TCP 握手，硬件只负责收发整帧）。
// 输入与 udp_rx 同一路字节流（含 FCS，末字节带错误标志）；FCS 错、格子满时丢弃（st_drop 计数）。
// 读出：rd_addr 选当前格的字（小端打包，字 k = 第 4k..4k+3 字节），下一拍 rd_q；pop 弹出当前格。
module raw_cap (
    input  wire        clk,
    input  wire        rst,
    input  wire        cfg_enable,
    input  wire [47:0] my_mac,
    input  wire [31:0] my_ip,
    input  wire        s_valid,
    input  wire [7:0]  s_data,
    input  wire        s_last,
    input  wire        s_err,
    input  wire [4:0]  rd_addr,
    output wire [31:0] rd_q,
    input  wire        pop,
    output wire [2:0]  n_avail,
    output wire [10:0] head_len,       // 当前格的整帧字节数（含 FCS；只存前 128 字节）
    output reg  [7:0]  st_drop
);
    reg  [10:0] bc;                    // 本帧已收字节数
    reg  [47:0] dmac;
    reg  [15:0] etype;
    reg  [15:0] arp_op;
    reg  [7:0]  proto;
    reg  [31:0] tip;                   // ARP 目标 IP / IPv4 目的 IP
    reg  [31:0] wacc;                  // 正在拼的字
    reg  [1:0]  wp, rp;
    reg  [2:0]  cnt;
    reg  [10:0] len [0:3];
    wire        full = (cnt == 3'd4);
    reg         blk;                   // 帧开始时缓冲已满：整帧不写（中途有格子空出来也不写，避免存半帧）
    wire        blk_now = (bc == 11'd0) ? full : blk;

    wire [31:0] wacc_n = {s_data, wacc[31:8]};
    wire        in_buf = (bc < 11'd128);
    wire        we     = s_valid && in_buf && ((bc[1:0] == 2'd3) || s_last);
    wire [31:0] wdata  = (bc[1:0] == 2'd3) ? wacc_n : (wacc_n >> (8 * (2'd3 - bc[1:0])));
    nav_ram_sdp #(.DW(32), .AW(7)) u_buf (
        .clk(clk), .we(we && !blk_now), .waddr({wp, bc[6:2]}), .wdata(wdata),
        .re(1'b1), .raddr({rp, rd_addr}), .rdata(rd_q)
    );

    // 帧末判定（末字节这一拍：目的 IP / ARP 目标 IP 的最后一个字节已在前面收过）
    wire mac_ok = (dmac == my_mac) || (dmac == 48'hFFFFFFFFFFFF);
    wire is_arp_rep = (etype == 16'h0806) && (arp_op == 16'd2) && (tip == my_ip);
    wire is_tcp     = (etype == 16'h0800) && (proto == 8'd6) && (tip == my_ip);
    wire keep = cfg_enable && mac_ok && (is_arp_rep || is_tcp) && !s_err && (bc >= 11'd41);

    assign n_avail  = cnt;
    assign head_len = len[rp];

    always @(posedge clk) begin
        if (rst) begin
            bc <= 0; wp <= 0; rp <= 0; cnt <= 0; st_drop <= 0; etype <= 0;
        end else begin
            if (s_valid) begin
                wacc <= wacc_n;
                if (bc == 11'd0) blk <= full;
                if (bc < 11'd6) dmac <= {dmac[39:0], s_data};
                if (bc == 11'd12) etype[15:8] <= s_data;
                if (bc == 11'd13) etype[7:0] <= s_data;
                if (bc == 11'd20) arp_op[15:8] <= s_data;
                if (bc == 11'd21) arp_op[7:0] <= s_data;
                if (bc == 11'd23) proto <= s_data;
                // IPv4 目的 IP 在 30–33，ARP 目标 IP 在 38–41：按以太类型分别取
                if ((etype == 16'h0800 && bc >= 11'd30 && bc < 11'd34) || (etype == 16'h0806 && bc >= 11'd38 && bc < 11'd42))
                    tip <= {tip[23:0], s_data};
                if (s_last) begin
                    bc <= 0; etype <= 0;
                    if (keep) begin
                        if (blk_now) st_drop <= st_drop + 1'b1;
                        else begin len[wp] <= bc + 1'b1; wp <= wp + 1'b1; end
                    end
                end else if (bc != 11'h7FF) bc <= bc + 1'b1;
            end
            // 计数：本拍入格与出格可能同时发生
            case ({s_valid && s_last && keep && !blk_now, pop && cnt != 0})
                2'b10: cnt <= cnt + 1'b1;
                2'b01: cnt <= cnt - 1'b1;
                default: ;
            endcase
            if (pop && cnt != 0) rp <= rp + 1'b1;
        end
    end
endmodule
