// XT-M60 图像帧接收：UDP 负载字节流 -> 乒乓帧缓冲 -> 校验通过后输出像素流。
//
// 输入：雷达端口 UDP 负载（每个 UDP 报文以 s_last 结束，s_err 表示该报文有错）。
// 输出：逐像素 (u, v, 原始深度)，深度 0 表示无效；m_sof/m_eof 标记帧首尾。
//       一帧在 END 标记、长度、分辨率都校验通过后才开始输出，丢包的坏帧整帧丢弃。
// 帧格式与可配置项见 model/xt_proto.py。
module xt_frame_rx #(
    parameter integer MAX_W      = 160,
    parameter integer MAX_H      = 60,
    parameter integer CLK_HZ     = 100000000,
    parameter integer TIMEOUT_US = 3000       // 帧内字节间隔超过此值视为丢包，放弃当前帧
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] time_us,

    // 配置
    input  wire        cfg_enable,
    input  wire        cfg_hdr_le,
    input  wire        cfg_pix_le,
    input  wire [1:0]  cfg_layout,     // type=0 时: 0 逐像素交织, 1 平面（整幅深度后整幅幅值）
    input  wire        cfg_amp_first,  // 交织时幅值在前
    input  wire [15:0] cfg_udp_skip,
    input  wire [15:0] cfg_size_adj,   // 有符号: data_len = size + size_adj
    input  wire [15:0] cfg_amp_min,
    input  wire [15:0] cfg_raw_inv,    // 非 0 时，原始深度 >= 此值视为无效
    input  wire        cfg_wait_pose,
    input  wire        pose_commit,    // 单拍

    // UDP 负载输入（不反压）
    input  wire        s_valid,
    input  wire [7:0]  s_data,
    input  wire        s_last,
    input  wire        s_err,

    // 像素输出
    output wire        m_valid,
    input  wire        m_ready,
    output wire [7:0]  m_u,
    output wire [7:0]  m_v,
    output wire [15:0] m_depth,
    output wire        m_sof,
    output wire        m_eof,

    // 当前输出帧（等待位姿或正在输出）的信息，帧期间保持
    output reg  [15:0] frm_sn,
    output reg  [15:0] frm_w,
    output reg  [15:0] frm_h,
    output reg  [31:0] frm_ts,
    output reg  [7:0]  frm_devstate,
    output reg  [7:0]  frm_proto,
    output wire        pose_waiting,
    output reg         frame_start,    // 单拍：开始输出一帧
    output reg         frame_ready,    // 单拍：一帧校验通过进入缓冲

    output reg  [31:0] stat_frames,
    output reg  [31:0] stat_bad,
    output reg  [31:0] stat_drop
);
    localparam integer NPIX = MAX_W * MAX_H;
    localparam integer AW   = $clog2(2 * NPIX);
    localparam [31:0] START_W = 32'h7EFFAA55;
    localparam [31:0] END_W   = 32'hFF7E55AA;
    localparam integer TW = $clog2(TIMEOUT_US + 2);

    // ------------------------------------------------------------------
    // 写侧：解析
    // ------------------------------------------------------------------
    localparam [3:0] S_HUNT = 4'd0, S_SIZE = 4'd1, S_HDR = 4'd2, S_PIX = 4'd3,
                     S_INFO = 4'd4, S_END = 4'd5;
    reg  [3:0]  st;
    reg  [15:0] skip_cnt;       // 本报文已跳过的字节
    reg         in_pkt;         // 报文中间（已过首字节）
    reg  [31:0] win;            // 最近 4 个数据字节
    reg  [2:0]  n4;             // 多字节字段已收字节数
    reg  [31:0] size_f;
    reg  [3:0]  hcnt;           // 头部（cmd..H）字节序号 0..7
    reg  [7:0]  cmdid, itype;
    reg  [15:0] sn_f, w_f, h_f;
    reg  [15:0] npix;           // W*H
    reg  [17:0] pix_bytes;      // 像素区总字节
    reg  [17:0] pcnt;           // 像素区已收字节
    reg  [20:0] info_left;      // FrameInfo 剩余字节
    reg  [7:0]  last1, last2;   // FrameInfo 最后两个字节
    reg  [15:0] wword;          // 像素字拼接
    reg  [15:0] dhold;          // 交织时暂存深度
    reg  [13:0] pidx;           // 像素序号
    reg         discard;        // 缓冲不可用，本帧只解析不写入
    reg         bad_pkt;        // 本帧内出现过错误报文
    reg  [31:0] ts_f;
    reg  [TW-1:0] idle_us;
    reg  [$clog2(CLK_HZ/1000000)-1:0] usdiv;

    // 帧缓冲管理
    reg        wb, rb;
    reg  [1:0] full;
    reg [15:0] m_sn [0:1];
    reg [15:0] m_w  [0:1];
    reg [15:0] m_h  [0:1];
    reg [31:0] m_ts [0:1];
    reg [7:0]  m_ds [0:1];
    reg [7:0]  m_pv [0:1];

    // 帧缓冲 RAM 写口
    reg          fb_we;
    reg [AW-1:0] fb_wa;
    reg [15:0]   fb_wd;

    wire [15:0] w16  = cfg_hdr_le ? {s_data, win[7:0]} : {win[7:0], s_data};
    wire [15:0] p16  = cfg_pix_le ? {s_data, wword[7:0]} : {wword[7:0], s_data};
    wire [31:0] w32n = {win[23:0], s_data};
    wire [31:0] w32  = cfg_hdr_le ? {s_data, win[7:0], win[15:8], win[23:16]} : w32n;
    reg  [20:0] data_len;       // size + SIZE_ADJ，头部阶段寄存
    reg         len_ok;         // data_len >= 像素字节 + 12
    reg  [20:0] info_init;      // FrameInfo 字节数
    wire        dvalid_raw = (p16 != 16'd0) && ((cfg_raw_inv == 16'd0) || (p16 < cfg_raw_inv));
    wire [AW-1:0] bank_base = wb ? NPIX[AW-1:0] : {AW{1'b0}};

    // 手册只给出标记的数值，未说明字节序：两种字节序都接受
    wire is_start = (w32n == START_W) || (w32n == {START_W[7:0], START_W[15:8], START_W[23:16], START_W[31:24]});
    wire is_end   = (w32n == END_W)   || (w32n == {END_W[7:0], END_W[15:8], END_W[23:16], END_W[31:24]});
    wire [15:0] wh_prod  = {8'd0, w_f[7:0]} * {8'd0, w16[7:0]};
    wire        end_ok   = (st == S_END) && (n4 == 3'd3) && is_end && !bad_pkt && !discard;

    wire in_byte = s_valid & cfg_enable;
    wire skipping = in_byte & (skip_cnt < cfg_udp_skip);
    wire dbyte = in_byte & ~skipping;         // 有效数据字节

    // 帧内空闲超时
    wire us_tick = (usdiv == CLK_HZ/1000000 - 1);
    wire timeout = (st != S_HUNT) && (idle_us >= TIMEOUT_US);

    task frame_fail;
        begin
            st <= S_HUNT;
            if (!discard) stat_bad <= stat_bad + 1'b1;
        end
    endtask

    always @(posedge clk) begin
        fb_we       <= 1'b0;
        frame_ready <= 1'b0;
        if (rst) begin
            st <= S_HUNT; skip_cnt <= 0; in_pkt <= 1'b0; win <= 0; n4 <= 0; hcnt <= 0;
            wb <= 1'b0; discard <= 1'b0; bad_pkt <= 1'b0; idle_us <= 0; usdiv <= 0;
            stat_frames <= 0; stat_bad <= 0; stat_drop <= 0;
            pcnt <= 0; pidx <= 0;
        end else begin
            // 空闲计时
            usdiv <= us_tick ? 0 : usdiv + 1'b1;
            if (in_byte) idle_us <= 0;
            else if (us_tick && idle_us < TIMEOUT_US) idle_us <= idle_us + 1'b1;

            if (in_byte) begin
                skip_cnt <= s_last ? 16'd0 : (skipping ? skip_cnt + 1'b1 : skip_cnt);
                if (s_last && s_err) bad_pkt <= 1'b1;
            end

            if (timeout) begin
                frame_fail;
            end else if (dbyte && st != S_HUNT && st != S_SIZE && is_start) begin
                // 帧中途出现起始标记：说明前面丢了包，放弃当前帧，从新帧重新同步
                win <= w32n;
                if (!discard) stat_bad <= stat_bad + 1'b1;
                st <= S_SIZE; n4 <= 0; bad_pkt <= 1'b0;
                ts_f <= time_us;
                discard <= full[wb];
                if (full[wb]) stat_drop <= stat_drop + 1'b1;
            end else if (dbyte) begin
                win <= w32n;
                case (st)
                S_HUNT: begin
                    if (is_start) begin
                        st <= S_SIZE; n4 <= 0; bad_pkt <= 1'b0;
                        ts_f <= time_us;
                        discard <= full[wb];
                        if (full[wb]) stat_drop <= stat_drop + 1'b1;
                    end
                end
                S_SIZE: begin
                    n4 <= n4 + 1'b1;
                    if (n4 == 3'd3) begin
                        size_f <= w32; st <= S_HDR; hcnt <= 0;
                    end
                end
                S_HDR: begin
                    hcnt <= hcnt + 1'b1;
                    if (hcnt == 4'd0) data_len <= size_f[20:0] + {{5{cfg_size_adj[15]}}, cfg_size_adj};
                    case (hcnt)
                        4'd0: cmdid <= s_data;
                        4'd1: itype <= s_data;
                        4'd3: sn_f  <= w16;
                        4'd5: w_f   <= w16;
                        4'd7: h_f   <= w16;
                        default: ;
                    endcase
                    if (hcnt == 4'd1 && cmdid != 8'd251) st <= S_HUNT;    // 非图像数据包
                    if (hcnt == 4'd7) begin
                        if (itype == 8'd2) begin
                            st <= S_HUNT;                                  // 灰度帧：忽略
                        end else if (w_f == 0 || w_f > MAX_W || w16 == 0 || w16 > MAX_H || itype > 8'd1) begin
                            frame_fail;
                        end else begin
                            npix      <= wh_prod;
                            pix_bytes <= (itype == 8'd1) ? {1'b0, wh_prod, 1'b0} : {wh_prod, 2'b00};
                            st <= S_PIX; pcnt <= 0; pidx <= 0;
                        end
                    end
                end
                S_PIX: begin
                    pcnt <= pcnt + 1'b1;
                    if (pcnt == 0) begin
                        len_ok    <= (data_len >= {3'd0, pix_bytes} + 21'd12);
                        info_init <= data_len - {3'd0, pix_bytes} - 21'd8;
                    end
                    wword <= {wword[7:0], s_data};
                    if (itype == 8'd1 || cfg_layout == 2'd1) begin
                        // 2 字节一个字
                        if (pcnt[0]) begin
                            if (itype == 8'd1 || pcnt < {npix, 1'b0}) begin
                                // 深度字
                                fb_we <= ~discard;
                                fb_wa <= bank_base + pidx;
                                fb_wd <= dvalid_raw ? p16 : 16'd0;
                                pidx  <= (pidx == npix - 1) ? 14'd0 : pidx + 1'b1;
                            end else begin
                                // 平面布局的幅值字：低于阈值时把已写入的深度清零
                                if (p16 < cfg_amp_min) begin
                                    fb_we <= ~discard;
                                    fb_wa <= bank_base + pidx;
                                    fb_wd <= 16'd0;
                                end
                                pidx <= pidx + 1'b1;
                            end
                        end
                    end else begin
                        // 交织：4 字节一个像素
                        if (pcnt[1:0] == 2'd1) dhold <= p16;
                        if (pcnt[1:0] == 2'd3) begin
                            fb_we <= ~discard;
                            fb_wa <= bank_base + pidx;
                            if (cfg_amp_first) begin
                                // dhold=幅值, p16=深度
                                fb_wd <= (dvalid_raw && dhold >= cfg_amp_min) ? p16 : 16'd0;
                            end else begin
                                // dhold=深度, p16=幅值
                                fb_wd <= ((dhold != 16'd0) && ((cfg_raw_inv == 16'd0) || (dhold < cfg_raw_inv))
                                          && p16 >= cfg_amp_min) ? dhold : 16'd0;
                            end
                            pidx <= pidx + 1'b1;
                        end
                    end
                    if (pcnt == pix_bytes - 1) begin
                        if (!len_ok) begin
                            frame_fail;
                        end else begin
                            info_left <= info_init;
                            st <= S_INFO;
                        end
                    end
                end
                S_INFO: begin
                    last2 <= last1; last1 <= s_data;
                    info_left <= info_left - 1'b1;
                    if (info_left == 21'd1) begin st <= S_END; n4 <= 0; end
                end
                S_END: begin
                    n4 <= n4 + 1'b1;
                    if (n4 == 3'd3) begin
                        st <= S_HUNT;
                        if (!is_end || bad_pkt) begin
                            frame_fail;
                        end else if (!discard) begin
                            m_sn[wb] <= sn_f; m_w[wb] <= w_f; m_h[wb] <= h_f; m_ts[wb] <= ts_f;
                            m_ds[wb] <= last2; m_pv[wb] <= last1;
                            wb <= ~wb;
                            stat_frames <= stat_frames + 1'b1;
                            frame_ready <= 1'b1;
                        end
                    end
                end
                default: st <= S_HUNT;
                endcase
            end
        end
    end

    // ------------------------------------------------------------------
    // 读侧：按帧输出像素
    // ------------------------------------------------------------------
    reg         rd_act;         // 正在输出
    reg         rd_wait;        // 帧已就绪，等待位姿
    reg  [13:0] rp;             // 下一个要读的像素序号
    reg  [13:0] rnpix;
    reg  [7:0]  ru, rv;
    reg  [7:0]  rw8;
    wire        rd_issue;
    wire [15:0] fb_q;
    reg  [7:0]  q_u, q_v;
    reg         q_sof, q_eof;
    reg         rd_done;        // 已发出最后一个读

    assign pose_waiting = rd_wait;

    // full 的置位在写侧、清零在读侧；用单独的清零脉冲合并
    reg clr_full;
    always @(posedge clk) begin
        frame_start <= 1'b0;
        clr_full    <= 1'b0;
        if (rst) begin
            rb <= 1'b0; rd_act <= 1'b0; rd_wait <= 1'b0; rp <= 0; rd_done <= 1'b0;
        end else begin
            if (!rd_act && !rd_wait && full[rb] && !clr_full) begin
                frm_sn <= m_sn[rb]; frm_w <= m_w[rb]; frm_h <= m_h[rb]; frm_ts <= m_ts[rb];
                frm_devstate <= m_ds[rb]; frm_proto <= m_pv[rb];
                rnpix <= {6'd0, m_w[rb][7:0]} * {6'd0, m_h[rb][7:0]};
                rw8   <= m_w[rb][7:0];
                rd_wait <= 1'b1;
            end
            if (rd_wait && (!cfg_wait_pose || pose_commit)) begin
                rd_wait <= 1'b0; rd_act <= 1'b1; rp <= 0; ru <= 0; rv <= 0; rd_done <= 1'b0;
                frame_start <= 1'b1;
            end
            if (rd_issue) begin
                q_u <= ru; q_v <= rv;
                q_sof <= (rp == 0);
                q_eof <= (rp == rnpix - 1);
                rp <= rp + 1'b1;
                if (ru == rw8 - 1) begin ru <= 0; rv <= rv + 1'b1; end
                else ru <= ru + 1'b1;
                if (rp == rnpix - 1) rd_done <= 1'b1;
            end
            if (rd_act && m_valid && m_ready && m_eof) begin
                rd_act <= 1'b0;
                clr_full <= 1'b1;
                rb <= ~rb;
            end
        end
    end

    // full：写侧置位与读侧清零
    always @(posedge clk) begin
        if (rst) full <= 2'b00;
        else begin
            if (dbyte && !timeout && end_ok)
                full[wb] <= 1'b1;
            if (clr_full) full[~rb] <= 1'b0;   // rb 已翻转，清的是刚读完的那一个
        end
    end

    wire [AW-1:0] rd_addr = (rb ? NPIX[AW-1:0] : {AW{1'b0}}) + rp;
    nav_ram_banked #(.DW(16), .DEPTH(2 * NPIX)) u_fb (
        .clk(clk), .we(fb_we), .waddr(fb_wa), .wdata(fb_wd),
        .re(rd_issue), .raddr(rd_addr), .rdata(fb_q)
    );

    wire [33:0] ob_q;
    nav_fifo_ob #(.DW(34)) u_ob (
        .clk(clk), .rst(rst), .ram_avail(rd_act & ~rd_done), .ram_rd(rd_issue),
        .ram_q({q_sof, q_eof, q_u, q_v, fb_q}),
        .m_valid(m_valid), .m_data(ob_q), .m_ready(m_ready), .held()
    );
    assign {m_sof, m_eof, m_u, m_v, m_depth} = ob_q;
endmodule
