// XT-M60 图像帧接收：UDP 负载 -> 逐分包解析 -> 乒乓帧缓冲 -> 校验通过后输出像素流。
//
// 格式依据厂家 SDK 源码（communicationNet.cpp udppacket、xtdaemon.cpp ParseFrame、cmdstructs.h），
// 行为模型与判定点见 model/xt_proto.py RxModel（两者逐项一致）。
//   分包 = 20 字节分包头 {帧序号 u16, 整帧字节数 u32, 数据字节数 u16, 帧内偏移 u32, 8 字节未用} + 数据；
//   一个 UDP 报文可连放多个分包。整帧字节数 < 1000 的分包（IMU 等）跳过。
//   偏移 0 开始新帧；同帧偏移不连续（丢包、乱序、重复）整帧丢弃；其他零散分包忽略。
//   帧 = START | len | cmdid 251 | type(0 深度+幅值 / 1 深度 / 3 灰度) | SN | W | H | 像素 | ... | FrameInfo | END。
// 输出：逐像素 (u, v, 原始深度)，深度 0 表示无效（原始值 0、>= RAW_INV、幅值 < AMP_MIN 或 >= RAW_INV）。
//       整帧校验通过后才开始输出；帧尾 FrameInfo（206 字节，V3）中的内参、深度单位、ROI、合并方式随帧锁存。
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
    input  wire        cfg_hdr_le,     // 分包头与帧头多字节字段小端（SDK V3 = 1）
    input  wire        cfg_pix_le,     // 像素字小端（SDK = 1）
    input  wire [7:0]  cfg_subhdr,     // 分包头字节数（SDK = 20，>= 16）
    input  wire [15:0] cfg_amp_min,
    input  wire [15:0] cfg_raw_inv,    // 非 0 时，原始深度/幅值 >= 此值视为无效（SDK 无效码从 64000 起）
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
    output reg         frm_iok,        // 本帧带有效 FrameInfo（下面 4 项有效）
    output reg  [7:0]  frm_unit,       // 深度单位 mm/LSB（unit_div）
    output reg  [1:0]  frm_bin,        // [1] 水平合并 [0] 竖直合并
    output reg  [7:0]  frm_roix,
    output reg  [7:0]  frm_roiy,
    output wire        pose_waiting,
    output reg         frame_start,    // 单拍：开始输出一帧
    output reg         frame_ready,    // 单拍：一帧校验通过进入缓冲

    // 最近一个带有效 FrameInfo 的成功帧
    output reg  [287:0] inf_lens,      // 9 个 float32（小端）：fx fy cx cy k1 k2 k3 p1 p2
    output reg  [31:0]  inf_cfg,       // {roiy0, roix0, binning, unit_div}
    output reg  [31:0]  inf_size,      // {hdrmode, imageflags, infosize}
    output reg  [31:0]  inf_temp,      // {灯板温度, 传感器温度}（int16）
    output reg  [31:0]  inf_seq,       // 锁存次数

    output reg  [31:0] stat_frames,
    output reg  [31:0] stat_bad,
    output reg  [31:0] stat_drop,
    output reg  [31:0] stat_skip       // 跳过的分包（非图像消息、不属于当前帧的零散分包）
);
    localparam integer NPIX = MAX_W * MAX_H;
    localparam integer AW   = $clog2(2 * NPIX);
    localparam [31:0] START_W = 32'h7EFFAA55;
    localparam [31:0] END_W   = 32'hFF7E55AA;
    localparam integer TW = $clog2(TIMEOUT_US + 2);
    localparam integer INFO_SZ = 206;
    localparam integer MIN_TOTAL = 1000;

    // ------------------------------------------------------------------
    // 分包层
    // ------------------------------------------------------------------
    localparam [1:0] M_HDR = 2'd0, M_FEED = 2'd1, M_SKIP = 2'd2;
    reg  [1:0]  mode;
    reg  [7:0]  sh_cnt;         // 分包头已收字节
    reg  [15:0] pl_left;        // 本分包剩余数据字节
    reg  [15:0] h_sn, h_n;
    reg  [31:0] h_total, h_off;
    reg  [15:0] cur_sn;
    reg  [19:0] cur_total;
    reg  [19:0] fcnt;           // 当前帧已收字节 = 下一个字节的帧内偏移
    reg         fr_active;
    reg         fed;            // 本报文向当前帧送过字节
    reg         bad_pkt;        // 本帧收到过错误报文
    reg         discard;        // 缓冲不可用，本帧只解析不写入

    // ------------------------------------------------------------------
    // 帧解析
    // ------------------------------------------------------------------
    localparam [2:0] S_START = 3'd0, S_LEN = 3'd1, S_HDR = 3'd2, S_PIX = 3'd3, S_INFO = 3'd4, S_END = 3'd5;
    reg  [2:0]  st;
    reg  [31:0] win;            // 最近 4 个帧字节
    reg  [19:0] data_len;
    reg  [7:0]  cmdid, itype;
    reg  [15:0] sn_f, w_f, h_f;
    reg  [15:0] npix;
    reg  [17:0] pix_bytes;
    reg  [17:0] pcnt;
    reg  [19:0] info_left;
    reg  [15:0] wword, dhold;
    reg  [13:0] pidx;
    reg  [31:0] ts_f;
    reg  [TW-1:0] idle_us;
    reg  [$clog2(CLK_HZ/1000000)-1:0] usdiv;
    // FrameInfo 采集
    reg         info_big;       // 帧尾 >= 206 字节
    reg  [287:0] lens_sr;
    reg  [7:0]  c_unit, c_bin, c_flags, c_hdr, c_roix, c_roiy;
    reg  [15:0] c_isz, c_t0, c_t1;
    reg  [7:0]  last1, last2;   // 负载最后两个字节：DevState、协议版本

    // 帧缓冲管理
    reg        wb, rb;
    reg  [1:0] full;
    reg [15:0] m_sn [0:1];
    reg [15:0] m_w  [0:1];
    reg [15:0] m_h  [0:1];
    reg [31:0] m_ts [0:1];
    reg [7:0]  m_ds [0:1];
    reg [7:0]  m_pv [0:1];
    reg        m_io [0:1];
    reg [7:0]  m_un [0:1];
    reg [1:0]  m_bn [0:1];
    reg [7:0]  m_rx [0:1];
    reg [7:0]  m_ry [0:1];

    reg          fb_we;
    reg [AW-1:0] fb_wa;
    reg [15:0]   fb_wd;

    reg  [31:0] hwin;           // 分包头最近 4 个字节
    wire        in_byte = s_valid & cfg_enable;
    wire        hdr_byte = in_byte && mode == M_HDR;
    wire [15:0] hw16  = cfg_hdr_le ? {s_data, hwin[7:0]} : {hwin[7:0], s_data};
    wire [31:0] hw32n = {hwin[23:0], s_data};
    wire [31:0] hw32  = cfg_hdr_le ? {s_data, hwin[7:0], hwin[15:8], hwin[23:16]} : hw32n;
    wire        fbyte    = in_byte && mode == M_FEED && fr_active;      // 送入帧解析的字节
    wire [15:0] w16  = cfg_hdr_le ? {s_data, win[7:0]} : {win[7:0], s_data};
    wire [31:0] w32n = {win[23:0], s_data};
    wire [31:0] w32  = cfg_hdr_le ? {s_data, win[7:0], win[15:8], win[23:16]} : w32n;
    wire [15:0] p16  = cfg_pix_le ? {s_data, wword[7:0]} : {wword[7:0], s_data};
    wire        inv_d = (p16 == 16'd0) || ((cfg_raw_inv != 16'd0) && (p16 >= cfg_raw_inv));
    wire        inv_h = (dhold == 16'd0) || ((cfg_raw_inv != 16'd0) && (dhold >= cfg_raw_inv));
    wire        inv_a = (p16 < cfg_amp_min) || ((cfg_raw_inv != 16'd0) && (p16 >= cfg_raw_inv));
    wire [AW-1:0] bank_base = wb ? NPIX[AW-1:0] : {AW{1'b0}};
    // 手册只给出标记数值；SDK 按字节 7E FF AA 55 比较。两种字节序都接受
    wire is_start = (w32n == START_W) || (w32n == {START_W[7:0], START_W[15:8], START_W[23:16], START_W[31:24]});
    wire is_end   = (w32n == END_W)   || (w32n == {END_W[7:0], END_W[15:8], END_W[23:16], END_W[31:24]});
    wire [15:0] wh_prod = {8'd0, w_f[7:0]} * {8'd0, w16[7:0]};
    wire [17:0] pix_w   = (itype == 8'd1) ? {1'b0, wh_prod, 1'b0} : {wh_prod, 2'b00};

    // 分包头字段（小端/大端），在分包头最后一个字节时判定
    wire [7:0]  hb = sh_cnt;
    wire        h_img  = (h_total >= MIN_TOTAL) && (h_total < 32'h00100000) && (h_n != 16'd0);
    wire [32:0] h_endo = {1'b0, h_off} + {17'd0, h_n};
    wire        h_fits = (h_endo <= {1'b0, h_total});
    wire        h_same = fr_active && (h_sn == cur_sn) && (h_total[19:0] == cur_total);
    wire        hdr_done = hdr_byte && (sh_cnt == cfg_subhdr - 8'd1);
    wire        new_frame = hdr_done && h_img && (h_off == 32'd0);

    // 帧内空闲超时
    wire us_tick = (usdiv == CLK_HZ/1000000 - 1);
    wire timeout = fr_active && (idle_us >= TIMEOUT_US);

    wire trunc  = in_byte && s_last && (mode == M_FEED) && (pl_left != 16'd1);  // 报文在分包数据中途结束
    wire last_err = in_byte && s_last && s_err;

    // 帧结束判定（S_END 第 4 字节）
    wire end_byte = fbyte && (st == S_END) && (fcnt == cur_total - 20'd1);
    wire end_ok   = end_byte && is_end && !bad_pkt && !last_err;
    wire iok      = info_big && (last1 == 8'd3) && (c_isz == INFO_SZ);

    task frame_fail;
        begin
            fr_active <= 1'b0;
            if (!discard) stat_bad <= stat_bad + 1'b1;
        end
    endtask

    always @(posedge clk) begin
        fb_we       <= 1'b0;
        frame_ready <= 1'b0;
        if (rst) begin
            mode <= M_HDR; sh_cnt <= 0; pl_left <= 0; fr_active <= 1'b0; fed <= 1'b0; bad_pkt <= 1'b0;
            wb <= 1'b0; discard <= 1'b0; idle_us <= 0; usdiv <= 0; st <= S_START; win <= 0;
            stat_frames <= 0; stat_bad <= 0; stat_drop <= 0; stat_skip <= 0; inf_seq <= 0;
            pcnt <= 0; pidx <= 0; fcnt <= 0;
            inf_lens <= 0; inf_cfg <= 0; inf_size <= 0; inf_temp <= 0;
        end else begin
            usdiv <= us_tick ? 0 : usdiv + 1'b1;
            if (in_byte) idle_us <= 0;
            else if (us_tick && idle_us < TIMEOUT_US) idle_us <= idle_us + 1'b1;

            // ---------------- 分包层 ----------------
            if (hdr_byte) begin
                sh_cnt <= sh_cnt + 1'b1;
                hwin   <= hw32n;
                case (hb)
                    8'd1:  h_sn    <= hw16;
                    8'd5:  h_total <= hw32;
                    8'd7:  h_n     <= hw16;
                    8'd11: h_off   <= hw32;
                    default: ;
                endcase
            end
            if (hdr_done) begin
                sh_cnt  <= 0;
                pl_left <= h_n;
                if (!h_img) begin
                    mode <= (h_n != 0) ? M_SKIP : M_HDR;
                    stat_skip <= stat_skip + 1'b1;
                end else if (h_off == 32'd0) begin
                    // 新帧：未完成的旧帧作废；首包就越界的当作零散分包
                    if (fr_active) frame_fail;
                    if (h_fits) begin
                        fr_active <= 1'b1; mode <= M_FEED;
                        cur_sn <= h_sn; cur_total <= h_total[19:0];
                        fcnt <= 0; st <= S_START; bad_pkt <= 1'b0; fed <= 1'b0;
                        ts_f <= time_us; info_big <= 1'b0;
                        discard <= full[wb];
                        if (full[wb]) stat_drop <= stat_drop + 1'b1;
                    end else begin
                        mode <= M_SKIP;
                        stat_skip <= stat_skip + 1'b1;
                    end
                end else if (h_same) begin
                    if (h_off[19:0] == fcnt && h_off[31:20] == 12'd0 && h_fits) mode <= M_FEED;
                    else begin
                        frame_fail;
                        mode <= M_SKIP;
                    end
                end else begin
                    mode <= M_SKIP;
                    stat_skip <= stat_skip + 1'b1;
                end
            end
            if (in_byte && mode != M_HDR) begin
                pl_left <= pl_left - 1'b1;
                if (pl_left == 16'd1) mode <= M_HDR;
            end

            // ---------------- 帧解析 ----------------
            if (fbyte) begin
                win  <= w32n;
                fcnt <= fcnt + 1'b1;
                fed  <= 1'b1;
                case (st)
                S_START: if (fcnt == 20'd3) begin
                    if (is_start) st <= S_LEN; else frame_fail;
                end
                S_LEN: if (fcnt == 20'd7) begin
                    data_len <= w32[19:0];
                    if (w32[31:20] == 12'd0 && w32[19:0] + 20'd12 == cur_total) st <= S_HDR;
                    else frame_fail;
                end
                S_HDR: begin
                    case (fcnt[3:0])
                        4'd8:  cmdid <= s_data;
                        4'd9:  begin
                            itype <= s_data;
                            if (cmdid != 8'd251 || s_data == 8'd3) fr_active <= 1'b0;   // 非图像 / 灰度：忽略
                            else if (s_data > 8'd1) frame_fail;
                        end
                        4'd11: sn_f <= w16;
                        4'd13: w_f  <= w16;
                        4'd15: begin
                            h_f <= w16;
                            npix      <= wh_prod;
                            pix_bytes <= pix_w;
                            if (w_f == 0 || w_f > MAX_W || w16 == 0 || w16 > MAX_H) frame_fail;
                            else begin st <= S_PIX; pcnt <= 0; pidx <= 0; end
                        end
                        default: ;
                    endcase
                end
                S_PIX: begin
                    pcnt  <= pcnt + 1'b1;
                    wword <= {wword[7:0], s_data};
                    // 长度检查放在第一个像素字节（偏移 16）：像素字节数已在上一拍寄存，缩短关键路径
                    if (pcnt == 0 && data_len < {2'd0, pix_bytes} + 20'd12) frame_fail;
                    if (itype == 8'd1) begin
                        if (pcnt[0]) begin
                            fb_we <= ~discard;
                            fb_wa <= bank_base + pidx;
                            fb_wd <= inv_d ? 16'd0 : p16;
                            pidx  <= pidx + 1'b1;
                        end
                    end else begin
                        // 逐像素交织：深度在前、幅值在后
                        if (pcnt[1:0] == 2'd1) dhold <= p16;
                        if (pcnt[1:0] == 2'd3) begin
                            fb_we <= ~discard;
                            fb_wa <= bank_base + pidx;
                            fb_wd <= (inv_h || inv_a) ? 16'd0 : dhold;
                            pidx  <= pidx + 1'b1;
                        end
                    end
                    if (pcnt == pix_bytes - 1) begin
                        info_left <= data_len - {2'd0, pix_bytes} - 20'd8;
                        info_big  <= (data_len - {2'd0, pix_bytes} - 20'd8) >= INFO_SZ;
                        st <= S_INFO;
                    end
                end
                S_INFO: begin
                    last2 <= last1; last1 <= s_data;
                    info_left <= info_left - 1'b1;
                    // FrameInfo 字段：结构体偏移 f 的字节距负载末尾 206-f
                    if (info_left <= 20'd153 && info_left >= 20'd118) lens_sr <= {s_data, lens_sr[287:8]};
                    case (info_left)
                        20'd113: c_flags <= s_data;
                        20'd112: c_hdr   <= s_data;
                        20'd93:  c_roix  <= s_data;
                        20'd89:  c_roiy  <= s_data;
                        20'd82:  c_bin   <= s_data;
                        20'd70:  c_unit  <= s_data;
                        20'd65:  c_t0[7:0]  <= s_data;
                        20'd64:  c_t0[15:8] <= s_data;
                        20'd63:  c_t1[7:0]  <= s_data;
                        20'd62:  c_t1[15:8] <= s_data;
                        20'd4:   c_isz[7:0]  <= s_data;
                        20'd3:   c_isz[15:8] <= s_data;
                        default: ;
                    endcase
                    if (info_left == 20'd1) st <= S_END;
                end
                S_END: if (fcnt == cur_total - 20'd1) begin
                    fr_active <= 1'b0;
                    if (!is_end || bad_pkt || last_err) begin
                        if (!discard) stat_bad <= stat_bad + 1'b1;
                    end else if (!discard) begin
                        m_sn[wb] <= sn_f; m_w[wb] <= w_f; m_h[wb] <= h_f; m_ts[wb] <= ts_f;
                        m_ds[wb] <= last2; m_pv[wb] <= last1;
                        m_io[wb] <= iok; m_un[wb] <= c_unit; m_bn[wb] <= {c_bin[1], c_bin[0]};
                        m_rx[wb] <= c_roix; m_ry[wb] <= c_roiy;
                        wb <= ~wb;
                        stat_frames <= stat_frames + 1'b1;
                        frame_ready <= 1'b1;
                        if (iok) begin
                            inf_lens <= lens_sr;
                            inf_cfg  <= {c_roiy, c_roix, c_bin, c_unit};
                            inf_size <= {c_hdr, c_flags, c_isz};
                            inf_temp <= {c_t1, c_t0};
                            inf_seq  <= inf_seq + 1'b1;
                        end
                    end
                end
                default: frame_fail;
                endcase
            end

            // 报文在分包数据中途结束、帧内超时
            if ((trunc && fr_active && !end_byte) || timeout) frame_fail;
            // 错误报文：只作废从该报文收到过字节的当前帧
            if (last_err && (fed || fbyte) && fr_active && !end_byte && !new_frame) bad_pkt <= 1'b1;
            if (in_byte && s_last) begin
                mode <= M_HDR; sh_cnt <= 0; fed <= 1'b0;
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
                frm_iok <= m_io[rb]; frm_unit <= m_un[rb]; frm_bin <= m_bn[rb];
                frm_roix <= m_rx[rb]; frm_roiy <= m_ry[rb];
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
            if (end_ok && !discard) full[wb] <= 1'b1;
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
