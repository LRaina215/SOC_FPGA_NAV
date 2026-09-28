// SLF 匹配定位硬件（结构点提取 + 结构图 + 打分 + 并图 + 似然场建图），与 model/slf_match.py 逐位一致。
//
// 每帧：
//   1. 提交帧时（点流旁听）提取结构点 -> 点缓冲，结束置 pts_ready；
//   2. EVAL：门控遍（每点：预测位置距窗口边 >= MARGIN 且该格 gate=1 -> 参与），再逐候选：
//      每点 p' = Rz(dyaw)(p − c) + c + (dx, dy)，取 1/16 格坐标，读 3×3 邻格：
//      有法向的结构格按点到线法向距离 |ex·nx + ey·ny|>>7，否则按 |ex|+|ey|，取最小并截断到 C_NEAR；
//      结果 {Σ代价, 近邻命中数, 参与点数}；
//   3. UPDATE：每点按修正量变换后写入结构图（一帧内同格只取第一个点：用"本遍已写"位图，写完再按记录清零），
//      然后启动似然场建图遍（slf_lf）。
// 点按步长 ceil(存量/1024) 抽取。所有顺序处理，每 (点, 候选) 约 15 拍：245 候选 × 1024 点约 38 ms（100 MHz）。
module slf_match #(
    parameter integer LOG2N  = 8,
    parameter integer PB_AW  = 12,
    parameter integer MARGIN = 8,
    parameter integer C_NEAR = 40,
    parameter integer HIT_FREEZE = 4,
    parameter integer HIT_MAX = 7,
    parameter SIN_INIT = "sin_lut.hex"
) (
    input  wire        clk,
    input  wire        rst,
    // 点流旁听
    input  wire        s_fire,
    input  wire signed [31:0] s_x,
    input  wire signed [31:0] s_y,
    input  wire signed [31:0] s_z,
    input  wire        s_pv,
    input  wire        s_sof,
    input  wire        s_eof,
    input  wire        frame_en,            // 本帧提取（帧开始时采样）
    input  wire signed [31:0] cfg_zlo,
    input  wire signed [31:0] cfg_zhi,
    input  wire signed [31:0] cfg_tx,
    input  wire signed [31:0] cfg_ty,
    input  wire [15:0] cfg_inv_res,
    // 命令
    input  wire        cmd_eval,
    input  wire        cmd_update,
    input  wire        cmd_clear,
    input  wire signed [31:0] cfg_cx_mm,     // 旋转中心（机器人位置）
    input  wire signed [31:0] cfg_cy_mm,
    input  wire [8:0]  cfg_ncand,
    input  wire signed [31:0] upd_dx,
    input  wire signed [31:0] upd_dy,
    input  wire [15:0] upd_dyaw,
    input  wire signed [31:0] lf_cx,         // 似然场窗口中心（世界 5 cm 格）
    input  wire signed [31:0] lf_cy,
    // 候选写口：地址 {k, w}，w=0: {dy[15:0], dx[15:0]}，w=1: {16'b0, dyaw}
    input  wire        cand_we,
    input  wire [8:0]  cand_waddr,
    input  wire [31:0] cand_wdata,
    // 结果读口：地址 {k, w}，w=0: cost，w=1: {n_in[15:0], n_valid[15:0]}（组合地址，下一拍出数）
    input  wire        res_re,
    input  wire [8:0]  res_raddr,
    output wire [31:0] res_q,
    // 结构图调试读（软核）：req 保持到 ack
    input  wire        dbg_req,
    input  wire [2*LOG2N-1:0] dbg_addr,
    output reg         dbg_ack,
    output reg  [22:0] dbg_q,
    // 状态
    output wire        busy,
    output reg         pts_ready,
    output wire [PB_AW:0] n_pts,
    output reg         eval_done
);
    localparam integer N  = 1 << LOG2N;
    localparam integer AW = 2 * LOG2N;

    // ================================================================ 结构点提取
    wire          pb_we;
    wire [PB_AW-1:0] pb_wa;
    wire [35:0]   pb_wd;
    wire          ex_done, ex_busy;
    reg  signed [31:0] base_x, base_y;       // 点缓冲的平移基准（本帧 t）
    slf_extract #(.PB_AW(PB_AW)) u_ex (
        .clk(clk), .rst(rst), .en(frame_en), .cfg_zlo(cfg_zlo), .cfg_zhi(cfg_zhi), .cfg_tx(cfg_tx), .cfg_ty(cfg_ty),
        .s_fire(s_fire), .s_x(s_x), .s_y(s_y), .s_z(s_z), .s_pv(s_pv), .s_sof(s_sof), .s_eof(s_eof),
        .pb_we(pb_we), .pb_waddr(pb_wa), .pb_wdata(pb_wd), .done(ex_done), .n_pts(n_pts), .busy(ex_busy)
    );
    reg          pb_re;
    reg [PB_AW-1:0] pb_ra;
    wire [35:0]  pb_q;
    nav_ram_sdp #(.DW(36), .AW(PB_AW)) u_pb (
        .clk(clk), .we(pb_we), .waddr(pb_wa), .wdata(pb_wd), .re(pb_re), .raddr(pb_ra), .rdata(pb_q)
    );

    // ================================================================ 结构图、位图、候选、结果
    reg          sm_re_m, sm_we_m;
    reg [AW-1:0] sm_ra_m, sm_wa_m;
    reg [22:0]   sm_wd_m;
    wire         lf_re, lf_we, lf_busy, lf_done;
    wire [AW-1:0] lf_ra, lf_wa;
    wire [22:0]  lf_wd;
    wire [22:0]  sm_q;
    wire         sm_we = lf_busy ? lf_we : sm_we_m;
    wire [AW-1:0] sm_wa = lf_busy ? lf_wa : sm_wa_m;
    wire [22:0]  sm_wd = lf_busy ? lf_wd : sm_wd_m;
    wire         sm_re = lf_busy ? lf_re : sm_re_m;
    wire [AW-1:0] sm_ra = lf_busy ? lf_ra : sm_ra_m;
    nav_ram_sdp #(.DW(23), .AW(AW)) u_sm (
        .clk(clk), .we(sm_we), .waddr(sm_wa), .wdata(sm_wd), .re(sm_re), .raddr(sm_ra), .rdata(sm_q)
    );
    reg          lf_start;
    reg signed [31:0] lfo_x, lfo_y;          // 当前似然场窗口原点（世界格）
    slf_lf #(.LOG2N(LOG2N)) u_lf (
        .clk(clk), .rst(rst), .start(lf_start), .cx(lf_cx), .cy(lf_cy), .busy(lf_busy), .done(lf_done),
        .sm_re(lf_re), .sm_raddr(lf_ra), .sm_q(sm_q), .sm_we(lf_we), .sm_waddr(lf_wa), .sm_wdata(lf_wd)
    );
    reg          tb_we, tb_wd, tb_re;
    reg [AW-1:0] tb_wa, tb_ra;
    wire         tb_q;
    nav_ram_sdp #(.DW(1), .AW(AW)) u_touch (
        .clk(clk), .we(tb_we), .waddr(tb_wa), .wdata(tb_wd), .re(tb_re), .raddr(tb_ra), .rdata(tb_q)
    );
    reg          sl_we, sl_re;
    reg [9:0]    sl_wa, sl_ra;
    reg [AW-1:0] sl_wd;
    wire [AW-1:0] sl_q;
    nav_ram_sdp #(.DW(AW), .AW(10)) u_slist (          // 本遍写过的槽（清位图用）
        .clk(clk), .we(sl_we), .waddr(sl_wa), .wdata(sl_wd), .re(sl_re), .raddr(sl_ra), .rdata(sl_q)
    );
    reg          ac_we, ac_wd, ac_re;
    reg [9:0]    ac_wa, ac_ra;
    wire         ac_q;
    nav_ram_sdp #(.DW(1), .AW(10)) u_acc (            // 门控结果（第 jj 个抽样点是否参与）
        .clk(clk), .we(ac_we), .waddr(ac_wa), .wdata(ac_wd), .re(ac_re), .raddr(ac_ra), .rdata(ac_q)
    );
    reg          cd_re;
    reg [8:0]    cd_ra;
    wire [31:0]  cd_q;
    nav_ram_sdp #(.DW(32), .AW(9)) u_cand (
        .clk(clk), .we(cand_we), .waddr(cand_waddr), .wdata(cand_wdata), .re(cd_re), .raddr(cd_ra), .rdata(cd_q)
    );
    reg          rs_we;
    reg [8:0]    rs_wa;
    reg [31:0]   rs_wd;
    nav_ram_sdp #(.DW(32), .AW(9)) u_res (
        .clk(clk), .we(rs_we), .waddr(rs_wa), .wdata(rs_wd), .re(res_re), .raddr(res_raddr), .rdata(res_q)
    );
    reg          lut_re;
    reg [9:0]    lut_si, lut_ci;
    wire [15:0]  lut_sq, lut_cq;
    nav_ram_sdp #(.DW(16), .AW(10), .INIT_FILE(SIN_INIT)) u_sin (
        .clk(clk), .we(1'b0), .waddr(10'd0), .wdata(16'd0), .re(lut_re), .raddr(lut_si), .rdata(lut_sq)
    );
    nav_ram_sdp #(.DW(16), .AW(10), .INIT_FILE(SIN_INIT)) u_cos (
        .clk(clk), .we(1'b0), .waddr(10'd0), .wdata(16'd0), .re(lut_re), .raddr(lut_ci), .rdata(lut_cq)
    );

    // ================================================================ 控制
    localparam [4:0] S_IDLE = 0, S_CLR = 1, S_EV0 = 2, S_CD0 = 3, S_CD1 = 4, S_CD2 = 5, S_LUT = 6,
                     S_T0 = 7, S_T1 = 8, S_T2 = 9, S_T3 = 10, S_T4 = 11, S_T5 = 12,
                     S_G0 = 13, S_G1 = 14, S_NB = 15, S_ACC = 16, S_RES = 17,
                     S_U0 = 18, S_U1 = 19, S_U2 = 30, S_U3 = 31, S_UC0 = 20, S_UC1 = 21, S_LFW = 22, S_DBG = 23, S_RES2 = 24,
                     S_T0W = 25, S_LUTW = 26, S_DBG2 = 28, S_UC2 = 29;
    localparam [1:0] MD_GATE = 0, MD_SCORE = 1, MD_UPD = 2;
    localparam [7:0] CN = C_NEAR;
    localparam [2:0] HM = HIT_MAX;
    // 注：RAM 读使能/地址在状态机里寄存，读数据在发读后第 2 拍可用（各处都多一个等待态）
    reg [5:0]  st;
    reg [1:0]  md;
    reg [AW:0] clr_a;
    reg [PB_AW:0] nsnap;                    // 本次处理时的点数
    reg [2:0]  stride;
    reg [PB_AW:0] j;                        // 点缓冲下标
    reg [9:0]  jj;                          // 抽样点序号
    reg [8:0]  k, ncand;
    reg signed [31:0] dx_c, dy_c, cxm, cym;
    reg [15:0] dyaw_c;
    reg signed [15:0] sn, cs;
    reg        acc_ok;
    reg signed [17:0] rx, ry;
    reg signed [34:0] pxc, pys, pxs, pyc;
    reg signed [31:0] xr, yr;
    reg signed [31:0] fxr, fyr;             // 1/16 格
    reg [15:0] cost_acc, near_acc, nval;
    reg [3:0]  nb;                          // 当前发读的邻格序号
    reg        nb_v1, nb_v; reg [3:0] nb_d1, nb_d;   // 读返回对应的邻格（发读后第 2 拍）
    reg        nq_v, np_v;                           // 邻格流水：寄存槽字 -> 求点积 -> 取小
    reg [3:0]  nq_d;
    reg [22:0] nq_w;
    reg        np_occ, np_has;
    reg signed [17:0] np_dot;
    reg [8:0]  np_l1;
    reg [22:0] uw_q; reg utb_q;                      // 并图读回（寄存一拍再算）
    reg [7:0]  best; reg any;
    reg [10:0] ncl;                         // 已记录的槽数（<= 1024）
    reg [10:0] cli;
    reg [AW-1:0] u_slot; reg [3:0] u_tag, u_mx, u_my;

    assign busy = (st != S_IDLE) || lf_busy || ex_busy;

    // 反向旋转用的正弦表索引：((dyaw + 32) & 0xFFFF) >> 6
    wire [15:0] da_r = dyaw_c + 16'd32;
    // 窗口坐标
    wire signed [31:0] wxa = fxr >>> 4, wya = fyr >>> 4;
    wire [3:0]  sxf = fxr[3:0], syf = fyr[3:0];
    wire signed [31:0] wxw = wxa - lfo_x, wyw = wya - lfo_y;
    wire        core = (wxw >= MARGIN) && (wxw < N - MARGIN) && (wyw >= MARGIN) && (wyw < N - MARGIN);
    // 邻格偏移
    wire signed [2:0] onx = (nb % 3) - 1, ony = (nb / 3) - 1;
    wire signed [31:0] nbx = wxa + onx, nby = wya + ony;
    // 邻格代价
    function signed [8:0] nvx(input [4:0] q);   // round(cos/sin(q·π/32)·128)，与 model/slf_match.py:NVEC 相同
        case (q)
            5'd0: nvx = 9'sd128;
            5'd1: nvx = 9'sd127;
            5'd2: nvx = 9'sd126;
            5'd3: nvx = 9'sd122;
            5'd4: nvx = 9'sd118;
            5'd5: nvx = 9'sd113;
            5'd6: nvx = 9'sd106;
            5'd7: nvx = 9'sd99;
            5'd8: nvx = 9'sd91;
            5'd9: nvx = 9'sd81;
            5'd10: nvx = 9'sd71;
            5'd11: nvx = 9'sd60;
            5'd12: nvx = 9'sd49;
            5'd13: nvx = 9'sd37;
            5'd14: nvx = 9'sd25;
            5'd15: nvx = 9'sd13;
            5'd16: nvx = 9'sd0;
            5'd17: nvx = -9'sd13;
            5'd18: nvx = -9'sd25;
            5'd19: nvx = -9'sd37;
            5'd20: nvx = -9'sd49;
            5'd21: nvx = -9'sd60;
            5'd22: nvx = -9'sd71;
            5'd23: nvx = -9'sd81;
            5'd24: nvx = -9'sd91;
            5'd25: nvx = -9'sd99;
            5'd26: nvx = -9'sd106;
            5'd27: nvx = -9'sd113;
            5'd28: nvx = -9'sd118;
            5'd29: nvx = -9'sd122;
            5'd30: nvx = -9'sd126;
            5'd31: nvx = -9'sd127;
            default: nvx = 9'sd0;
        endcase
    endfunction
    function signed [8:0] nvy(input [4:0] q);   // round(cos/sin(q·π/32)·128)，与 model/slf_match.py:NVEC 相同
        case (q)
            5'd0: nvy = 9'sd0;
            5'd1: nvy = 9'sd13;
            5'd2: nvy = 9'sd25;
            5'd3: nvy = 9'sd37;
            5'd4: nvy = 9'sd49;
            5'd5: nvy = 9'sd60;
            5'd6: nvy = 9'sd71;
            5'd7: nvy = 9'sd81;
            5'd8: nvy = 9'sd91;
            5'd9: nvy = 9'sd99;
            5'd10: nvy = 9'sd106;
            5'd11: nvy = 9'sd113;
            5'd12: nvy = 9'sd118;
            5'd13: nvy = 9'sd122;
            5'd14: nvy = 9'sd126;
            5'd15: nvy = 9'sd127;
            5'd16: nvy = 9'sd128;
            5'd17: nvy = 9'sd127;
            5'd18: nvy = 9'sd126;
            5'd19: nvy = 9'sd122;
            5'd20: nvy = 9'sd118;
            5'd21: nvy = 9'sd113;
            5'd22: nvy = 9'sd106;
            5'd23: nvy = 9'sd99;
            5'd24: nvy = 9'sd91;
            5'd25: nvy = 9'sd81;
            5'd26: nvy = 9'sd71;
            5'd27: nvy = 9'sd60;
            5'd28: nvy = 9'sd49;
            5'd29: nvy = 9'sd37;
            5'd30: nvy = 9'sd25;
            5'd31: nvy = 9'sd13;
            default: nvy = 9'sd0;
        endcase
    endfunction
    wire [4:0]  q_q = nq_w[6:2];
    wire signed [2:0] qdx = (nq_d % 3) - 1, qdy = (nq_d / 3) - 1;
    wire signed [7:0] ex_n = $signed({qdx, 4'd0}) + $signed({4'd0, nq_w[15:12]}) - $signed({4'd0, sxf});
    wire signed [7:0] ey_n = $signed({qdy, 4'd0}) + $signed({4'd0, nq_w[11:8]}) - $signed({4'd0, syf});
    wire signed [17:0] dot_n = ex_n * nvx(q_q) + ey_n * nvy(q_q);
    wire [7:0]  aex = ex_n[7] ? -ex_n : ex_n, aey = ey_n[7] ? -ey_n : ey_n;
    wire [17:0] adot = np_dot[17] ? -np_dot : np_dot;
    wire [10:0] dd_n = np_has ? {3'd0, adot[14:7]} : {2'd0, np_l1};
    wire [7:0]  dd_c = (dd_n > 11'd255) ? 8'd255 : dd_n[7:0];
    // 并图：新字
    wire [3:0]  w_tag = uw_q[22:19];
    wire [2:0]  w_hit = uw_q[18:16];
    wire [3:0]  w_cx = uw_q[15:12], w_cy = uw_q[11:8];
    wire        fresh_w = (w_tag != u_tag) || (w_hit == 0);
    reg         fresh;                               // 并图：S_U2 寄存 fresh/h0，S_U3 求新字
    reg  [2:0]  h0;
    wire        updc = (h0 < HIT_FREEZE);
    wire [2:0]  n1 = h0 + 3'd1;
    // 向下取整除法 floor(t / d)，d ∈ {2,3,4}，t ∈ [-16, 31]（除以 3 查表）
    function signed [5:0] fdiv3(input signed [5:0] t);
        case (t)
            -6'sd32: fdiv3 = -6'sd11;
            -6'sd31: fdiv3 = -6'sd11;
            -6'sd30: fdiv3 = -6'sd10;
            -6'sd29: fdiv3 = -6'sd10;
            -6'sd28: fdiv3 = -6'sd10;
            -6'sd27: fdiv3 = -6'sd9;
            -6'sd26: fdiv3 = -6'sd9;
            -6'sd25: fdiv3 = -6'sd9;
            -6'sd24: fdiv3 = -6'sd8;
            -6'sd23: fdiv3 = -6'sd8;
            -6'sd22: fdiv3 = -6'sd8;
            -6'sd21: fdiv3 = -6'sd7;
            -6'sd20: fdiv3 = -6'sd7;
            -6'sd19: fdiv3 = -6'sd7;
            -6'sd18: fdiv3 = -6'sd6;
            -6'sd17: fdiv3 = -6'sd6;
            -6'sd16: fdiv3 = -6'sd6;
            -6'sd15: fdiv3 = -6'sd5;
            -6'sd14: fdiv3 = -6'sd5;
            -6'sd13: fdiv3 = -6'sd5;
            -6'sd12: fdiv3 = -6'sd4;
            -6'sd11: fdiv3 = -6'sd4;
            -6'sd10: fdiv3 = -6'sd4;
            -6'sd9: fdiv3 = -6'sd3;
            -6'sd8: fdiv3 = -6'sd3;
            -6'sd7: fdiv3 = -6'sd3;
            -6'sd6: fdiv3 = -6'sd2;
            -6'sd5: fdiv3 = -6'sd2;
            -6'sd4: fdiv3 = -6'sd2;
            -6'sd3: fdiv3 = -6'sd1;
            -6'sd2: fdiv3 = -6'sd1;
            -6'sd1: fdiv3 = -6'sd1;
            6'sd0: fdiv3 = 6'sd0;
            6'sd1: fdiv3 = 6'sd0;
            6'sd2: fdiv3 = 6'sd0;
            6'sd3: fdiv3 = 6'sd1;
            6'sd4: fdiv3 = 6'sd1;
            6'sd5: fdiv3 = 6'sd1;
            6'sd6: fdiv3 = 6'sd2;
            6'sd7: fdiv3 = 6'sd2;
            6'sd8: fdiv3 = 6'sd2;
            6'sd9: fdiv3 = 6'sd3;
            6'sd10: fdiv3 = 6'sd3;
            6'sd11: fdiv3 = 6'sd3;
            6'sd12: fdiv3 = 6'sd4;
            6'sd13: fdiv3 = 6'sd4;
            6'sd14: fdiv3 = 6'sd4;
            6'sd15: fdiv3 = 6'sd5;
            6'sd16: fdiv3 = 6'sd5;
            6'sd17: fdiv3 = 6'sd5;
            6'sd18: fdiv3 = 6'sd6;
            6'sd19: fdiv3 = 6'sd6;
            6'sd20: fdiv3 = 6'sd6;
            6'sd21: fdiv3 = 6'sd7;
            6'sd22: fdiv3 = 6'sd7;
            6'sd23: fdiv3 = 6'sd7;
            6'sd24: fdiv3 = 6'sd8;
            6'sd25: fdiv3 = 6'sd8;
            6'sd26: fdiv3 = 6'sd8;
            6'sd27: fdiv3 = 6'sd9;
            6'sd28: fdiv3 = 6'sd9;
            6'sd29: fdiv3 = 6'sd9;
            6'sd30: fdiv3 = 6'sd10;
            6'sd31: fdiv3 = 6'sd10;
            default: fdiv3 = 6'sd0;
        endcase
    endfunction
    function signed [5:0] fdiv(input signed [5:0] t, input [2:0] d);
        case (d)
            3'd2: fdiv = t >>> 1;
            3'd4: fdiv = t >>> 2;
            default: fdiv = fdiv3(t);
        endcase
    endfunction
    reg  signed [5:0] dcx, dcy;                       // S_U2 寄存：m − c
    wire signed [5:0] tcx = dcx + $signed({3'd0, n1[2:1]});
    wire signed [5:0] tcy = dcy + $signed({3'd0, n1[2:1]});
    wire signed [5:0] ncx_s = $signed({2'd0, w_cx}) + fdiv(tcx, n1);
    wire signed [5:0] ncy_s = $signed({2'd0, w_cy}) + fdiv(tcy, n1);
    wire [3:0]  ncx = fresh ? u_mx : updc ? ncx_s[3:0] : w_cx;
    wire [3:0]  ncy = fresh ? u_my : updc ? ncy_s[3:0] : w_cy;
    wire [2:0]  nhit = (h0 == 3'd7 || h0 + 3'd1 > HM) ? HM : h0 + 3'd1;
    // 定点坐标
    wire signed [35:0] rotx = pxc - pys;
    wire signed [35:0] roty = pxs + pyc;
    function signed [31:0] clip25(input signed [31:0] v);
        clip25 = (v > 32'sd33554431) ? 32'sd33554431 : (v < -32'sd33554432) ? -32'sd33554432 : v;
    endfunction
    wire signed [48:0] fxm = xr * $signed({1'b0, cfg_inv_res});
    wire signed [48:0] fym = yr * $signed({1'b0, cfg_inv_res});
    wire signed [31:0] pbx = base_x + {{14{pb_q[17]}}, pb_q[17:0]};
    wire signed [31:0] pby = base_y + {{14{pb_q[35]}}, pb_q[35:18]};
    wire signed [31:0] rxw = pbx - cxm, ryw = pby - cym;
    wire [PB_AW:0] jn = j + stride;

    always @(posedge clk) begin
        pb_re <= 1'b0; sm_re_m <= 1'b0; sm_we_m <= 1'b0; tb_we <= 1'b0; tb_re <= 1'b0; sl_we <= 1'b0; sl_re <= 1'b0;
        ac_we <= 1'b0; ac_re <= 1'b0; cd_re <= 1'b0; rs_we <= 1'b0; lut_re <= 1'b0; lf_start <= 1'b0;
        eval_done <= 1'b0; dbg_ack <= 1'b0;
        nb_v1 <= 1'b0; nb_v <= nb_v1; nb_d <= nb_d1;
        nq_v <= nb_v; nq_d <= nb_d; nq_w <= sm_q;
        np_v <= nq_v; np_occ <= nq_w[7]; np_has <= nq_w[1]; np_dot <= dot_n; np_l1 <= {1'b0, aex} + {1'b0, aey};
        if (rst) begin
            st <= S_CLR; clr_a <= 0; pts_ready <= 1'b0; lfo_x <= -(N/2); lfo_y <= -(N/2);
        end else begin
            if (s_fire && s_sof && frame_en) begin base_x <= cfg_tx; base_y <= cfg_ty; end
            if (ex_done) pts_ready <= 1'b1;
            // 邻格读返回（打分）
            if (np_v) begin
                if (np_occ) begin any <= 1'b1; if (dd_c < best) best <= dd_c; end
            end
            case (st)
            S_IDLE: begin
                if (cmd_clear) begin st <= S_CLR; clr_a <= 0; end
                else if (cmd_eval || cmd_update) begin
                    nsnap <= n_pts;
                    stride <= (n_pts <= 1024) ? 3'd1 : (n_pts <= 2048) ? 3'd2 : (n_pts <= 3072) ? 3'd3 : 3'd4;
                    cxm <= cfg_cx_mm; cym <= cfg_cy_mm;
                    j <= 0; jj <= 0;
                    if (cmd_eval) begin
                        md <= MD_GATE; nval <= 0; ncand <= cfg_ncand; k <= 0;
                        dx_c <= 0; dy_c <= 0; dyaw_c <= 0; st <= S_LUT;
                    end else begin
                        md <= MD_UPD; pts_ready <= 1'b0; ncl <= 0;
                        dx_c <= upd_dx; dy_c <= upd_dy; dyaw_c <= upd_dyaw; st <= S_LUT;
                    end
                end else if (dbg_req && !dbg_ack) begin
                    sm_re_m <= 1'b1; sm_ra_m <= dbg_addr; st <= S_DBG;
                end
            end
            S_DBG:  st <= S_DBG2;
            S_DBG2: begin dbg_q <= sm_q; dbg_ack <= 1'b1; st <= S_IDLE; end
            S_CLR: begin
                sm_we_m <= 1'b1; sm_wa_m <= clr_a[AW-1:0]; sm_wd_m <= 23'd0;
                clr_a <= clr_a + 1'b1;
                if (clr_a[AW-1:0] == {AW{1'b1}}) st <= S_IDLE;
            end
            // ---------------- 候选 ----------------
            S_CD0: begin cd_re <= 1'b1; cd_ra <= {k[7:0], 1'b0}; st <= S_CD1; end
            S_CD1: begin cd_re <= 1'b1; cd_ra <= {k[7:0], 1'b1}; st <= S_CD2; end
            S_CD2: begin dx_c <= {{16{cd_q[15]}}, cd_q[15:0]}; dy_c <= {{16{cd_q[31]}}, cd_q[31:16]}; st <= S_EV0; end
            S_EV0: begin dyaw_c <= cd_q[15:0]; st <= S_LUT; end
            S_LUT: begin
                lut_re <= 1'b1; lut_si <= da_r[15:6]; lut_ci <= da_r[15:6] + 10'd256;
                j <= 0; jj <= 0; cost_acc <= 0; near_acc <= 0;
                st <= S_LUTW;
            end
            S_LUTW: st <= S_T0;
            // ---------------- 每点变换 ----------------
            S_T0: begin
                if (j == 0) begin sn <= lut_sq; cs <= lut_cq; end
                if (j >= nsnap) begin
                    st <= (md == MD_GATE) ? S_CD0 : (md == MD_SCORE) ? S_RES : S_UC0;
                    if (md == MD_GATE) begin md <= MD_SCORE; k <= 0; end
                    cli <= 0;
                end else begin
                    pb_re <= 1'b1; pb_ra <= j[PB_AW-1:0];
                    ac_re <= 1'b1; ac_ra <= jj;
                    st <= S_T0W;
                end
            end
            S_T0W: st <= S_T1;
            S_T1: begin
                rx <= rxw[17:0]; ry <= ryw[17:0];
                acc_ok <= ac_q;
                if (md == MD_SCORE && !ac_q) begin
                    j <= jn; jj <= jj + 1'b1; st <= S_T0;          // 门控未过：跳过
                end else st <= S_T2;
            end
            S_T2: begin
                pxc <= rx * cs; pys <= ry * sn; pxs <= rx * sn; pyc <= ry * cs;
                st <= S_T3;
            end
            S_T3: begin
                xr <= clip25(cxm + (rotx >>> 14) + dx_c);
                yr <= clip25(cym + (roty >>> 14) + dy_c);
                st <= S_T4;
            end
            S_T4: begin
                fxr <= fxm >>> 12; fyr <= fym >>> 12;
                st <= S_T5;
            end
            S_T5: begin
                case (md)
                MD_GATE: begin
                    sm_re_m <= 1'b1; sm_ra_m <= {wya[LOG2N-1:0], wxa[LOG2N-1:0]};
                    acc_ok <= core; st <= S_G0;
                end
                MD_SCORE: begin nb <= 0; best <= 8'd255; any <= 1'b0; st <= S_NB; end
                default: begin
                    u_slot <= {wya[LOG2N-1:0], wxa[LOG2N-1:0]};
                    u_tag <= {wya[LOG2N+1:LOG2N], wxa[LOG2N+1:LOG2N]};
                    u_mx <= sxf; u_my <= syf;
                    sm_re_m <= 1'b1; sm_ra_m <= {wya[LOG2N-1:0], wxa[LOG2N-1:0]};
                    tb_re <= 1'b1; tb_ra <= {wya[LOG2N-1:0], wxa[LOG2N-1:0]};
                    st <= S_U0;
                end
                endcase
            end
            // ---------------- 门控 ----------------
            S_G0: st <= S_G1;
            S_G1: begin
                ac_we <= 1'b1; ac_wa <= jj; ac_wd <= acc_ok & sm_q[0];
                if (acc_ok & sm_q[0]) nval <= nval + 1'b1;
                j <= jn; jj <= jj + 1'b1; st <= S_T0;
            end
            // ---------------- 打分：9 个邻格 ----------------
            S_NB: begin
                sm_re_m <= 1'b1; sm_ra_m <= {nby[LOG2N-1:0], nbx[LOG2N-1:0]};
                nb_v1 <= 1'b1; nb_d1 <= nb;
                if (nb == 4'd8) st <= S_ACC;
                nb <= nb + 1'b1;
            end
            S_ACC: if (!nb_v && !nb_v1 && !nq_v && !np_v) begin
                cost_acc <= cost_acc + ((any && best < CN) ? best : CN);
                near_acc <= near_acc + any;
                j <= jn; jj <= jj + 1'b1; st <= S_T0;
            end
            S_RES: begin
                rs_we <= 1'b1; rs_wa <= {k[7:0], 1'b0}; rs_wd <= {16'd0, cost_acc};
                st <= S_RES2;
            end
            // ---------------- 并图 ----------------
            S_U0: st <= S_U1;
            S_U1: begin uw_q <= sm_q; utb_q <= tb_q; st <= S_U2; end
            S_U2: begin
                fresh <= fresh_w; h0 <= fresh_w ? 3'd0 : w_hit;
                dcx <= $signed({2'd0, u_mx}) - $signed({2'd0, w_cx});
                dcy <= $signed({2'd0, u_my}) - $signed({2'd0, w_cy});
                st <= S_U3;
            end
            S_U3: begin
                if (!utb_q) begin
                    tb_we <= 1'b1; tb_wa <= u_slot; tb_wd <= 1'b1;
                    sl_we <= 1'b1; sl_wa <= ncl[9:0]; sl_wd <= u_slot; ncl <= ncl + 1'b1;
                    sm_we_m <= 1'b1; sm_wa_m <= u_slot; sm_wd_m <= {u_tag, nhit, ncx, ncy, uw_q[7:0]};
                end
                j <= jn; jj <= jj + 1'b1; st <= S_T0;
            end
            S_UC0: begin
                if (cli == ncl) begin
                    lf_start <= 1'b1; lfo_x <= lf_cx - N/2; lfo_y <= lf_cy - N/2; st <= S_LFW;
                end else begin
                    sl_re <= 1'b1; sl_ra <= cli[9:0]; st <= S_UC1;
                end
            end
            S_UC1: st <= S_UC2;
            S_UC2: begin tb_we <= 1'b1; tb_wa <= sl_q; tb_wd <= 1'b0; cli <= cli + 1'b1; st <= S_UC0; end
            S_LFW: if (lf_done) st <= S_IDLE;
            S_RES2: begin
                rs_we <= 1'b1; rs_wa <= {k[7:0], 1'b1}; rs_wd <= {near_acc, nval};
                if (k + 1'b1 >= ncand) begin st <= S_IDLE; eval_done <= 1'b1; end
                else begin k <= k + 1'b1; st <= S_CD0; end
            end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
