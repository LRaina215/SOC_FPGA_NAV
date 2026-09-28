// 可通行性分析：扫描窗口内全部栅格，生成代价图（写入后台缓冲）。
// 第 1 遍（(N+1)^2 拍）：按窗口坐标光栅扫描高程图，3×3 邻域计算台阶/坡度/粗糙度 -> 原始代价、致命位图；
//                        同时清除过期（年龄超限）或不属于当前窗口（标签不符）的槽。
// 第 2 遍（(N+R)^2 拍）：致命位图做 (2R+1)^2 方形膨胀，膨胀区原始代价 < 253 的改为 253。
// 运算与 model/nav_fixed.py:trav_analysis 逐位一致。
module trav_analysis #(
    parameter integer LOG2N = 7,
    parameter integer TAGB  = 4
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire signed [31:0] cx,
    input  wire signed [31:0] cy,
    input  wire [7:0]  fc,
    input  wire [7:0]  cfg_age_max,
    input  wire [3:0]  cfg_cnt_min,
    input  wire [15:0] cfg_step_max,
    input  wire [15:0] cfg_rough_max,
    input  wire [15:0] cfg_slope_max,
    input  wire [7:0]  cfg_k_slope,
    input  wire [7:0]  cfg_k_step,
    input  wire [7:0]  cfg_k_rough,
    input  wire [3:0]  cfg_infl_r,
    output reg         busy,
    output reg         done,

    // 高程图口
    output wire        em_re,
    output wire [2*LOG2N-1:0] em_raddr,
    input  wire [51:0] em_q,
    output reg         em_we,
    output reg  [2*LOG2N-1:0] em_waddr,
    output reg  [51:0] em_wdata,

    // 代价图后台缓冲口
    output reg         cm_we,
    output reg  [2*LOG2N-1:0] cm_waddr,
    output reg  [7:0]  cm_wdata,
    output wire        cm_re,
    output wire [2*LOG2N-1:0] cm_raddr,
    input  wire [7:0]  cm_q
);
    localparam integer N  = 1 << LOG2N;
    localparam integer AW = 2 * LOG2N;
    localparam [7:0] C_INS = 8'd253, C_LETHAL = 8'd254, C_UNK = 8'd255, C_FREE_MAX = 8'd200;

    localparam [1:0] P_IDLE = 2'd0, P_ONE = 2'd1, P_TWO = 2'd2, P_FLUSH = 2'd3;
    reg [1:0] ph;
    reg [LOG2N+1:0] r, c;           // 0..N+R
    reg signed [31:0] cx_l, cy_l;
    reg [3:0] R;
    reg [3:0] flush;
    reg p2_phase;
    reg p2x1_valid;

    // 窗口坐标 -> 世界栅格
    wire signed [31:0] gx  = cx_l - N/2 + $signed({1'b0, c});
    wire signed [31:0] gy  = cy_l - N/2 + $signed({1'b0, r});
    wire signed [31:0] gxm = gx - 1;
    wire signed [31:0] gym = gy - 1;

    // ================================================================ 第 1 遍
    // T0：发读
    wire inr0 = (r < N) && (c < N);
    reg  v1, inr1, top_ok1, colN1, out1;
    reg  [LOG2N:0] c1;
    reg  [2*TAGB-1:0] etag1;
    reg  [AW-1:0] sl1;              // 读出格的槽
    reg  [AW-1:0] csl1;             // 中心格 (c-1, r-1) 的槽
    reg  [AW-1:0] cwi1;             // 中心格窗口序号

    // T1：派生值
    wire [7:0]  q_stamp = em_q[51:44];
    wire [7:0]  q_tag   = em_q[43:36];
    wire [3:0]  q_cnt   = em_q[35:32];
    wire signed [15:0] q_zmin = em_q[31:16];
    wire signed [15:0] q_zmax = em_q[15:0];
    wire [7:0]  q_age = fc - q_stamp;
    wire        q_live = (q_cnt != 0) && (q_tag == etag1) && (q_age <= cfg_age_max);
    wire        q_valid = inr1 && q_live && (q_cnt >= cfg_cnt_min);
    wire [16:0] q_rough = {q_zmax[15], q_zmax} - {q_zmin[15], q_zmin};

    // 行缓冲：LB1 = 上一行，LB2 = 上上行，存 {valid, h, rough}
    reg [32:0] lb1 [0:N];
    reg [16:0] lb2 [0:N];
    // 3×3 窗口寄存器：[行][列]，行 0=上 1=中 2=下，列 0=左 1=中 2=右
    reg        wv [0:2][0:2];
    reg signed [15:0] wh [0:2][0:2];
    reg [15:0] crough [0:2];        // 中行粗糙度
    reg        v2, out2;
    reg [AW-1:0] csl2, cwi2;

    wire [32:0] lb1_q = lb1[c1];
    wire [16:0] lb2_q = lb2[c1];
    integer i, j;

    // T2：差值
    function [16:0] absd(input signed [15:0] a, input signed [15:0] b);
        reg signed [16:0] d;
        begin
            d = {a[15], a} - {b[15], b};
            absd = d[16] ? -d : d;
        end
    endfunction
    function signed [17:0] gradf(input va, input signed [15:0] ha, input vb, input signed [15:0] hb,
                                  input signed [15:0] hc);
        begin
            if (va && vb)      gradf = {{2{hb[15]}}, hb} - {{2{ha[15]}}, ha};
            else if (vb)       gradf = ({{2{hb[15]}}, hb} - {{2{hc[15]}}, hc}) <<< 1;
            else if (va)       gradf = ({{2{hc[15]}}, hc} - {{2{ha[15]}}, ha}) <<< 1;
            else               gradf = 18'sd0;
        end
    endfunction

    wire signed [15:0] hc = wh[1][1];
    wire [16:0] d00 = wv[0][0] ? absd(wh[0][0], hc) : 17'd0;
    wire [16:0] d01 = wv[0][1] ? absd(wh[0][1], hc) : 17'd0;
    wire [16:0] d02 = wv[0][2] ? absd(wh[0][2], hc) : 17'd0;
    wire [16:0] d10 = wv[1][0] ? absd(wh[1][0], hc) : 17'd0;
    wire [16:0] d12 = wv[1][2] ? absd(wh[1][2], hc) : 17'd0;
    wire [16:0] d20 = wv[2][0] ? absd(wh[2][0], hc) : 17'd0;
    wire [16:0] d21 = wv[2][1] ? absd(wh[2][1], hc) : 17'd0;
    wire [16:0] d22 = wv[2][2] ? absd(wh[2][2], hc) : 17'd0;
    function [16:0] mx2(input [16:0] a, input [16:0] b); mx2 = (a > b) ? a : b; endfunction
    // T2 寄存 8 个差值，T2b 求最大值
    reg  [16:0] dq [0:7];
    reg  [16:0] p4 [0:3];           // T2b：两两取大
    reg         v2d, out2d, cval2d;
    reg  [AW-1:0] csl2d, cwi2d;
    reg  [17:0] agx2d, agy2d;
    reg  [15:0] rough2d;
    reg  [16:0] mh0, mh1;           // T2c：两组 4 取 1
    wire [16:0] step_w = mx2(mh0, mh1);
    reg         v2c, out2c, cval2c;
    reg  [AW-1:0] csl2c, cwi2c;
    reg  [17:0] agx2c, agy2c;
    reg  [15:0] rough2c;
    reg         v2b, out2b, cval2b;
    reg  [AW-1:0] csl2b, cwi2b;
    reg  [17:0] agx2b, agy2b;
    reg  [15:0] rough2b;
    wire signed [17:0] gx_w = gradf(wv[1][0], wh[1][0], wv[1][2], wh[1][2], hc);
    wire signed [17:0] gy_w = gradf(wv[0][1], wh[0][1], wv[2][1], wh[2][1], hc);

    reg        v3, out3, cval3;
    reg [AW-1:0] csl3, cwi3;
    reg [16:0] step3;
    reg [17:0] agx3, agy3;
    reg [15:0] rough3;

    // T3：代价
    wire [17:0] smax = (agx3 > agy3) ? agx3 : agy3;
    wire [17:0] smin = (agx3 > agy3) ? agy3 : agx3;
    wire [18:0] slope = {1'b0, smax} + {2'b0, smin[17:1]};
    wire lethal_w = (step3 > {1'b0, cfg_step_max}) || ({1'b0, rough3} > {1'b0, cfg_rough_max})
                    || (slope > {3'b0, cfg_slope_max});
    wire [15:0] slope_c = (slope > 19'd65535) ? 16'hFFFF : slope[15:0];
    wire [15:0] step_c  = (step3 > 17'd65535) ? 16'hFFFF : step3[15:0];
    wire [25:0] wsum = cfg_k_slope * slope_c + cfg_k_step * step_c + cfg_k_rough * rough3;
    wire [17:0] wsh  = wsum[25:8];
    wire [7:0]  cfree = (wsh > C_FREE_MAX) ? C_FREE_MAX : wsh[7:0];

    // 致命位图（窗口坐标）
    reg          bm_we;
    reg [AW-1:0] bm_wa;
    reg          bm_wd;
    wire          bm_re;
    wire [AW-1:0] bm_ra;
    wire         bm_q;
    nav_ram_sdp #(.DW(1), .AW(AW)) u_bm (
        .clk(clk), .we(bm_we), .waddr(bm_wa), .wdata(bm_wd),
        .re(bm_re), .raddr(bm_ra), .rdata(bm_q)
    );

    // ================================================================ 第 2 遍
    reg        p2v1, p2in1, p2first_c1, p2first_r1, p2out1;
    reg [LOG2N:0] p2x1;
    reg [AW-1:0]  p2sl1;
    reg [4:0]  hc_cnt;
    reg [4:0]  vc [0:N-1];
    wire [4:0] two_r = {R, 1'b0};
    wire       b2 = p2in1 & bm_q;
    wire [4:0] hc_new = b2 ? 5'd0 : (p2first_c1 ? 5'd31 : ((hc_cnt == 5'd31) ? 5'd31 : hc_cnt + 5'd1));
    wire       hd = (hc_new <= two_r);
    wire [4:0] vc_old = vc[p2x1[LOG2N-1:0]];
    wire [4:0] vc_new = hd ? 5'd0 : (p2first_r1 ? 5'd31 : ((vc_old == 5'd31) ? 5'd31 : vc_old + 5'd1));
    wire       infl1 = (vc_new <= two_r);
    reg        p2v2, infl2;
    reg [AW-1:0] p2sl2;

    // 读口组合驱动：同步读 RAM 的数据在下一拍（T1）返回
    assign em_re    = (ph == P_ONE) && inr0;
    assign em_raddr = {gy[LOG2N-1:0], gx[LOG2N-1:0]};
    assign bm_re    = (ph == P_TWO) && (r < N) && (c < N);
    assign bm_ra    = {r[LOG2N-1:0], c[LOG2N-1:0]};
    assign cm_re    = p2v1 && p2out1;
    assign cm_raddr = p2sl1;

    wire [LOG2N+1:0] lim = (ph == P_TWO) ? (N - 1 + R) : N;

    always @(posedge clk) begin
        em_we <= 1'b0; cm_we <= 1'b0;
        bm_we <= 1'b0; done <= 1'b0;
        if (rst) begin
            ph <= P_IDLE; busy <= 1'b0;
            v1 <= 1'b0; v2 <= 1'b0; v2b <= 1'b0; v2d <= 1'b0; v2c <= 1'b0; v3 <= 1'b0; p2v1 <= 1'b0; p2v2 <= 1'b0;
        end else begin
            // ---------------- 扫描计数 ----------------
            case (ph)
            P_IDLE: if (start) begin
                ph <= P_ONE; busy <= 1'b1; r <= 0; c <= 0;
                cx_l <= cx; cy_l <= cy; R <= (cfg_infl_r > 4'd8) ? 4'd8 : cfg_infl_r;
            end
            P_ONE, P_TWO: begin
                if (c == lim) begin
                    c <= 0;
                    if (r == lim) begin
                        r <= 0;
                        ph <= P_FLUSH; flush <= 4'd8;
                    end else r <= r + 1'b1;
                end else c <= c + 1'b1;
            end
            P_FLUSH: begin
                flush <= flush - 1'b1;
                if (flush == 0) begin
                    if (p2_phase) begin
                        ph <= P_IDLE; busy <= 1'b0; done <= 1'b1;
                    end else begin
                        ph <= P_TWO; r <= 0; c <= 0;
                    end
                end
            end
            endcase

            // ---------------- 第 1 遍流水 ----------------
            // T0
            v1 <= (ph == P_ONE);
            if (ph == P_ONE) begin
                inr1  <= inr0;
                c1    <= c[LOG2N:0];
                colN1 <= (c == N);
                top_ok1 <= (r >= 2);
                out1  <= (r >= 1) && (c >= 1);
                etag1 <= {gy[LOG2N+TAGB-1:LOG2N], gx[LOG2N+TAGB-1:LOG2N]};
                sl1   <= {gy[LOG2N-1:0], gx[LOG2N-1:0]};
                csl1  <= {gym[LOG2N-1:0], gxm[LOG2N-1:0]};
                cwi1  <= {r[LOG2N-1:0] - 1'b1, c[LOG2N-1:0] - 1'b1};
            end
            // T1
            v2 <= v1;
            if (v1) begin
                // 过期/不属于窗口的槽清零（cnt=0）
                if (inr1 && q_cnt != 0 && !q_live) begin
                    em_we    <= 1'b1;
                    em_waddr <= sl1;
                    em_wdata <= {em_q[51:36], 4'd0, em_q[31:0]};
                end
                lb1[c1] <= {q_valid, q_zmax, q_rough[15:0]};
                lb2[c1] <= lb1_q[32:16];
                for (i = 0; i < 3; i = i + 1) begin
                    wv[i][0] <= wv[i][1]; wh[i][0] <= wh[i][1];
                    wv[i][1] <= wv[i][2]; wh[i][1] <= wh[i][2];
                end
                crough[0] <= crough[1]; crough[1] <= crough[2];
                if (colN1) begin
                    wv[0][2] <= 1'b0; wv[1][2] <= 1'b0; wv[2][2] <= 1'b0;
                    wh[0][2] <= 16'sd0; wh[1][2] <= 16'sd0; wh[2][2] <= 16'sd0;
                    crough[2] <= 16'd0;
                end else begin
                    wv[0][2] <= top_ok1 & lb2_q[16]; wh[0][2] <= lb2_q[15:0];
                    wv[1][2] <= lb1_q[32];           wh[1][2] <= lb1_q[31:16];
                    crough[2] <= lb1_q[15:0];
                    wv[2][2] <= q_valid;             wh[2][2] <= q_zmax;
                end
                out2 <= out1; csl2 <= csl1; cwi2 <= cwi1;
            end
            // T2：差值与梯度
            v2b <= v2;
            if (v2) begin
                out2b <= out2; csl2b <= csl2; cwi2b <= cwi2;
                cval2b <= wv[1][1];
                dq[0] <= d00; dq[1] <= d01; dq[2] <= d02; dq[3] <= d10;
                dq[4] <= d12; dq[5] <= d20; dq[6] <= d21; dq[7] <= d22;
                agx2b <= gx_w[17] ? -gx_w : gx_w;
                agy2b <= gy_w[17] ? -gy_w : gy_w;
                rough2b <= crough[1];
            end
            // T2b：台阶最大值，两两取大
            v2d <= v2b;
            if (v2b) begin
                out2d <= out2b; csl2d <= csl2b; cwi2d <= cwi2b; cval2d <= cval2b;
                p4[0] <= mx2(dq[0], dq[1]); p4[1] <= mx2(dq[2], dq[3]);
                p4[2] <= mx2(dq[4], dq[5]); p4[3] <= mx2(dq[6], dq[7]);
                agx2d <= agx2b; agy2d <= agy2b; rough2d <= rough2b;
            end
            // T2c：两组
            v2c <= v2d;
            if (v2d) begin
                out2c <= out2d; csl2c <= csl2d; cwi2c <= cwi2d; cval2c <= cval2d;
                mh0 <= mx2(p4[0], p4[1]);
                mh1 <= mx2(p4[2], p4[3]);
                agx2c <= agx2d; agy2c <= agy2d; rough2c <= rough2d;
            end
            // T2c：台阶最大值
            v3 <= v2c;
            if (v2c) begin
                out3  <= out2c; csl3 <= csl2c; cwi3 <= cwi2c;
                cval3 <= cval2c;
                step3 <= step_w;
                agx3  <= agx2c;
                agy3  <= agy2c;
                rough3 <= rough2c;
            end
            // T3
            if (v3 && out3) begin
                cm_we    <= 1'b1;
                cm_waddr <= csl3;
                cm_wdata <= !cval3 ? C_UNK : lethal_w ? C_LETHAL : cfree;
                bm_we    <= 1'b1;
                bm_wa    <= cwi3;
                bm_wd    <= cval3 & lethal_w;
            end

            // ---------------- 第 2 遍流水 ----------------
            p2v1 <= (ph == P_TWO);
            if (ph == P_TWO) begin
                p2in1 <= (r < N) && (c < N);
                p2first_c1 <= (c == 0);
                p2first_r1 <= (r == 0);   // 每列的竖向计数在第 0 行复位
                p2x1  <= c - R;
                p2out1 <= (r >= R) && (c >= R);
                p2sl1 <= {gy[LOG2N-1:0] - R, gx[LOG2N-1:0] - R};
            end
            p2v2 <= 1'b0;
            if (p2v1) begin
                hc_cnt <= hc_new;
                if (p2x1_valid) vc[p2x1[LOG2N-1:0]] <= vc_new;
                if (p2out1) begin
                    p2v2     <= 1'b1;
                    p2sl2    <= p2sl1;
                    infl2    <= infl1;
                end
            end
            if (p2v2) begin
                if (infl2 && cm_q < C_INS) begin
                    cm_we    <= 1'b1;
                    cm_waddr <= p2sl2;
                    cm_wdata <= C_INS;
                end
            end
        end
    end

    always @(posedge clk) begin
        if (rst || (ph == P_IDLE)) p2_phase <= 1'b0;
        else if (ph == P_TWO) p2_phase <= 1'b1;
    end
    // x = c - R 是否已进入窗口（c >= R）
    always @(posedge clk) if (ph == P_TWO) p2x1_valid <= (c >= R);
endmodule
