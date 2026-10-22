// 可通行性分析 v3（门控基线坡度 + 坡度外推落差 + 空洞填补 + 近区保持），与 model/trav_v3.py 逐位一致。
//
// 四遍扫描（每遍约 (N+2)^2 拍，N=128 时共约 6.8 万拍，100 MHz 下 0.7 ms）：
//   A 正向：读高程图（同时清除过期/标签不符的槽；窗口中心 ±KEEP_R 格不过期），5×5 窗口求
//           台阶（8 邻域最大高差）、x/y 向 4 格门控基线净升高、粗糙度，以及向 -x/-y 的遮挡落差
//           -> 原始代价写后台代价图，致命位写位图。
//   B 反向：同一前端倒序扫描，求向 +x/+y 的遮挡落差（倒序时它们正好是"已扫描方向"）-> 补写致命。
//   C 正向：空洞填补：未观测格若 8 邻域足够多已观测、高差极差小、邻居无致命 -> 写填补代价。
//   D 正向：致命位图方形膨胀（与 v1 第 2 遍相同）。
// 遮挡落差的"隔着空洞最近的已观测格"用扫描寄存器求：行方向一组寄存器，列方向每列一组（N 项小数组）。
// 基线长度固定 4 格。
module trav_v3 #(
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
    input  wire [3:0]  cfg_keep_r,
    input  wire [15:0] cfg_step_max,
    input  wire        cfg_step_high_only,
    input  wire [15:0] cfg_rough_max,
    input  wire [15:0] cfg_slope_rise,
    input  wire [7:0]  cfg_jump,
    input  wire [4:0]  cfg_drop_k,
    input  wire [7:0]  cfg_drop_slope,
    input  wire [7:0]  cfg_k_slope,
    input  wire [7:0]  cfg_k_step,
    input  wire [7:0]  cfg_k_rough,
    input  wire [3:0]  cfg_fill_min,
    input  wire [7:0]  cfg_fill_tol,
    input  wire [7:0]  cfg_fill_cost,
    input  wire [3:0]  cfg_infl_r,
    input  wire        cfg_infl_disk,       // 1：圆形膨胀（半径 <= 4 时；否则方形）
    output reg         busy,
    output reg         done,

    output wire        em_re,
    output wire [2*LOG2N-1:0] em_raddr,
    input  wire [52:0] em_q,
    output reg         em_we,
    output reg  [2*LOG2N-1:0] em_waddr,
    output reg  [52:0] em_wdata,

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
    localparam [2:0] P_IDLE = 3'd0, P_A = 3'd1, P_B = 3'd2, P_C = 3'd3, P_D = 3'd4, P_FLUSH = 3'd5;
    localparam integer LB = N + 2;              // 每行扫描列数（多 2 列无效格作行间隔）
    localparam integer CW = 34;                 // 窗口格 {b, v, h16, ro16}

    reg [2:0] ph, ph_next;
    reg [LOG2N+1:0] r, c;
    reg signed [31:0] cx_l, cy_l;
    reg [3:0]  R;
    reg [4:0]  flush;
    reg        rev;                             // 当前前端扫描为倒序（B 遍）

    // ---------------------------------------------------------------- T0：读地址
    wire front_ph = (ph == P_A) || (ph == P_B) || (ph == P_C);
    wire inr0 = (r < N) && (c < N);
    wire [LOG2N-1:0] rr = r[LOG2N-1:0], ccl = c[LOG2N-1:0];
    wire [LOG2N-1:0] ty0 = rev ? (N - 1 - rr) : rr;          // 真实窗口坐标
    wire [LOG2N-1:0] tx0 = rev ? (N - 1 - ccl) : ccl;
    wire signed [31:0] gx0 = cx_l - N/2 + $signed({1'b0, tx0});
    wire signed [31:0] gy0 = cy_l - N/2 + $signed({1'b0, ty0});

    // 致命位图（窗口坐标）
    reg          bm_we;
    reg [AW-1:0] bm_wa;
    reg          bm_wd;
    wire         bm_re;
    wire [AW-1:0] bm_ra;
    wire         bm_q;
    nav_ram_sdp #(.DW(1), .AW(AW)) u_bm (
        .clk(clk), .we(bm_we), .waddr(bm_wa), .wdata(bm_wd), .re(bm_re), .raddr(bm_ra), .rdata(bm_q)
    );

    // ---------------------------------------------------------------- T1：解码
    reg        v1, inr1;
    reg [2:0]  ph1, phw;
    reg [LOG2N+1:0] r1, c1;
    reg [LOG2N-1:0] tx1, ty1;
    reg [2*TAGB-1:0] etag1;
    reg [AW-1:0] sl1;
    wire [7:0]  q_stamp = em_q[51:44];
    wire [7:0]  q_tag   = em_q[43:36];
    wire [3:0]  q_cnt   = em_q[35:32];
    wire signed [15:0] q_zmin = em_q[31:16];
    wire signed [15:0] q_zmax = em_q[15:0];
    wire [7:0]  q_age = fc - q_stamp;
    wire [LOG2N-1:0] kdx = (tx1 >= N/2) ? (tx1 - N/2) : (N/2 - tx1);
    wire [LOG2N-1:0] kdy = (ty1 >= N/2) ? (ty1 - N/2) : (N/2 - ty1);
    wire        q_keep = (kdx <= cfg_keep_r) && (kdy <= cfg_keep_r);
    wire q_stale=em_q[52];
    wire        q_live = (q_cnt != 0) && (q_tag == etag1) && ((!q_stale && q_age <= cfg_age_max) || q_keep);
    wire        q_valid = inr1 && q_live && (q_cnt >= cfg_cnt_min);
    wire [16:0] q_rough17 = {q_zmax[15], q_zmax} - {q_zmin[15], q_zmin};
    wire [15:0] q_rough = q_rough17[16] ? 16'd0 : q_rough17[15:0];     // zmax >= zmin 恒成立
    wire        q_b = inr1 && bm_q && (ph1 == P_C);
    wire [CW-1:0] cur = {q_b, q_valid, q_zmax, q_rough};

    // 行缓存：lbk[c] = 第 r-k 行、第 c 列
    reg [CW-1:0] lb1 [0:LB-1];
    reg [CW-1:0] lb2 [0:LB-1];
    reg [CW-1:0] lb3 [0:LB-1];
    reg [CW-1:0] lb4 [0:LB-1];
    wire [CW-1:0] l1 = lb1[c1], l2 = lb2[c1], l3 = lb3[c1], l4 = lb4[c1];
    // 窗口 W[行][列]（扫描方向：行 0 最早，列 0 最早）
    reg [CW-1:0] W [0:4][0:4];
    reg        wval;                            // 窗口中心有效（r1>=2 且 c1>=2）
    reg [LOG2N-1:0] wcr, wcc;                   // 中心的扫描行/列
    integer i, j;

    // ---------------------------------------------------------------- 窗口按真实方向取格
    // 正序：列 0 = x-2；倒序：列 0 = x+2。O[i][j]：i=0 为 y-2，j=0 为 x-2（真实方向）
    wire [CW-1:0] O [0:4][0:4];
    genvar gi, gj;
    generate
        for (gi = 0; gi < 5; gi = gi + 1) begin : g_or
            for (gj = 0; gj < 5; gj = gj + 1) begin : g_oc
                assign O[gi][gj] = rev ? W[4-gi][4-gj] : W[gi][gj];
            end
        end
    endgenerate
    function [16:0] absd(input signed [15:0] a, input signed [15:0] b);
        reg signed [16:0] d;
        begin
            d = {a[15], a} - {b[15], b};
            absd = d[16] ? -d : d;
        end
    endfunction
    function [16:0] stepd(input signed [15:0] center, input signed [15:0] neighbor);
        reg signed [16:0] d;
        begin
            d = {center[15], center} - {neighbor[15], neighbor};
            stepd = d[16] ? (cfg_step_high_only ? 17'd0 : -d) : d;
        end
    endfunction
    function signed [16:0] sdiff(input signed [15:0] a, input signed [15:0] b);   // a - b
        sdiff = {a[15], a} - {b[15], b};
    endfunction
    function [16:0] mx17(input [16:0] a, input [16:0] b); mx17 = (a > b) ? a : b; endfunction
    function signed [15:0] smax16(input signed [15:0] a, input signed [15:0] b); smax16 = (a > b) ? a : b; endfunction
    function signed [15:0] smin16(input signed [15:0] a, input signed [15:0] b); smin16 = (a < b) ? a : b; endfunction

    wire        c_v  = O[2][2][32];
    wire signed [15:0] c_h = O[2][2][31:16];
    wire [15:0] c_ro = O[2][2][15:0];
    // 8 邻域（真实方向），顺序 (-1,-1) (-1,0) (-1,1) (0,-1) (0,1) (1,-1) (1,0) (1,1)
    wire [CW-1:0] nbw [0:7];
    assign nbw[0] = O[1][1]; assign nbw[1] = O[1][2]; assign nbw[2] = O[1][3]; assign nbw[3] = O[2][1];
    assign nbw[4] = O[2][3]; assign nbw[5] = O[3][1]; assign nbw[6] = O[3][2]; assign nbw[7] = O[3][3];

    // 扫描方向的"前一格"（行向：同行前一列；列向：同列前一行）——落差扫描寄存器用
    wire        sl_v = W[2][1][32];
    wire signed [15:0] sl_h = W[2][1][31:16];
    wire        su_v = W[1][2][32];
    wire signed [15:0] su_h = W[1][2][31:16];
    // 行向扫描寄存器
    reg        trv;
    reg signed [15:0] trh;
    reg [4:0]  trd;
    // 列向扫描寄存器（每列一组）
    // 同步读 RAM：T1 按下一个中心的列号发读，T2 取数并写回同列（避免"同址读写"的单口推断，布局布线不支持）
    reg        ct_we;
    reg [LOG2N-1:0] ct_wa;
    reg [21:0] ct_wd;
    wire [21:0] ct_q;
    wire        ct_re = v1 && (c1 >= 2) && (c1 < N + 2);
    wire [LOG2N-1:0] ct_ra = c1[LOG2N-1:0] - 2'd2;
    nav_ram_sdp #(.DW(22), .AW(LOG2N)) u_ct (
        .clk(clk), .we(ct_we), .waddr(ct_wa), .wdata(ct_wd), .re(ct_re), .raddr(ct_ra), .rdata(ct_q)
    );
    wire       e_rv = (wcc == 0) ? 1'b0 : trv;
    wire       e_cv = (wcr == 0) ? 1'b0 : ct_q[21];
    wire signed [15:0] e_ch = ct_q[20:5];
    wire [4:0] e_cd = ct_q[4:0];
    wire [4:0] trd_inc = (trd == 5'd31) ? 5'd31 : trd + 5'd1;
    wire [4:0] ctd_inc = (e_cd == 5'd31) ? 5'd31 : e_cd + 5'd1;

    // 中心的真实窗口坐标与槽
    wire [LOG2N-1:0] cty = rev ? (N - 1 - wcr) : wcr;
    wire [LOG2N-1:0] ctx = rev ? (N - 1 - wcc) : wcc;
    wire signed [31:0] cgx = cx_l - N/2 + $signed({1'b0, ctx});
    wire signed [31:0] cgy = cy_l - N/2 + $signed({1'b0, cty});

    // ---------------------------------------------------------------- 流水
    // T2：按真实方向寄存原始格（中心、8 邻域、中心行 5 格、中心列 5 格）与落差扫描寄存器的结果
    reg        s2;  reg [2:0] ph2; reg [AW-1:0] sl2, wi2;
    reg        cv2; reg signed [15:0] ch2; reg [15:0] cro2;
    reg        nv2 [0:7]; reg signed [15:0] nh2 [0:7]; reg nb2 [0:7];
    reg        xv2 [0:4]; reg signed [15:0] xh2 [0:4];
    reg        yv2 [0:4]; reg signed [15:0] yh2 [0:4];
    reg        fr2, fcl2, nr2, ncl2;
    reg signed [15:0] hr2, hcl2;
    reg [4:0]  kr2, kcl2;
    // T3
    reg        s3;  reg [2:0] ph3; reg [AW-1:0] sl3, wi3;
    reg        cv3; reg signed [15:0] ch3; reg [15:0] cro3;
    reg [16:0] ad3 [0:7];
    reg signed [16:0] dx3 [0:3], dy3 [0:3];
    reg        okx3, oky3;
    reg [3:0]  nval3; reg nleth3;
    reg        mv3 [0:3]; reg signed [15:0] mxh3 [0:3], mnh3 [0:3];
    reg        fr3, fcl3, nr3, ncl3; reg signed [15:0] hr3, hcl3; reg [4:0] kr3, kcl3;
    // T4
    reg        s4;  reg [2:0] ph4; reg [AW-1:0] sl4, wi4;
    reg        cv4; reg signed [15:0] ch4; reg [15:0] cro4;
    reg [16:0] st4 [0:3];
    reg signed [16:0] gx4 [0:3], gy4 [0:3];
    reg        okx4, oky4;
    reg [3:0]  nval4; reg nleth4;
    reg        mv4 [0:1]; reg signed [15:0] mxh4 [0:1], mnh4 [0:1];
    reg        fr4, fcl4, nr4, ncl4; reg signed [15:0] hr4, hcl4; reg [4:0] kr4, kcl4;
    // T5
    reg        s5;  reg [2:0] ph5; reg [AW-1:0] sl5, wi5;
    reg        cv5; reg signed [15:0] ch5; reg [15:0] cro5;
    reg [16:0] st5a, st5b;
    reg signed [18:0] sx5, sy5;
    reg        okx5, oky5;
    reg        fb5, fmv5;
    reg signed [15:0] fmx5, fmn5;
    reg        fr5, fcl5, nr5, ncl5; reg signed [15:0] hr5, hcl5; reg [4:0] kr5, kcl5;
    // T6
    reg        s6;  reg [2:0] ph6; reg [AW-1:0] sl6, wi6;
    reg        cv6; reg signed [15:0] ch6; reg [15:0] cro6;
    reg [16:0] step6;
    reg [18:0] rx6, ry6;
    reg signed [18:0] sdr6, sdc6;          // 行/列扫描方向上的有符号基线升高
    reg        okr6, okc6;
    reg        fill6;
    reg        fr6, fcl6, nr6, ncl6; reg signed [15:0] hr6, hcl6; reg [4:0] kr6, kcl6;
    // T7
    reg        s7;  reg [2:0] ph7; reg [AW-1:0] sl7, wi7;
    reg        cv7; reg [15:0] cro7;
    reg [16:0] step7;
    reg [19:0] rise7;
    reg signed [25:0] er7, ec7;            // 外推高度 - 远处高度
    reg        fill7;
    reg        fr7, fcl7, nr7, ncl7;
    // T8
    reg        s8;  reg [2:0] ph8; reg [AW-1:0] sl8, wi8;
    reg        cv8, leth8, drop8, fill8;
    reg [25:0] wsum8;

    // 组合：T5 -> T6 的外推所需
    function signed [25:0] hexp_minus_far(input ok, input signed [18:0] sd, input [4:0] k,
                                          input signed [15:0] h, input signed [15:0] hf, input [7:0] dsl);
        reg signed [25:0] prod;
        begin
            prod = sd * $signed({1'b0, k});
            if (ok) hexp_minus_far = $signed({{10{h[15]}}, h}) + (prod >>> 2) - $signed({{10{hf[15]}}, hf});
            else    hexp_minus_far = $signed({{10{h[15]}}, h}) - $signed({13'd0, k} * {5'd0, dsl}) - $signed({{10{hf[15]}}, hf});
        end
    endfunction
    function signed [16:0] gate(input signed [16:0] d, input [7:0] jmp);
        reg [16:0] ad;
        begin
            ad = d[16] ? -d : d;
            gate = (ad <= {9'd0, jmp}) ? d : 17'sd0;
        end
    endfunction
    function [18:0] abs19(input signed [18:0] a); abs19 = a[18] ? -a : a; endfunction

    wire [18:0] mxr = (rx6 > ry6) ? rx6 : ry6;
    wire [18:0] mnr = (rx6 > ry6) ? ry6 : rx6;
    wire [15:0] rise_c = (rise7 > 20'd65535) ? 16'hFFFF : rise7[15:0];
    wire [15:0] step_c = (step7 > 17'd65535) ? 16'hFFFF : step7[15:0];
    wire [17:0] wsh = wsum8[25:8];
    wire [7:0]  cfree = (wsh > C_FREE_MAX) ? C_FREE_MAX : wsh[7:0];

    // ---------------------------------------------------------------- 第 D 遍（膨胀）
    // 方形：行向"距上个致命格的列数"计数 + 列向"距上个行命中的行数"计数（同 v1 第 2 遍）。
    // 圆形（R <= 4）：行内 2R+1 位移位寄存器求输出列到最近致命格的水平距离 d（0..R，否则 7），
    // 每列保存最近 2R 行的 d（8 × 3 位），第 k 行（相对输出行）满足 d <= w(R,|k|) 即膨胀，
    // w = floor(sqrt(R² + 1 − k²))，与 model/nav_fast.py:dilate_disk 相同。
    reg        p2v1, p2in1, p2first_c1, p2first_r1, p2out1, p2x1_valid;
    reg [LOG2N:0] p2x1;
    reg [AW-1:0]  p2sl1;
    reg [4:0]  hc_cnt;
    reg [4:0]  vc [0:N-1];
    wire [4:0] two_r = {R, 1'b0};
    wire       b2 = p2in1 & bm_q;
    wire [4:0] hc_new = b2 ? 5'd0 : (p2first_c1 ? 5'd31 : ((hc_cnt == 5'd31) ? 5'd31 : hc_cnt + 5'd1));
    wire       hd = (hc_new <= two_r);
    // 行向结果先寄存一拍，再做列向计数（切断 行计数 -> 列计数 -> 输出 的长组合路径）
    reg        p2va, hda, p2first_ra, p2outa, p2x_valida;
    reg [LOG2N:0] p2xa;
    reg [AW-1:0]  p2sla;
    wire [4:0] vc_rd = vc[p2xa[LOG2N-1:0]];
    // 再寄存一拍：列计数读出 -> 计算/写回 -> 膨胀判定分开
    reg        p2vb, hdb, p2first_rb, p2outb, p2x_validb;
    reg [LOG2N:0] p2xb;
    reg [AW-1:0]  p2slb;
    reg [4:0]  vc_old;
    wire [4:0] vc_new = hdb ? 5'd0 : (p2first_rb ? 5'd31 : ((vc_old == 5'd31) ? 5'd31 : vc_old + 5'd1));
    wire       infl1 = (vc_new <= two_r);
    reg        p2v2, infl2;
    reg [AW-1:0] p2sl2;
    // 圆形膨胀
    function [2:0] dwid(input [3:0] rr, input [2:0] ak);   // w(R, |k|)
        case ({rr, ak})
            {4'd1, 3'd0}, {4'd1, 3'd1}: dwid = 3'd1;
            {4'd2, 3'd0}, {4'd2, 3'd1}: dwid = 3'd2;
            {4'd2, 3'd2}: dwid = 3'd1;
            {4'd3, 3'd0}, {4'd3, 3'd1}: dwid = 3'd3;
            {4'd3, 3'd2}: dwid = 3'd2;
            {4'd3, 3'd3}: dwid = 3'd1;
            {4'd4, 3'd0}, {4'd4, 3'd1}: dwid = 3'd4;
            {4'd4, 3'd2}: dwid = 3'd3;
            {4'd4, 3'd3}: dwid = 3'd2;
            {4'd4, 3'd4}: dwid = 3'd1;
            default: dwid = 3'd0;
        endcase
    endfunction
    wire       disk = cfg_infl_disk && (R != 4'd0) && (R <= 4'd4);
    reg  [8:0] hsr;                                  // 行内最近 9 个输入格的致命位（位 i = 前 i 格）
    wire [8:0] hsr_new = p2first_c1 ? {8'd0, b2} : {hsr[7:0], b2};
    reg  [2:0] dh1;                                  // 输出列（= 当前列 − R）到最近致命格的水平距离
    integer    di;
    always @(*) begin
        dh1 = 3'd7;
        for (di = 8; di >= 0; di = di - 1)
            if (di <= 2 * R && hsr_new[di] && ((di >= R ? di - R : R - di) < dh1)) dh1 = (di >= R ? di - R : R - di);
    end
    reg  [2:0]  dha, dhb;
    reg  [23:0] dhist [0:N-1];                       // 每列最近 8 行的 d（位 [3j+2:3j] = 前 j+1 行）
    wire [23:0] dhist_rd = dhist[p2xa[LOG2N-1:0]];
    reg  [23:0] dh_old;
    wire [23:0] dh_hist = p2first_rb ? 24'hFFFFFF : dh_old;
    reg         dinfl;
    integer     dk;
    always @(*) begin
        // 当前行（最新）对应 k = +R；前 j 行对应 k = R − j
        dinfl = (dhb <= dwid(R, R[2:0]));
        for (dk = 1; dk <= 8; dk = dk + 1)
            if (dk <= 2 * R) begin
                if (dh_hist[3*(dk-1) +: 3] <= dwid(R, (dk >= R) ? dk - R : R - dk)) dinfl = 1'b1;
            end
    end
    wire signed [31:0] dgx = cx_l - N/2 + $signed({1'b0, c});
    wire signed [31:0] dgy = cy_l - N/2 + $signed({1'b0, r});

    assign em_re    = front_ph && inr0;
    assign em_raddr = {gy0[LOG2N-1:0], gx0[LOG2N-1:0]};
    assign bm_re    = ((ph == P_C) && inr0) || ((ph == P_D) && (r < N) && (c < N));
    assign bm_ra    = (ph == P_D) ? {r[LOG2N-1:0], c[LOG2N-1:0]} : {ty0, tx0};
    assign cm_re    = p2vb && p2outb;
    assign cm_raddr = p2slb;
    wire [LOG2N+1:0] lim = (ph == P_D) ? (N - 1 + R) : (N + 1);

    always @(posedge clk) begin
        em_we <= 1'b0; cm_we <= 1'b0; bm_we <= 1'b0; done <= 1'b0; ct_we <= 1'b0;
        if (rst) begin
            ph <= P_IDLE; busy <= 1'b0; rev <= 1'b0;
            v1 <= 1'b0; wval <= 1'b0; s2 <= 1'b0; s3 <= 1'b0; s4 <= 1'b0; s5 <= 1'b0; s6 <= 1'b0;
            s7 <= 1'b0; s8 <= 1'b0; p2v1 <= 1'b0; p2va <= 1'b0; p2vb <= 1'b0; p2v2 <= 1'b0;
        end else begin
            // ---------------- 扫描计数 ----------------
            case (ph)
            P_IDLE: if (start) begin
                ph <= P_A; busy <= 1'b1; rev <= 1'b0; r <= 0; c <= 0;
                cx_l <= cx; cy_l <= cy; R <= (cfg_infl_r > 4'd8) ? 4'd8 : cfg_infl_r;
            end
            P_A, P_B, P_C, P_D: begin
                if (c == lim) begin
                    c <= 0;
                    if (r == lim) begin
                        r <= 0; flush <= 5'd16;
                        ph_next <= (ph == P_A) ? P_B : (ph == P_B) ? P_C : (ph == P_C) ? P_D : P_IDLE;
                        ph <= P_FLUSH;
                    end else r <= r + 1'b1;
                end else c <= c + 1'b1;
            end
            P_FLUSH: begin
                flush <= flush - 1'b1;
                if (flush == 0) begin
                    ph <= ph_next; rev <= (ph_next == P_B); r <= 0; c <= 0;
                    if (ph_next == P_IDLE) begin busy <= 1'b0; done <= 1'b1; end
                end
            end
            default: ph <= P_IDLE;
            endcase

            // ---------------- T0 -> T1 ----------------
            v1 <= front_ph;
            if (front_ph) begin
                ph1 <= ph; inr1 <= inr0; r1 <= r; c1 <= c; tx1 <= tx0; ty1 <= ty0;
                etag1 <= {gy0[LOG2N+TAGB-1:LOG2N], gx0[LOG2N+TAGB-1:LOG2N]};
                sl1 <= {gy0[LOG2N-1:0], gx0[LOG2N-1:0]};
            end
            // ---------------- T1：清除、行缓存、窗口 ----------------
            wval <= v1 && (r1 >= 2) && (c1 >= 2);
            if (v1) begin
                if ((ph1 == P_A) && inr1 && q_cnt != 0 && (!q_live || (!q_stale && q_age>cfg_age_max))) begin
                    // 近区保留危险证据，但锁存过期状态；8bit帧龄回绕不能使它重新变新。
                    em_we <= 1'b1; em_waddr <= sl1;
                    em_wdata <= {1'b1,em_q[51:36],q_live?q_cnt:4'd0,em_q[31:0]};
                end
                lb1[c1] <= cur; lb2[c1] <= l1; lb3[c1] <= l2; lb4[c1] <= l3;
                for (i = 0; i < 5; i = i + 1)
                    for (j = 0; j < 4; j = j + 1) W[i][j] <= W[i][j+1];
                W[0][4] <= (r1 >= 4) ? l4 : {CW{1'b0}};
                W[1][4] <= (r1 >= 3) ? l3 : {CW{1'b0}};
                W[2][4] <= (r1 >= 2) ? l2 : {CW{1'b0}};
                W[3][4] <= (r1 >= 1) ? l1 : {CW{1'b0}};
                W[4][4] <= cur;
                wcr <= r1[LOG2N-1:0] - 2'd2; wcc <= c1[LOG2N-1:0] - 2'd2; phw <= ph1;
            end
            // ---------------- T2：取格 + 落差扫描寄存器 ----------------
            s2 <= wval;
            if (wval) begin
                ph2 <= phw; sl2 <= {cgy[LOG2N-1:0], cgx[LOG2N-1:0]}; wi2 <= {cty, ctx};
                cv2 <= c_v; ch2 <= c_h; cro2 <= c_ro;
                for (i = 0; i < 8; i = i + 1) begin
                    nv2[i] <= nbw[i][32]; nh2[i] <= nbw[i][31:16]; nb2[i] <= nbw[i][33];
                end
                for (i = 0; i < 5; i = i + 1) begin
                    xv2[i] <= O[2][i][32]; xh2[i] <= O[2][i][31:16];
                    yv2[i] <= O[i][2][32]; yh2[i] <= O[i][2][31:16];
                end
                // 行向：评估（隔空洞最近的已观测格在扫描方向距离 trd 处），再并入前一格
                fr2 <= e_rv && (trd <= cfg_drop_k); hr2 <= trh; kr2 <= trd; nr2 <= sl_v;
                if (sl_v) begin trv <= 1'b1; trh <= sl_h; trd <= 5'd2; end
                else begin trv <= e_rv; trd <= trd_inc; end
                // 列向
                fcl2 <= e_cv && (e_cd <= cfg_drop_k); hcl2 <= e_ch; kcl2 <= e_cd; ncl2 <= su_v;
                ct_we <= 1'b1; ct_wa <= wcc;
                ct_wd <= su_v ? {1'b1, su_h, 5'd2} : {e_cv, e_ch, ctd_inc};
            end
            // ---------------- T3 ----------------
            s3 <= s2;
            if (s2) begin
                ph3 <= ph2; sl3 <= sl2; wi3 <= wi2; cv3 <= cv2; ch3 <= ch2; cro3 <= cro2;
                for (i = 0; i < 8; i = i + 1) ad3[i] <= nv2[i] ? stepd(ch2, nh2[i]) : 17'd0;
                for (i = 0; i < 4; i = i + 1) begin
                    dx3[i] <= sdiff(xh2[i+1], xh2[i]);
                    dy3[i] <= sdiff(yh2[i+1], yh2[i]);
                end
                okx3 <= xv2[0] & xv2[1] & xv2[2] & xv2[3] & xv2[4];
                oky3 <= yv2[0] & yv2[1] & yv2[2] & yv2[3] & yv2[4];
                nval3 <= nv2[0] + nv2[1] + nv2[2] + nv2[3] + nv2[4] + nv2[5] + nv2[6] + nv2[7];
                nleth3 <= nb2[0] | nb2[1] | nb2[2] | nb2[3] | nb2[4] | nb2[5] | nb2[6] | nb2[7];
                for (i = 0; i < 4; i = i + 1) begin
                    mv3[i] <= nv2[2*i] | nv2[2*i+1];
                    mxh3[i] <= (nv2[2*i] && nv2[2*i+1]) ? smax16(nh2[2*i], nh2[2*i+1]) : nv2[2*i] ? nh2[2*i] : nh2[2*i+1];
                    mnh3[i] <= (nv2[2*i] && nv2[2*i+1]) ? smin16(nh2[2*i], nh2[2*i+1]) : nv2[2*i] ? nh2[2*i] : nh2[2*i+1];
                end
                fr3 <= fr2; fcl3 <= fcl2; nr3 <= nr2; ncl3 <= ncl2; hr3 <= hr2; hcl3 <= hcl2; kr3 <= kr2; kcl3 <= kcl2;
            end
            // ---------------- T4 ----------------
            s4 <= s3;
            if (s3) begin
                ph4 <= ph3; sl4 <= sl3; wi4 <= wi3; cv4 <= cv3; ch4 <= ch3; cro4 <= cro3;
                for (i = 0; i < 4; i = i + 1) begin
                    st4[i] <= mx17(ad3[2*i], ad3[2*i+1]);
                    gx4[i] <= gate(dx3[i], cfg_jump);
                    gy4[i] <= gate(dy3[i], cfg_jump);
                end
                okx4 <= okx3; oky4 <= oky3; nval4 <= nval3; nleth4 <= nleth3;
                for (i = 0; i < 2; i = i + 1) begin
                    mv4[i] <= mv3[2*i] | mv3[2*i+1];
                    mxh4[i] <= (mv3[2*i] && mv3[2*i+1]) ? smax16(mxh3[2*i], mxh3[2*i+1]) : mv3[2*i] ? mxh3[2*i] : mxh3[2*i+1];
                    mnh4[i] <= (mv3[2*i] && mv3[2*i+1]) ? smin16(mnh3[2*i], mnh3[2*i+1]) : mv3[2*i] ? mnh3[2*i] : mnh3[2*i+1];
                end
                fr4 <= fr3; fcl4 <= fcl3; nr4 <= nr3; ncl4 <= ncl3; hr4 <= hr3; hcl4 <= hcl3; kr4 <= kr3; kcl4 <= kcl3;
            end
            // ---------------- T5 ----------------
            s5 <= s4;
            if (s4) begin
                ph5 <= ph4; sl5 <= sl4; wi5 <= wi4; cv5 <= cv4; ch5 <= ch4; cro5 <= cro4;
                st5a <= mx17(st4[0], st4[1]); st5b <= mx17(st4[2], st4[3]);
                sx5 <= gx4[0] + gx4[1] + gx4[2] + gx4[3];
                sy5 <= gy4[0] + gy4[1] + gy4[2] + gy4[3];
                okx5 <= okx4; oky5 <= oky4;
                // 空洞填补（本拍合并邻居高度极值，下一拍比较）：未观测、>= FILL_MIN 个已观测邻居、邻居无致命
                fb5 <= !cv4 && (nval4 >= cfg_fill_min) && !nleth4;
                fmv5 <= mv4[0] | mv4[1];
                fmx5 <= (mv4[0] && mv4[1]) ? smax16(mxh4[0], mxh4[1]) : mv4[0] ? mxh4[0] : mxh4[1];
                fmn5 <= (mv4[0] && mv4[1]) ? smin16(mnh4[0], mnh4[1]) : mv4[0] ? mnh4[0] : mnh4[1];
                fr5 <= fr4; fcl5 <= fcl4; nr5 <= nr4; ncl5 <= ncl4; hr5 <= hr4; hcl5 <= hcl4; kr5 <= kr4; kcl5 <= kcl4;
            end
            // ---------------- T6 ----------------
            s6 <= s5;
            if (s5) begin
                ph6 <= ph5; sl6 <= sl5; wi6 <= wi5; cv6 <= cv5; ch6 <= ch5; cro6 <= cro5; step6 <= mx17(st5a, st5b);
                rx6 <= okx5 ? abs19(sx5) : 19'd0;
                ry6 <= oky5 ? abs19(sy5) : 19'd0;
                // 正序时扫描的"前方"是 -x/-y，倒序时是 +x/+y
                sdr6 <= (ph5 == P_B) ? sx5 : -sx5;
                sdc6 <= (ph5 == P_B) ? sy5 : -sy5;
                okr6 <= okx5; okc6 <= oky5;
                fill6 <= fb5 && (!fmv5 || (sdiff(fmx5, fmn5) < $signed({9'd0, cfg_fill_tol})));   // 邻居高差极差 < FILL_TOL
                fr6 <= fr5; fcl6 <= fcl5; nr6 <= nr5; ncl6 <= ncl5; hr6 <= hr5; hcl6 <= hcl5; kr6 <= kr5; kcl6 <= kcl5;
            end
            // ---------------- T7 ----------------
            s7 <= s6;
            if (s6) begin
                ph7 <= ph6; sl7 <= sl6; wi7 <= wi6; cv7 <= cv6; cro7 <= cro6; step7 <= step6; fill7 <= fill6;
                rise7 <= {1'b0, mxr} + {2'b0, mnr[18:1]};
                er7 <= hexp_minus_far(okr6, sdr6, kr6, ch6, hr6, cfg_drop_slope);
                ec7 <= hexp_minus_far(okc6, sdc6, kcl6, ch6, hcl6, cfg_drop_slope);
                fr7 <= fr6; fcl7 <= fcl6; nr7 <= nr6; ncl7 <= ncl6;
            end
            // ---------------- T8 ----------------
            s8 <= s7;
            if (s7) begin
                ph8 <= ph7; sl8 <= sl7; wi8 <= wi7; cv8 <= cv7; fill8 <= fill7;
                drop8 <= cv7 && ((!nr7 && fr7 && (er7 > $signed({10'd0, cfg_step_max}))) ||
                                 (!ncl7 && fcl7 && (ec7 > $signed({10'd0, cfg_step_max}))));
                leth8 <= cv7 && ((step7 > {1'b0, cfg_step_max}) || (cro7 > cfg_rough_max) ||
                                 (rise7 > {4'd0, cfg_slope_rise}) ||
                                 (!nr7 && fr7 && (er7 > $signed({10'd0, cfg_step_max}))) ||
                                 (!ncl7 && fcl7 && (ec7 > $signed({10'd0, cfg_step_max}))));
                wsum8 <= cfg_k_slope * rise_c + cfg_k_step * step_c + cfg_k_rough * cro7;
            end
            // ---------------- T9：写代价图 / 位图 ----------------
            if (s8) begin
                case (ph8)
                P_A: begin
                    cm_we <= 1'b1; cm_waddr <= sl8; cm_wdata <= !cv8 ? C_UNK : leth8 ? C_LETHAL : cfree;
                    bm_we <= 1'b1; bm_wa <= wi8; bm_wd <= leth8;
                end
                P_B: if (drop8) begin
                    cm_we <= 1'b1; cm_waddr <= sl8; cm_wdata <= C_LETHAL;
                    bm_we <= 1'b1; bm_wa <= wi8; bm_wd <= 1'b1;
                end
                P_C: if (fill8) begin
                    cm_we <= 1'b1; cm_waddr <= sl8; cm_wdata <= cfg_fill_cost;
                end
                default: ;
                endcase
            end

            // ---------------- 第 D 遍：膨胀 ----------------
            p2v1 <= (ph == P_D);
            if (ph == P_D) begin
                p2in1 <= (r < N) && (c < N);
                p2first_c1 <= (c == 0);
                p2first_r1 <= (r == 0);
                p2x1  <= c - R;
                p2x1_valid <= (c >= R);
                p2out1 <= (r >= R) && (c >= R);
                p2sl1 <= {dgy[LOG2N-1:0] - R, dgx[LOG2N-1:0] - R};
            end
            p2va <= p2v1;
            if (p2v1) begin
                hc_cnt <= hc_new;
                hsr <= hsr_new; dha <= dh1;
                hda <= hd; p2first_ra <= p2first_r1; p2outa <= p2out1; p2x_valida <= p2x1_valid;
                p2xa <= p2x1; p2sla <= p2sl1;
            end
            p2vb <= p2va;
            if (p2va) begin
                hdb <= hda; p2first_rb <= p2first_ra; p2outb <= p2outa; p2x_validb <= p2x_valida;
                p2xb <= p2xa; p2slb <= p2sla; vc_old <= vc_rd;
                dhb <= dha; dh_old <= dhist_rd;
            end
            p2v2 <= 1'b0;
            if (p2vb) begin
                if (p2x_validb) begin
                    vc[p2xb[LOG2N-1:0]] <= vc_new;
                    dhist[p2xb[LOG2N-1:0]] <= {dh_hist[20:0], dhb};
                end
                if (p2outb) begin
                    p2v2 <= 1'b1; p2sl2 <= p2slb; infl2 <= disk ? dinfl : infl1;
                end
            end
            if (p2v2 && infl2 && cm_q < C_INS) begin
                cm_we <= 1'b1; cm_waddr <= p2sl2; cm_wdata <= C_INS;
            end
        end
    end
endmodule
