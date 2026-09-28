// SLF 似然场建图遍：按窗口坐标扫描结构图，为每格计算"结构格 / 法向 / 门控"并写回同一槽字，
// 与 model/slf_match.py 的 StructMap.build_lf + cell_normals 逐位一致。
//
// 槽字（23 位）：{tag[22:19], hit[18:16], cx[15:12], cy[11:8], occ[7], q[6:2], has[1], gate[0]}
//   occ  = 标签符合当前窗口 且 hit >= HIT_MIN
//   q    = 法向扇区（32 个方向），has = 线状结构（5×5 邻域质心协方差的主方向明显，且 >= 3 个结构格）
//   gate = 3×3 邻域或正交方向 2 格处有结构格（= 八边形距离 2·切比雪夫+曼哈顿 <= 6）
// 5×5 窗口的矩用"列和"分解：每拍只对新进入窗口的一列（5 格）求 Σw、Σw·cx、Σw·Y、Σw·cx²、Σw·cx·Y、Σw·Y²，
// 再对 5 列按 dx 权重（16dx、256dx²）组合；A=nΣXX−(ΣX)² 等平移不变，直接用相对中心的坐标。
module slf_lf #(
    parameter integer LOG2N = 8,
    parameter integer HIT_MIN = 2
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire signed [31:0] cx,           // 窗口中心（世界 5 cm 格）
    input  wire signed [31:0] cy,
    output reg         busy,
    output reg         done,
    output wire        sm_re,
    output wire [2*LOG2N-1:0] sm_raddr,
    input  wire [22:0] sm_q,
    output reg         sm_we,
    output reg  [2*LOG2N-1:0] sm_waddr,
    output reg  [22:0] sm_wdata
);
    localparam integer N  = 1 << LOG2N;
    localparam integer AW = 2 * LOG2N;
    localparam integer LB = N + 2;
    localparam integer CW = 1 + 4 + 4 + 15;          // 窗口格 {occ, cx, cy, 原字高 15 位}

    reg              run;
    reg [LOG2N+1:0]  r, c;
    reg signed [31:0] cx_l, cy_l;
    reg [4:0]        flush;
    wire inr0 = (r < N) && (c < N);
    wire signed [31:0] gx0 = cx_l - N/2 + $signed({1'b0, c});
    wire signed [31:0] gy0 = cy_l - N/2 + $signed({1'b0, r});
    assign sm_re    = run && inr0;
    assign sm_raddr = {gy0[LOG2N-1:0], gx0[LOG2N-1:0]};

    // ---------------------------------------------------------------- T1：解码、行缓存、窗口
    reg        v1, inr1;
    reg [LOG2N+1:0] r1, c1;
    reg [3:0]  etag1;
    wire [3:0] q_tag = sm_q[22:19];
    wire [2:0] q_hit = sm_q[18:16];
    wire       q_occ = inr1 && (q_tag == etag1) && (q_hit >= HIT_MIN);
    wire [CW-1:0] cur = {q_occ, sm_q[15:12], sm_q[11:8], sm_q[22:8]};
    reg [CW-1:0] lb1 [0:LB-1];
    reg [CW-1:0] lb2 [0:LB-1];
    reg [CW-1:0] lb3 [0:LB-1];
    reg [CW-1:0] lb4 [0:LB-1];
    wire [CW-1:0] l1 = lb1[c1], l2 = lb2[c1], l3 = lb3[c1], l4 = lb4[c1];
    wire [CW-1:0] ncol [0:4];
    assign ncol[0] = (r1 >= 4) ? l4 : {CW{1'b0}};
    assign ncol[1] = (r1 >= 3) ? l3 : {CW{1'b0}};
    assign ncol[2] = (r1 >= 2) ? l2 : {CW{1'b0}};
    assign ncol[3] = (r1 >= 1) ? l1 : {CW{1'b0}};
    assign ncol[4] = cur;
    // 窗口占用位（5×5）与中心原字
    reg        wo [0:4][0:4];
    reg [14:0] o2_2, o2_3, o2_4;          // 中心行的原字 {tag, hit, cx, cy}，列 2..4（列 2 为中心）
    // 列矩移位寄存器（列 0 最早，dx = 列号 - 2）
    reg [2:0]  m0 [0:4];
    reg signed [9:0]  mX [0:4];           // Σw·cx（0..75）
    reg signed [10:0] mY [0:4];           // Σw·Y，Y = 16dy + cy（-32..47）
    reg [13:0] mXX [0:4];                 // Σw·cx²（<= 1125）
    reg signed [13:0] mXY [0:4];
    reg [14:0] mYY [0:4];
    // 新列的矩（组合）
    integer i, j;
    reg [2:0]  n0_w;
    reg signed [9:0]  nX_w;
    reg signed [10:0] nY_w;
    reg [13:0] nXX_w;
    reg signed [13:0] nXY_w;
    reg [14:0] nYY_w;
    reg signed [6:0] yy;
    reg signed [14:0] yy2;
    reg [3:0] cxv;
    always @(*) begin
        n0_w = 0; nX_w = 0; nY_w = 0; nXX_w = 0; nXY_w = 0; nYY_w = 0;
        for (i = 0; i < 5; i = i + 1) begin
            cxv = ncol[i][22:19];
            yy  = $signed(7'd16) * (i - 2) + $signed({3'b0, ncol[i][18:15]});
            if (ncol[i][23]) begin
                n0_w  = n0_w + 1'b1;
                nX_w  = nX_w + $signed({6'd0, cxv});
                nY_w  = nY_w + yy;
                nXX_w = nXX_w + cxv * cxv;
                nXY_w = nXY_w + $signed({1'b0, cxv}) * yy;
                yy2   = yy * yy;
                nYY_w = nYY_w + yy2[14:0];
            end
        end
    end
    reg        wval;
    reg [LOG2N-1:0] wr_, wc_;

    // ---------------------------------------------------------------- T2：窗口矩
    reg        s2; reg [AW-1:0] sl2; reg [14:0] ow2; reg occ2, gate2;
    reg [4:0]  n2;
    reg signed [12:0] sx2, sy2;
    reg [16:0] sxx2, syy2;
    reg signed [16:0] sxy2;
    reg [4:0]  n_w;
    reg signed [12:0] sx_w, sy_w;
    reg signed [18:0] sxx_w, syy_w;
    reg signed [16:0] sxy_w;
    reg signed [4:0] dxj;
    always @(*) begin
        n_w = 0; sx_w = 0; sy_w = 0; sxx_w = 0; sxy_w = 0; syy_w = 0;
        for (j = 0; j < 5; j = j + 1) begin
            dxj = j - 2;
            n_w   = n_w + m0[j];
            sx_w  = sx_w + (dxj <<< 4) * $signed({1'b0, m0[j]}) + mX[j];
            sy_w  = sy_w + mY[j];
            sxx_w = sxx_w + ((dxj * dxj) <<< 8) * $signed({1'b0, m0[j]}) + (dxj <<< 5) * mX[j] + $signed({1'b0, mXX[j]});
            sxy_w = sxy_w + (dxj <<< 4) * mY[j] + mXY[j];
            syy_w = syy_w + $signed({1'b0, mYY[j]});
        end
    end
    wire signed [31:0] cgx = cx_l - N/2 + $signed({1'b0, wc_});
    wire signed [31:0] cgy = cy_l - N/2 + $signed({1'b0, wr_});
    wire gate_w = wo[1][1] | wo[1][2] | wo[1][3] | wo[2][1] | wo[2][2] | wo[2][3] | wo[3][1] | wo[3][2] | wo[3][3] |
                  wo[2][0] | wo[2][4] | wo[0][2] | wo[4][2];
    // ---------------------------------------------------------------- T3：A B C
    reg        s3; reg [AW-1:0] sl3; reg [14:0] ow3; reg occ3, gate3, n3ok3;
    reg signed [23:0] a3, b3, c3;
    // ---------------------------------------------------------------- T4：X Y
    reg        s4; reg [AW-1:0] sl4; reg [14:0] ow4; reg occ4, gate4, n3ok4;
    reg signed [24:0] x4, y4;
    reg signed [24:0] apc4;
    // ---------------------------------------------------------------- T5：线状判定、归一化
    reg        s5; reg [AW-1:0] sl5; reg [14:0] ow5; reg occ5, gate5, has5;
    reg signed [24:0] x5, y5;
    reg [4:0]  shv5;
    wire signed [49:0] sq_x = x4 * x4;
    wire signed [49:0] sq_y = y4 * y4;
    wire signed [49:0] sq_s = apc4 * apc4;
    wire aniso4 = ((sq_x + sq_y) * 25) > (sq_s * 9);
    function [4:0] bitlen25(input [24:0] v);
        integer k;
        begin
            bitlen25 = 0;
            for (k = 0; k < 25; k = k + 1) if (v[k]) bitlen25 = k + 1;
        end
    endfunction
    wire [24:0] ax4 = x4[24] ? -x4 : x4;
    wire [24:0] ay4 = y4[24] ? -y4 : y4;
    wire [24:0] am4 = (ax4 > ay4) ? ax4 : ay4;
    wire [4:0]  bl4 = bitlen25(am4);
    // CORDIC：10 次迭代，每拍 1 次
    localparam integer NIT = 10;
    reg        cs [0:NIT];
    reg [AW-1:0] csl [0:NIT];
    reg [14:0] cow [0:NIT];
    reg        cocc [0:NIT], cgate [0:NIT], chas [0:NIT];
    reg signed [18:0] cxr [0:NIT], cyr [0:NIT];
    reg [15:0] cang [0:NIT];
    function [15:0] atan_t(input integer k);
        case (k)
            0: atan_t = 16'd8192; 1: atan_t = 16'd4836; 2: atan_t = 16'd2555; 3: atan_t = 16'd1297;
            4: atan_t = 16'd651;  5: atan_t = 16'd326;  6: atan_t = 16'd163;  7: atan_t = 16'd81;
            8: atan_t = 16'd41;   default: atan_t = 16'd20;
        endcase
    endfunction
    wire [15:0] ang_f = cang[NIT] + 16'd32768 + 16'd1024;
    wire [4:0]  q_f = ang_f[15:11];

    always @(posedge clk) begin
        sm_we <= 1'b0; done <= 1'b0;
        if (rst) begin
            run <= 1'b0; busy <= 1'b0; v1 <= 1'b0; wval <= 1'b0; s2 <= 1'b0; s3 <= 1'b0; s4 <= 1'b0; s5 <= 1'b0;
            flush <= 0;
            for (i = 0; i <= NIT; i = i + 1) cs[i] <= 1'b0;
        end else begin
            // ---------------- 扫描 ----------------
            if (start && !busy) begin
                busy <= 1'b1; run <= 1'b1; r <= 0; c <= 0; cx_l <= cx; cy_l <= cy;
            end else if (run) begin
                if (c == N + 1) begin
                    c <= 0;
                    if (r == N + 1) begin run <= 1'b0; flush <= 5'd24; end
                    else r <= r + 1'b1;
                end else c <= c + 1'b1;
            end else if (busy) begin
                if (flush == 0) begin busy <= 1'b0; done <= 1'b1; end
                else flush <= flush - 1'b1;
            end
            // ---------------- T0 -> T1 ----------------
            v1 <= run;
            if (run) begin
                inr1 <= inr0; r1 <= r; c1 <= c;
                etag1 <= {gy0[LOG2N+1:LOG2N], gx0[LOG2N+1:LOG2N]};
            end
            // ---------------- T1 ----------------
            wval <= v1 && (r1 >= 2) && (c1 >= 2);
            if (v1) begin
                lb1[c1] <= cur; lb2[c1] <= l1; lb3[c1] <= l2; lb4[c1] <= l3;
                for (i = 0; i < 5; i = i + 1) begin
                    for (j = 0; j < 4; j = j + 1) wo[i][j] <= wo[i][j+1];
                    wo[i][4] <= ncol[i][23];
                end
                for (j = 0; j < 4; j = j + 1) begin
                    m0[j] <= m0[j+1]; mX[j] <= mX[j+1]; mY[j] <= mY[j+1];
                    mXX[j] <= mXX[j+1]; mXY[j] <= mXY[j+1]; mYY[j] <= mYY[j+1];
                end
                m0[4] <= n0_w; mX[4] <= nX_w; mY[4] <= nY_w; mXX[4] <= nXX_w; mXY[4] <= nXY_w; mYY[4] <= nYY_w;
                o2_4 <= ncol[2][14:0]; o2_3 <= o2_4; o2_2 <= o2_3;
                wr_ <= r1[LOG2N-1:0] - 2'd2; wc_ <= c1[LOG2N-1:0] - 2'd2;
            end
            // ---------------- T2 ----------------
            s2 <= wval;
            if (wval) begin
                sl2 <= {cgy[LOG2N-1:0], cgx[LOG2N-1:0]}; ow2 <= o2_2; occ2 <= wo[2][2]; gate2 <= gate_w;
                n2 <= n_w; sx2 <= sx_w; sy2 <= sy_w; sxx2 <= sxx_w[16:0]; sxy2 <= sxy_w; syy2 <= syy_w[16:0];
            end
            // ---------------- T3 ----------------
            s3 <= s2;
            if (s2) begin
                sl3 <= sl2; ow3 <= ow2; occ3 <= occ2; gate3 <= gate2; n3ok3 <= (n2 >= 3);
                a3 <= $signed({1'b0, n2}) * $signed({1'b0, sxx2}) - sx2 * sx2;
                b3 <= $signed({1'b0, n2}) * sxy2 - sx2 * sy2;
                c3 <= $signed({1'b0, n2}) * $signed({1'b0, syy2}) - sy2 * sy2;
            end
            // ---------------- T4 ----------------
            s4 <= s3;
            if (s3) begin
                sl4 <= sl3; ow4 <= ow3; occ4 <= occ3; gate4 <= gate3; n3ok4 <= n3ok3;
                x4 <= a3 - c3; y4 <= b3 <<< 1; apc4 <= a3 + c3;
            end
            // ---------------- T5 ----------------
            s5 <= s4;
            if (s4) begin
                sl5 <= sl4; ow5 <= ow4; occ5 <= occ4; gate5 <= gate4;
                has5 <= occ4 && n3ok4 && aniso4;
                x5 <= x4; y5 <= y4;
                shv5 <= (bl4 > 5'd17) ? (bl4 - 5'd17) : 5'd0;
            end
            // ---------------- CORDIC 输入：转到右半平面 ----------------
            cs[0] <= s5;
            if (s5) begin
                csl[0] <= sl5; cow[0] <= ow5; cocc[0] <= occ5; cgate[0] <= gate5; chas[0] <= has5;
                if (x5 < 0) begin
                    cxr[0] <= -(x5 >>> shv5); cyr[0] <= -(y5 >>> shv5); cang[0] <= 16'd32768;
                end else begin
                    cxr[0] <= x5 >>> shv5;    cyr[0] <= y5 >>> shv5;    cang[0] <= 16'd0;
                end
            end
            for (i = 0; i < NIT; i = i + 1) begin
                cs[i+1] <= cs[i];
                if (cs[i]) begin
                    csl[i+1] <= csl[i]; cow[i+1] <= cow[i]; cocc[i+1] <= cocc[i]; cgate[i+1] <= cgate[i];
                    chas[i+1] <= chas[i];
                    if (cyr[i] > 0) begin
                        cxr[i+1] <= cxr[i] + (cyr[i] >>> i); cyr[i+1] <= cyr[i] - (cxr[i] >>> i);
                        cang[i+1] <= cang[i] + atan_t(i);
                    end else begin
                        cxr[i+1] <= cxr[i] - (cyr[i] >>> i); cyr[i+1] <= cyr[i] + (cxr[i] >>> i);
                        cang[i+1] <= cang[i] - atan_t(i);
                    end
                end
            end
            // ---------------- 写回 ----------------
            if (cs[NIT]) begin
                sm_we <= 1'b1; sm_waddr <= csl[NIT];
                sm_wdata <= {cow[NIT], cocc[NIT], q_f, chas[NIT], cgate[NIT]};
            end
        end
    end
endmodule
