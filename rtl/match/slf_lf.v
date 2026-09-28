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
    reg [4:0]  wo [0:4];                  // wo[行][列]
    reg [14:0] o2_2, o2_3, o2_4;          // 中心行的原字 {tag, hit, cx, cy}，列 2..4（列 2 为中心）
    // 列矩移位寄存器（列 0 最早，dx = 列号 - 2）
    reg [2:0]  m0 [0:4];
    reg signed [9:0]  mX [0:4];           // Σw·cx（0..75）
    reg signed [10:0] mY [0:4];           // Σw·Y，Y = 16dy + cy（-32..47）
    reg [13:0] mXX [0:4];                 // Σw·cx²（<= 1125）
    reg signed [13:0] mXY [0:4];
    reg [14:0] mYY [0:4];
    reg [CW-1:0] ncr [0:4];               // 寄存后的新列（切断 行缓存读 -> 列矩 的组合路径）
    // 新列的矩（组合，逐行展开）
    wire [3:0] ncx0, ncx1, ncx2, ncx3, ncx4;
    wire       nw0 = ncr[0][23];
    wire [3:0] ncxv0 = ncr[0][22:19];
    wire signed [6:0] nyv0 = $signed(7'd96) + $signed({3'd0, ncr[0][18:15]});
    wire signed [14:0] nyy0 = nyv0 * nyv0;
    wire [7:0]  nxx0 = ncxv0 * ncxv0;
    wire signed [12:0] nxy0 = $signed({1'b0, ncxv0}) * nyv0;
    wire       nw1 = ncr[1][23];
    wire [3:0] ncxv1 = ncr[1][22:19];
    wire signed [6:0] nyv1 = $signed(7'd112) + $signed({3'd0, ncr[1][18:15]});
    wire signed [14:0] nyy1 = nyv1 * nyv1;
    wire [7:0]  nxx1 = ncxv1 * ncxv1;
    wire signed [12:0] nxy1 = $signed({1'b0, ncxv1}) * nyv1;
    wire       nw2 = ncr[2][23];
    wire [3:0] ncxv2 = ncr[2][22:19];
    wire signed [6:0] nyv2 = $signed(7'd0) + $signed({3'd0, ncr[2][18:15]});
    wire signed [14:0] nyy2 = nyv2 * nyv2;
    wire [7:0]  nxx2 = ncxv2 * ncxv2;
    wire signed [12:0] nxy2 = $signed({1'b0, ncxv2}) * nyv2;
    wire       nw3 = ncr[3][23];
    wire [3:0] ncxv3 = ncr[3][22:19];
    wire signed [6:0] nyv3 = $signed(7'd16) + $signed({3'd0, ncr[3][18:15]});
    wire signed [14:0] nyy3 = nyv3 * nyv3;
    wire [7:0]  nxx3 = ncxv3 * ncxv3;
    wire signed [12:0] nxy3 = $signed({1'b0, ncxv3}) * nyv3;
    wire       nw4 = ncr[4][23];
    wire [3:0] ncxv4 = ncr[4][22:19];
    wire signed [6:0] nyv4 = $signed(7'd32) + $signed({3'd0, ncr[4][18:15]});
    wire signed [14:0] nyy4 = nyv4 * nyv4;
    wire [7:0]  nxx4 = ncxv4 * ncxv4;
    wire signed [12:0] nxy4 = $signed({1'b0, ncxv4}) * nyv4;
    wire [2:0]  n0_w  = {2'd0, nw0} + {2'd0, nw1} + {2'd0, nw2} + {2'd0, nw3} + {2'd0, nw4};
    wire signed [9:0]  nX_w  = (nw0 ? $signed({6'd0, ncxv0}) : 10'sd0) + (nw1 ? $signed({6'd0, ncxv1}) : 10'sd0) + (nw2 ? $signed({6'd0, ncxv2}) : 10'sd0) + (nw3 ? $signed({6'd0, ncxv3}) : 10'sd0) + (nw4 ? $signed({6'd0, ncxv4}) : 10'sd0);
    wire signed [10:0] nY_w  = (nw0 ? {{4{nyv0[6]}}, nyv0} : 11'sd0) + (nw1 ? {{4{nyv1[6]}}, nyv1} : 11'sd0) + (nw2 ? {{4{nyv2[6]}}, nyv2} : 11'sd0) + (nw3 ? {{4{nyv3[6]}}, nyv3} : 11'sd0) + (nw4 ? {{4{nyv4[6]}}, nyv4} : 11'sd0);
    wire [13:0] nXX_w = (nw0 ? {6'd0, nxx0} : 14'd0) + (nw1 ? {6'd0, nxx1} : 14'd0) + (nw2 ? {6'd0, nxx2} : 14'd0) + (nw3 ? {6'd0, nxx3} : 14'd0) + (nw4 ? {6'd0, nxx4} : 14'd0);
    wire signed [13:0] nXY_w = (nw0 ? {nxy0[12], nxy0} : 14'sd0) + (nw1 ? {nxy1[12], nxy1} : 14'sd0) + (nw2 ? {nxy2[12], nxy2} : 14'sd0) + (nw3 ? {nxy3[12], nxy3} : 14'sd0) + (nw4 ? {nxy4[12], nxy4} : 14'sd0);
    wire [14:0] nYY_w = (nw0 ? nyy0[14:0] : 15'd0) + (nw1 ? nyy1[14:0] : 15'd0) + (nw2 ? nyy2[14:0] : 15'd0) + (nw3 ? nyy3[14:0] : 15'd0) + (nw4 ? nyy4[14:0] : 15'd0);
    integer i, j;
    reg        vb;
    reg [LOG2N+1:0] r1b, c1b;
    reg        wval;
    reg [LOG2N-1:0] wrow, wcol;

    // ---------------------------------------------------------------- T2：窗口矩
    reg        s2; reg [AW-1:0] sl2; reg [14:0] ow2; reg occ2, gate2;
    reg [4:0]  n2;
    reg signed [12:0] sx2, sy2;
    reg [16:0] sxx2, syy2;
    reg signed [16:0] sxy2;
    // 窗口矩分两拍：T2a 求列 0..1 与列 2..4 的部分和，T2 相加
    wire [4:0]  n_pa = {2'd0, m0[0]} + {2'd0, m0[1]};
    wire [4:0]  n_pb = {2'd0, m0[2]} + {2'd0, m0[3]} + {2'd0, m0[4]};
    reg  [4:0]  n_ra, n_rb;
    wire signed [12:0] sx_pa = $signed(-32) * $signed({1'b0, m0[0]}) + mX[0] + $signed(-16) * $signed({1'b0, m0[1]}) + mX[1];
    wire signed [12:0] sx_pb = mX[2] + $signed(16) * $signed({1'b0, m0[3]}) + mX[3] + $signed(32) * $signed({1'b0, m0[4]}) + mX[4];
    reg  signed [12:0] sx_ra, sx_rb;
    wire signed [12:0] sy_pa = mY[0] + mY[1];
    wire signed [12:0] sy_pb = mY[2] + mY[3] + mY[4];
    reg  signed [12:0] sy_ra, sy_rb;
    wire signed [18:0] sxx_pa = $signed(1024) * $signed({1'b0, m0[0]}) + $signed(-64) * mX[0] + $signed({1'b0, mXX[0]}) + $signed(256) * $signed({1'b0, m0[1]}) + $signed(-32) * mX[1] + $signed({1'b0, mXX[1]});
    wire signed [18:0] sxx_pb = $signed({1'b0, mXX[2]}) + $signed(256) * $signed({1'b0, m0[3]}) + $signed(32) * mX[3] + $signed({1'b0, mXX[3]}) + $signed(1024) * $signed({1'b0, m0[4]}) + $signed(64) * mX[4] + $signed({1'b0, mXX[4]});
    reg  signed [18:0] sxx_ra, sxx_rb;
    wire signed [16:0] sxy_pa = $signed(-32) * mY[0] + mXY[0] + $signed(-16) * mY[1] + mXY[1];
    wire signed [16:0] sxy_pb = mXY[2] + $signed(16) * mY[3] + mXY[3] + $signed(32) * mY[4] + mXY[4];
    reg  signed [16:0] sxy_ra, sxy_rb;
    wire signed [18:0] syy_pa = $signed({1'b0, mYY[0]}) + $signed({1'b0, mYY[1]});
    wire signed [18:0] syy_pb = $signed({1'b0, mYY[2]}) + $signed({1'b0, mYY[3]}) + $signed({1'b0, mYY[4]});
    reg  signed [18:0] syy_ra, syy_rb;
    reg        s2a; reg [AW-1:0] sl2a; reg [14:0] ow2a; reg occ2a, gate2a;
    wire [4:0]  n_w = n_ra + n_rb;
    wire signed [12:0] sx_w = sx_ra + sx_rb;
    wire signed [12:0] sy_w = sy_ra + sy_rb;
    wire signed [18:0] sxx_w = sxx_ra + sxx_rb;
    wire signed [16:0] sxy_w = sxy_ra + sxy_rb;
    wire signed [18:0] syy_w = syy_ra + syy_rb;
    wire signed [31:0] cgx = cx_l - N/2 + $signed({1'b0, wcol});
    wire signed [31:0] cgy = cy_l - N/2 + $signed({1'b0, wrow});
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
    // T5a：平方项与绝对值最大者寄存；T5：线状判定与归一化移位
    reg        s5a; reg [AW-1:0] sl5a; reg [14:0] ow5a; reg occ5a, gate5a, n3ok5a;
    reg signed [24:0] x5a, y5a;
    reg signed [49:0] sqd5a, sqs5a;
    reg [24:0] am5a;
    wire aniso5a = (sqd5a * 25) > (sqs5a * 9);
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
    wire [4:0]  bl5a = bitlen25(am5a);
    // CORDIC：10 次迭代，每拍 1 次（逐级展开为独立寄存器）
    localparam integer NIT = 10;
    reg        cs0; reg [AW-1:0] csl0; reg [14:0] cow0; reg cocc0, cgate0, chas0;
    reg signed [18:0] cxr0, cyr0; reg [15:0] cang0;
    reg        cs1; reg [AW-1:0] csl1; reg [14:0] cow1; reg cocc1, cgate1, chas1;
    reg signed [18:0] cxr1, cyr1; reg [15:0] cang1;
    reg        cs2; reg [AW-1:0] csl2; reg [14:0] cow2; reg cocc2, cgate2, chas2;
    reg signed [18:0] cxr2, cyr2; reg [15:0] cang2;
    reg        cs3; reg [AW-1:0] csl3; reg [14:0] cow3; reg cocc3, cgate3, chas3;
    reg signed [18:0] cxr3, cyr3; reg [15:0] cang3;
    reg        cs4; reg [AW-1:0] csl4; reg [14:0] cow4; reg cocc4, cgate4, chas4;
    reg signed [18:0] cxr4, cyr4; reg [15:0] cang4;
    reg        cs5; reg [AW-1:0] csl5; reg [14:0] cow5; reg cocc5, cgate5, chas5;
    reg signed [18:0] cxr5, cyr5; reg [15:0] cang5;
    reg        cs6; reg [AW-1:0] csl6; reg [14:0] cow6; reg cocc6, cgate6, chas6;
    reg signed [18:0] cxr6, cyr6; reg [15:0] cang6;
    reg        cs7; reg [AW-1:0] csl7; reg [14:0] cow7; reg cocc7, cgate7, chas7;
    reg signed [18:0] cxr7, cyr7; reg [15:0] cang7;
    reg        cs8; reg [AW-1:0] csl8; reg [14:0] cow8; reg cocc8, cgate8, chas8;
    reg signed [18:0] cxr8, cyr8; reg [15:0] cang8;
    reg        cs9; reg [AW-1:0] csl9; reg [14:0] cow9; reg cocc9, cgate9, chas9;
    reg signed [18:0] cxr9, cyr9; reg [15:0] cang9;
    reg        cs10; reg [AW-1:0] csl10; reg [14:0] cow10; reg cocc10, cgate10, chas10;
    reg signed [18:0] cxr10, cyr10; reg [15:0] cang10;
    // 归一化右移（0..8 位，逐档选择）
    function signed [18:0] shr(input signed [24:0] v, input [4:0] n);
        case (n)
            5'd0: shr = v[18:0];
            5'd1: shr = v[19:1];
            5'd2: shr = v[20:2];
            5'd3: shr = v[21:3];
            5'd4: shr = v[22:4];
            5'd5: shr = v[23:5];
            5'd6: shr = v[24:6];
            5'd7: shr = {v[24], v[24:7]};
            default: shr = {{2{v[24]}}, v[24:8]};
        endcase
    endfunction
    wire signed [18:0] xs5 = shr(x5, shv5), ys5 = shr(y5, shv5);
    wire [15:0] ang_f = cang10 + 16'd32768 + 16'd1024;
    wire [4:0]  q_f = ang_f[15:11];

    always @(posedge clk) begin
        sm_we <= 1'b0; done <= 1'b0;
        if (rst) begin
            run <= 1'b0; busy <= 1'b0; v1 <= 1'b0; vb <= 1'b0; wval <= 1'b0; s2a <= 1'b0; s5a <= 1'b0; s2 <= 1'b0; s3 <= 1'b0; s4 <= 1'b0; s5 <= 1'b0;
            flush <= 0;
            cs0 <= 1'b0; cs1 <= 1'b0; cs2 <= 1'b0; cs3 <= 1'b0; cs4 <= 1'b0; cs5 <= 1'b0; cs6 <= 1'b0; cs7 <= 1'b0; cs8 <= 1'b0; cs9 <= 1'b0; cs10 <= 1'b0;
        end else begin
            // ---------------- 扫描 ----------------
            if (start && !busy) begin
                busy <= 1'b1; run <= 1'b1; r <= 0; c <= 0; cx_l <= cx; cy_l <= cy;
            end else if (run) begin
                if (c == N + 1) begin
                    c <= 0;
                    if (r == N + 1) begin run <= 1'b0; flush <= 5'd28; end
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
            // ---------------- T1：行缓存、寄存新列 ----------------
            vb <= v1;
            if (v1) begin
                lb1[c1] <= cur; lb2[c1] <= l1; lb3[c1] <= l2; lb4[c1] <= l3;
                for (i = 0; i < 5; i = i + 1) ncr[i] <= ncol[i];
                r1b <= r1; c1b <= c1;
            end
            // ---------------- T1b：窗口移位、列矩 ----------------
            wval <= vb && (r1b >= 2) && (c1b >= 2);
            if (vb) begin
                for (i = 0; i < 5; i = i + 1) begin
                    for (j = 0; j < 4; j = j + 1) wo[i][j] <= wo[i][j+1];
                    wo[i][4] <= ncr[i][23];
                end
                for (j = 0; j < 4; j = j + 1) begin
                    m0[j] <= m0[j+1]; mX[j] <= mX[j+1]; mY[j] <= mY[j+1];
                    mXX[j] <= mXX[j+1]; mXY[j] <= mXY[j+1]; mYY[j] <= mYY[j+1];
                end
                m0[4] <= n0_w; mX[4] <= nX_w; mY[4] <= nY_w; mXX[4] <= nXX_w; mXY[4] <= nXY_w; mYY[4] <= nYY_w;
                o2_4 <= ncr[2][14:0]; o2_3 <= o2_4; o2_2 <= o2_3;
                wrow <= r1b[LOG2N-1:0] - 2'd2; wcol <= c1b[LOG2N-1:0] - 2'd2;
            end
            // ---------------- T2a：部分和 ----------------
            s2a <= wval;
            if (wval) begin
                sl2a <= {cgy[LOG2N-1:0], cgx[LOG2N-1:0]}; ow2a <= o2_2; occ2a <= wo[2][2]; gate2a <= gate_w;
                n_ra <= n_pa; n_rb <= n_pb; sx_ra <= sx_pa; sx_rb <= sx_pb; sy_ra <= sy_pa; sy_rb <= sy_pb;
                sxx_ra <= sxx_pa; sxx_rb <= sxx_pb; sxy_ra <= sxy_pa; sxy_rb <= sxy_pb; syy_ra <= syy_pa; syy_rb <= syy_pb;
            end
            // ---------------- T2 ----------------
            s2 <= s2a;
            if (s2a) begin
                sl2 <= sl2a; ow2 <= ow2a; occ2 <= occ2a; gate2 <= gate2a;
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
            // ---------------- T5a ----------------
            s5a <= s4;
            if (s4) begin
                sl5a <= sl4; ow5a <= ow4; occ5a <= occ4; gate5a <= gate4; n3ok5a <= n3ok4;
                x5a <= x4; y5a <= y4;
                sqd5a <= x4 * x4 + y4 * y4;          // (A−C)² + (2B)²
                sqs5a <= apc4 * apc4;                // (A+C)²
                am5a <= am4;
            end
            // ---------------- T5 ----------------
            s5 <= s5a;
            if (s5a) begin
                sl5 <= sl5a; ow5 <= ow5a; occ5 <= occ5a; gate5 <= gate5a;
                has5 <= occ5a && n3ok5a && aniso5a;
                x5 <= x5a; y5 <= y5a;
                shv5 <= (bl5a > 5'd17) ? (bl5a - 5'd17) : 5'd0;
            end
            // ---------------- CORDIC 输入：转到右半平面 ----------------
            cs0 <= s5;
            if (s5) begin
                csl0 <= sl5; cow0 <= ow5; cocc0 <= occ5; cgate0 <= gate5; chas0 <= has5;
                if (x5 < 0) begin
                    cxr0 <= -xs5; cyr0 <= -ys5; cang0 <= 16'd32768;
                end else begin
                    cxr0 <= xs5;  cyr0 <= ys5;  cang0 <= 16'd0;
                end
            end
            cs1 <= cs0;
            if (cs0) begin
                csl1 <= csl0; cow1 <= cow0; cocc1 <= cocc0; cgate1 <= cgate0; chas1 <= chas0;
                if (cyr0 > 0) begin
                    cxr1 <= cxr0 + (cyr0 >>> 0); cyr1 <= cyr0 - (cxr0 >>> 0); cang1 <= cang0 + 16'd8192;
                end else begin
                    cxr1 <= cxr0 - (cyr0 >>> 0); cyr1 <= cyr0 + (cxr0 >>> 0); cang1 <= cang0 - 16'd8192;
                end
            end
            cs2 <= cs1;
            if (cs1) begin
                csl2 <= csl1; cow2 <= cow1; cocc2 <= cocc1; cgate2 <= cgate1; chas2 <= chas1;
                if (cyr1 > 0) begin
                    cxr2 <= cxr1 + (cyr1 >>> 1); cyr2 <= cyr1 - (cxr1 >>> 1); cang2 <= cang1 + 16'd4836;
                end else begin
                    cxr2 <= cxr1 - (cyr1 >>> 1); cyr2 <= cyr1 + (cxr1 >>> 1); cang2 <= cang1 - 16'd4836;
                end
            end
            cs3 <= cs2;
            if (cs2) begin
                csl3 <= csl2; cow3 <= cow2; cocc3 <= cocc2; cgate3 <= cgate2; chas3 <= chas2;
                if (cyr2 > 0) begin
                    cxr3 <= cxr2 + (cyr2 >>> 2); cyr3 <= cyr2 - (cxr2 >>> 2); cang3 <= cang2 + 16'd2555;
                end else begin
                    cxr3 <= cxr2 - (cyr2 >>> 2); cyr3 <= cyr2 + (cxr2 >>> 2); cang3 <= cang2 - 16'd2555;
                end
            end
            cs4 <= cs3;
            if (cs3) begin
                csl4 <= csl3; cow4 <= cow3; cocc4 <= cocc3; cgate4 <= cgate3; chas4 <= chas3;
                if (cyr3 > 0) begin
                    cxr4 <= cxr3 + (cyr3 >>> 3); cyr4 <= cyr3 - (cxr3 >>> 3); cang4 <= cang3 + 16'd1297;
                end else begin
                    cxr4 <= cxr3 - (cyr3 >>> 3); cyr4 <= cyr3 + (cxr3 >>> 3); cang4 <= cang3 - 16'd1297;
                end
            end
            cs5 <= cs4;
            if (cs4) begin
                csl5 <= csl4; cow5 <= cow4; cocc5 <= cocc4; cgate5 <= cgate4; chas5 <= chas4;
                if (cyr4 > 0) begin
                    cxr5 <= cxr4 + (cyr4 >>> 4); cyr5 <= cyr4 - (cxr4 >>> 4); cang5 <= cang4 + 16'd651;
                end else begin
                    cxr5 <= cxr4 - (cyr4 >>> 4); cyr5 <= cyr4 + (cxr4 >>> 4); cang5 <= cang4 - 16'd651;
                end
            end
            cs6 <= cs5;
            if (cs5) begin
                csl6 <= csl5; cow6 <= cow5; cocc6 <= cocc5; cgate6 <= cgate5; chas6 <= chas5;
                if (cyr5 > 0) begin
                    cxr6 <= cxr5 + (cyr5 >>> 5); cyr6 <= cyr5 - (cxr5 >>> 5); cang6 <= cang5 + 16'd326;
                end else begin
                    cxr6 <= cxr5 - (cyr5 >>> 5); cyr6 <= cyr5 + (cxr5 >>> 5); cang6 <= cang5 - 16'd326;
                end
            end
            cs7 <= cs6;
            if (cs6) begin
                csl7 <= csl6; cow7 <= cow6; cocc7 <= cocc6; cgate7 <= cgate6; chas7 <= chas6;
                if (cyr6 > 0) begin
                    cxr7 <= cxr6 + (cyr6 >>> 6); cyr7 <= cyr6 - (cxr6 >>> 6); cang7 <= cang6 + 16'd163;
                end else begin
                    cxr7 <= cxr6 - (cyr6 >>> 6); cyr7 <= cyr6 + (cxr6 >>> 6); cang7 <= cang6 - 16'd163;
                end
            end
            cs8 <= cs7;
            if (cs7) begin
                csl8 <= csl7; cow8 <= cow7; cocc8 <= cocc7; cgate8 <= cgate7; chas8 <= chas7;
                if (cyr7 > 0) begin
                    cxr8 <= cxr7 + (cyr7 >>> 7); cyr8 <= cyr7 - (cxr7 >>> 7); cang8 <= cang7 + 16'd81;
                end else begin
                    cxr8 <= cxr7 - (cyr7 >>> 7); cyr8 <= cyr7 + (cxr7 >>> 7); cang8 <= cang7 - 16'd81;
                end
            end
            cs9 <= cs8;
            if (cs8) begin
                csl9 <= csl8; cow9 <= cow8; cocc9 <= cocc8; cgate9 <= cgate8; chas9 <= chas8;
                if (cyr8 > 0) begin
                    cxr9 <= cxr8 + (cyr8 >>> 8); cyr9 <= cyr8 - (cxr8 >>> 8); cang9 <= cang8 + 16'd41;
                end else begin
                    cxr9 <= cxr8 - (cyr8 >>> 8); cyr9 <= cyr8 + (cxr8 >>> 8); cang9 <= cang8 - 16'd41;
                end
            end
            cs10 <= cs9;
            if (cs9) begin
                csl10 <= csl9; cow10 <= cow9; cocc10 <= cocc9; cgate10 <= cgate9; chas10 <= chas9;
                if (cyr9 > 0) begin
                    cxr10 <= cxr9 + (cyr9 >>> 9); cyr10 <= cyr9 - (cxr9 >>> 9); cang10 <= cang9 + 16'd20;
                end else begin
                    cxr10 <= cxr9 - (cyr9 >>> 9); cyr10 <= cyr9 + (cxr9 >>> 9); cang10 <= cang9 - 16'd20;
                end
            end
            // ---------------- 写回 ----------------
            if (cs10) begin
                sm_we <= 1'b1; sm_waddr <= csl10;
                sm_wdata <= {cow10, cocc10, q_f, chas10, cgate10};
            end
        end
    end
endmodule
