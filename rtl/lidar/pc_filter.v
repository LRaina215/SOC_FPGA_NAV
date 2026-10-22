// 点过滤与栅格化：里程计坐标点 -> 滚动窗口内的栅格更新 {槽地址, 标签, 高度}。
//   世界栅格 ix = floor(x * INV_RES / 65536)（INV_RES = 65536 / 分辨率mm）
//   窗口 = 以 (cx, cy) 为中心、边长 N 的世界栅格范围 [c - N/2, c + N/2)
//   槽地址 = (iy mod N) * N + (ix mod N)；标签 = 世界栅格高位 (ix >> LOG2N) 的低 TAGB 位
// 保留条件：点有效、z 在 [zmin, zmax]、落在窗口内。
// 帧尾一定输出一拍（m_upd=0 时仅表示帧结束），保证下游能按帧处理。
// cx/cy 在帧首点锁存；m_cx/m_cy 输出本帧使用的中心，供可通行性分析对齐。
module pc_filter #(
    parameter integer LOG2N = 7,
    parameter integer TAGB  = 4
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] cfg_inv_res,
    input  wire signed [31:0] cfg_cx,
    input  wire signed [31:0] cfg_cy,
    input  wire signed [15:0] cfg_zmin,
    input  wire signed [15:0] cfg_zmax,

    input  wire        s_valid,
    output wire        s_ready,
    input  wire signed [31:0] s_x,
    input  wire signed [31:0] s_y,
    input  wire signed [31:0] s_z,
    input  wire        s_pv,
    input  wire        s_sof,
    input  wire        s_eof,

    output reg         m_valid,
    input  wire        m_ready,
    output reg         m_upd,
    output reg  [2*LOG2N-1:0] m_addr,
    output reg  [2*TAGB-1:0]  m_tag,     // {tag_y, tag_x}
    output reg  signed [15:0] m_z,
    output reg         m_sof,
    output reg         m_eof,
    output reg  signed [31:0] m_cx,
    output reg  signed [31:0] m_cy,
    output reg [13:0] m_cov_addr,
    output reg        m_cov_upd,

    output reg  [31:0] stat_upd,         // 上一帧写入点数（帧尾更新）
    output reg  [31:0] stat_drop         // 上一帧有效但被剔除的点数
);
    localparam integer N = 1 << LOG2N;
    wire ce = ~m_valid | m_ready;
    assign s_ready = ce;

    // 坐标限幅到 ±2^25 mm（±33 km）再乘，乘积 42 位
    function signed [25:0] clip26(input signed [31:0] a);
        clip26 = (a > 32'sd33554431) ? 26'sd33554431 : (a < -32'sd33554432) ? -26'sd33554432 : a[25:0];
    endfunction

    reg signed [31:0] cx_l, cy_l;
    wire signed [31:0] cxs = (s_valid && s_sof) ? cfg_cx : cx_l;
    wire signed [31:0] cys = (s_valid && s_sof) ? cfg_cy : cy_l;

    // S1：乘法、z 判定
    reg signed [42:0] mx, my;
    reg        v1, sof1, eof1, keep1, pv1;
    reg signed [15:0] z1;
    reg signed [31:0] cx1, cy1;
    wire signed [26:0] xc = clip26(s_x);
    wire signed [26:0] yc = clip26(s_y);
    wire signed [16:0] ir = {1'b0, cfg_inv_res};
    wire zin = (s_z >= cfg_zmin) && (s_z <= cfg_zmax);

    // S2：世界栅格、相对窗口起点的偏移
    wire signed [26:0] ix = mx >>> 16;
    wire signed [26:0] iy = my >>> 16;
    reg  signed [31:0] dx2, dy2;
    reg  [LOG2N+TAGB-1:0] ixl2, iyl2;
    reg [1:0] subx2,suby2;
    reg        v2, sof2, eof2, keep2, pv2;
    reg signed [15:0] z2;
    reg signed [31:0] cx2, cy2;

    // S3：窗口判定与输出
    wire inwin = (dx2 >= 0) && (dx2 < N) && (dy2 >= 0) && (dy2 < N);
    wire kept  = keep2 && inwin;

    reg [31:0] cnt_upd, cnt_drop;

    always @(posedge clk) begin
        if (rst) begin
            v1 <= 1'b0; v2 <= 1'b0; m_valid <= 1'b0; cx_l <= 0; cy_l <= 0;
            cnt_upd <= 0; cnt_drop <= 0; stat_upd <= 0; stat_drop <= 0;
        end else if (ce) begin
            if (s_valid && s_sof) begin cx_l <= cfg_cx; cy_l <= cfg_cy; end
            v1 <= s_valid; sof1 <= s_sof; eof1 <= s_eof; pv1 <= s_pv;
            keep1 <= s_pv && zin;
            z1 <= s_z[15:0];
            mx <= xc * ir;
            my <= yc * ir;
            cx1 <= cxs; cy1 <= cys;

            v2 <= v1; sof2 <= sof1; eof2 <= eof1; pv2 <= pv1; keep2 <= keep1; z2 <= z1;
            dx2 <= ix - (cx1 - N/2);
            dy2 <= iy - (cy1 - N/2);
            ixl2 <= ix[LOG2N+TAGB-1:0];
            iyl2 <= iy[LOG2N+TAGB-1:0];
            subx2<=mx[15:14];suby2<=my[15:14];
            cx2 <= cx1; cy2 <= cy1;

            m_valid <= v2 && (kept || eof2);
            m_upd   <= kept;
            m_addr  <= {iyl2[LOG2N-1:0], ixl2[LOG2N-1:0]};
            m_tag   <= {iyl2[LOG2N+TAGB-1:LOG2N], ixl2[LOG2N+TAGB-1:LOG2N]};
            m_z     <= z2;
            m_sof   <= sof2;
            m_eof   <= eof2;
            m_cx    <= cx2;
            m_cy    <= cy2;
            m_cov_addr<={iyl2[4:0],ixl2[4:0],suby2,subx2};
            m_cov_upd<=kept && dx2>=N/2-16 && dx2<N/2+16 && dy2>=N/2-16 && dy2<N/2+16;

            if (v2) begin
                if (sof2) begin
                    cnt_upd  <= kept ? 32'd1 : 32'd0;
                    cnt_drop <= (pv2 && !kept) ? 32'd1 : 32'd0;
                end else begin
                    cnt_upd  <= cnt_upd  + kept;
                    cnt_drop <= cnt_drop + (pv2 && !kept);
                end
                if (eof2) begin
                    stat_upd  <= (sof2 ? 32'd0 : cnt_upd)  + kept;
                    stat_drop <= (sof2 ? 32'd0 : cnt_drop) + (pv2 && !kept);
                end
            end
        end
    end
endmodule
