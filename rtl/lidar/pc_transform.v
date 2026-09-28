// 雷达坐标 -> 里程计坐标： p_o = R * p_s + t
//   R：Q1.14 有符号 16 位，t：mm int32。R/t 在每帧首个点（s_sof）时锁存，整帧使用同一位姿。
//   软核负责把 "机身位姿 × 雷达外参" 合成为这一组 R/t（见 fw/nav/pose.c）。
// 输出 x/y/z 为 int32 mm，向下取整。每拍 1 点，3 级流水。
module pc_transform #(
    parameter integer ROT_FRAC = 14
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [143:0] cfg_r,     // {r22,r21,r20,r12,r11,r10,r02,r01,r00}，各 16 位
    input  wire [95:0]  cfg_t,     // {tz,ty,tx}，各 32 位

    input  wire        s_valid,
    output wire        s_ready,
    input  wire signed [15:0] s_x,
    input  wire signed [15:0] s_y,
    input  wire signed [15:0] s_z,
    input  wire        s_pv,
    input  wire        s_sof,
    input  wire        s_eof,

    output reg         m_valid,
    input  wire        m_ready,
    output reg  signed [31:0] m_x,
    output reg  signed [31:0] m_y,
    output reg  signed [31:0] m_z,
    output reg         m_pv,
    output reg         m_sof,
    output reg         m_eof
);
    wire ce = ~m_valid | m_ready;
    assign s_ready = ce;

    reg signed [15:0] r [0:8];
    reg signed [31:0] tx, ty, tz;
    integer k;

    // 帧首点使用新位姿：S0 选择锁存值或输入值
    wire signed [15:0] rs [0:8];
    genvar g;
    generate
        for (g = 0; g < 9; g = g + 1) begin : g_r
            assign rs[g] = (s_valid && s_sof) ? cfg_r[16*g +: 16] : r[g];
        end
    endgenerate
    wire signed [31:0] txs = (s_valid && s_sof) ? cfg_t[31:0]  : tx;
    wire signed [31:0] tys = (s_valid && s_sof) ? cfg_t[63:32] : ty;
    wire signed [31:0] tzs = (s_valid && s_sof) ? cfg_t[95:64] : tz;

    // S1：9 个乘积
    reg signed [31:0] p [0:8];
    reg signed [31:0] t1x, t1y, t1z;
    reg v1, pv1, sof1, eof1;
    // S2：求和
    reg signed [33:0] ax, ay, az;
    reg signed [31:0] t2x, t2y, t2z;
    reg v2, pv2, sof2, eof2;

    always @(posedge clk) begin
        if (rst) begin
            v1 <= 1'b0; v2 <= 1'b0; m_valid <= 1'b0;
            for (k = 0; k < 9; k = k + 1) r[k] <= (k == 0 || k == 4 || k == 8) ? 16'sd16384 : 16'sd0;
            tx <= 0; ty <= 0; tz <= 0;
        end else if (ce) begin
            if (s_valid && s_sof) begin
                for (k = 0; k < 9; k = k + 1) r[k] <= cfg_r[16*k +: 16];
                tx <= cfg_t[31:0]; ty <= cfg_t[63:32]; tz <= cfg_t[95:64];
            end
            v1 <= s_valid; pv1 <= s_pv; sof1 <= s_sof; eof1 <= s_eof;
            p[0] <= rs[0] * s_x; p[1] <= rs[1] * s_y; p[2] <= rs[2] * s_z;
            p[3] <= rs[3] * s_x; p[4] <= rs[4] * s_y; p[5] <= rs[5] * s_z;
            p[6] <= rs[6] * s_x; p[7] <= rs[7] * s_y; p[8] <= rs[8] * s_z;
            t1x <= txs; t1y <= tys; t1z <= tzs;

            v2 <= v1; pv2 <= pv1; sof2 <= sof1; eof2 <= eof1;
            ax <= p[0] + p[1] + p[2];
            ay <= p[3] + p[4] + p[5];
            az <= p[6] + p[7] + p[8];
            t2x <= t1x; t2y <= t1y; t2z <= t1z;

            m_valid <= v2; m_pv <= pv2; m_sof <= sof2; m_eof <= eof2;
            m_x <= (ax >>> ROT_FRAC) + t2x;
            m_y <= (ay >>> ROT_FRAC) + t2y;
            m_z <= (az >>> ROT_FRAC) + t2z;
        end
    end
endmodule
