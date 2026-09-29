// 深度 -> 雷达坐标系三维点（X 前 Y 左 Z 上，mm）。
//   depth_mm = (raw * DEPTH_MUL) >> 8，饱和到 32767
//   p = (depth_mm * ray) >>> LUT_FRAC  （向下取整）
// ray 为每像素视线单位向量（12 位有符号 Q0.11，36 位字 {rz, ry, rx}），由内参 + 去畸变生成：
//   XT-M60 的深度是沿视线的径向距离（厂家 SDK 核实），所以点 = 距离 × 单位向量；坐标轴换算也已在表里。
//   表在上电后由固件按雷达帧尾带来的内参写入（fw/nav/lidar_cal.c，经 lut_we 口）；全零项 = 该像素无效
//   （四角切除、尚未写表）。
// 查表地址 = ((v << 竖直合并) + ROI_Y0) * MAX_W + (u << 水平合并) + ROI_X0（同 SDK），越界的像素无效。
// 每拍 1 像素；无效像素照常输出（pv=0），保证帧首尾标记完整。
module pc_project #(
    parameter integer MAX_W    = 160,
    parameter integer MAX_H    = 60,
    parameter integer LUT_FRAC = 11
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] cfg_depth_mul,
    input  wire [15:0] cfg_min_mm,
    input  wire [15:0] cfg_max_mm,
    input  wire [7:0]  cfg_roi_x0,
    input  wire [7:0]  cfg_roi_y0,
    input  wire [1:0]  cfg_bin,       // [1] 水平合并 [0] 竖直合并

    input  wire        lut_we,
    input  wire [13:0] lut_waddr,
    input  wire [35:0] lut_wdata,     // {rz, ry, rx}

    input  wire        s_valid,
    output wire        s_ready,
    input  wire [7:0]  s_u,
    input  wire [7:0]  s_v,
    input  wire [15:0] s_depth,
    input  wire        s_sof,
    input  wire        s_eof,

    output reg         m_valid,
    input  wire        m_ready,
    output reg  signed [15:0] m_x,
    output reg  signed [15:0] m_y,
    output reg  signed [15:0] m_z,
    output reg         m_pv,
    output reg         m_sof,
    output reg         m_eof
);
    localparam integer NPIX = MAX_W * MAX_H;
    localparam integer AW   = $clog2(NPIX);

    wire ce = ~m_valid | m_ready;
    assign s_ready = ce;

    // S0：查表地址、深度换算
    reg        v0, sof0, eof0, nz0;
    reg [31:0] dprod0;
    wire [9:0]  uu = ({2'd0, s_u} << cfg_bin[1]) + {2'd0, cfg_roi_x0};
    wire [9:0]  vv = ({2'd0, s_v} << cfg_bin[0]) + {2'd0, cfg_roi_y0};
    wire        inr = (uu < MAX_W) && (vv < MAX_H);
    wire [AW-1:0] vv_e = {{(AW-10){1'b0}}, vv};
    wire [AW-1:0] uu_e = {{(AW-10){1'b0}}, uu};
    wire [AW-1:0] laddr = inr ? (vv_e * MAX_W + uu_e) : {AW{1'b0}};
    wire [23:0] dmm_w = dprod0[31:8];
    wire [15:0] dmm   = (dmm_w > 24'd32767) ? 16'd32767 : dmm_w[15:0];

    // S1：有效判定，锁存查表结果
    reg        v1, sof1, eof1, ok1, inr0;
    reg [15:0] d1;
    reg [35:0] ray1;
    wire [35:0] ray;

    nav_ram_banked #(.DW(36), .DEPTH(NPIX)) u_lut (
        .clk(clk), .we(lut_we), .waddr(lut_waddr[AW-1:0]), .wdata(lut_wdata),
        .re(ce), .raddr(laddr), .rdata(ray)
    );

    // S2：乘法
    reg        v2, sof2, eof2, ok2;
    reg signed [31:0] px, py, pz;
    wire signed [16:0] ds = {1'b0, d1};
    wire signed [11:0] rx = ray1[11:0], ry = ray1[23:12], rz = ray1[35:24];

    always @(posedge clk) begin
        if (rst) begin
            v0 <= 1'b0; v1 <= 1'b0; v2 <= 1'b0; m_valid <= 1'b0;
        end else if (ce) begin
            v0     <= s_valid;
            sof0   <= s_sof; eof0 <= s_eof;
            nz0    <= (s_depth != 16'd0);
            inr0   <= inr;
            dprod0 <= s_depth * cfg_depth_mul;

            v1   <= v0;
            sof1 <= sof0; eof1 <= eof0;
            d1   <= dmm;
            ok1  <= nz0 && inr0 && (ray != 36'd0) && (dmm >= cfg_min_mm) && (dmm <= cfg_max_mm);
            ray1 <= ray;

            v2   <= v1;
            sof2 <= sof1; eof2 <= eof1; ok2 <= ok1;
            px <= ds * rx;
            py <= ds * ry;
            pz <= ds * rz;

            m_valid <= v2;
            m_sof <= sof2; m_eof <= eof2; m_pv <= ok2;
            m_x <= sat16(px >>> LUT_FRAC);
            m_y <= sat16(py >>> LUT_FRAC);
            m_z <= sat16(pz >>> LUT_FRAC);
        end
    end

    function signed [15:0] sat16(input signed [31:0] a);
        sat16 = (a > 32'sd32767) ? 16'sd32767 : (a < -32'sd32768) ? -16'sd32768 : a[15:0];
    endfunction
endmodule
