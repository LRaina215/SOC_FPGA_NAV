// 深度 -> 雷达坐标系三维点（X 前 Y 左 Z 上，mm）。
//   depth_mm = (raw * DEPTH_MUL) >> 8，饱和到 32767
//   p = (depth_mm * ray) >>> LUT_FRAC  （向下取整）
// ray 为每像素方向向量（Q2.13），离线由内参 + 去畸变生成，已包含：
//   坐标轴换算、深度定义（光轴 Z 距离 / 径向距离）。所以本模块与这两件事无关。
// 查找表可由 INIT_FILE 初始化，也可运行期经 lut_we 口改写（软核从 Flash 载入）。
// 每拍 1 像素；无效像素照常输出（pv=0），保证帧首尾标记完整。
module pc_project #(
    parameter integer MAX_W    = 160,
    parameter integer MAX_H    = 60,
    parameter integer LUT_FRAC = 13,
    parameter LUT_INIT = ""
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] cfg_depth_mul,
    input  wire [15:0] cfg_min_mm,
    input  wire [15:0] cfg_max_mm,
    input  wire [7:0]  cfg_roi_x0,
    input  wire [7:0]  cfg_roi_y0,

    input  wire        lut_we,
    input  wire [13:0] lut_waddr,
    input  wire [47:0] lut_wdata,     // {rz, ry, rx}

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

    // S0 只做乘法；S1 饱和与有效判定，同时锁存查表结果
    reg        v0, sof0, eof0, nz0;
    reg [31:0] dprod0;
    reg [47:0] ray1;

    // S0：地址、深度换算
    wire [7:0]  uu = s_u + cfg_roi_x0;
    wire [7:0]  vv = s_v + cfg_roi_y0;
    wire [AW-1:0] laddr = vv * MAX_W + uu;
    wire [23:0] dmm_w = dprod0[31:8];
    wire [15:0] dmm   = (dmm_w > 24'd32767) ? 16'd32767 : dmm_w[15:0];

    reg        v1, sof1, eof1, ok1;
    reg [15:0] d1;
    wire [47:0] ray;

    nav_ram_sdp #(.DW(48), .AW(AW), .DEPTH(NPIX), .INIT_FILE(LUT_INIT)) u_lut (
        .clk(clk), .we(lut_we), .waddr(lut_waddr[AW-1:0]), .wdata(lut_wdata),
        .re(ce), .raddr(laddr), .rdata(ray)
    );

    // S1：乘法
    reg        v2, sof2, eof2, ok2;
    reg signed [31:0] px, py, pz;
    wire signed [16:0] ds = {1'b0, d1};
    wire signed [15:0] rx = ray1[15:0], ry = ray1[31:16], rz = ray1[47:32];

    always @(posedge clk) begin
        if (rst) begin
            v0 <= 1'b0; v1 <= 1'b0; v2 <= 1'b0; m_valid <= 1'b0;
        end else if (ce) begin
            v0     <= s_valid;
            sof0   <= s_sof; eof0 <= s_eof;
            nz0    <= (s_depth != 16'd0);
            dprod0 <= s_depth * cfg_depth_mul;

            v1   <= v0;
            sof1 <= sof0; eof1 <= eof0;
            d1   <= dmm;
            ok1  <= nz0 && (dmm >= cfg_min_mm) && (dmm <= cfg_max_mm);
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
