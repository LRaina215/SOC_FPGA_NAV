// SLF 结构点提取（流式，旁听 pc_transform 的输出）：与 model/slf_match.py:structure_mask + subsample 的前半一致。
// 同一列上下相邻两像素（行优先 W×H）都有效，且 |Δz| >= DZ_MIN、|Δz| > 2·max(|Δx|,|Δy|) -> 两点都是结构点。
// 上一行的点要等本行同列处理完才知道标记，因此输出滞后一行（行缓存），帧尾再把最后一行冲出来——输出顺序仍是行优先。
// 结构点再按高度带 [zlo, zhi] 过滤，存入点缓冲（相对帧位姿平移 (tx, ty) 的 18 位有符号 mm），最多 PB_MAX 个。
module slf_extract #(
    parameter integer W = 160,
    parameter integer H = 60,
    parameter integer PB_AW = 12,
    parameter integer DZ_MIN = 20
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        en,                  // 本帧提取（帧开始时采样）
    input  wire signed [31:0] cfg_zlo,
    input  wire signed [31:0] cfg_zhi,
    input  wire signed [31:0] cfg_tx,
    input  wire signed [31:0] cfg_ty,
    // 点流（已被下游接收的传输）
    input  wire        s_fire,
    input  wire signed [31:0] s_x,
    input  wire signed [31:0] s_y,
    input  wire signed [31:0] s_z,
    input  wire        s_pv,
    input  wire        s_sof,
    input  wire        s_eof,
    // 点缓冲写口
    output reg         pb_we,
    output reg  [PB_AW-1:0] pb_waddr,
    output reg  [35:0] pb_wdata,            // {ry[17:0], rx[17:0]}
    output reg         done,                // 单拍：本帧结构点已全部写入
    output reg  [PB_AW:0] n_pts,            // 本帧结构点数（饱和于 2^PB_AW）
    output reg         busy
);
    localparam integer UW = $clog2(W);
    localparam integer RB = 1 + 1 + 96;      // {mark, pv, z, y, x}
    localparam [PB_AW:0] PB_MAX = (1 << PB_AW);

    // 行缓存（同步读）
    reg          rb_we;
    reg [UW-1:0] rb_wa;
    reg [RB-1:0] rb_wd;
    wire         rb_re;
    wire [UW-1:0] rb_ra;
    wire [RB-1:0] rb_q;
    nav_ram_sdp #(.DW(RB), .AW(UW), .DEPTH(W)) u_rb (
        .clk(clk), .we(rb_we), .waddr(rb_wa), .wdata(rb_wd), .re(rb_re), .raddr(rb_ra), .rdata(rb_q)
    );

    reg          act;                       // 本帧在提取
    reg [UW-1:0] u;
    reg [7:0]    r;
    reg signed [31:0] tx_l, ty_l, zlo_l, zhi_l;
    reg          flushing;
    reg [UW:0]   fu;                        // 冲出计数

    // S0 -> S1
    reg          v1, f1;                    // f1: 冲出遍的读
    reg [UW-1:0] u1;
    reg          top1;                      // 当前像素不在第 0 行
    reg signed [31:0] x1, y1, z1;
    reg          pv1;
    assign rb_re = (act && s_fire) || (flushing && fu < W);
    assign rb_ra = flushing ? fu[UW-1:0] : u;

    // S1：行缓存数据返回
    wire         p_mark = rb_q[97];
    wire         p_pv   = rb_q[96];
    wire signed [31:0] p_z = rb_q[95:64];
    wire signed [31:0] p_y = rb_q[63:32];
    wire signed [31:0] p_x = rb_q[31:0];
    function [32:0] absd32(input signed [31:0] a, input signed [31:0] b);
        reg signed [32:0] d;
        begin d = {a[31], a} - {b[31], b}; absd32 = d[32] ? -d : d; end
    endfunction
    // S2：比较
    reg          v2, f2, top2, pv2c, pvp2, markp2;
    reg [UW-1:0] u2;
    reg [32:0]   dz2, dx2, dy2;
    reg signed [31:0] x2, y2, z2, px2, py2, pz2;
    reg          last2;
    reg          last1;
    wire [32:0]  dxy2 = (dx2 > dy2) ? dx2 : dy2;
    wire         vert2 = !f2 && top2 && pv2c && pvp2 && (dz2 >= DZ_MIN) && ({1'b0, dz2} > {dxy2, 1'b0});
    // 被输出的是上一行同列的点（冲出遍时是行缓存里的最后一行）
    wire         emit2 = (f2 || top2) && pvp2 && (markp2 || vert2) && (pz2 >= zlo_l) && (pz2 <= zhi_l);

    always @(posedge clk) begin
        rb_we <= 1'b0; pb_we <= 1'b0; done <= 1'b0;
        if (rst) begin
            act <= 1'b0; flushing <= 1'b0; busy <= 1'b0; v1 <= 1'b0; v2 <= 1'b0; n_pts <= 0;
        end else begin
            // ---------------- S0：像素到达 ----------------
            v1 <= 1'b0;
            if (s_fire && s_sof) begin
                act <= en; u <= 0; r <= 0; n_pts <= 0; busy <= en;
                tx_l <= cfg_tx; ty_l <= cfg_ty; zlo_l <= cfg_zlo; zhi_l <= cfg_zhi;
            end
            if (s_fire && (act || (s_sof && en))) begin
                v1 <= 1'b1; f1 <= 1'b0;
                u1 <= s_sof ? {UW{1'b0}} : u;
                top1 <= s_sof ? 1'b0 : (r != 0);
                x1 <= s_x; y1 <= s_y; z1 <= s_z; pv1 <= s_pv;
                last1 <= s_eof;
                if (s_sof) begin u <= 1; r <= 0; end
                else if (u == W - 1) begin u <= 0; r <= r + 1'b1; end
                else u <= u + 1'b1;
                if (s_eof) act <= 1'b0;
            end else if (flushing && fu < W) begin
                v1 <= 1'b1; f1 <= 1'b1; u1 <= fu[UW-1:0]; top1 <= 1'b1; last1 <= (fu == W - 1);
                fu <= fu + 1'b1;
            end
            // ---------------- S1 ----------------
            v2 <= v1;
            if (v1) begin
                f2 <= f1; u2 <= u1; top2 <= top1; last2 <= last1;
                pv2c <= pv1; x2 <= x1; y2 <= y1; z2 <= z1;
                pvp2 <= p_pv; markp2 <= p_mark; px2 <= p_x; py2 <= p_y; pz2 <= p_z;
                dz2 <= absd32(z1, p_z); dx2 <= absd32(x1, p_x); dy2 <= absd32(y1, p_y);
            end
            // ---------------- S2：写回行缓存、输出结构点 ----------------
            if (v2) begin
                if (!f2) begin
                    rb_we <= 1'b1; rb_wa <= u2; rb_wd <= {vert2, pv2c, z2, y2, x2};
                end
                if (emit2 && n_pts < PB_MAX) begin
                    pb_we <= 1'b1; pb_waddr <= n_pts[PB_AW-1:0];
                    pb_wdata <= {py2[17:0] - ty_l[17:0], px2[17:0] - tx_l[17:0]};
                    n_pts <= n_pts + 1'b1;
                end
                if (!f2 && last2) begin flushing <= 1'b1; fu <= 0; end
                if (f2 && last2) begin flushing <= 1'b0; busy <= 1'b0; done <= 1'b1; end
            end
        end
    end
endmodule
