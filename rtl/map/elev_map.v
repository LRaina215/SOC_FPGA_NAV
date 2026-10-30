// N×N槽，每槽53位 {retained_stale,stamp[8],tag[8],cnt[4],zmin[16],zmax[16]}。
// 更新规则（每点读-改-写，每拍 1 点；读数据先寄存一拍再合并写回，同址连续更新用两级前递解决）：
// cfg_avg=0（逐帧极值）：
//   槽空 / 标签不符 / 不是本帧写入的 -> 以该点重新初始化（每帧用最新观测替换，动态障碍可消失）
//   否则 -> zmax/zmin 取极值，cnt 饱和加 1
// cfg_avg=1（跨帧平均，与 model/nav_fixed.py ElevMap(avg=1) 逐位一致）：
//   zmax 位置存 Q3 高度均值 A（点高度先限幅 ±4095 mm），zmin 位置存 Q3 平均绝对偏差 D（无符号），cnt 为累计点数；
//   槽空 / 标签不符 / 已标记过期 -> A = 该点、D = 0、cnt = 1；
//   否则 A += (z - A) >>> s，D += (|z - A| - D) >>> s，s 按已有点数 1/2-3/4-7/8-14 取 1/2/3/4，
//   计满 15 点后取 cfg_avg_shift，且不超过 cfg_avg_shift。切换模式后须清图。
// 帧尾（s_eof）帧计数 fc 加 1。
// 扫描口（sweep_*）供可通行性分析在更新空闲时读写（过期清除）；swp_sel=1 时更新输入被阻塞。
// clear 脉冲：逐槽写 0（N*N 拍），期间阻塞输入。复位后自动清一次。
module elev_map #(
    parameter integer LOG2N = 7,
    parameter integer TAGB  = 4
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        clear,
    input  wire        cfg_avg,
    input  wire [2:0]  cfg_avg_shift,

    input  wire        s_valid,
    output wire        s_ready,
    input  wire        s_upd,
    input  wire [2*LOG2N-1:0] s_addr,
    input  wire [2*TAGB-1:0]  s_tag,
    input  wire signed [15:0] s_z,
    input  wire        s_eof,

    output reg  [7:0]  fc,
    output reg         frame_done,     // 单拍：一帧的最后一个更新已写入
    output wire        busy,           // 清除中或流水线未排空

    input  wire        swp_sel,        // 扫描占用存储
    input  wire        swp_re,
    input  wire [2*LOG2N-1:0] swp_raddr,
    output wire [52:0] swp_rdata,
    input  wire        swp_we,
    input  wire [2*LOG2N-1:0] swp_waddr,
    input  wire [52:0] swp_wdata,

    // 软核读口：读口空闲（无扫描、本拍无点更新、非清除）时服务；req 保持到 ack，ack 拍 swp_rdata 即数据
    input  wire        cpu_req,
    input  wire [2*LOG2N-1:0] cpu_raddr,
    output reg         cpu_ack
);
    localparam integer AW = 2 * LOG2N;

    reg          clr_act;
    reg [AW-1:0] clr_a;

    // S0：发读
    wire acc = s_valid & s_ready;
    assign s_ready = ~clr_act & ~swp_sel;

    // S1：RAM 数据返回并寄存；S2：合并并写回
    reg          v1, upd1, eof1;
    reg [AW-1:0] a1;
    reg [2*TAGB-1:0] tag1;
    reg signed [15:0] z1;
    reg          v2, upd2, eof2;
    reg [AW-1:0] a2;
    reg [2*TAGB-1:0] tag2;
    reg signed [15:0] z2;
    reg signed [15:0] zq2;                      // 平均模式：限幅后的 Q3 高度
    reg [52:0]   q2;
    // S3：写回（寄存后再写 RAM）
    // 前递：前 1/2/3 个点的新值（它们写入 RAM 的时刻不早于本点读 RAM 的时刻）
    reg          fw1_v, fw2_v, fw3_v;
    reg [AW-1:0] fw1_a, fw2_a, fw3_a;
    reg [52:0]   fw1_d, fw2_d, fw3_d;

    wire [52:0] q;
    // 前递命中在 S1 预先比较（S1 时的 S2/fw1/fw2 即为 S2 时的 fw1/fw2/fw3）
    reg m1, m2, m3;
    wire [52:0] old = m1 ? fw1_d : m2 ? fw2_d : m3 ? fw3_d : q2;
    wire [7:0]  o_stamp = old[51:44];
    wire [7:0]  o_tag   = old[43:36];
    wire [3:0]  o_cnt   = old[35:32];
    wire signed [15:0] o_zmin = old[31:16];
    wire signed [15:0] o_zmax = old[15:0];
    wire reset_slot = old[52] || (o_cnt == 4'd0) || (o_tag != tag2);
    wire fresh = reset_slot || (o_stamp != fc);
    wire [3:0]  cnt_inc = (o_cnt == 4'hF) ? 4'hF : o_cnt + 4'd1;
    wire [52:0] nw_ext = fresh ? {1'b0,fc, tag2, 4'd1, z2, z2}
                               : {1'b0,fc, tag2, cnt_inc,
                                  (z2 < o_zmin) ? z2 : o_zmin, (z2 > o_zmax) ? z2 : o_zmax};
    // 平均模式
    wire signed [15:0] o_a = old[15:0];
    wire        [15:0] o_d = old[31:16];
    wire [2:0] s_pts = (o_cnt == 4'd1) ? 3'd1 : (o_cnt <= 4'd3) ? 3'd2 : (o_cnt <= 4'd7) ? 3'd3 : 3'd4;
    wire [2:0] s_avg = (o_cnt == 4'hF) ? cfg_avg_shift : (s_pts < cfg_avg_shift) ? s_pts : cfg_avg_shift;
    wire signed [16:0] dif  = {zq2[15], zq2} - {o_a[15], o_a};
    wire signed [16:0] ndif = {o_a[15], o_a} - {zq2[15], zq2};
    wire        [15:0] adif = dif[16] ? ndif[15:0] : dif[15:0];          // |z - A| <= 65520
    wire signed [16:0] dev  = {1'b0, adif} - {1'b0, o_d};
    wire signed [16:0] dif_s = dif >>> s_avg;
    wire signed [16:0] dev_s = dev >>> s_avg;
    wire        [15:0] a_new = o_a + dif_s[15:0];
    wire        [15:0] d_new = o_d + dev_s[15:0];
    wire [52:0] nw_avg = reset_slot ? {1'b0, fc, tag2, 4'd1, 16'd0, zq2}
                                    : {1'b0, fc, tag2, cnt_inc, d_new, a_new};
    wire [52:0] nw = cfg_avg ? nw_avg : nw_ext;

    wire          we_u = v2 & upd2;
    // fw1 即 S3 写口寄存器
    wire          ram_we    = clr_act ? 1'b1 : swp_sel ? swp_we : fw1_v;
    wire [AW-1:0] ram_waddr = clr_act ? clr_a : swp_sel ? swp_waddr : fw1_a;
    wire [52:0]   ram_wdata = clr_act ? 53'd0 : swp_sel ? swp_wdata : fw1_d;
    reg           cpu_pend;
    wire          cpu_go    = cpu_req & ~cpu_pend & ~cpu_ack & ~swp_sel & ~acc & ~clr_act;
    wire          ram_re    = swp_sel ? swp_re : (acc | cpu_go);
    wire [AW-1:0] ram_raddr = swp_sel ? swp_raddr : acc ? s_addr : cpu_raddr;

    nav_ram_sdp #(.DW(53), .AW(AW)) u_ram (
        .clk(clk), .we(ram_we), .waddr(ram_waddr), .wdata(ram_wdata),
        .re(ram_re), .raddr(ram_raddr), .rdata(q)
    );
    assign swp_rdata = q;
    assign busy = clr_act | v1 | v2 | fw1_v;

    always @(posedge clk) begin
        cpu_ack <= 1'b0;
        if (rst) cpu_pend <= 1'b0;
        else begin
            cpu_pend <= cpu_go;
            if (cpu_pend) cpu_ack <= 1'b1;
        end
    end

    always @(posedge clk) begin
        frame_done <= 1'b0;
        if (rst) begin
            v1 <= 1'b0; v2 <= 1'b0; fw1_v <= 1'b0; fw2_v <= 1'b0; fw3_v <= 1'b0; fc <= 8'd0;
            clr_act <= 1'b1; clr_a <= 0;
        end else begin
            if (clear && !clr_act) begin
                clr_act <= 1'b1; clr_a <= 0;
            end else if (clr_act) begin
                clr_a <= clr_a + 1'b1;
                if (clr_a == {AW{1'b1}}) clr_act <= 1'b0;
            end
            v1 <= acc; upd1 <= s_upd; eof1 <= s_eof;
            a1 <= s_addr; tag1 <= s_tag; z1 <= s_z;
            v2 <= v1; upd2 <= upd1; eof2 <= eof1; a2 <= a1; tag2 <= tag1; z2 <= z1; q2 <= q;
            zq2 <= ((z1 > 16'sd4095) ? 16'sd4095 : (z1 < -16'sd4095) ? -16'sd4095 : z1) <<< 3;
            m1 <= we_u  && (a2 == a1);
            m2 <= fw1_v && (fw1_a == a1);
            m3 <= fw2_v && (fw2_a == a1);
            fw1_v <= we_u; fw1_a <= a2; fw1_d <= nw;
            fw2_v <= fw1_v; fw2_a <= fw1_a; fw2_d <= fw1_d;
            fw3_v <= fw2_v; fw3_a <= fw2_a; fw3_d <= fw2_d;
            if (v2 && eof2) begin
                fc <= fc + 1'b1;
                frame_done <= 1'b1;
            end
        end
    end
endmodule
