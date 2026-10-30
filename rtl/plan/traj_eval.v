// 候选轨迹评估：对 K 条候选速度各前推 T 步，沿途查前台代价图。
// 每步：th += dth；x += (dvx*cos - dvy*sin) >>> 14；y += (dvx*sin + dvy*cos) >>> 14
//       (x, y) 为世界栅格 Q8；出窗口或代价 255 计未知；代价 >= CMAX 视为碰撞并停止该条。
// 结果（每条 4 字）：
//   w0 [7:0]max_cost [15:8]n_unknown [23:16]coll_step [24]collide [25]left
//   w1 sum_cost   w2 end_x(Q8)   w3 end_y(Q8)
// 运算与 model/nav_fixed.py:traj_eval 逐位一致。单套数据通路逐条评估；窗口内每步9拍。
// 健康运行条件下周期上界 K*(9*T+9)+2；不含CPU装载、读回、打分及地形细检。
module traj_eval #(
    parameter integer LOG2N    = 7,
    parameter integer KMAX_LOG = 6,
    parameter SIN_INIT = "sin_lut.hex"
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire [7:0]  cfg_k,
    input  wire [7:0]  cfg_t,
    input  wire signed [31:0] cfg_x0,
    input  wire signed [31:0] cfg_y0,
    input  wire [15:0] cfg_th0,
    input  wire [7:0]  cfg_cmax,
    input  wire signed [31:0] map_cx,     // 前台代价图中心（启动时锁存）
    input  wire signed [31:0] map_cy,
    input  wire [15:0] map_seq,
    output reg         busy,
    output reg         done,              // 单拍
    output reg         stale,             // 评估期间代价图被更新过

    // 候选存储写口（总线）：地址 = k*2 + w
    input  wire        cand_we,
    input  wire [KMAX_LOG:0] cand_addr,
    input  wire [31:0] cand_wdata,
    // 结果读口（总线）：地址 = k*4 + w，下一拍出数
    input  wire        res_re,
    input  wire [KMAX_LOG+1:0] res_addr,
    output wire [31:0] res_rdata,

    // 代价图读口（map_subsys A 口：地址同拍给出，下一拍出数）
    output reg         cm_re,
    output reg  [2*LOG2N-1:0] cm_addr,
    input  wire [7:0]  cm_q
);
    localparam integer N = 1 << LOG2N;

    localparam [4:0] S_IDLE = 5'd0, S_C0 = 5'd1, S_C1 = 5'd2, S_C2 = 5'd3, S_C3 = 5'd4,
                     S_A = 5'd5, S_B = 5'd6, S_C = 5'd7, S_D = 5'd8, S_E = 5'd9, S_F = 5'd10,
                     S_Q1 = 5'd11, S_Q2 = 5'd12, S_W0 = 5'd13, S_W1 = 5'd14, S_W2 = 5'd15,
                     S_W3 = 5'd16, S_DONE = 5'd17, S_F0 = 5'd18, S_Q3 = 5'd19;
    reg [7:0] cq_r;
    reg [4:0] st;

    reg [7:0] K, T, k, t, cmax;
    reg signed [31:0] cx, cy, x, y, x0, y0;
    reg [15:0] th, th00, seq0;
    reg signed [15:0] dvx, dvy, dth;
    reg [7:0]  mxc, nun, cstep;
    reg        coll, left;
    reg [31:0] csum;

    // 候选存储（寄存器驱动读口，数据在 re 的后第 2 拍可用）
    reg              c_re;
    reg [KMAX_LOG:0] c_ra;
    wire [31:0]      c_q;
    nav_ram_sdp #(.DW(32), .AW(KMAX_LOG + 1)) u_cand (
        .clk(clk), .we(cand_we), .waddr(cand_addr), .wdata(cand_wdata),
        .re(c_re), .raddr(c_ra), .rdata(c_q)
    );

    // 结果存储
    reg                r_we;
    reg [KMAX_LOG+1:0] r_wa;
    reg [31:0]         r_wd;
    nav_ram_sdp #(.DW(32), .AW(KMAX_LOG + 2)) u_res (
        .clk(clk), .we(r_we), .waddr(r_wa), .wdata(r_wd),
        .re(res_re), .raddr(res_addr), .rdata(res_rdata)
    );

    // 正弦表：两个只读实例，sin 与 cos 同拍查
    reg         lut_re;
    reg  [9:0]  si, ci;
    wire [15:0] sq, cq;
    nav_ram_sdp #(.DW(16), .AW(10), .INIT_FILE(SIN_INIT)) u_sin (
        .clk(clk), .we(1'b0), .waddr(10'd0), .wdata(16'd0), .re(lut_re), .raddr(si), .rdata(sq)
    );
    nav_ram_sdp #(.DW(16), .AW(10), .INIT_FILE(SIN_INIT)) u_cos (
        .clk(clk), .we(1'b0), .waddr(10'd0), .wdata(16'd0), .re(lut_re), .raddr(ci), .rdata(cq)
    );

    wire [15:0] th_n = th + dth;
    reg signed [31:0] p_xc, p_ys, p_xs, p_yc;
    wire signed [32:0] ddx = {p_xc[31], p_xc} - {p_ys[31], p_ys};
    wire signed [32:0] ddy = {p_xs[31], p_xs} + {p_yc[31], p_yc};
    wire signed [32:0] ddx_s = ddx >>> 14;
    wire signed [32:0] ddy_s = ddy >>> 14;
    wire signed [31:0] ix = x >>> 8;
    wire signed [31:0] iy = y >>> 8;
    reg  signed [31:0] wx, wy;           // S_F0 寄存
    reg  [2*LOG2N-1:0] slot;
    wire inwin = (wx >= 0) && (wx < N) && (wy >= 0) && (wy < N);

    always @(posedge clk) begin
        done <= 1'b0; r_we <= 1'b0; c_re <= 1'b0; lut_re <= 1'b0; cm_re <= 1'b0;
        if (rst) begin
            st <= S_IDLE; busy <= 1'b0; stale <= 1'b0;
        end else begin
            case (st)
            S_IDLE: if (start) begin
                busy <= 1'b1; stale <= 1'b0;
                K <= (cfg_k > (1 << KMAX_LOG)) ? (1 << KMAX_LOG) : cfg_k;
                T <= cfg_t; cmax <= cfg_cmax;
                x0 <= cfg_x0; y0 <= cfg_y0; th00 <= cfg_th0;
                cx <= map_cx; cy <= map_cy; seq0 <= map_seq;
                k <= 0;
                st <= (cfg_k == 0) ? S_DONE : S_C0;
            end
            // 读候选 k 的两个字
            S_C0: begin c_re <= 1'b1; c_ra <= {k[KMAX_LOG-1:0], 1'b0}; st <= S_C1; end
            S_C1: begin c_re <= 1'b1; c_ra <= {k[KMAX_LOG-1:0], 1'b1}; st <= S_C2; end
            S_C2: begin
                dvx <= c_q[15:0]; dvy <= c_q[31:16];         // 字 0
                st <= S_C3;
            end
            S_C3: begin
                dth <= c_q[15:0];                             // 字 1
                x <= x0; y <= y0; th <= th00;
                mxc <= 0; nun <= 0; cstep <= 0; coll <= 0; left <= 0; csum <= 0; t <= 0;
                st <= S_B;
            end
            S_B: begin
                if (t == T) st <= S_W0;
                else begin
                    th <= th_n;
                    si <= th_n[15:6];
                    ci <= th_n[15:6] + 10'd256;
                    lut_re <= 1'b1;
                    t <= t + 1'b1;
                    st <= S_C;
                end
            end
            S_C: st <= S_D;                                   // 等 sin/cos
            S_D: begin
                p_xc <= dvx * $signed(cq);
                p_ys <= dvy * $signed(sq);
                p_xs <= dvx * $signed(sq);
                p_yc <= dvy * $signed(cq);
                st <= S_E;
            end
            S_E: begin
                x <= x + ddx_s[31:0];
                y <= y + ddy_s[31:0];
                st <= S_F0;
            end
            S_F0: begin
                wx <= ix - (cx - N/2);
                wy <= iy - (cy - N/2);
                slot <= {iy[LOG2N-1:0], ix[LOG2N-1:0]};
                st <= S_F;
            end
            S_F: begin
                if (!inwin) begin
                    left <= 1'b1;
                    if (nun != 8'hFF) nun <= nun + 1'b1;
                    st <= S_B;
                end else begin
                    cm_re   <= 1'b1;
                    cm_addr <= slot;
                    st <= S_Q1;
                end
            end
            S_Q1: st <= S_Q3;                                 // cm_q 在 S_Q3 有效
            S_Q3: begin cq_r <= cm_q; st <= S_Q2; end
            S_Q2: begin
                if (cq_r == 8'hFF) begin
                    if (nun != 8'hFF) nun <= nun + 1'b1;
                    st <= S_B;
                end else if (cq_r >= cmax) begin
                    coll <= 1'b1; cstep <= t;
                    st <= S_W0;
                end else begin
                    csum <= csum + cq_r;
                    if (cq_r > mxc) mxc <= cq_r;
                    st <= S_B;
                end
            end
            S_W0: begin
                r_we <= 1'b1; r_wa <= {k[KMAX_LOG-1:0], 2'd0};
                r_wd <= {6'd0, left, coll, cstep, nun, mxc};
                st <= S_W1;
            end
            S_W1: begin r_we <= 1'b1; r_wa <= {k[KMAX_LOG-1:0], 2'd1}; r_wd <= csum; st <= S_W2; end
            S_W2: begin r_we <= 1'b1; r_wa <= {k[KMAX_LOG-1:0], 2'd2}; r_wd <= x;    st <= S_W3; end
            S_W3: begin
                r_we <= 1'b1; r_wa <= {k[KMAX_LOG-1:0], 2'd3}; r_wd <= y;
                k <= k + 1'b1;
                st <= (k + 1'b1 == K) ? S_DONE : S_C0;
            end
            S_DONE: begin
                busy <= 1'b0; done <= 1'b1;
                stale <= (map_seq != seq0);
                st <= S_IDLE;
            end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
