// 地图子系统：滚动高程图 + 可通行性分析 + 双缓冲代价图。
//   点更新 -> 帧尾（AUTO_SWEEP）-> 扫描生成后台代价图 -> 交换前后台，seq 加 1。
//   扫描期间阻塞点输入（上游帧缓冲吸收，约 0.35 ms/帧）。
// 前台代价图读口：
//   A 口（轨迹评估，组合地址，下一拍出数，始终优先）
//   B 口（软核总线，req 保持到 ack；A 口空闲时服务）
module map_subsys #(
    parameter integer LOG2N = 7,
    parameter integer TAGB  = 4
) (
    input  wire        clk,
    input  wire        rst,

    input  wire        cfg_enable,
    input  wire        cfg_auto_sweep,
    input  wire        cmd_sweep,
    input  wire        cmd_clear,
    input  wire signed [31:0] cfg_cx,
    input  wire signed [31:0] cfg_cy,
    input  wire [7:0]  cfg_age_max,
    input  wire [3:0]  cfg_cnt_min,
    input  wire [3:0]  cfg_keep_r,
    input  wire [15:0] cfg_step_max,
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
    input  wire        cfg_infl_disk,

    // 点更新流（来自 pc_filter）
    input  wire        s_valid,
    output wire        s_ready,
    input  wire        s_upd,
    input  wire [2*LOG2N-1:0] s_addr,
    input  wire [2*TAGB-1:0]  s_tag,
    input  wire signed [15:0] s_z,
    input  wire        s_eof,
    input  wire signed [31:0] s_cx,
    input  wire signed [31:0] s_cy,

    // 前台代价图读口 A
    input  wire        a_re,
    input  wire [2*LOG2N-1:0] a_addr,
    output wire [7:0]  a_q,
    // 读口 B
    input  wire        b_req,
    input  wire [2*LOG2N-1:0] b_addr,
    output reg         b_ack,
    output reg  [7:0]  b_q,
    // 高程图读口（软核）：req 保持到 ack
    input  wire        e_req,
    input  wire [2*LOG2N-1:0] e_addr,
    output wire        e_ack,
    output wire [51:0] e_q,

    output wire        busy,
    output reg         front,
    output reg  [15:0] seq,
    output reg  signed [31:0] lat_cx,
    output reg  signed [31:0] lat_cy,
    output wire [7:0]  frame_cnt,
    output reg         map_ready        // 单拍：新代价图已切到前台
);
    localparam integer AW = 2 * LOG2N;

    wire        swp_busy;
    wire        em_re, em_we, cm_we, cm_re;
    wire [AW-1:0] em_raddr, em_waddr, cm_waddr, cm_raddr;
    wire [51:0] em_q, em_wdata;
    wire [7:0]  cm_wdata;
    wire        frame_done, em_busy;
    reg         sweep_pend;
    reg  signed [31:0] sw_cx, sw_cy;
    reg         tv_start;
    wire        tv_done;
    wire        em_s_ready;
    wire [7:0]  cm_q_back;
    reg         started;
    wire        eof_acc = s_valid & s_ready & s_eof & cfg_enable;

    elev_map #(.LOG2N(LOG2N), .TAGB(TAGB)) u_em (
        .clk(clk), .rst(rst), .clear(cmd_clear),
        .s_valid(s_valid & cfg_enable & ~sweep_pend), .s_ready(em_s_ready),
        .s_upd(s_upd), .s_addr(s_addr), .s_tag(s_tag), .s_z(s_z), .s_eof(s_eof),
        .fc(frame_cnt), .frame_done(frame_done), .busy(em_busy),
        .swp_sel(swp_busy), .swp_re(em_re), .swp_raddr(em_raddr), .swp_rdata(em_q),
        .swp_we(em_we), .swp_waddr(em_waddr), .swp_wdata(em_wdata),
        .cpu_req(e_req), .cpu_raddr(e_addr), .cpu_ack(e_ack)
    );
    assign e_q = em_q;
    assign s_ready = (em_s_ready & ~sweep_pend) | ~cfg_enable;   // 关闭时丢弃输入

    trav_v3 #(.LOG2N(LOG2N), .TAGB(TAGB)) u_tv (
        .clk(clk), .rst(rst), .start(tv_start), .cx(sw_cx), .cy(sw_cy), .fc(frame_cnt),
        .cfg_age_max(cfg_age_max), .cfg_cnt_min(cfg_cnt_min), .cfg_keep_r(cfg_keep_r),
        .cfg_step_max(cfg_step_max), .cfg_rough_max(cfg_rough_max), .cfg_slope_rise(cfg_slope_rise),
        .cfg_jump(cfg_jump), .cfg_drop_k(cfg_drop_k), .cfg_drop_slope(cfg_drop_slope),
        .cfg_k_slope(cfg_k_slope), .cfg_k_step(cfg_k_step), .cfg_k_rough(cfg_k_rough),
        .cfg_fill_min(cfg_fill_min), .cfg_fill_tol(cfg_fill_tol), .cfg_fill_cost(cfg_fill_cost),
        .cfg_infl_r(cfg_infl_r), .cfg_infl_disk(cfg_infl_disk),
        .busy(swp_busy), .done(tv_done),
        .em_re(em_re), .em_raddr(em_raddr), .em_q(em_q), .em_we(em_we), .em_waddr(em_waddr), .em_wdata(em_wdata),
        .cm_we(cm_we), .cm_waddr(cm_waddr), .cm_wdata(cm_wdata), .cm_re(cm_re), .cm_raddr(cm_raddr), .cm_q(cm_q_back)
    );

    // 双缓冲代价图
    reg        b_pend;
    // 一次只服务一个请求：在途（b_pend）或刚应答（b_ack）时不再发读
    wire       b_serve = b_req & ~b_ack & ~b_pend & ~a_re;
    wire [7:0] q0, q1;
    wire       re0 = front ? cm_re : (a_re | b_serve);
    wire       re1 = front ? (a_re | b_serve) : cm_re;
    wire [AW-1:0] fa = a_re ? a_addr : b_addr;
    nav_ram_sdp #(.DW(8), .AW(AW), .INIT_VAL(8'hFF)) u_cm0 (
        .clk(clk), .we(cm_we & front), .waddr(cm_waddr), .wdata(cm_wdata),
        .re(re0), .raddr(front ? cm_raddr : fa), .rdata(q0)
    );
    nav_ram_sdp #(.DW(8), .AW(AW), .INIT_VAL(8'hFF)) u_cm1 (
        .clk(clk), .we(cm_we & ~front), .waddr(cm_waddr), .wdata(cm_wdata),
        .re(re1), .raddr(front ? fa : cm_raddr), .rdata(q1)
    );
    // front=0：前台为 cm0，后台 cm1
    assign cm_q_back = front ? q0 : q1;
    assign a_q       = front ? q1 : q0;

    always @(posedge clk) begin
        b_ack <= 1'b0;
        if (rst) b_pend <= 1'b0;
        else begin
            b_pend <= b_serve;
            if (b_pend) begin b_ack <= 1'b1; b_q <= a_q; end
        end
    end

    assign busy = swp_busy | sweep_pend | em_busy;

    always @(posedge clk) begin
        tv_start  <= 1'b0;
        map_ready <= 1'b0;
        if (rst) begin
            sweep_pend <= 1'b0; front <= 1'b0; seq <= 0; lat_cx <= 0; lat_cy <= 0;
        end else begin
            if (!sweep_pend) begin
                // 帧尾被接收的同一拍即停止接收，保证扫描看到的正好是完整的这一帧
                if (eof_acc && cfg_auto_sweep) begin
                    sweep_pend <= 1'b1; sw_cx <= s_cx; sw_cy <= s_cy;
                end else if (cmd_sweep) begin
                    sweep_pend <= 1'b1; sw_cx <= cfg_cx; sw_cy <= cfg_cy;
                end
            end else if (!swp_busy && !em_busy && !tv_start && !started) begin
                tv_start <= 1'b1;
            end
            if (tv_done) begin
                sweep_pend <= 1'b0;
                front  <= ~front;
                seq    <= seq + 1'b1;
                lat_cx <= sw_cx;
                lat_cy <= sw_cy;
                map_ready <= 1'b1;
            end
        end
    end
    always @(posedge clk) begin
        if (rst || tv_done) started <= 1'b0;
        else if (tv_start) started <= 1'b1;
    end
endmodule
