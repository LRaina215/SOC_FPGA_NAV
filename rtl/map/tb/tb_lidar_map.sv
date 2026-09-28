// 点云链路 + 地图子系统联合测试：
//   像素流 -> pc_project -> pc_transform -> pc_filter -> map_subsys
//   检查 1：pc_filter 输出与定点模型逐项一致
//   检查 2：每帧扫描后的前台代价图与定点模型逐格一致
`timescale 1ns/1ps
module tb_lidar_map;
    logic clk = 0, rst = 1;
    always #5 clk = ~clk;

    localparam int NCELL = 128 * 128;
    logic [33:0] pix  [0:9600*8-1];
    logic [31:0] pose [0:14*8-1];
    logic [39:0] fexp [0:9600*8-1];
    logic [7:0]  cexp [0:NCELL*8-1];
    logic [15:0] mcfg [0:18];
    int n_pix, n_flt, n_frm;

    logic [143:0] cfg_r;
    logic [95:0]  cfg_t;
    logic signed [31:0] cfg_cx, cfg_cy;

    // 像素源
    logic        p_valid, p_ready;
    logic [33:0] p_cur;
    // pc_project -> pc_transform
    logic        a_valid, a_ready, a_pv, a_sof, a_eof;
    logic signed [15:0] a_x, a_y, a_z;
    // pc_transform -> pc_filter
    logic        b_valid, b_ready, b_pv, b_sof, b_eof;
    logic signed [31:0] b_x, b_y, b_z;
    // pc_filter -> map
    logic        f_valid, f_ready, f_upd, f_sof, f_eof;
    logic [13:0] f_addr;
    logic [7:0]  f_tag;
    logic signed [15:0] f_z;
    logic signed [31:0] f_cx, f_cy;

    pc_project #(.LUT_INIT("lut.hex")) u_pp (
        .clk(clk), .rst(rst), .cfg_depth_mul(16'd256), .cfg_min_mm(16'd100), .cfg_max_mm(16'd20000),
        .cfg_roi_x0(8'd0), .cfg_roi_y0(8'd0), .lut_we(1'b0), .lut_waddr(14'd0), .lut_wdata(48'd0),
        .s_valid(p_valid), .s_ready(p_ready), .s_u(p_cur[31:24]), .s_v(p_cur[23:16]), .s_depth(p_cur[15:0]),
        .s_sof(p_cur[33]), .s_eof(p_cur[32]),
        .m_valid(a_valid), .m_ready(a_ready), .m_x(a_x), .m_y(a_y), .m_z(a_z), .m_pv(a_pv),
        .m_sof(a_sof), .m_eof(a_eof));

    pc_transform u_pt (
        .clk(clk), .rst(rst), .cfg_r(cfg_r), .cfg_t(cfg_t),
        .s_valid(a_valid), .s_ready(a_ready), .s_x(a_x), .s_y(a_y), .s_z(a_z), .s_pv(a_pv),
        .s_sof(a_sof), .s_eof(a_eof),
        .m_valid(b_valid), .m_ready(b_ready), .m_x(b_x), .m_y(b_y), .m_z(b_z), .m_pv(b_pv),
        .m_sof(b_sof), .m_eof(b_eof));

    pc_filter u_pf (
        .clk(clk), .rst(rst), .cfg_inv_res(mcfg[16]), .cfg_cx(cfg_cx), .cfg_cy(cfg_cy),
        .cfg_zmin(mcfg[17]), .cfg_zmax(mcfg[18]),
        .s_valid(b_valid), .s_ready(b_ready), .s_x(b_x), .s_y(b_y), .s_z(b_z), .s_pv(b_pv),
        .s_sof(b_sof), .s_eof(b_eof),
        .m_valid(f_valid), .m_ready(f_ready), .m_upd(f_upd), .m_addr(f_addr), .m_tag(f_tag), .m_z(f_z),
        .m_sof(f_sof), .m_eof(f_eof), .m_cx(f_cx), .m_cy(f_cy), .stat_upd(), .stat_drop());

    logic        a_re;
    logic [13:0] a_addr;
    logic [7:0]  a_q;
    logic        map_ready, busy;
    logic [15:0] seq;
    map_subsys u_map (
        .clk(clk), .rst(rst), .cfg_enable(1'b1), .cfg_auto_sweep(1'b1), .cmd_sweep(1'b0), .cmd_clear(1'b0),
        .cfg_cx(cfg_cx), .cfg_cy(cfg_cy),
        .cfg_age_max(mcfg[0][7:0]), .cfg_cnt_min(mcfg[1][3:0]), .cfg_keep_r(mcfg[2][3:0]),
        .cfg_step_max(mcfg[3]), .cfg_rough_max(mcfg[4]), .cfg_slope_rise(mcfg[5]), .cfg_jump(mcfg[6][7:0]),
        .cfg_drop_k(mcfg[7][4:0]), .cfg_drop_slope(mcfg[8][7:0]),
        .cfg_k_slope(mcfg[9][7:0]), .cfg_k_step(mcfg[10][7:0]), .cfg_k_rough(mcfg[11][7:0]),
        .cfg_fill_min(mcfg[12][3:0]), .cfg_fill_tol(mcfg[13][7:0]), .cfg_fill_cost(mcfg[14][7:0]),
        .cfg_infl_r(mcfg[15][3:0]),
        .s_valid(f_valid), .s_ready(f_ready), .s_upd(f_upd), .s_addr(f_addr), .s_tag(f_tag), .s_z(f_z),
        .s_eof(f_eof), .s_cx(f_cx), .s_cy(f_cy),
        .a_re(a_re), .a_addr(a_addr), .a_q(a_q), .b_req(1'b0), .b_addr(14'd0), .b_ack(), .b_q(),
        .busy(busy), .front(), .seq(seq), .lat_cx(), .lat_cy(), .frame_cnt(), .map_ready(map_ready));

    int errs = 0, fi = 0, pi = 0, frame = 0;
    int pend_frames = 0;   // 已发送、尚未扫描完成的帧

    // 滤波输出检查（旁路监视）
    always @(posedge clk) if (!rst && f_valid && f_ready) begin
        if ({f_upd, f_eof, f_addr, f_tag, f_z} !== fexp[fi]) begin
            errs++;
            if (errs < 10) $display("filter #%0d 期望 %h 实得 %h", fi, fexp[fi], {f_upd, f_eof, f_addr, f_tag, f_z});
        end
        fi++;
    end

    // 像素源：按帧发送
    logic sending = 0;
    always @(posedge clk) begin
        if (rst) p_valid <= 0;
        else begin
            if (p_valid && p_ready) begin
                p_valid <= 0;
                if (p_cur[32]) sending = 0;
            end
            if (sending && (!p_valid || p_ready) && ($urandom % 4 != 0)) begin
                p_cur   <= pix[pi];
                p_valid <= 1;
                pi++;
            end
        end
    end

    task automatic check_cost(int k);
        int bad = 0;
        a_re <= 1;
        for (int i = 0; i <= NCELL; i++) begin
            if (i < NCELL) a_addr <= i[13:0];
            else a_re <= 0;
            @(posedge clk);
            #1;
            if (i < NCELL && a_q !== cexp[k * NCELL + i]) begin
                bad++;
                if (bad < 6) $display("帧%0d 代价 槽%0d 期望 %0d 实得 %0d", k, i, cexp[k * NCELL + i], a_q);
            end
        end
        a_re <= 0;
        if (bad) begin errs += bad; $display("帧%0d 代价图不一致 %0d 格", k, bad); end
        else $display("帧%0d 代价图一致", k);
    endtask

    initial begin
        int fd;
        a_re = 0; a_addr = 0;
        fd = $fopen("counts.txt", "r");
        void'($fscanf(fd, "%d %d %d", n_pix, n_flt, n_frm));
        $fclose(fd);
        $readmemh("pix.hex", pix);
        $readmemh("pose.hex", pose);
        $readmemh("flt_exp.hex", fexp);
        $readmemh("cost_exp.hex", cexp);
        $readmemh("mapcfg.hex", mcfg);
        repeat (20) @(posedge clk);
        rst = 0;
        wait (!busy);    // 上电清除
        for (int k = 0; k < n_frm; k++) begin
            for (int j = 0; j < 9; j++) cfg_r[16*j +: 16] = pose[k*14 + j][15:0];
            for (int j = 0; j < 3; j++) cfg_t[32*j +: 32] = pose[k*14 + 9 + j];
            cfg_cx = pose[k*14 + 12];
            cfg_cy = pose[k*14 + 13];
            @(posedge clk);
            sending = 1;
            @(posedge map_ready);
            @(posedge clk);
            check_cost(k);
        end
        repeat (10) @(posedge clk);
        $display("filter 输出 %0d/%0d", fi, n_flt);
        if (errs == 0 && fi == n_flt) $display("TEST PASSED");
        else $display("TEST FAILED errs=%0d", errs);
        $finish;
    end

    initial begin
        #200ms;
        $display("TEST FAILED 超时");
        $finish;
    end
endmodule
