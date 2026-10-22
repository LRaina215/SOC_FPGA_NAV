// xt_frame_rx 测试：按 100 Mbps 线速（每 8 拍 1 字节）注入 UDP 负载，随机反压读出像素，逐项比对；
// 结束时核对帧数 / 失败帧数 / 跳过分包数与最近一帧 FrameInfo 锁存值（期望来自 model/xt_proto.py RxModel）。
`timescale 1ns/1ps
module tb_xt_frame_rx;
    logic clk = 0, rst = 1;
    always #5 clk = ~clk;

    localparam int MAXS = 1 << 19;
    logic [9:0]  stim [0:MAXS-1];
    logic [33:0] expv [0:MAXS-1];
    logic [15:0] cfg  [0:4];
    logic [31:0] chk  [0:8];
    int n_stim, n_exp;

    logic [31:0] time_us = 0;
    always @(posedge clk) if ($time % 1000 == 0) time_us <= time_us + 1;

    logic s_valid, s_last, s_err;
    logic [7:0] s_data;
    logic m_valid, m_ready, m_sof, m_eof;
    logic [7:0] m_u, m_v;
    logic [15:0] m_depth;
    logic [31:0] st_frames, st_bad, st_drop, st_skip;
    logic [287:0] inf_lens;
    logic [31:0] inf_cfg, inf_size, inf_temp, inf_seq;

    xt_frame_rx dut (
        .clk(clk), .rst(rst), .time_us(time_us),
        .cfg_enable(1'b1), .cfg_hdr_le(cfg[0][0]), .cfg_pix_le(cfg[1][0]), .cfg_subhdr(cfg[2][7:0]),
        .cfg_amp_min(cfg[3]), .cfg_raw_inv(cfg[4]), .cfg_wait_pose(1'b0), .pose_commit(1'b0), .cfg_retain(1'b0),
        .s_valid(s_valid), .s_data(s_data), .s_last(s_last), .s_err(s_err),
        .m_valid(m_valid), .m_ready(m_ready), .m_u(m_u), .m_v(m_v), .m_depth(m_depth),
        .m_sof(m_sof), .m_eof(m_eof),
        .frm_sn(), .frm_w(), .frm_h(), .frm_ts(), .frm_devstate(), .frm_proto(), .frm_iok(), .frm_unit(),
        .frm_bin(), .frm_roix(), .frm_roiy(), .pose_waiting(), .frame_start(), .frame_ready(),
        .inf_lens(inf_lens), .inf_cfg(inf_cfg), .inf_size(inf_size), .inf_temp(inf_temp), .inf_seq(inf_seq),
        .stat_frames(st_frames), .stat_bad(st_bad), .stat_drop(st_drop), .stat_skip(st_skip));

    int si = 0, ei = 0, errs = 0, gap = 0;

    always @(posedge clk) begin
        if (rst) begin
            s_valid <= 0; m_ready <= 0;
        end else begin
            // 输入：每 8 拍一个字节
            s_valid <= 0;
            if (gap == 7 && si < n_stim) begin
                {s_err, s_last, s_data} <= stim[si];
                s_valid <= 1;
                si <= si + 1;
                gap <= 0;
            end else if (gap < 7) gap <= gap + 1;
            m_ready <= ($urandom % 2);
            if (m_valid && m_ready) begin
                if (ei >= n_exp) begin
                    errs++; if (errs < 10) $display("多余输出 u=%0d v=%0d", m_u, m_v);
                end else if ({m_sof, m_eof, m_u, m_v, m_depth} !== expv[ei]) begin
                    errs++;
                    if (errs < 10) $display("#%0d 期望 %h 实得 %h", ei, expv[ei], {m_sof, m_eof, m_u, m_v, m_depth});
                end
                ei <= ei + 1;
            end
        end
    end

    task automatic cmp(string what, input [31:0] got, input [31:0] exp);
        if (got !== exp) begin errs++; $display("%s: 期望 %h 实得 %h", what, exp, got); end
    endtask

    initial begin
        int fd;
        fd = $fopen("counts.txt", "r");
        void'($fscanf(fd, "%d %d", n_stim, n_exp));
        $fclose(fd);
        $readmemh("stim.hex", stim);
        $readmemh("exp.hex", expv);
        $readmemh("cfg.hex", cfg);
        $readmemh("chk.hex", chk);
        repeat (10) @(posedge clk);
        rst = 0;
        wait (si == n_stim);
        repeat (40000) @(posedge clk);
        $display("frames=%0d bad=%0d drop=%0d skip=%0d info=%0d out=%0d/%0d", st_frames, st_bad, st_drop, st_skip,
                 inf_seq, ei, n_exp);
        cmp("STAT_FRAMES", st_frames, chk[0]);
        cmp("STAT_BAD", st_bad, chk[1]);
        cmp("STAT_SKIP", st_skip, chk[2]);
        cmp("STAT_DROP", st_drop, 0);
        cmp("INFO_SEQ", inf_seq, chk[3]);
        if (chk[3] != 0) begin
            cmp("LENS0", inf_lens[31:0], chk[4]);
            cmp("LENS8", inf_lens[287:256], chk[5]);
            cmp("INFO_CFG", inf_cfg, chk[6]);
            cmp("INFO_SIZE", inf_size, chk[7]);
            cmp("INFO_TEMP", inf_temp, chk[8]);
        end
        if (errs == 0 && ei == n_exp) $display("TEST PASSED");
        else $display("TEST FAILED errs=%0d", errs);
        $finish;
    end
endmodule
