// SLF 匹配硬件测试：逐帧 点流 -> 结构点数 / 245 候选打分结果 / 并图后整张结构图，与模型逐位比较。
`timescale 1ns/1ps
module tb_slf;
    logic clk = 0, rst = 1;
    always #5 clk = ~clk;

    localparam int NPIX = 9600, NSLOT = 65536, MAXF = 8;
    logic [31:0] pts [0:NPIX*4*MAXF-1];
    logic [31:0] frm [0:12*MAXF-1];
    logic [31:0] cand [0:511];
    logic [31:0] rexp [0:512*MAXF-1];
    logic [23:0] smx [0:NSLOT*MAXF-1];
    logic [23:0] smm [0:NSLOT*MAXF-1];
    int n_frm, n_cand;

    logic        s_fire = 0, s_pv = 0, s_sof = 0, s_eof = 0;
    logic signed [31:0] s_x = 0, s_y = 0, s_z = 0;
    logic signed [31:0] zlo, zhi, tx, ty, cxm, cym, lcx, lcy, udx, udy;
    logic [15:0] udyaw;
    logic        cmd_eval = 0, cmd_update = 0;
    logic        cand_we = 0; logic [8:0] cand_wa; logic [31:0] cand_wd;
    logic        res_re = 0; logic [8:0] res_ra; logic [31:0] res_q;
    logic        dbg_req = 0; logic [15:0] dbg_addr; logic dbg_ack; logic [22:0] dbg_q;
    logic        busy, pts_ready, eval_done;
    logic [12:0] n_pts;

    slf_match #(.SIN_INIT("sin_lut.hex")) dut (
        .clk(clk), .rst(rst),
        .s_fire(s_fire), .s_x(s_x), .s_y(s_y), .s_z(s_z), .s_pv(s_pv), .s_sof(s_sof), .s_eof(s_eof),
        .frame_en(1'b1), .cfg_zlo(zlo), .cfg_zhi(zhi), .cfg_tx(tx), .cfg_ty(ty), .cfg_inv_res(16'd1311),
        .cmd_eval(cmd_eval), .cmd_update(cmd_update), .cmd_clear(1'b0),
        .cfg_cx_mm(cxm), .cfg_cy_mm(cym), .cfg_ncand(n_cand[8:0]),
        .upd_dx(udx), .upd_dy(udy), .upd_dyaw(udyaw), .lf_cx(lcx), .lf_cy(lcy),
        .cand_we(cand_we), .cand_waddr(cand_wa), .cand_wdata(cand_wd),
        .res_re(res_re), .res_raddr(res_ra), .res_q(res_q),
        .dbg_req(dbg_req), .dbg_addr(dbg_addr), .dbg_ack(dbg_ack), .dbg_q(dbg_q),
        .busy(busy), .pts_ready(pts_ready), .n_pts(n_pts), .eval_done(eval_done));

    int errs = 0;
    initial begin
        int fd;
        fd = $fopen("counts.txt", "r");
        void'($fscanf(fd, "%d %d", n_frm, n_cand));
        $fclose(fd);
        $readmemh("pts.hex", pts);
        $readmemh("frm.hex", frm);
        $readmemh("cand.hex", cand);
        $readmemh("res.hex", rexp);
        $readmemh("sm.hex", smx);
        $readmemh("smm.hex", smm);
        repeat (20) @(posedge clk);
        rst = 0;
        repeat (10) @(posedge clk);
        wait (!busy);
        // 候选（各帧相同）
        for (int i = 0; i < 2 * n_cand; i++) begin
            @(posedge clk); cand_we <= 1; cand_wa <= i[8:0]; cand_wd <= cand[i];
        end
        @(posedge clk); cand_we <= 0;
        for (int k = 0; k < n_frm; k++) begin
            {zlo, zhi, tx, ty, cxm, cym, lcx, lcy, udx, udy} = {frm[k*12+0], frm[k*12+1], frm[k*12+2], frm[k*12+3],
                frm[k*12+4], frm[k*12+5], frm[k*12+6], frm[k*12+7], frm[k*12+8], frm[k*12+9]};
            udyaw = frm[k*12+10][15:0];
            // 点流（随机间隔）
            for (int i = 0; i < NPIX; i++) begin
                @(posedge clk);
                while ($urandom % 3 == 0) begin s_fire <= 0; @(posedge clk); end
                s_fire <= 1;
                s_x <= pts[(k*NPIX + i)*4]; s_y <= pts[(k*NPIX + i)*4 + 1]; s_z <= pts[(k*NPIX + i)*4 + 2];
                {s_eof, s_sof, s_pv} <= pts[(k*NPIX + i)*4 + 3][2:0];
            end
            @(posedge clk); s_fire <= 0;
            wait (pts_ready);
            @(posedge clk);
            if (n_pts !== frm[k*12+11][12:0]) begin errs++; $display("帧%0d 结构点数 期望 %0d 实得 %0d", k, frm[k*12+11], n_pts); end
            // 打分
            @(posedge clk); cmd_eval <= 1; @(posedge clk); cmd_eval <= 0;
            wait (eval_done);
            begin
                int bad = 0;
                for (int i = 0; i < 2 * n_cand; i++) begin
                    @(posedge clk); res_re <= 1; res_ra <= i[8:0];
                    @(posedge clk); res_re <= 0;
                    #1;
                    if (res_q !== rexp[k*512 + i]) begin
                        bad++;
                        if (bad < 6) $display("帧%0d 候选%0d 字%0d 期望 %h 实得 %h", k, i / 2, i % 2, rexp[k*512 + i], res_q);
                    end
                end
                if (bad) begin errs += bad; $display("帧%0d 打分不一致 %0d", k, bad); end
                else $display("帧%0d 打分一致（%0d 候选）", k, n_cand);
            end
            // 并图 + 似然场
            @(posedge clk); cmd_update <= 1; @(posedge clk); cmd_update <= 0;
            repeat (5) @(posedge clk);
            wait (!busy);
            begin
                int bad = 0;
                for (int a = 0; a < NSLOT; a++) begin
                    dbg_req <= 1; dbg_addr <= a[15:0];
                    do begin @(posedge clk); #1; end while (!dbg_ack);
                    dbg_req <= 0;
                    if ((({1'b0, dbg_q} ^ smx[k*NSLOT + a]) & smm[k*NSLOT + a]) != 0) begin
                        bad++;
                        if (bad < 6) $display("帧%0d 槽%0d 期望 %h 实得 %h（掩码 %h）", k, a, smx[k*NSLOT + a], dbg_q, smm[k*NSLOT + a]);
                    end
                end
                if (bad) begin errs += bad; $display("帧%0d 结构图不一致 %0d 槽", k, bad); end
                else $display("帧%0d 结构图一致", k);
            end
        end
        if (errs == 0) $display("TEST PASSED");
        else $display("TEST FAILED errs=%0d", errs);
        $finish;
    end
endmodule
