// traj_eval 测试：行为级代价图（1 拍读延迟）+ 多组候选，结果与定点模型逐字比对。
`timescale 1ns/1ps
module tb_traj_eval;
    logic clk = 0, rst = 1;
    always #5 clk = ~clk;

    logic [7:0]  cmem [0:16383];
    logic [31:0] runs [0:4095];
    int n_runs;

    logic        start, busy, done, stale;
    logic [7:0]  cfg_k, cfg_t, cfg_cmax;
    logic signed [31:0] cfg_x0, cfg_y0, map_cx, map_cy;
    logic [15:0] cfg_th0;
    logic        cand_we, res_re;
    logic [6:0]  cand_addr;
    logic [7:0]  res_addr;
    logic [31:0] cand_wdata, res_rdata;
    logic        cm_re;
    logic [13:0] cm_addr;
    logic [7:0]  cm_q;

    always @(posedge clk) if (cm_re) cm_q <= cmem[cm_addr];

    traj_eval dut (
        .clk(clk), .rst(rst), .start(start), .cfg_k(cfg_k), .cfg_t(cfg_t), .cfg_x0(cfg_x0), .cfg_y0(cfg_y0),
        .cfg_th0(cfg_th0), .cfg_cmax(cfg_cmax), .map_cx(map_cx), .map_cy(map_cy), .map_seq(16'd0),
        .busy(busy), .done(done), .stale(stale),
        .cand_we(cand_we), .cand_addr(cand_addr), .cand_wdata(cand_wdata),
        .res_re(res_re), .res_addr(res_addr), .res_rdata(res_rdata),
        .cm_re(cm_re), .cm_addr(cm_addr), .cm_q(cm_q));

    int errs = 0;
    initial begin
        int fd, p, K;
        start = 0; cand_we = 0; res_re = 0;
        fd = $fopen("counts.txt", "r");
        void'($fscanf(fd, "%d", n_runs));
        $fclose(fd);
        $readmemh("cost.hex", cmem);
        $readmemh("runs.hex", runs);
        repeat (5) @(posedge clk);
        rst = 0;
        p = 0;
        for (int r = 0; r < n_runs; r++) begin
            map_cx = runs[p]; map_cy = runs[p+1]; cfg_x0 = runs[p+2]; cfg_y0 = runs[p+3];
            cfg_th0 = runs[p+4]; cfg_t = runs[p+5]; K = runs[p+6]; cfg_k = K; cfg_cmax = runs[p+7];
            p += 8;
            for (int i = 0; i < 2 * K; i++) begin
                @(posedge clk);
                cand_we <= 1; cand_addr <= i; cand_wdata <= runs[p + i];
            end
            @(posedge clk); cand_we <= 0;
            p += 2 * K;
            @(posedge clk); start <= 1;
            @(posedge clk); start <= 0;
            @(posedge done);
            for (int i = 0; i < 4 * K; i++) begin
                @(posedge clk); res_re <= 1; res_addr <= i;
                @(posedge clk); res_re <= 0;
                @(posedge clk); #1;
                if (res_rdata !== runs[p + i]) begin
                    errs++;
                    if (errs < 10) $display("run%0d 候选%0d 字%0d 期望 %h 实得 %h", r, i / 4, i % 4, runs[p + i], res_rdata);
                end
            end
            p += 4 * K;
            $display("run%0d 完成", r);
        end
        if (errs == 0) $display("TEST PASSED");
        else $display("TEST FAILED errs=%0d", errs);
        $finish;
    end
    initial begin #100ms; $display("TEST FAILED 超时"); $finish; end
endmodule
