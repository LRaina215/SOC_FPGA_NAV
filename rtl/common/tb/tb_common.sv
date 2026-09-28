// 公共库随机测试：同步 FIFO、异步 FIFO、skid 在随机 valid/ready 下保序不丢不重。
`timescale 1ns/1ps
module tb_common;
    localparam int N = 20000;
    logic clk = 0, clk2 = 0, rst = 1;
    always #5    clk  = ~clk;    // 100 MHz
    always #3.9  clk2 = ~clk2;   // ~128 MHz，与 clk 异步

    // ---------- 同步 FIFO ----------
    logic        f_sv, f_sr, f_mv, f_mr;
    logic [15:0] f_sd, f_md;
    nav_fifo #(.DW(16), .AW(4)) u_f (
        .clk(clk), .rst(rst), .s_valid(f_sv), .s_data(f_sd), .s_ready(f_sr),
        .m_valid(f_mv), .m_data(f_md), .m_ready(f_mr), .level(), .free());

    // ---------- skid ----------
    logic        k_sv, k_sr, k_mv, k_mr;
    logic [15:0] k_sd, k_md;
    nav_skid #(.DW(16)) u_k (
        .clk(clk), .rst(rst), .s_valid(k_sv), .s_data(k_sd), .s_ready(k_sr),
        .m_valid(k_mv), .m_data(k_md), .m_ready(k_mr));

    // ---------- 异步 FIFO：clk2 写，clk 读 ----------
    logic        a_sv, a_sr, a_mv, a_mr, rst2;
    logic [15:0] a_sd, a_md;
    nav_rst_sync u_rs (.clk(clk2), .arst(rst), .rst(rst2));
    nav_async_fifo #(.DW(16), .AW(4)) u_a (
        .wclk(clk2), .wrst(rst2), .s_valid(a_sv), .s_data(a_sd), .s_ready(a_sr), .wfree(),
        .rclk(clk), .rrst(rst), .m_valid(a_mv), .m_data(a_md), .m_ready(a_mr));

    int f_tx = 0, f_rx = 0, k_tx = 0, k_rx = 0, a_tx = 0, a_rx = 0, errs = 0;

    // 同步域驱动与检查
    always @(posedge clk) begin
        if (rst) begin
            f_sv <= 0; f_mr <= 0; k_sv <= 0; k_mr <= 0; a_mr <= 0;
            f_sd <= 0; k_sd <= 0;
        end else begin
            if (f_sv && f_sr) begin f_tx++; f_sd <= f_sd + 1; end
            if (f_mv && f_mr) begin
                if (f_md !== f_rx[15:0]) begin errs++; $display("fifo mismatch %0d vs %0d", f_md, f_rx); end
                f_rx++;
            end
            f_sv <= (f_tx + (f_sv && f_sr) < N) && ($urandom % 4 != 0);
            f_mr <= ($urandom % 3 != 0);

            if (k_sv && k_sr) begin k_tx++; k_sd <= k_sd + 1; end
            if (k_mv && k_mr) begin
                if (k_md !== k_rx[15:0]) begin errs++; $display("skid mismatch %0d vs %0d", k_md, k_rx); end
                k_rx++;
            end
            // 按 valid/ready 协议：valid 拉高后在握手前保持
            if (!k_sv || k_sr) k_sv <= (k_tx + (k_sv && k_sr) < N) && ($urandom % 3 != 0);
            k_mr <= ($urandom % 2 == 0);

            if (a_mv && a_mr) begin
                if (a_md !== a_rx[15:0]) begin errs++; $display("afifo mismatch %0d vs %0d", a_md, a_rx); end
                a_rx++;
            end
            a_mr <= ($urandom % 3 != 0);
        end
    end

    // 异步写域
    always @(posedge clk2) begin
        if (rst2) begin
            a_sv <= 0; a_sd <= 0;
        end else begin
            if (a_sv && a_sr) begin a_tx++; a_sd <= a_sd + 1; end
            if (!a_sv || a_sr) a_sv <= (a_tx + (a_sv && a_sr) < N) && ($urandom % 2 == 0);
        end
    end

    initial begin
        repeat (10) @(posedge clk);
        rst = 0;
        fork
            wait (f_rx == N && k_rx == N && a_rx == N);
            begin #5ms; $display("超时 f=%0d k=%0d a=%0d", f_rx, k_rx, a_rx); errs++; end
        join_any
        repeat (20) @(posedge clk);
        if (errs == 0 && f_rx == N && k_rx == N && a_rx == N) $display("TEST PASSED");
        else $display("TEST FAILED errs=%0d", errs);
        $finish;
    end
endmodule
