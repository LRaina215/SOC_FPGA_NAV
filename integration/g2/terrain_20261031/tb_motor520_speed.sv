// motor520_speed 编码器滤波测试：干净正交信号、模拟 PWM 干扰（两相带时差的共模窄脉冲，
// 未滤波时每个脉冲凑成一整个假正交周期）、超过物理上限的计数。
// 闸门缩短到 1 ms（50000 拍）以加快仿真；上限按比例取 12（10 ms 闸门下对应 120）。
`timescale 1ns/1ps
module tb_motor520_speed;
    localparam integer GATE = 50000, FILT = 150, MAXD = 12;
    reg clk = 0, rst_n = 0;
    always #10 clk = ~clk;
    reg enc_a = 0, enc_b = 0;
    wire signed [31:0] pos_f, del_f, pos_r, del_r;
    wire signed [15:0] cms_f, cms_r;
    wire val_f, val_r;
    wire [31:0] inv_f, inv_r;
    wire [15:0] rej_f, rej_r;
    motor520_speed #(.SPEED_GATE_CYCLES(GATE), .FILTER_CYCLES(FILT), .MAX_DELTA(MAXD)) dut (
        .clk(clk), .reset_n(rst_n), .enc_a(enc_a), .enc_b(enc_b), .position(pos_f), .speed_delta(del_f),
        .speed_cms(cms_f), .sample_valid(val_f), .invalid_transitions(inv_f), .rejected_samples(rej_f));
    motor520_speed #(.SPEED_GATE_CYCLES(GATE)) ref_raw (
        .clk(clk), .reset_n(rst_n), .enc_a(enc_a), .enc_b(enc_b), .position(pos_r), .speed_delta(del_r),
        .speed_cms(cms_r), .sample_valid(val_r), .invalid_transitions(inv_r), .rejected_samples(rej_r));

    integer errors = 0, phase = 0, dir = 1;
    integer period = 0;          // 真实边沿间隔（拍）；0 = 停止。正常速度每闸门约 8 个边沿
    integer noise_gap = 0;       // 干扰脉冲间隔（拍）；0 = 无
    integer true_edges = 0;
    reg [1:0] q = 0;             // 真实正交状态（格雷码 00 01 11 10）
    task automatic set_q(input [1:0] v); begin enc_a = v[1]; enc_b = v[0]; end endtask
    function [1:0] gray(input integer k);
        case (k & 3) 0: gray = 2'b00; 1: gray = 2'b01; 2: gray = 2'b11; default: gray = 2'b10; endcase
    endfunction
    integer k = 0, t_edge = 0, t_noise = 0, cyc = 0, nt = 0;
    reg in_noise = 0;
    reg [1:0] g;
    // 激励在下降沿改变，避免与被测模块的上升沿采样竞争
    always @(negedge clk) if (rst_n) begin
        cyc = cyc + 1;
        if (period && cyc >= t_edge) begin
            k = k + dir; t_edge = cyc + period; true_edges = true_edges + dir;
        end
        g = gray(k);
        // 干扰：A 先翻，20 拍后 B 翻，再 20 拍 A 回，再 20 拍 B 回（都窄于 FILTER_CYCLES）
        if (noise_gap && !in_noise && cyc >= t_noise) begin in_noise = 1; nt = 0; end
        if (in_noise) begin
            enc_a = (nt < 40) ? ~g[1] : g[1];
            enc_b = (nt >= 20 && nt < 60) ? ~g[0] : g[0];
            nt = nt + 1;
            if (nt == 60) begin in_noise = 0; t_noise = cyc + noise_gap; end
        end else set_q(g);
    end

    // 每个采样：记录滤波实例与未滤波实例的计数，与真实边沿数比较
    integer last_true = 0, gates = 0;
    integer exp_now;
    task automatic check_gate(input integer expect_hold, input integer prev_del);
        begin
            exp_now = true_edges - last_true;
            if (expect_hold) begin
                if (del_f !== prev_del) begin errors = errors + 1; $display("FAIL hold: del_f=%0d prev=%0d", del_f, prev_del); end
            end else if ((del_f - exp_now) > 1 || (exp_now - del_f) > 1) begin
                errors = errors + 1; $display("FAIL count: phase %0d del_f=%0d true=%0d", phase, del_f, exp_now);
            end
        end
    endtask
    integer prev_del = 0, hold_expected = 0, raw_max = 0, rej_before = 0, skip = 0;
    always @(posedge clk) if (val_f) begin
        if (skip) skip = skip - 1;
        else check_gate(hold_expected, prev_del);
        if (del_r > raw_max) raw_max = del_r;
        if (-del_r > raw_max) raw_max = -del_r;
        $display("gate %0d phase %0d: filtered %0d raw %0d true %0d rejected %0d", gates, phase, del_f, del_r, true_edges - last_true, rej_f);
        prev_del = del_f;
        last_true = true_edges;
        gates = gates + 1;
    end

    initial begin
        repeat (20) @(posedge clk);
        rst_n = 1;
        // 1) 干净信号，正转：每 6000 拍一个边沿（每闸门约 8 个）
        phase = 1; period = 6000; dir = 1;
        repeat (6) @(posedge val_f);
        // 2) 反转
        phase = 2; dir = -1;
        repeat (4) @(posedge val_f);
        // 3) 正转 + 每 400 拍一个干扰脉冲（未滤波实例每脉冲多计 ±4）
        phase = 3; dir = 1; noise_gap = 400; raw_max = 0;
        repeat (6) @(posedge val_f);
        // 4) 停止 + 干扰（静止时每个干扰脉冲方向相同，未滤波实例计数最大）
        phase = 4; period = 0;
        repeat (4) @(posedge val_f);
        if (raw_max < 100) begin errors = errors + 1; $display("FAIL: 未滤波实例没有被干扰放大（max %0d）", raw_max); end
        $display("phase3-4: 未滤波实例最大 |计数| %0d / 闸门，滤波实例与真值相差不超过 1", raw_max);
        // 5) 超限：每 1000 拍一个真实边沿（50/闸门 > 12），滤波实例应保持上一个值
        phase = 5; noise_gap = 0; period = 1000; dir = 1; rej_before = rej_f; skip = 1;
        @(posedge val_f);                       // 跨越切换的闸门不检查
        hold_expected = 1;
        repeat (3) @(posedge val_f);
        hold_expected = 0;
        if (rej_f - rej_before < 3) begin errors = errors + 1; $display("FAIL: rejected_samples %0d", rej_f - rej_before); end
        // 6) 恢复正常速度：先跨过一个切换闸门
        phase = 6; period = 6000; skip = 1;
        @(posedge val_f);
        repeat (3) @(posedge val_f);
        if (errors == 0) $display("TEST PASSED (%0d gates, invalid transitions filtered %0d raw %0d)", gates, inv_f, inv_r);
        else $display("TEST FAILED: %0d errors", errors);
        $finish;
    end
endmodule
