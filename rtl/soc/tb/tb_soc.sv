// 软核 SoC 测试：PicoRV32 跑自检程序（fw/bsp/selftest.c，经 nav_bus_arb 访问真实 nav_subsystem 寄存器），
// 再经调试口保持复位、重新装载程序、放开，检查再次运行；控制台缓冲经调试口读出。
`timescale 1ns/1ps
module tb_soc;
    logic clk = 0, rst = 1;
    always #10 clk = ~clk;                       // 50 MHz

    logic        m_req, m_we, m_ack; logic [15:0] m_addr; logic [31:0] m_wdata, m_rdata;
    logic        d_req = 0, d_we = 0, d_ack; logic [15:0] d_addr; logic [31:0] d_wdata, d_rdata;
    logic [3:0]  led; logic trap;
    nav_soc #(.RAM_KB(16), .INIT_B0("ram_b0.hex"), .INIT_B1("ram_b1.hex"), .INIT_B2("ram_b2.hex"), .INIT_B3("ram_b3.hex")) dut (
        .clk(clk), .rst(rst), .m_req(m_req), .m_we(m_we), .m_addr(m_addr), .m_wdata(m_wdata), .m_ack(m_ack), .m_rdata(m_rdata),
        .d_req(d_req), .d_we(d_we), .d_addr(d_addr), .d_wdata(d_wdata), .d_ack(d_ack), .d_rdata(d_rdata), .led(led), .trap(trap));

    logic s_req, s_we, s_ack; logic [15:0] s_addr; logic [31:0] s_wdata, s_rdata;
    nav_bus_arb u_arb (.clk(clk), .rst(rst),
        .m0_req(m_req), .m0_we(m_we), .m0_addr(m_addr), .m0_wdata(m_wdata), .m0_ack(m_ack), .m0_rdata(m_rdata),
        .m1_req(1'b0), .m1_we(1'b0), .m1_addr(16'd0), .m1_wdata(32'd0), .m1_ack(), .m1_rdata(),
        .s_req(s_req), .s_we(s_we), .s_addr(s_addr), .s_wdata(s_wdata), .s_ack(s_ack), .s_rdata(s_rdata));
    nav_subsystem #(.CLK_HZ(50000000), .LUT_INIT("lut_default.hex"), .SIN_INIT("sin_lut.hex")) u_nav (
        .clk(clk), .rst(rst),
        .bus_req(s_req), .bus_we(s_we), .bus_addr(s_addr), .bus_wdata(s_wdata), .bus_ack(s_ack), .bus_rdata(s_rdata), .irq(),
        .l_rx_clk(clk), .l_rx_rst(rst), .l_rx_valid(1'b0), .l_rx_data(8'd0), .l_rx_last(1'b0), .l_rx_error(1'b0),
        .l_tx_clk(clk), .l_tx_rst(rst), .l_tx_valid(), .l_tx_data(), .l_tx_last(), .l_tx_ready(1'b1),
        .h_rx_clk(clk), .h_rx_rst(rst), .h_rx_valid(1'b0), .h_rx_data(8'd0), .h_rx_last(1'b0), .h_rx_error(1'b0),
        .h_tx_clk(clk), .h_tx_rst(rst), .h_tx_valid(), .h_tx_data(), .h_tx_last(), .h_tx_ready(1'b1),
        .imu_rx(1'b1), .imu_tx(),
        .ch_vx(), .ch_vy(), .ch_wz(), .ch_mode(), .ch_stop(), .ch_wdog(), .ch_valid(), .ch_seq(),
        .odo_valid(1'b0), .odo_vx(16'd0), .odo_vy(16'd0), .odo_wz(16'd0), .ext_stop(4'd0));

    // 控制台字节直接打印（便于看日志）
    always @(posedge clk) if (dut.con_we) $write("%c", dut.mem_wdata[7:0]);

    task automatic dwr(input [15:0] a, input [31:0] v);
        @(posedge clk); d_req <= 1; d_we <= 1; d_addr <= a; d_wdata <= v;
        do @(posedge clk); while (!d_ack);
        d_req <= 0; d_we <= 0; @(posedge clk);
    endtask
    task automatic drd(input [15:0] a, output [31:0] v);
        @(posedge clk); d_req <= 1; d_we <= 0; d_addr <= a;
        do @(posedge clk); while (!d_ack);
        v = d_rdata; d_req <= 0; @(posedge clk);
    endtask

    logic [31:0] img [0:4095];
    int errs = 0;
    initial begin
        logic [31:0] v, v2, wp0, wp1;
        $readmemh("ram_w.hex", img);
        repeat (10) @(posedge clk); rst = 0;
        // 1. 自检（随初始化镜像运行）
        fork
            begin wait (dut.status == 32'h600DC0DE || dut.status == 32'h00000BAD || trap); end
            begin repeat (400000) @(posedge clk); end
        join_any
        if (dut.status != 32'h600DC0DE) begin errs++; $display("\n自检未通过：status=%h trap=%0d", dut.status, trap); end
        // 2. 控制台缓冲经调试口读出：前 8 字节应为 "selftest"
        drd(16'hF00C, wp0);
        if (wp0 < 20) begin errs++; $display("控制台字节数 %0d", wp0); end
        drd(16'hE000, v); drd(16'hE004, v2);
        // "selftest" 小端打包：'s''e''l''f' = 0x666c6573，'t''e''s''t' = 0x74736574
        if (v != 32'h666c6573 || v2 != 32'h74736574) begin errs++; $display("控制台内容 %h %h", v, v2); end
        drd(16'hF014, v);
        if (v[31:16] != 16) begin errs++; $display("INFO %h", v); end
        // 3. 调试口装载：保持复位 -> 清状态字所在程序不变，重新写入镜像并回读校验 -> 放开 -> 再次通过
        dwr(16'hF000, 1);
        dwr(16'hF004, 0);
        for (int i = 0; i < 512; i++) dwr(16'hF008, img[i]);
        dwr(16'hF004, 5);
        drd(16'hF008, v);
        if (v !== img[5]) begin errs++; $display("RAM 回读 %h 期望 %h", v, img[5]); end
        force dut.status = 32'd0; @(posedge clk); release dut.status;
        drd(16'hF00C, wp0);
        dwr(16'hF000, 0);
        fork
            begin wait (dut.status == 32'h600DC0DE || trap); end
            begin repeat (400000) @(posedge clk); end
        join_any
        drd(16'hF00C, wp1);
        if (dut.status != 32'h600DC0DE || wp1 <= wp0) begin errs++; $display("重新装载后未运行：status=%h 控制台 %0d->%0d", dut.status, wp0, wp1); end
        if (trap) begin errs++; $display("trap"); end
        if (errs == 0) $display("\nTEST PASSED");
        else $display("\nTEST FAILED errs=%0d", errs);
        $finish;
    end
endmodule
