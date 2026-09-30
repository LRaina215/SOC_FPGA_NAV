// 调试串口桥测试：经 921600 波特串口读 SYS.ID、写读 SCRATCH、读 TIME_US 递增。
`timescale 1ns/1ps
module tb_uart_bridge;
`include "nav_regs_map.vh"
    logic clk = 0, rst = 1;
    always #10 clk = ~clk;                 // 50 MHz
    localparam real BIT = 1e9 / 921600.0;

    logic rxd = 1;
    wire  txd;
    wire  req, we, ack;
    wire [15:0] addr;
    wire [31:0] wdata, rdata;
    uart_bus_bridge #(.CLK_HZ(50000000), .BAUD(921600)) u_b (
        .clk(clk), .rst(rst), .rxd(rxd), .txd(txd),
        .bus_req(req), .bus_we(we), .bus_addr(addr), .bus_wdata(wdata), .bus_ack(ack), .bus_rdata(rdata));

    // 简化从机：只实现 ID、SCRATCH、TIME
    logic [31:0] scratch = 0, tus = 0;
    logic ack_r = 0;
    logic [31:0] rd_r;
    always @(posedge clk) begin
        tus <= tus + 1;
        ack_r <= 0;
        if (req && !ack_r) begin
            ack_r <= 1;
            if (we && addr == RA_SYS_SCRATCH) scratch <= wdata;
            rd_r <= (addr == RA_SYS_ID) ? 32'h4E415630 : (addr == RA_SYS_SCRATCH) ? scratch : tus;
        end
    end
    assign ack = ack_r;
    assign rdata = rd_r;

    task automatic send(input [7:0] b);
        rxd = 0; #(BIT);
        for (int i = 0; i < 8; i++) begin rxd = b[i]; #(BIT); end
        rxd = 1; #(BIT);
    endtask
    task automatic recv(output [7:0] b);
        @(negedge txd);
        #(BIT * 1.5);
        for (int i = 0; i < 8; i++) begin b[i] = txd; #(BIT); end
    endtask
    task automatic rd32(input [15:0] a, output [31:0] d);
        logic [7:0] c, x;
        fork
            begin send("R"); send(a[15:8]); send(a[7:0]); end
            begin
                recv(c);
                if (c != "D") $display("TEST FAILED 读应答 %h", c);
                for (int i = 0; i < 4; i++) begin recv(x); d = {d[23:0], x}; end
            end
        join
    endtask
    task automatic wr32(input [15:0] a, input [31:0] v);
        logic [7:0] c;
        fork
            begin send("W"); send(a[15:8]); send(a[7:0]); send(v[31:24]); send(v[23:16]); send(v[15:8]); send(v[7:0]); end
            begin recv(c); if (c != "K") $display("TEST FAILED 写应答 %h", c); end
        join
    endtask

    initial begin
        logic [31:0] d, t0, t1;
        int errs = 0;
        #200 rst = 0;
        #1000;
        rd32(RA_SYS_ID, d);        if (d != 32'h4E415630) begin errs++; $display("ID %h", d); end
        wr32(RA_SYS_SCRATCH, 32'hCAFEF00D);
        rd32(RA_SYS_SCRATCH, d);   if (d != 32'hCAFEF00D) begin errs++; $display("SCRATCH %h", d); end
        // 连发 3 条写（字节之间没有间隔，PC 端 wr_burst 就是这样发的）：3 个 'K'，SCRATCH = 最后一条
        begin
            logic [7:0] c;
            int nk;
            nk = 0;
            fork
                for (int k = 1; k <= 3; k++) begin
                    send("W"); send(RA_SYS_SCRATCH[15:8]); send(RA_SYS_SCRATCH[7:0]);
                    send(8'h00); send(8'h00); send(8'h00); send(k[7:0]);
                end
                for (int k = 0; k < 3; k++) begin recv(c); if (c == "K") nk++; end
            join
            if (nk != 3) begin errs++; $display("连发写应答 %0d / 3", nk); end
            rd32(RA_SYS_SCRATCH, d);   if (d != 32'd3) begin errs++; $display("连发后 SCRATCH %h", d); end
        end
        send(8'h00); send(8'h55);  // 杂散字节应被忽略
        rd32(RA_SYS_TIME_US, t0);
        rd32(RA_SYS_TIME_US, t1);  if (t1 <= t0) begin errs++; $display("TIME 未递增"); end
        if (errs == 0) $display("TEST PASSED"); else $display("TEST FAILED errs=%0d", errs);
        $finish;
    end
endmodule
