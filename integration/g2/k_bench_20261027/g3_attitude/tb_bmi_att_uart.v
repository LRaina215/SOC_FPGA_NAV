`timescale 1ns/1ps
// Decode the serial stream of bmi_att_uart with a reference UART receiver and
// check both frames' contents and checksums against the published samples.
module tb_bmi_att_uart;
    localparam integer DIV = 20;
    reg clk = 0, rst = 1;
    always #10 clk = ~clk;
    reg valid = 0;
    reg signed [23:0] gx, gy, gz, wx, wy, wz;
    wire txd;
    wire [15:0] seq, dropped;
    bmi_att_uart #(.DIV(DIV)) dut (.clk(clk), .rst(rst), .valid(valid), .gx(gx), .gy(gy), .gz(gz),
        .wx(wx), .wy(wy), .wz(wz), .txd(txd), .seq(seq), .dropped(dropped));

    // reference receiver: sample at mid-bit
    reg [7:0] rx [0:63];
    integer nrx = 0;
    integer b;
    reg [7:0] byte_v;
    initial begin
        forever begin
            @(negedge txd);
            repeat (DIV / 2) @(posedge clk);
            if (txd !== 1'b0) $display("ERROR bad start bit");
            for (b = 0; b < 8; b = b + 1) begin
                repeat (DIV) @(posedge clk);
                byte_v[b] = txd;
            end
            repeat (DIV) @(posedge clk);
            if (txd !== 1'b1) $display("ERROR bad stop bit");
            rx[nrx % 64] = byte_v;
            nrx = nrx + 1;
        end
    end

    function [15:0] sat16;
        input signed [23:0] v;
        begin
            if (v > 24'sd32767) sat16 = 16'h7FFF;
            else if (v < -24'sd32768) sat16 = 16'h8000;
            else sat16 = v[15:0];
        end
    endfunction

    integer i, errors = 0, s, base;
    reg [7:0] sum;
    reg [15:0] e [0:6];
    integer seed = 7;
    initial begin
        repeat (5) @(posedge clk);
        rst = 0;
        for (s = 0; s < 40; s = s + 1) begin
            gx = $random(seed) % 5000; gy = $random(seed) % 5000; gz = -4096 + ($random(seed) % 300);
            wx = $random(seed); wy = $random(seed) % 40000; wz = (s == 3) ? 24'sh7FFFFF : $random(seed) % 3000;
            if (s == 5) begin gx = 24'sh800000; gy = 24'sh7FFFFF; end
            e[0] = sat16(gx); e[1] = sat16(gy); e[2] = sat16(gz);
            e[3] = sat16(wx >>> 3); e[4] = sat16(wy >>> 3); e[5] = sat16(wz >>> 3); e[6] = s;
            base = nrx;
            @(posedge clk); #1 valid = 1; @(posedge clk); #1 valid = 0;
            wait (nrx >= base + 22);
            repeat (3 * DIV) @(posedge clk);
            // frame A
            sum = 0; for (i = 0; i < 10; i = i + 1) sum = sum + rx[(base + i) % 64];
            if (rx[base % 64] !== 8'h55 || rx[(base + 1) % 64] !== 8'h5B || sum !== rx[(base + 10) % 64] ||
                {rx[(base + 3) % 64], rx[(base + 2) % 64]} !== e[0] || {rx[(base + 5) % 64], rx[(base + 4) % 64]} !== e[1] ||
                {rx[(base + 7) % 64], rx[(base + 6) % 64]} !== e[2] || {rx[(base + 9) % 64], rx[(base + 8) % 64]} !== e[6]) begin
                errors = errors + 1; $display("ERROR frame A sample %0d", s);
            end
            sum = 0; for (i = 11; i < 21; i = i + 1) sum = sum + rx[(base + i) % 64];
            if (rx[(base + 11) % 64] !== 8'h55 || rx[(base + 12) % 64] !== 8'h5C || sum !== rx[(base + 21) % 64] ||
                {rx[(base + 14) % 64], rx[(base + 13) % 64]} !== e[3] || {rx[(base + 16) % 64], rx[(base + 15) % 64]} !== e[4] ||
                {rx[(base + 18) % 64], rx[(base + 17) % 64]} !== e[5]) begin
                errors = errors + 1; $display("ERROR frame B sample %0d", s);
            end
        end
        // a publication while busy must be dropped and counted, not corrupt the stream
        base = nrx;
        @(posedge clk); #1 valid = 1; @(posedge clk); #1 valid = 0;
        repeat (5 * DIV) @(posedge clk);
        #1 valid = 1; @(posedge clk); #1 valid = 0;
        wait (nrx >= base + 22);
        repeat (40 * DIV) @(posedge clk);
        if (dropped !== 1 || nrx !== base + 22) begin errors = errors + 1; $display("ERROR drop handling dropped=%0d n=%0d", dropped, nrx - base); end
        $display("DONE samples=40 bytes=%0d errors=%0d", nrx, errors);
        $finish;
    end
endmodule
