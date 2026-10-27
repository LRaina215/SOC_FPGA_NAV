`timescale 1ns/1ps
module tb_motor_test;
    reg clk = 0;
    always #10 clk = ~clk;
    wire scl, sda;
    pullup(scl); pullup(sda);
    wire m1p, m1d1, m1d2, m2p, m2d1, m2d2, s1, s2, s3, s4, txd;
    motor_test_top dut(.clk50(clk), .M1_ENCA(1'b1), .M1_ENCB(1'b1), .M2_ENCA(1'b1), .M2_ENCB(1'b1),
        .BMI088_SCL(scl), .BMI088_SDA(sda),
        .M1_PWM(m1p), .M1_DIR1(m1d1), .M1_DIR2(m1d2), .M2_PWM(m2p), .M2_DIR1(m2d1), .M2_DIR2(m2d2),
        .SERVO1(s1), .SERVO2(s2), .SERVO3(s3), .SERVO4(s4), .control_txd(txd));
    defparam dut.wheel_left.SPEED_GATE_CYCLES = 5000;
    defparam dut.wheel_right.SPEED_GATE_CYCLES = 5000;

    integer hi_l, hi_r, n, d1l, d2l, d1r, d2r, errors;
    reg [7:0] last_phase;
    initial begin
        errors = 0; last_phase = 8'hAA; hi_l = 0; hi_r = 0; n = 0; d1l = 0; d2l = 0; d1r = 0; d2r = 0;
    end
    always @(posedge clk) begin
        if (dut.phase !== last_phase) begin
            if (n > 0)
                $display("phase %h duty %0d  L pwm %0.3f d1 %0.2f d2 %0.2f | R pwm %0.3f d1 %0.2f d2 %0.2f",
                    last_phase, $signed(dut.duty), hi_l*1.0/n, d1l*1.0/n, d2l*1.0/n, hi_r*1.0/n, d1r*1.0/n, d2r*1.0/n);
            last_phase = dut.phase; hi_l = 0; hi_r = 0; n = 0; d1l = 0; d2l = 0; d1r = 0; d2r = 0;
        end else begin
            n = n + 1; hi_l = hi_l + m2p; hi_r = hi_r + m1p;
            d1l = d1l + m2d1; d2l = d2l + m2d2; d1r = d1r + m1d1; d2r = d2r + m1d2;
            if ((m2d1 && m2d2) || (m1d1 && m1d2)) errors = errors + 1;
            if ((m1p || m1d1 || m1d2) && (m2p || m2d1 || m2d2)) errors = errors + 1;
        end
    end

    // UART receiver at 217 clocks/bit; check frame header and checksum
    integer frames = 0, bad = 0, k, i;
    reg [7:0] b [0:17];
    reg [7:0] s;
    task rx_byte(output [7:0] v);
        integer j;
        begin
            @(negedge txd);
            repeat (217 + 108) @(posedge clk);
            for (j = 0; j < 8; j = j + 1) begin v[j] = txd; repeat (217) @(posedge clk); end
        end
    endtask
    initial begin
        forever begin
            rx_byte(b[0]);
            if (b[0] == 8'hA5) begin
                for (k = 1; k < 18; k = k + 1) rx_byte(b[k]);
                s = 0; for (i = 2; i < 17; i = i + 1) s = s + b[i];
                frames = frames + 1;
                if (b[1] != 8'h5A || s != b[17]) bad = bad + 1;
                if (frames < 4 || frames % 200 == 0)
                    $display("frame %0d phase %h duty %0d sum_ok %0d", frames, b[2], $signed({b[4], b[3]}), s == b[17]);
            end
        end
    end
    initial begin
        wait (dut.phase == 8'hFE);
        repeat (200000) @(posedge clk);
        $display("DONE errors %0d frames %0d bad %0d servo_hi %0d", errors, frames, bad, s1);
        $finish;
    end
endmodule
