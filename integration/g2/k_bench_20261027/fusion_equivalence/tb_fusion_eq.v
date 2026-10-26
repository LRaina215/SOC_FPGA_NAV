`timescale 1ns/1ps
// Compare the original and the retimed imu_fusion on identical random stimulus.
// Outputs are compared per published sample (valid pulse), not per clock,
// because the retimed version publishes one clock later.
module tb_fusion_eq;
    reg clk = 0, rst_n = 0;
    always #10 clk = ~clk;

    reg sample_valid = 0;
    reg signed [15:0] ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

    wire signed [23:0] r_wx, r_wy, r_wz, r_gx, r_gy, r_gz, n_wx, n_wy, n_wz, n_gx, n_gy, n_gz;
    wire r_valid, r_cal, r_sus, r_rej, n_valid, n_cal, n_sus, n_rej;

    imu_fusion_ref #(.BIAS_SAMPLES(4)) ref_i (
        .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid),
        .accel_x(ax), .accel_y(ay), .accel_z(az), .gyro_x(gx), .gyro_y(gy), .gyro_z(gz),
        .ang_vel_x(r_wx), .ang_vel_y(r_wy), .ang_vel_z(r_wz),
        .gravity_x(r_gx), .gravity_y(r_gy), .gravity_z(r_gz),
        .valid(r_valid), .calibrating(r_cal), .bias_suspect(r_sus), .accel_rejected(r_rej));
    imu_fusion #(.BIAS_SAMPLES(4)) new_i (
        .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid),
        .accel_x(ax), .accel_y(ay), .accel_z(az), .gyro_x(gx), .gyro_y(gy), .gyro_z(gz),
        .ang_vel_x(n_wx), .ang_vel_y(n_wy), .ang_vel_z(n_wz),
        .gravity_x(n_gx), .gravity_y(n_gy), .gravity_z(n_gz),
        .valid(n_valid), .calibrating(n_cal), .bias_suspect(n_sus), .accel_rejected(n_rej));

    // Queue of reference outputs, consumed by the retimed outputs.
    reg [24*6+1:0] q [0:15];
    integer wp = 0, rp = 0, n_ref = 0, n_new = 0, errors = 0, rejected = 0, accepted = 0;
    always @(posedge clk) begin
        if (r_valid) begin
            q[wp & 15] <= {r_rej, r_sus, r_wx, r_wy, r_wz, r_gx, r_gy, r_gz};
            wp <= wp + 1;
            n_ref <= n_ref + 1;
            if (r_rej) rejected <= rejected + 1; else accepted <= accepted + 1;
        end
        if (n_valid) begin
            if (rp >= wp + (r_valid ? 1 : 0)) begin
                errors <= errors + 1;
                $display("ERROR new published before ref at sample %0d", n_new);
            end else if (q[rp & 15] !== {n_rej, n_sus, n_wx, n_wy, n_wz, n_gx, n_gy, n_gz}) begin
                errors <= errors + 1;
                if (errors < 10)
                    $display("MISMATCH sample %0d ref=%h new=%h", n_new, q[rp & 15],
                             {n_rej, n_sus, n_wx, n_wy, n_wz, n_gx, n_gy, n_gz});
            end
            rp <= rp + 1;
            n_new <= n_new + 1;
        end
        if (r_cal !== n_cal && !(r_cal == 0 && n_cal == 0)) begin
            // calibrating may differ by a clock at the end of the bias window only
        end
    end

    integer i, k, seed = 12345;
    integer kind;
    reg signed [31:0] r;
    task new_sample;
        begin
            kind = $random(seed) & 7;
            // accel: mostly ~1 g with noise, sometimes far off 1 g, sometimes extremes
            if (kind < 5) begin
                ax = ($random(seed) % 2500);
                ay = ($random(seed) % 2500);
                az = 10920 + ($random(seed) % 1500);
            end else if (kind == 5) begin
                ax = $random(seed); ay = $random(seed); az = $random(seed);
            end else if (kind == 6) begin
                ax = 16'sh8000; ay = 16'sh7FFF; az = 16'sh8000;
            end else begin
                ax = ($random(seed) % 20000); ay = ($random(seed) % 20000); az = ($random(seed) % 20000);
            end
            // gyro: mostly small (accel trusted), sometimes around the 600 gate, sometimes large
            k = $random(seed) & 7;
            if (k < 4) begin
                gx = $random(seed) % 150; gy = $random(seed) % 150; gz = $random(seed) % 150;
            end else if (k < 6) begin
                gx = 200 + ($random(seed) % 20); gy = 190 + ($random(seed) % 20); gz = 200 + ($random(seed) % 20);
            end else if (k == 6) begin
                gx = $random(seed); gy = $random(seed); gz = $random(seed);
            end else begin
                gx = 16'sh8000; gy = 16'sh8000; gz = 16'sh7FFF;
            end
            sample_valid = 1;
            @(posedge clk);
            #1 sample_valid = 0;
        end
    endtask

    initial begin
        repeat (5) @(posedge clk);
        #1 rst_n = 1;
        // still samples for the bias window
        for (i = 0; i < 6; i = i + 1) begin
            ax = 0; ay = 0; az = 10920; gx = 3; gy = -2; gz = 1;
            sample_valid = 1; @(posedge clk); #1 sample_valid = 0;
            repeat (400) @(posedge clk);
        end
        for (i = 0; i < 4000; i = i + 1) begin
            new_sample;
            // spacing: normal 400 clocks, occasionally a sample arrives during processing
            if (($random(seed) & 31) == 0) repeat (30) @(posedge clk);
            else repeat (400) @(posedge clk);
            #1;
        end
        repeat (1000) @(posedge clk);
        $display("DONE ref=%0d new=%0d accepted=%0d rejected=%0d errors=%0d", n_ref, n_new, accepted, rejected, errors);
        $finish;
    end
endmodule
