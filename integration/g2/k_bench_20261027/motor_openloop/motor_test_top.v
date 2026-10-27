// Open-loop wheel test with the control core's own PWM and encoder modules and parameters.
// After a 2 s idle: left wheel then right wheel, duty 500/1000/1500/2000 of 2500 ticks,
// each level +duty 1 s, off 0.5 s, -duty 1 s, off 0.5 s; then all outputs stay off.
// Every 10 ms encoder sample: A5 5A phase duty lspd rspd lpos rpos sum on control_txd (230400).
// Servos hold 1500 us; BMI088 I2C bus recovery runs as in the hold image.
module motor_test_top (
    input  wire clk50,
    input  wire M1_ENCA, M1_ENCB, M2_ENCA, M2_ENCB,
    inout  wire BMI088_SCL,
    inout  wire BMI088_SDA,
    output wire M1_PWM, M1_DIR1, M1_DIR2, M2_PWM, M2_DIR1, M2_DIR2,
    output wire SERVO1, SERVO2, SERVO3, SERVO4,
    output wire control_txd
);
    reg [7:0] por = 8'd0;
    always @(posedge clk50) if (!(&por)) por <= por + 8'd1;
    wire rst_n = &por;

    reg [19:0] frame = 20'd0;
    always @(posedge clk50) frame <= (frame == 20'd999_999) ? 20'd0 : frame + 20'd1;
    assign {SERVO1, SERVO2, SERVO3, SERVO4} = {4{frame < 20'd75_000}};

    reg [10:0] div = 11'd0;
    reg [11:0] step = 12'd0;
    always @(posedge clk50) begin
        if (div == 11'd1249) begin div <= 11'd0; step <= (step == 12'd3999) ? 12'd0 : step + 12'd1; end
        else div <= div + 11'd1;
    end
    wire scl_low = (step < 12'd18) ? ~step[0] : (step == 12'd18 || step == 12'd19);
    wire sda_low = (step == 12'd19 || step == 12'd20);
    assign BMI088_SCL = scl_low ? 1'b0 : 1'bz;
    assign BMI088_SDA = sda_low ? 1'b0 : 1'bz;

    wire signed [31:0] lpos, rpos, ldelta, rdelta;
    wire signed [15:0] lspd, rspd;
    wire lvalid, rvalid;
    motor520_speed #(.ENCODER_CPR(1320), .WHEEL_DIAMETER_UM(65000), .ENCODER_SIGN(1)) wheel_left (
        .clk(clk50), .reset_n(rst_n), .enc_a(M2_ENCA), .enc_b(M2_ENCB),
        .speed_cms(lspd), .sample_valid(lvalid), .position(lpos), .speed_delta(ldelta), .invalid_transitions());
    motor520_speed #(.ENCODER_CPR(1320), .WHEEL_DIAMETER_UM(65000), .ENCODER_SIGN(-1)) wheel_right (
        .clk(clk50), .reset_n(rst_n), .enc_a(M1_ENCA), .enc_b(M1_ENCB),
        .speed_cms(rspd), .sample_valid(rvalid), .position(rpos), .speed_delta(rdelta), .invalid_transitions());

    // phase: 0xFF initial idle, 0..31 test segments, 0xFE finished
    reg [7:0] phase = 8'hFF;
    reg [7:0] ticks = 8'd0;
    wire [1:0] sub = phase[1:0];
    wire [7:0] seg_len = (phase == 8'hFF) ? 8'd200 : (sub[0] ? 8'd50 : 8'd100);
    wire running = phase[7:5] == 3'd0;
    wire [15:0] level = ({14'd0, phase[3:2]} + 16'd1) * 16'd500;   // 500, 1000, 1500, 2000
    wire signed [15:0] duty = !running || sub[0] ? 16'sd0 : (sub[1] ? -$signed(level) : $signed(level));
    wire right_side = phase[4];
    always @(posedge clk50) begin
        if (!rst_n) begin phase <= 8'hFF; ticks <= 8'd0; end
        else if (lvalid && phase != 8'hFE) begin
            if (ticks == seg_len - 8'd1) begin
                ticks <= 8'd0;
                phase <= (phase == 8'hFF) ? 8'd0 : (phase == 8'd31 ? 8'hFE : phase + 8'd1);
            end else ticks <= ticks + 8'd1;
        end
    end
    motor520_pwm #(.PERIOD_CYCLES(2500), .DRIVE_SIGN(1)) left_pwm (
        .clk(clk50), .reset_n(rst_n), .enable(running && !right_side), .duty_ticks(right_side ? 16'sd0 : duty),
        .pwm(M2_PWM), .dir1(M2_DIR1), .dir2(M2_DIR2));
    motor520_pwm #(.PERIOD_CYCLES(2500), .DRIVE_SIGN(1)) right_pwm (
        .clk(clk50), .reset_n(rst_n), .enable(running && right_side), .duty_ticks(right_side ? duty : 16'sd0),
        .pwm(M1_PWM), .dir1(M1_DIR1), .dir2(M1_DIR2));

    // 18-byte record per left encoder sample
    reg [127:0] payload = 128'd0;
    reg [4:0] idx = 5'd0;
    reg sending = 1'b0;
    reg [7:0] sum = 8'd0;
    wire tx_ready;
    wire [7:0] byte_out = (idx == 5'd0) ? 8'hA5 : (idx == 5'd1) ? 8'h5A :
                          (idx == 5'd17) ? sum : payload[8*(idx-5'd2) +: 8];
    wire tx_go = sending && tx_ready;
    always @(posedge clk50) begin
        if (!rst_n) begin sending <= 1'b0; idx <= 5'd0; end
        else if (!sending) begin
            if (lvalid) begin
                payload <= {8'd0, rpos, lpos, rspd, lspd, duty, phase};
                sum <= 8'd0; idx <= 5'd0; sending <= 1'b1;
            end
        end else if (tx_go) begin
            if (idx >= 5'd2 && idx <= 5'd16) sum <= sum + byte_out;
            if (idx == 5'd17) sending <= 1'b0;
            idx <= idx + 5'd1;
        end
    end
    board_uart_tx #(.CLKS_PER_BIT(217)) u_tx (
        .clk(clk50), .rst_n(rst_n), .data(byte_out), .valid(tx_go), .ready(tx_ready), .txd(control_txd), .busy());
endmodule
