// Hold image: motor PWM/DIR low, four servos at 1500 us / 50 Hz (the control image's pre-policy
// pose), and a BMI088 I2C bus recovery sequence (9 SCL pulses + STOP) every 100 ms.
module hold_top (
    input  wire clk50,
    inout  wire BMI088_SCL,
    inout  wire BMI088_SDA,
    output wire M1_PWM, M1_DIR1, M1_DIR2, M2_PWM, M2_DIR1, M2_DIR2,
    output wire SERVO1, SERVO2, SERVO3, SERVO4
);
    reg [19:0] frame = 20'd0;   // 1_000_000 clocks = 20 ms
    always @(posedge clk50) frame <= (frame == 20'd999_999) ? 20'd0 : frame + 20'd1;
    wire servo_pulse = frame < 20'd75_000;  // 1500 us
    assign {SERVO1, SERVO2, SERVO3, SERVO4} = {4{servo_pulse}};
    assign {M1_PWM, M1_DIR1, M1_DIR2, M2_PWM, M2_DIR1, M2_DIR2} = 6'b0;

    reg [10:0] div = 11'd0;     // 25 us per step
    reg [11:0] step = 12'd0;    // 100 ms per sequence
    always @(posedge clk50) begin
        if (div == 11'd1249) begin
            div <= 11'd0;
            step <= (step == 12'd3999) ? 12'd0 : step + 12'd1;
        end else
            div <= div + 11'd1;
    end
    wire scl_low = (step < 12'd18) ? ~step[0] : (step == 12'd18 || step == 12'd19);
    wire sda_low = (step == 12'd19 || step == 12'd20);
    assign BMI088_SCL = scl_low ? 1'b0 : 1'bz;
    assign BMI088_SDA = sda_low ? 1'b0 : 1'bz;
endmodule
