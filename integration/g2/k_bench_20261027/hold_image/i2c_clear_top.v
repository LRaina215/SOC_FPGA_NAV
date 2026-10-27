// BMI088 I2C bus recovery image: every 100 ms send 9 SCL pulses (10 kHz) and a STOP, so a slave
// left mid-byte by an FPGA reconfiguration releases SDA. All motor/servo outputs are held low.
module i2c_clear_top (
    input  wire clk50,
    inout  wire BMI088_SCL,
    inout  wire BMI088_SDA,
    output wire M1_PWM, M1_DIR1, M1_DIR2, M2_PWM, M2_DIR1, M2_DIR2,
    output wire SERVO1, SERVO2, SERVO3, SERVO4
);
    reg [10:0] div = 11'd0;   // 1250 clocks = 25 us per step
    reg [11:0] step = 12'd0;  // 4000 steps = 100 ms per sequence
    always @(posedge clk50) begin
        if (div == 11'd1249) begin
            div <= 11'd0;
            step <= (step == 12'd3999) ? 12'd0 : step + 12'd1;
        end else
            div <= div + 11'd1;
    end
    // steps 0..17: SCL low on even steps, released on odd steps (9 pulses, SDA released)
    // 18: SCL low; 19: SCL low + SDA low; 20: SCL released, SDA low; 21: SDA released (STOP)
    wire scl_low = (step < 12'd18) ? ~step[0] : (step == 12'd18 || step == 12'd19);
    wire sda_low = (step == 12'd19 || step == 12'd20);
    assign BMI088_SCL = scl_low ? 1'b0 : 1'bz;
    assign BMI088_SDA = sda_low ? 1'b0 : 1'bz;
    assign {M1_PWM, M1_DIR1, M1_DIR2, M2_PWM, M2_DIR1, M2_DIR2} = 6'b0;
    assign {SERVO1, SERVO2, SERVO3, SERVO4} = 4'b0;
endmodule
