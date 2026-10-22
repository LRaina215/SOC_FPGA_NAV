`timescale 1ns/1ps
// POR is the only board reset source. No actuator/arming logic exists here.
module fc_v4_board_top #(
    parameter integer CLK_HZ=50000000,
    parameter integer BAUD=115200,
    parameter integer CLKS_PER_BIT=CLK_HZ/BAUD,
    parameter integer RX_TIMEOUT_CLKS=CLK_HZ/50,
    parameter integer POR_CLKS=1024,
    parameter WEIGHTS_FILE="/home/cyd/fpga/rl_ws2/vehicle_integration/assets/weights.mem",
    parameter BIASES_FILE="/home/cyd/fpga/rl_ws2/vehicle_integration/assets/bias.mem",
    parameter ELU_FILE="/home/cyd/fpga/rl_ws2/vehicle_integration/assets/elu.mem"
)(
    input wire clk, input wire UART_RXD, output wire UART_TXD,
    input wire SPI_MISO, output wire SPI_SCLK, SPI_MOSI, SPI_CSB1, SPI_CSB2,
    input wire MT6701_A, output wire MT6701_B, MT6701_I,
    input wire MT6701_A_R, output wire MT6701_B_R, MT6701_I_R,
    output wire MOTOR_EN, output wire PWM_U, output wire PWM_V, output wire PWM_W,
    output wire MOTOR_EN2, output wire PWM_U2, output wire PWM_V2, output wire PWM_W2,
    output wire SERVO1, output wire SERVO2, output wire SERVO3, output wire SERVO4
);
    reg [31:0] por_count=0;
    wire rst_n=(por_count>=POR_CLKS);
    always @(posedge clk) if(por_count<POR_CLKS) por_count<=por_count+1'b1;
    assign {MOTOR_EN,PWM_U,PWM_V,PWM_W,MOTOR_EN2,PWM_U2,PWM_V2,PWM_W2,SERVO1,SERVO2,SERVO3,SERVO4}=12'd0;
    wire imu_ready, imu_error, imu_valid, enc_ready, enc_error, enc_valid;
    wire signed [15:0] ax,ay,az,gx,gy,gz;
    wire [13:0] enc_angle;
    wire [3:0] enc_status,enc_error_code;
    wire [23:0] enc_frame;
    wire cs_swapped,probe_complete;
    wire [7:0] probe_cs1,probe_cs2;
    wire [7:0] diag_operation,diag_actual,diag_expected,accel_id,gyro_id;
    wire er_ready,er_error,er_valid;
    wire [13:0] er_angle;
    wire [3:0] er_status,er_error_code;
    wire [23:0] er_frame;
    bmi088 imu(.clk(clk),.reset_n(rst_n),.SPI_MISO(SPI_MISO),.SPI_SCLK(SPI_SCLK),
        .SPI_MOSI(SPI_MOSI),.SPI_CSB1(SPI_CSB1),.SPI_CSB2(SPI_CSB2),
        .diag_operation(diag_operation),.diag_actual(diag_actual),.diag_expected(diag_expected),
        .accel_id(accel_id),.gyro_id(gyro_id),.cs_swapped(cs_swapped),.probe_cs1(probe_cs1),.probe_cs2(probe_cs2),.probe_complete(probe_complete),.init_done(imu_ready),.init_error(imu_error),.data_valid(imu_valid),
        .accel_x(ax),.accel_y(ay),.accel_z(az),.gyro_x(gx),.gyro_y(gy),.gyro_z(gz));
    mt6701_ssi #(.ALLOW_WEAK_FIELD(1)) encoder(.clk(clk),.reset_n(rst_n),.SSI_DO(MT6701_A),.SSI_CLK(MT6701_B),
        .SSI_CSN(MT6701_I),.ready(enc_ready),.error(enc_error),.data_valid(enc_valid),
        .angle_raw(enc_angle),.status_raw(enc_status),.frame_raw(enc_frame),.error_code(enc_error_code));
    mt6701_ssi #(.ALLOW_WEAK_FIELD(1)) encoder_right(.clk(clk),.reset_n(rst_n),.SSI_DO(MT6701_A_R),.SSI_CLK(MT6701_B_R),
        .SSI_CSN(MT6701_I_R),.ready(er_ready),.error(er_error),.data_valid(er_valid),
        .angle_raw(er_angle),.status_raw(er_status),.frame_raw(er_frame),.error_code(er_error_code));
    reg [31:0] er_sequence=0,er_ticks=0;
    always @(posedge clk) begin
        if(!rst_n) begin er_sequence<=0; er_ticks<=0; end
        else if(er_valid) begin er_sequence<=er_sequence+1'b1; er_ticks<=ticks; end
    end
    reg [31:0] ticks=0, imu_sequence=0, imu_ticks=0, enc_sequence=0, enc_ticks=0;
    always @(posedge clk) begin
        if(!rst_n) begin ticks<=0; imu_sequence<=0; imu_ticks<=0; enc_sequence<=0; enc_ticks<=0; end
        else begin
            ticks<=ticks+1'b1;
            if(imu_valid) begin imu_sequence<=imu_sequence+1'b1; imu_ticks<=ticks; end
            if(enc_valid) begin enc_sequence<=enc_sequence+1'b1; enc_ticks<=ticks; end
        end
    end
    // Frozen atomically by the UART bridge on an accepted telemetry request.
    // LE offsets: ticks0, imu_seq4, imu_ticks8, acc12, gyro18,
    // enc_seq24, enc_ticks28, enc_angle32, enc_status34, enc_err35,
    // flags36 (IMU ready/error, encoder ready/error), enc_frame37.
    wire [511:0] sensor_snapshot={3'd0,probe_complete,cs_swapped,MT6701_A_R,MT6701_A,SPI_MISO,probe_cs2,probe_cs1,gyro_id,accel_id,diag_actual,diag_expected,diag_operation,
        er_frame,6'd0,er_error,er_ready,4'd0,er_error_code,4'd0,er_status,2'd0,er_angle,er_ticks,er_sequence,enc_frame,4'd0,enc_error,enc_ready,imu_error,imu_ready,
        4'd0,enc_error_code,4'd0,enc_status,2'd0,enc_angle,enc_ticks,enc_sequence,
        gz,gy,gx,az,ay,ax,imu_ticks,imu_sequence,ticks};
    board_rl_bridge #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.CLKS_PER_BIT(CLKS_PER_BIT),
        .RX_TIMEOUT_CLKS(RX_TIMEOUT_CLKS),.WEIGHTS_FILE(WEIGHTS_FILE),
        .BIASES_FILE(BIASES_FILE),.ELU_FILE(ELU_FILE)) bridge(
        .clk(clk),.rst_n(rst_n),.UART_RXD(UART_RXD),.UART_TXD(UART_TXD),.sensor_snapshot(sensor_snapshot));
endmodule
