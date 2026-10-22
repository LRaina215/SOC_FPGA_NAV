`timescale 1ns/1ps
// BMI088：SPI四线模式；CSB1加速度计，CSB2陀螺仪。
// Bosch BST-BMI088-DS001：加速度计读地址后有1字节dummy，陀螺仪没有。
// 上电先用一次无效读切换加速度计到SPI，再检查ID和配置回读。
// 输出为16位有符号原始值：加速度±3g，10920 LSB/g；陀螺±2000度/s，16.384 LSB/(度/s)。
// 定时轮询最新数据，六轴连续读取，不保证加速度与陀螺仪采样时刻完全同步。
module bmi088 #(
    parameter STARTUP_CYCLES = 1_500_000, // 50 MHz下30 ms
    parameter WRITE_WAIT_CYCLES = 50_000,// 写寄存器后等待1 ms
    parameter GYRO_WAIT_CYCLES = 1_500_000,
    parameter POLL_CYCLES = 500_000,     // 每10 ms开始一次采集
    parameter SPI_HALF_CYCLES = 25
) (
    input wire clk,
    input wire reset_n,
    input wire SPI_MISO,
    output wire SPI_SCLK,
    output wire SPI_MOSI,
    output wire SPI_CSB1,
    output wire SPI_CSB2,
    output reg [7:0] diag_operation=0,
    output reg [7:0] diag_actual=0,
    output reg [7:0] diag_expected=0,
    output reg [7:0] accel_id=0,
    output reg [7:0] gyro_id=0,
    output reg cs_swapped=0,
    output reg [7:0] probe_cs1=0,
    output reg [7:0] probe_cs2=0,
    output reg probe_complete=0,
    output reg init_done = 1'b0 /* synthesis syn_preserve = 1 */,
    output reg init_error = 1'b0 /* synthesis syn_preserve = 1 */,
    output reg data_valid = 1'b0 /* synthesis syn_preserve = 1 */,
    output reg signed [15:0] accel_x = 16'd0 /* synthesis syn_preserve = 1 */,
    output reg signed [15:0] accel_y = 16'd0 /* synthesis syn_preserve = 1 */,
    output reg signed [15:0] accel_z = 16'd0 /* synthesis syn_preserve = 1 */,
    output reg signed [15:0] gyro_x = 16'd0 /* synthesis syn_preserve = 1 */,
    output reg signed [15:0] gyro_y = 16'd0 /* synthesis syn_preserve = 1 */,
    output reg signed [15:0] gyro_z = 16'd0 /* synthesis syn_preserve = 1 */
);
    localparam [1:0] WAIT = 2'd0, ISSUE = 2'd1, RECEIVE = 2'd2, ERROR = 2'd3;
    reg [1:0] state = WAIT;
    reg [4:0] operation = 5'd0;
    reg [2:0] probe_state=0;
    reg [31:0] wait_cnt = 32'd0;
    reg [31:0] poll_cnt = 32'd0;
    reg start = 1'b0;
    reg target_gyro = 1'b0;
    reg [63:0] tx_data = 64'd0;
    reg [3:0] byte_count = 4'd0;
    wire [63:0] rx_data;
    wire cs_n, spi_done;
    reg [47:0] accel_pending = 48'd0;
    reg [47:0] gyro_pending = 48'd0;
    wire selected_gyro = target_gyro ^ cs_swapped;
    assign SPI_CSB1 = selected_gyro ? 1'b1 : cs_n;
    assign SPI_CSB2 = selected_gyro ? cs_n : 1'b1;
    spi #(.HALF_CYCLES(SPI_HALF_CYCLES)) u_spi (
        .clk(clk), .reset_n(reset_n), .start(start),
        .tx_data(tx_data), .byte_count(byte_count), .MISO(SPI_MISO),
        .SCLK(SPI_SCLK), .MOSI(SPI_MOSI), .CS_N(cs_n),
        .busy(), .done(spi_done), .rx_data(rx_data)
    );
    // 每项{目标,地址,数值}：目标0为加速度，1为陀螺。
    function [16:0] config_item;
        input [4:0] index;
        begin
            case (index)
                5'd3: config_item = {1'b0, 8'h7d, 8'h04}; // 加速度开启
                5'd4: config_item = {1'b0, 8'h7c, 8'h00}; // 正常模式
                5'd5: config_item = {1'b0, 8'h40, 8'ha8}; // 100 Hz，正常滤波
                5'd6: config_item = {1'b0, 8'h41, 8'h00}; // ±3 g
                5'd7: config_item = {1'b1, 8'h11, 8'h00}; // 陀螺正常模式，随后等30 ms
                5'd8: config_item = {1'b1, 8'h0f, 8'h00}; // ±2000度/s
                5'd9: config_item = {1'b1, 8'h10, 8'h07}; // 100 Hz，32 Hz带宽
                default: config_item = 17'd0;
            endcase
        end
    endfunction
    wire [16:0] write_item = config_item(operation);
    wire [16:0] verify_item = config_item(operation - 5'd7);
    wire [7:0] verify_mask = operation == 5'd16 ? 8'h7f : 8'hff;
    always @(posedge clk) begin
        start <= 1'b0;
        data_valid <= 1'b0;
        if (!reset_n) begin
            probe_state<=0;probe_cs1<=0;probe_cs2<=0;probe_complete<=0;
            cs_swapped<=0; diag_operation<=0; diag_actual<=0; diag_expected<=0; accel_id<=0; gyro_id<=0;
            init_done <= 1'b0;
            init_error <= 1'b0;
            operation <= 5'd0;
            wait_cnt <= STARTUP_CYCLES - 1;
            poll_cnt <= 32'd0;
            state <= WAIT;
            target_gyro <= 1'b0;
            tx_data <= 64'd0;
            byte_count <= 4'd0;
            accel_pending <= 48'd0;
            gyro_pending <= 48'd0;
            accel_x <= 16'd0; accel_y <= 16'd0; accel_z <= 16'd0;
            gyro_x <= 16'd0; gyro_y <= 16'd0; gyro_z <= 16'd0;
        end else begin
            if (poll_cnt != 0)
                poll_cnt <= poll_cnt - 32'd1;
            case (state)
                WAIT: begin
                    if (wait_cnt != 0)
                        wait_cnt <= wait_cnt - 32'd1;
                    else if (operation != 5'd17 || poll_cnt == 0)
                        state <= ISSUE;
                end
                ISSUE: begin
                    start <= 1'b1;
                    state <= RECEIVE;
                    if (operation <= 5'd2 || operation >= 5'd19) begin
                        // 0为切换SPI的dummy读；1/19检查加速度ID；2/20检查陀螺ID。
                        target_gyro <= operation == 5'd2 || operation == 5'd20;
                        tx_data <= {8'h80, 56'd0};
                        byte_count <= (operation == 5'd2 || operation == 5'd20) ? 4'd2 : 4'd3;
                    end else if (operation <= 5'd9) begin
                        target_gyro <= write_item[16];
                        tx_data <= {write_item[15:0], 48'd0};
                        byte_count <= 4'd2;
                    end else if (operation <= 5'd16) begin
                        target_gyro <= verify_item[16];
                        tx_data <= {1'b1, verify_item[14:8], 56'd0};
                        byte_count <= verify_item[16] ? 4'd2 : 4'd3;
                    end else if (operation == 5'd17) begin
                        target_gyro <= 1'b0;
                        tx_data <= {8'h92, 56'd0}; // 0x12起，地址+dummy+六字节
                        byte_count <= 4'd8;
                        poll_cnt <= POLL_CYCLES - 1;
                    end else begin
                        target_gyro <= 1'b1;
                        tx_data <= {8'h82, 56'd0}; // 0x02起，地址+六字节
                        byte_count <= 4'd7;
                    end
                end
                RECEIVE: begin
                    if (spi_done) begin
                        diag_operation<={3'd0,operation};diag_actual<=rx_data[7:0];
                        diag_expected<=operation==1 || operation==19 ? 8'h1e :
                            (operation==2 || operation==20 ? 8'h0f : verify_item[7:0]);
                        if(operation==1 || operation==19) accel_id<=rx_data[7:0];
                        if(operation==2 || operation==20) gyro_id<=rx_data[7:0];
                        if (((operation == 5'd1 || operation == 5'd19) && rx_data[7:0] != 8'h1e) ||
                            ((operation == 5'd2 || operation == 5'd20) && rx_data[7:0] != 8'h0f) ||
                            (operation >= 5'd10 && operation <= 5'd16 &&
                             (rx_data[7:0] & verify_mask) != verify_item[7:0])) begin
                            // Retry only the two documented CS lines once. No data is
                            // published until both IDs and all config readbacks pass.
                            if ((operation == 1 || operation == 2) && !cs_swapped) begin
                                cs_swapped <= 1; operation <= 0;
                                accel_id <= 0; gyro_id <= 0;
                                wait_cnt <= WRITE_WAIT_CYCLES - 1; state <= WAIT;
                            end else begin
                                init_done <= 1'b0; init_error <= 1'b1; state <= ERROR;
                            end
                        end else begin
                            operation <= operation + 5'd1;
                            state <= WAIT;
                            wait_cnt <= WRITE_WAIT_CYCLES - 1;
                            if (operation == 5'd7)
                                wait_cnt <= GYRO_WAIT_CYCLES - 1;
                            if (operation == 5'd16) begin
                                init_done <= 1'b1;
                                // 配置完成后再等陀螺稳定，随后进入采集循环。
                                wait_cnt <= GYRO_WAIT_CYCLES - 1;
                            end
                            if (operation == 5'd17)
                                accel_pending <= rx_data[47:0];
                            if (operation == 5'd18)
                                gyro_pending <= rx_data[47:0];
                            if (operation >= 5'd17)
                                wait_cnt <= 32'd0;
                            if (operation == 5'd20) begin
                                // 连读顺序X低/X高/Y低/Y高/Z低/Z高，统一在此发布。
                                accel_x <= {accel_pending[39:32], accel_pending[47:40]};
                                accel_y <= {accel_pending[23:16], accel_pending[31:24]};
                                accel_z <= {accel_pending[7:0], accel_pending[15:8]};
                                gyro_x <= {gyro_pending[39:32], gyro_pending[47:40]};
                                gyro_y <= {gyro_pending[23:16], gyro_pending[31:24]};
                                gyro_z <= {gyro_pending[7:0], gyro_pending[15:8]};
                                data_valid <= 1'b1;
                                operation <= 5'd17;
                            end
                        end
                    end
                end
                ERROR: begin
                    // Fail closed. Probe only the gyro CHIP_ID register on each
                    // documented physical CS line, once, without configuration writes.
                    init_done <= 1'b0;
                    case(probe_state)
                        0: begin
                            target_gyro<=cs_swapped;tx_data<={8'h80,56'd0};
                            byte_count<=2;start<=1;probe_state<=1;
                        end
                        1: if(spi_done)begin probe_cs1<=rx_data[7:0];probe_state<=2;end
                        2: begin
                            target_gyro<=!cs_swapped;tx_data<={8'h80,56'd0};
                            byte_count<=2;start<=1;probe_state<=3;
                        end
                        3: if(spi_done)begin probe_cs2<=rx_data[7:0];probe_complete<=1;probe_state<=4;end
                        default: begin end
                    endcase
                end
                default: state <= ERROR;
            endcase
        end
    end
endmodule
