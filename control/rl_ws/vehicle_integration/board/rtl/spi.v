`timescale 1ns/1ps
// SPI主机：模式0，MSB先发送，50 MHz输入时默认1 MHz SPI。
// 一次片选保持期间传输1~8字节；tx_data最高字节先发送。
module spi #(
    parameter HALF_CYCLES = 25
) (
    input wire clk,
    input wire reset_n,
    input wire start,
    input wire [63:0] tx_data,
    input wire [3:0] byte_count,
    input wire MISO,
    output reg SCLK = 1'b0,
    output reg MOSI = 1'b0,
    output reg CS_N = 1'b1,
    output reg busy = 1'b0,
    output reg done = 1'b0,
    output reg [63:0] rx_data = 64'd0
);
    localparam [1:0] IDLE = 2'd0, TRANSFER = 2'd1, HOLD = 2'd2;
    reg [1:0] state = IDLE;
    reg [15:0] half_cnt = 16'd0;
    reg [6:0] bits_left = 7'd0;
    reg [63:0] tx_shift = 64'd0;
    // 在低速SPI下两级同步MISO，有充分的半周期建立时间。
    reg miso_sync_0 = 1'b0;
    reg miso_sync_1 = 1'b0;
    always @(posedge clk) begin
        if (!reset_n) begin
            miso_sync_0 <= 1'b0;
            miso_sync_1 <= 1'b0;
        end else begin
            miso_sync_0 <= MISO;
            miso_sync_1 <= miso_sync_0;
        end
    end
    always @(posedge clk) begin
        done <= 1'b0;
        if (!reset_n) begin
            state <= IDLE;
            half_cnt <= 16'd0;
            bits_left <= 7'd0;
            tx_shift <= 64'd0;
            rx_data <= 64'd0;
            SCLK <= 1'b0;
            MOSI <= 1'b0;
            CS_N <= 1'b1;
            busy <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    if (start && byte_count >= 1 && byte_count <= 8) begin
                        tx_shift <= tx_data;
                        MOSI <= tx_data[63];
                        rx_data <= 64'd0;
                        bits_left <= {byte_count, 3'b000};
                        half_cnt <= 16'd0;
                        CS_N <= 1'b0;
                        busy <= 1'b1;
                        state <= TRANSFER;
                    end
                end
                TRANSFER: begin
                    if (half_cnt == HALF_CYCLES - 1) begin
                        half_cnt <= 16'd0;
                        if (!SCLK) begin
                            SCLK <= 1'b1;
                            rx_data <= {rx_data[62:0], miso_sync_1};
                        end else begin
                            SCLK <= 1'b0;
                            if (bits_left == 7'd1)
                                state <= HOLD;
                            else begin
                                bits_left <= bits_left - 7'd1;
                                tx_shift <= {tx_shift[62:0], 1'b0};
                                MOSI <= tx_shift[62];
                            end
                        end
                    end else
                        half_cnt <= half_cnt + 16'd1;
                end
                HOLD: begin
                    // 最后一个下降沿后再等待半周期，满足片选保持时间。
                    if (half_cnt == HALF_CYCLES - 1) begin
                        CS_N <= 1'b1;
                        MOSI <= 1'b0;
                        busy <= 1'b0;
                        done <= 1'b1;
                        state <= IDLE;
                    end else
                        half_cnt <= half_cnt + 16'd1;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
