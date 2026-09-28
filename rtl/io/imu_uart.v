// IMU 串口：8N1 收发，协议交给固件解析（IMU 型号/报文格式变化时无需改硬件）。
//   接收字节进 FIFO，每个"突发"（空闲超过 IDLE_US 后的第一个字节）打 FIRST 标记，
//   并把该字节起始位的到达时间压入时间戳 FIFO——这是 IMU 数据与雷达帧对齐的时间依据。
module imu_uart #(
    parameter integer FIFO_AW = 8,
    parameter integer TS_AW   = 5
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] time_us,
    input  wire        tick_us,
    input  wire [15:0] cfg_div,         // 时钟 / 波特率
    input  wire [15:0] cfg_idle_us,

    input  wire        uart_rx,
    output reg         uart_tx,

    output wire        rx_valid,
    output wire [7:0]  rx_data,
    output wire        rx_first,
    input  wire        rx_pop,
    output wire [FIFO_AW+1:0] rx_count,
    output wire        ts_valid,
    output wire [31:0] ts_data,
    input  wire        ts_pop,
    output reg  [31:0] st_ovf,
    output reg         irq,              // 单拍：突发首字节到达

    input  wire        tx_we,
    input  wire [7:0]  tx_data,
    output wire        tx_busy,
    output wire        tx_full
);
    // ---------------- 接收 ----------------
    wire rxs;
    nav_sync2 #(.W(1)) u_s (.clk(clk), .d(uart_rx), .q(rxs));
    reg  [15:0] bc;
    reg  [3:0]  bit_i;
    reg         busy_r;
    reg  [7:0]  sh;
    reg  [31:0] t_start;
    reg  [15:0] idle;
    reg         first_pend;
    reg         push;
    reg  [8:0]  push_d;
    reg         tpush;
    reg  [31:0] tpush_d;
    wire        rf_ready;

    always @(posedge clk) begin
        push <= 1'b0; tpush <= 1'b0; irq <= 1'b0;
        if (rst) begin
            busy_r <= 1'b0; idle <= 16'hFFFF; st_ovf <= 0;
        end else begin
            if (tick_us && !busy_r && idle != 16'hFFFF) idle <= idle + 1'b1;
            if (!busy_r) begin
                if (!rxs) begin                          // 起始位
                    busy_r <= 1'b1; bc <= {1'b0, cfg_div[15:1]}; bit_i <= 0;
                    t_start <= time_us;
                    first_pend <= (idle >= cfg_idle_us);
                end
            end else begin
                if (bc == 0) begin
                    bc <= cfg_div - 1'b1;
                    if (bit_i == 0) begin
                        if (rxs) busy_r <= 1'b0;        // 起始位毛刺
                        bit_i <= 1;
                    end else if (bit_i <= 8) begin
                        sh <= {rxs, sh[7:1]};
                        bit_i <= bit_i + 1'b1;
                    end else begin                       // 停止位
                        busy_r <= 1'b0;
                        idle <= 0;
                        if (rxs) begin
                            push <= 1'b1; push_d <= {first_pend, sh};
                            if (first_pend) begin tpush <= 1'b1; tpush_d <= t_start; irq <= 1'b1; end
                        end
                    end
                end else bc <= bc - 1'b1;
            end
            if (push && !rf_ready) st_ovf <= st_ovf + 1'b1;
        end
    end

    nav_fifo #(.DW(9), .AW(FIFO_AW)) u_rf (
        .clk(clk), .rst(rst), .s_valid(push), .s_data(push_d), .s_ready(rf_ready),
        .m_valid(rx_valid), .m_data({rx_first, rx_data}), .m_ready(rx_pop), .level(rx_count), .free()
    );
    nav_fifo #(.DW(32), .AW(TS_AW)) u_tf (
        .clk(clk), .rst(rst), .s_valid(tpush), .s_data(tpush_d), .s_ready(),
        .m_valid(ts_valid), .m_data(ts_data), .m_ready(ts_pop), .level(), .free()
    );

    // ---------------- 发送 ----------------
    wire       tf_v;
    wire [7:0] tf_d;
    reg        tx_act;
    reg [15:0] tbc;
    reg [3:0]  tbit;
    reg [9:0]  tsh;
    wire       tf_pop = tf_v & ~tx_act;
    wire       tf_sr;
    assign tx_full = ~tf_sr;
    assign tx_busy = tx_act | tf_v;
    nav_fifo #(.DW(8), .AW(4)) u_txf (
        .clk(clk), .rst(rst), .s_valid(tx_we), .s_data(tx_data), .s_ready(tf_sr),
        .m_valid(tf_v), .m_data(tf_d), .m_ready(tf_pop), .level(), .free()
    );
    always @(posedge clk) begin
        if (rst) begin
            tx_act <= 1'b0; uart_tx <= 1'b1;
        end else if (!tx_act) begin
            uart_tx <= 1'b1;
            if (tf_v) begin
                tx_act <= 1'b1; tsh <= {1'b1, tf_d, 1'b0}; tbit <= 0; tbc <= 0;
            end
        end else begin
            if (tbc == 0) begin
                tbc <= cfg_div - 1'b1;
                uart_tx <= tsh[0];
                tsh <= {1'b1, tsh[9:1]};
                if (tbit == 10) tx_act <= 1'b0;
                tbit <= tbit + 1'b1;
            end else tbc <= tbc - 1'b1;
        end
    end
endmodule
