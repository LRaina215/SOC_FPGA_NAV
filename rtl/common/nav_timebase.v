// 全局微秒时基：CLK_HZ 必须是 1 MHz 的整数倍。
// time_us 是整个导航子系统的时间戳来源（报文到达、点云帧、底盘回传）。
module nav_timebase #(
    parameter integer CLK_HZ = 100000000,
    parameter integer TW     = 32
) (
    input  wire          clk,
    input  wire          rst,
    output reg  [TW-1:0] time_us,
    output reg           tick_us
);
    localparam integer DIV = CLK_HZ / 1000000;
    localparam integer CW  = (DIV > 1) ? $clog2(DIV) : 1;
    reg [CW-1:0] c;
    always @(posedge clk) begin
        if (rst) begin
            c       <= 0;
            time_us <= 0;
            tick_us <= 1'b0;
        end else if (c == DIV - 1) begin
            c       <= 0;
            time_us <= time_us + 1'b1;
            tick_us <= 1'b1;
        end else begin
            c       <= c + 1'b1;
            tick_us <= 1'b0;
        end
    end
endmodule
