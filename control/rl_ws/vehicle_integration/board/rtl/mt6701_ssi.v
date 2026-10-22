`timescale 1ns/1ps
// MT6701 SSI：CLK空闲高，CSN低有效，下降沿采样，24位MSB优先。
// 对照SSI例程MODE2：首个下降沿为无效位，丢弃后再采集24位。
// 帧格式：角度[23:10]、磁场/按压/超速状态[9:6]、CRC6[5:0]。
// CRC覆盖前18位，X^6+X+1，初值0，无反射、无结果异或。
module mt6701_ssi #(
    parameter STARTUP_CYCLES=1_000_000,
    parameter POLL_CYCLES=50_000,       // 50MHz下约1kHz采样
    parameter CLK_HALF_CYCLES=25,      // SSI CLK为1MHz
    parameter CS_SETUP_CYCLES=50,      // CSN拉低后等待1us再发第一个下降沿
    parameter ALLOW_WEAK_FIELD=0,     // 1：弱磁仅保留status标志
    parameter IGNORE_STATUS=0        // 1：CRC正确即发布角度，状态位只记录
) (
    input wire clk,
    input wire reset_n,
    input wire SSI_DO,
    output reg SSI_CLK=1'b1,
    output reg SSI_CSN=1'b1,
    output reg ready=1'b0 /* synthesis syn_preserve = 1 */,
    output reg error=1'b0 /* synthesis syn_preserve = 1 */,
    output reg data_valid=1'b0 /* synthesis syn_preserve = 1 */,
    output reg [13:0] angle_raw=14'd0 /* synthesis syn_preserve = 1 */,
    output reg [3:0] status_raw=4'd0 /* synthesis syn_preserve = 1 */,
    output reg [23:0] frame_raw=24'd0 /* synthesis syn_preserve = 1 */,
    output reg [3:0] error_code=4'd0 /* synthesis syn_preserve = 1 */
);
    localparam [2:0] WAIT=0, SETUP=1, LOW=2, HIGH=3, HOLD=4, CHECK=5;
    reg [2:0] state=WAIT;
    reg [31:0] startup_cnt=0, poll_cnt=0, edge_cnt=0;
    reg [4:0] bit_index=5'd24;
    reg [23:0] shift_data=24'd0;
    reg do_sync_0=1'b1, do_sync_1=1'b1;
    function [5:0] crc6;
        input [17:0] payload;
        reg [5:0] crc;
        reg feedback;
        integer i;
        begin
            crc=6'd0;
            for(i=17;i>=0;i=i-1) begin
                feedback=crc[5]^payload[i];
                crc={crc[4:0],1'b0};
                if(feedback) crc=crc^6'h03;
            end
            crc6=crc;
        end
    endfunction
    always @(posedge clk) begin
        if(!reset_n) begin do_sync_0<=1'b1; do_sync_1<=1'b1; end
        else begin do_sync_0<=SSI_DO; do_sync_1<=do_sync_0; end
    end
    always @(posedge clk) begin
        data_valid<=1'b0;
        if(!reset_n) begin
            state<=WAIT; startup_cnt<=STARTUP_CYCLES-1; poll_cnt<=0; edge_cnt<=0;
            SSI_CLK<=1; SSI_CSN<=1; bit_index<=24; shift_data<=0;
            ready<=0; error<=0; error_code<=0; angle_raw<=0; status_raw<=0; frame_raw<=0;
        end else begin
            if(poll_cnt!=0) poll_cnt<=poll_cnt-32'd1;
            case(state)
                WAIT: begin
                    SSI_CLK<=1; SSI_CSN<=1;
                    if(startup_cnt!=0) startup_cnt<=startup_cnt-32'd1;
                    else if(poll_cnt==0) begin
                        SSI_CSN<=0; edge_cnt<=0; shift_data<=0; bit_index<=24;
                        poll_cnt<=POLL_CYCLES-1; state<=SETUP;
                    end
                end
                SETUP: begin
                    if(edge_cnt==CS_SETUP_CYCLES-1) begin
                        // 尚无发送上升沿，此处不采样；下一上升沿才发送D13。
                        SSI_CLK<=0;
                        edge_cnt<=0; state<=LOW;
                    end else edge_cnt<=edge_cnt+32'd1;
                end
                LOW: begin
                    if(edge_cnt==CLK_HALF_CYCLES-1) begin
                        SSI_CLK<=1; edge_cnt<=0;
                        state<=bit_index==0 ? HOLD : HIGH;
                    end else edge_cnt<=edge_cnt+32'd1;
                end
                HIGH: begin
                    if(edge_cnt==CLK_HALF_CYCLES-1) begin
                        SSI_CLK<=0; shift_data<={shift_data[22:0],do_sync_1};
                        bit_index<=bit_index-5'd1; edge_cnt<=0; state<=LOW;
                    end else edge_cnt<=edge_cnt+32'd1;
                end
                HOLD: begin
                    if(edge_cnt==CLK_HALF_CYCLES-1) begin
                        SSI_CSN<=1; edge_cnt<=0; state<=CHECK;
                    end else edge_cnt<=edge_cnt+32'd1;
                end
                CHECK: begin
                    frame_raw<=shift_data; state<=WAIT;
                    if(crc6(shift_data[23:6])!=shift_data[5:0]) begin
                        ready<=0; error<=1; error_code<=1; // CRC错误，不发布角度
                    end else begin
                        angle_raw<=shift_data[23:10]; status_raw<=shift_data[9:6];
                        data_valid<=1; // CRC正确；ready表示按配置允许用于电机反馈
                        if(IGNORE_STATUS) begin ready<=1; error<=0; error_code<=0; end
                        else if(shift_data[7:6]!=0 && !(ALLOW_WEAK_FIELD && shift_data[7:6]==2)) begin
                            ready<=0; error<=1;
                            case(shift_data[7:6])
                                1:error_code<=2; // 磁场过强
                                2:error_code<=3; // 磁场过弱
                                default:error_code<=4; // 磁场状态保留值
                            endcase
                        end else if(shift_data[9]) begin
                            ready<=0; error<=1; error_code<=5; // 超速标志
                        end else begin ready<=1; error<=0; error_code<=0; end
                    end
                end
                default: begin
                    state<=WAIT; SSI_CSN<=1; SSI_CLK<=1;
                    ready<=0; error<=1; error_code<=6;
                end
            endcase
        end
    end
endmodule
