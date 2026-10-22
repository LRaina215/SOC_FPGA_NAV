// Chassis command: A5 5A 10 04 forward_mmps_LE yaw_mradps_LE CRC16_LE.
// Legacy length02 forward-only frames remain accepted and atomically set yaw=0.
// CRC16/CCITT-FALSE over version, length and payload. 230400 baud, 8N1 at 50MHz by default.
// A valid frame atomically updates speed; held until next frame/reset. No UART height control.
module uart_speed_rx #(
    parameter integer CLKS_PER_BIT=217,
    parameter integer FRAME_TIMEOUT_CLKS=500000 // parser only: discard an incomplete packet after10ms
)(
    input wire clk,reset_n,rx,
    output reg signed [23:0] forward_q12,yaw_q12,
    output reg command_seen,
    output reg command_valid
);
    (* syn_preserve=1 *) reg rx_meta,rx_sync;
    reg [1:0] rx_state;
    reg [15:0] rx_count;
    reg [2:0] rx_bit;
    reg [7:0] shift,byte_data;
    reg byte_valid;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin rx_meta<=1;rx_sync<=1;end
        else begin rx_meta<=rx;rx_sync<=rx_meta;end
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            rx_state<=0;rx_count<=0;rx_bit<=0;shift<=0;byte_data<=0;byte_valid<=0;
        end else begin
            byte_valid<=0;
            case(rx_state)
                0:if(!rx_sync) begin rx_count<=0;rx_state<=1;end
                1:if(rx_count==(CLKS_PER_BIT/2)-1) begin
                    rx_count<=0;
                    if(!rx_sync) begin rx_bit<=0;rx_state<=2;end else rx_state<=0;
                end else rx_count<=rx_count+1'b1;
                2:if(rx_count==CLKS_PER_BIT-1) begin
                    rx_count<=0;shift[rx_bit]<=rx_sync;
                    if(rx_bit==7) rx_state<=3;else rx_bit<=rx_bit+1'b1;
                end else rx_count<=rx_count+1'b1;
                3:if(rx_count==CLKS_PER_BIT-1) begin
                    rx_count<=0;rx_state<=0;
                    if(rx_sync) begin byte_data<=shift;byte_valid<=1;end
                end else rx_count<=rx_count+1'b1;
            endcase
        end
    end
    function [15:0] crc_byte;
        input [15:0] previous;
        input [7:0] value;
        reg [15:0] c;
        integer i;
        begin
            c=previous^{value,8'b0};
            for(i=0;i<8;i=i+1) c=c[15] ? (c<<1)^16'h1021 : (c<<1);
            crc_byte=c;
        end
    endfunction
    reg [3:0] index;
    reg extended_frame;
    reg [15:0] crc;
    reg [7:0] speed_low,yaw_low,crc_low;
    reg signed [15:0] pending_speed,pending_yaw;
    reg [31:0] frame_age;
    wire signed [31:0] scaled=$signed(pending_speed)*32'sd4096;
    wire signed [31:0] rounded=(scaled<0) ? (scaled-32'sd500) : (scaled+32'sd500);
    wire signed [31:0] converted=rounded/32'sd1000;
    wire signed [31:0] yaw_scaled=$signed(pending_yaw)*32'sd4096;
    wire signed [31:0] yaw_rounded=(yaw_scaled<0) ? (yaw_scaled-32'sd500) : (yaw_scaled+32'sd500);
    wire signed [31:0] yaw_converted=yaw_rounded/32'sd1000;
    wire values_in_range=pending_speed>=-16'sd600 && pending_speed<=16'sd600 &&
        pending_yaw>=-16'sd2900 && pending_yaw<=16'sd2900;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            index<=0;extended_frame<=0;crc<=16'hffff;speed_low<=0;yaw_low<=0;crc_low<=0;pending_speed<=0;pending_yaw<=0;
            frame_age<=0;forward_q12<=0;yaw_q12<=0;command_seen<=0;command_valid<=0;
        end else begin
            command_valid<=0;
            if(index!=0) begin
                if(frame_age==FRAME_TIMEOUT_CLKS-1) begin index<=0;frame_age<=0;end
                else frame_age<=frame_age+1'b1;
            end
            if(byte_valid) begin
                frame_age<=0;
                case(index)
                    0:if(byte_data==8'ha5) index<=1;
                    1:if(byte_data==8'h5a) index<=2;else index<=(byte_data==8'ha5) ? 1 : 0;
                    2:if(byte_data==8'h10) begin crc<=crc_byte(16'hffff,byte_data);index<=3;end
                      else index<=(byte_data==8'ha5) ? 1 : 0;
                    3:if(byte_data==8'h02 || byte_data==8'h04) begin
                        extended_frame<=(byte_data==8'h04);pending_yaw<=0;
                        crc<=crc_byte(crc,byte_data);index<=4;end
                      else index<=(byte_data==8'ha5) ? 1 : 0;
                    4:begin speed_low<=byte_data;crc<=crc_byte(crc,byte_data);index<=5;end
                    5:begin pending_speed<={byte_data,speed_low};crc<=crc_byte(crc,byte_data);index<=6;end
                    6:if(extended_frame) begin yaw_low<=byte_data;crc<=crc_byte(crc,byte_data);index<=7;end
                      else begin crc_low<=byte_data;index<=7;end
                    7:if(extended_frame) begin pending_yaw<={byte_data,yaw_low};crc<=crc_byte(crc,byte_data);index<=8;end
                      else begin
                        index<=0;
                        if({byte_data,crc_low}==crc && values_in_range) begin
                            forward_q12<=converted[23:0];yaw_q12<=0;command_seen<=1;command_valid<=1;
                        end
                      end
                    8:begin crc_low<=byte_data;index<=9;end
                    9:begin
                        index<=0;
                        if({byte_data,crc_low}==crc && values_in_range) begin
                            forward_q12<=converted[23:0];yaw_q12<=yaw_converted[23:0];command_seen<=1;command_valid<=1;
                        end
                    end
                endcase
            end
        end
    end
endmodule
