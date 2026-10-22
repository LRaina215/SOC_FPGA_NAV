// Default V2:100Hz,230400 8N1;120-byte LE payload,CRC16 LE.
// V1 remains selectable with VERSION=1,PAYLOAD_BYTES=100,FRAME_HZ=50,CLKS_PER_BIT=434.
// CRC16/CCITT-FALSE covers version, length and payload (not A5 5A).
module uart_telemetry #(
    parameter integer CLOCK_FREQ_HZ=50_000_000,
    parameter integer VERSION=2,
    parameter integer PAYLOAD_BYTES=120,
    parameter integer FRAME_HZ=100,
    parameter integer CLKS_PER_BIT=217
)(
    input wire clk,reset_n,
    input wire [PAYLOAD_BYTES*8-1:0] payload,
    output wire tx
);
    localparam integer FRAME_CLKS=CLOCK_FREQ_HZ/FRAME_HZ;
    localparam integer MS_CLKS=CLOCK_FREQ_HZ/1000;
    reg [31:0] frame_count,ms_count,timestamp_ms;
    reg [15:0] packet_sequence;
    reg [PAYLOAD_BYTES*8-1:0] snapshot;
    reg active;
    reg [6:0] index;
    reg [15:0] crc;
    reg [7:0] data;
    wire ready;
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
    always @* begin
        case(index)
            0:data=8'ha5;
            1:data=8'h5a;
            2:data=VERSION;
            3:data=PAYLOAD_BYTES;
            PAYLOAD_BYTES+4:data=crc[7:0];
            PAYLOAD_BYTES+5:data=crc[15:8];
            default:data=snapshot[7:0];
        endcase
    end
    board_uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) transmitter(
        .clk(clk),.rst_n(reset_n),.data(data),.valid(active),
        .ready(ready),.txd(tx),.busy());
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            frame_count<=0;ms_count<=0;timestamp_ms<=0;packet_sequence<=0;
            snapshot<=0;active<=0;index<=0;crc<=16'hffff;
        end else begin
            if(ms_count==MS_CLKS-1) begin ms_count<=0;timestamp_ms<=timestamp_ms+1'b1;end
            else ms_count<=ms_count+1'b1;
            if(frame_count==FRAME_CLKS-1) begin
                frame_count<=0;
                if(!active && ready) begin
                    snapshot<={payload[PAYLOAD_BYTES*8-1:48],packet_sequence,timestamp_ms};
                    packet_sequence<=packet_sequence+1'b1;active<=1;index<=0;crc<=16'hffff;
                end
            end else frame_count<=frame_count+1'b1;
            if(active && ready) begin
                if(index>=2 && index<=PAYLOAD_BYTES+3) crc<=crc_byte(crc,data);
                if(index>=4 && index<=PAYLOAD_BYTES+3) snapshot<={8'b0,snapshot[PAYLOAD_BYTES*8-1:8]};
                if(index==PAYLOAD_BYTES+5) active<=0;
                else index<=index+1'b1;
            end
        end
    end
endmodule
