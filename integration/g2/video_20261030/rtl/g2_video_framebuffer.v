// 160x120 RGB565 double buffer. Each independent bank is 19200x16.
// The capture source owns write_bank until frame_ack acknowledges frame_req.
// No reset/initialization of pixel RAM; valid is published only for a full frame.
module g2_video_framebuffer #(
    parameter integer WIDTH=160, HEIGHT=120
)(
    input wire write_clk, write_reset_n,
    input wire pixel_valid, write_bank,
    input wire [16:0] pixel_addr,
    input wire [15:0] pixel_data,
    input wire frame_req,
    input wire read_clk, read_reset_n, frame_start,
    input wire read_enable,
    input wire [14:0] read_addr,
    input wire invalidate,
    output reg frame_ack,
    output reg display_ready,
    output reg [15:0] read_data
);
    localparam integer PIXELS=WIDTH*HEIGHT;
    (* syn_ramstyle="block_ram" *) reg [15:0] bank0 [0:PIXELS-1];
    (* syn_ramstyle="block_ram" *) reg [15:0] bank1 [0:PIXELS-1];
    // A declared reset does not clear any RAM location.
    always @(posedge write_clk) begin
        if(write_reset_n && pixel_valid && pixel_addr<PIXELS) begin
            if(write_bank) bank1[pixel_addr[14:0]]<=pixel_data;
            else bank0[pixel_addr[14:0]]<=pixel_data;
        end
    end
    (* syn_preserve=1 *) reg req_meta,req_sync;
    reg read_bank;
    // One synchronous RAM read; callers delay coordinates/DE by the same cycle.
    always @(posedge read_clk) begin
        if(read_enable && read_addr<PIXELS) begin
            if(read_bank) read_data<=bank1[read_addr];
            else read_data<=bank0[read_addr];
        end
    end
    always @(posedge read_clk or negedge read_reset_n) begin
        if(!read_reset_n) begin
            req_meta<=0;req_sync<=0;read_bank<=1;frame_ack<=0;display_ready<=0;
        end else begin
            req_meta<=frame_req;req_sync<=req_meta;
            if(invalidate) display_ready<=0;
            if(frame_start && req_sync!=frame_ack) begin
                read_bank<=~read_bank;frame_ack<=req_sync;
                if(!invalidate) display_ready<=1;
            end
        end
    end
endmodule
