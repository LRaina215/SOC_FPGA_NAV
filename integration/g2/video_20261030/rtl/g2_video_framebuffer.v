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
    // Explicit 1024x16 slices prevent deep-array mapping to 32768x1 planes.
    // Nineteen slices per frame, including a partially used final slice.
    localparam integer SLICES=(PIXELS+1023)/1024;
    wire [15:0] q0[0:SLICES-1],q1[0:SLICES-1];
    reg [4:0] read_slice;
    reg selected_bank;
    reg read_bank;
    genvar slice;
    generate for(slice=0;slice<SLICES;slice=slice+1)begin:segments
        (* syn_ramstyle="block_ram" *) reg [15:0] bank0[0:1023];
        (* syn_ramstyle="block_ram" *) reg [15:0] bank1[0:1023];
        reg [15:0] data0,data1;
        always @(posedge write_clk)begin
            if(write_reset_n && pixel_valid && pixel_addr<PIXELS && pixel_addr[16:10]==slice)begin
                if(write_bank)bank1[pixel_addr[9:0]]<=pixel_data;
                else bank0[pixel_addr[9:0]]<=pixel_data;
            end
        end
        always @(posedge read_clk)begin
            if(read_enable && read_addr<PIXELS && read_addr[14:10]==slice)begin
                if(read_bank)data1<=bank1[read_addr[9:0]];
                else data0<=bank0[read_addr[9:0]];
            end
        end
        assign q0[slice]=data0;assign q1[slice]=data1;
    end endgenerate
    (* syn_preserve=1 *) reg req_meta,req_sync;
    // One synchronous RAM read; selection is registered on that same edge.
    always @(posedge read_clk) begin
        if(read_enable && read_addr<PIXELS) begin
            read_slice<=read_addr[14:10];selected_bank<=read_bank;
        end
    end
    always @* read_data=selected_bank ? q1[read_slice] : q0[read_slice];
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
