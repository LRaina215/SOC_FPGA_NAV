// Direct-mapped, write-through CPU memory candidate. 128 KiB ADDRESS SPACE
// remains in DDR; default cache holds 4 KiB. Every write reaches the native
// controller before ACK. Warm CPU reset must not reset this backend.
module nav_ddr_cache #(
    parameter integer RAM_WORDS = 32768,
    parameter integer INDEX_BITS = 7,
    parameter integer IMAGE_WORDS = 12928,
    parameter INIT_PREFIX = "boot_b"
) (
    input wire clk, cold_rst,
    input wire req,
    input wire [14:0] word_addr,
    input wire [31:0] wdata,
    input wire [3:0] wstrb,
    output reg ack,
    output reg [31:0] rdata,
    output reg initialized,
    output reg line_req, line_we,
    output reg [24:0] line_addr,
    output reg [255:0] line_wdata,
    output reg [31:0] line_mask,
    input wire line_ack,
    input wire [255:0] line_rdata
);
    localparam integer LINES = 1 << INDEX_BITS;
    localparam integer TAG_BITS = 12 - INDEX_BITS;
    // Ordinary synchronous write/hold mode avoids single-port read-first
    // mapping. Each cache word is written as a complete 32-bit word.
    (* syn_ramstyle = "block_ram" *) reg [31:0] data [0:LINES*8-1];
    reg [TAG_BITS-1:0] tags [0:LINES-1];
    reg [LINES-1:0] valid;
    wire [31:0] image_q;
    reg [31:0] data_q;
    reg [14:0] image_addr;
    reg [INDEX_BITS+2:0] data_addr;
    reg data_we;
    reg [31:0] data_wdata;
    nav_ddr_boot_rom #(.IMAGE_WORDS(IMAGE_WORDS),.INIT_PREFIX(INIT_PREFIX))
        u_boot(clk,image_addr,image_q);
    always @(posedge clk) begin
        if (data_we) data[data_addr] <= data_wdata;
        else data_q <= data[data_addr];
    end
    reg [4:0] state;
    localparam BOOT_ADDR=0, BOOT_READ=1, BOOT_PACK=2, BOOT_SEND=3,
        BOOT_DROP=4, IDLE=5, LOOKUP=6, HIT=7, NATIVE=8,
        DROP=9, FILL=10, FILL_DONE=11, RESPONSE=12, CACHE_WRITE=13;
    reg [14:0] boot_word, pending_addr;
    reg [31:0] pending_data;
    reg [3:0] pending_strb;
    reg [255:0] fill_line;
    reg [2:0] fill_word;
    wire [INDEX_BITS-1:0] index = pending_addr[INDEX_BITS+2:3];
    wire [TAG_BITS-1:0] tag = pending_addr[14:INDEX_BITS+3];
    wire hit = valid[index] && tags[index] == tag;
    function [31:0] merge;
        input [31:0] old_data, new_data;
        input [3:0] be;
        integer b;
        begin
            for (b=0;b<4;b=b+1)
                merge[b*8+:8] = be[b] ? new_data[b*8+:8] : old_data[b*8+:8];
        end
    endfunction
    always @(posedge clk or posedge cold_rst) begin
        if (cold_rst) begin
            state<=BOOT_ADDR; initialized<=0; ack<=0; rdata<=0;
            line_req<=0; line_we<=0; line_addr<=0; line_wdata<=0; line_mask<=0;
            boot_word<=0; image_addr<=0; data_addr<=0; data_we<=0; data_wdata<=0;
            valid<=0; pending_addr<=0; pending_data<=0; pending_strb<=0;
            fill_line<=0; fill_word<=0;
        end else begin
            data_we <= 0;
            case(state)
                BOOT_ADDR: begin image_addr<=boot_word; state<=BOOT_READ; end
                BOOT_READ: state<=BOOT_PACK;
                BOOT_PACK: begin
                    line_wdata[boot_word[2:0]*32+:32] <= image_q;
                    if (boot_word[2:0] == 7) state<=BOOT_SEND;
                    else begin boot_word<=boot_word+1'b1; state<=BOOT_ADDR; end
                end
                BOOT_SEND: begin
                    line_we<=1; line_addr<={13'd0,boot_word[14:3]};
                    line_mask<=0; line_req<=1;
                    if(line_ack) begin line_req<=0; state<=BOOT_DROP; end
                end
                BOOT_DROP: if(!line_ack) begin
                    if(boot_word == RAM_WORDS-1) begin initialized<=1; state<=IDLE; end
                    else begin boot_word<=boot_word+1'b1; state<=BOOT_ADDR; end
                end
                IDLE: if(req) begin
                    pending_addr<=word_addr; pending_data<=wdata; pending_strb<=wstrb;
                    data_addr<=word_addr[INDEX_BITS+2:0]; state<=LOOKUP;
                end
                LOOKUP: state<=HIT;
                HIT: begin
                    if(pending_strb == 0 && hit) begin rdata<=data_q; ack<=1; state<=RESPONSE; end
                    else begin
                        line_addr<={13'd0,pending_addr[14:3]};
                        line_we<=pending_strb!=0; line_wdata<=0; line_mask<=32'hffffffff;
                        line_wdata[pending_addr[2:0]*32+:32]<=pending_data;
                        line_mask[pending_addr[2:0]*4+:4]<=~pending_strb;
                        line_req<=1; state<=NATIVE;
                    end
                end
                NATIVE: if(line_ack) begin
                    line_req<=0;
                    if(pending_strb!=0) begin
                        if(hit) begin
                            data_wdata<=merge(data_q,pending_data,pending_strb);
                            data_we<=1;
                        end
                        state<=CACHE_WRITE;
                    end else begin fill_line<=line_rdata; fill_word<=0; state<=DROP; end
                end
                CACHE_WRITE: if(!line_ack) begin ack<=1; state<=RESPONSE; end
                DROP: if(!line_ack) state<=FILL;
                FILL: begin
                    data_addr<={index,fill_word}; data_wdata<=fill_line[fill_word*32+:32]; data_we<=1;
                    if(fill_word==7) state<=FILL_DONE;
                    else fill_word<=fill_word+1'b1;
                end
                FILL_DONE: begin
                    valid[index]<=1; tags[index]<=tag;
                    rdata<=fill_line[pending_addr[2:0]*32+:32]; ack<=1; state<=RESPONSE;
                end
                RESPONSE: if(!req) begin ack<=0; state<=IDLE; end
                default: state<=BOOT_ADDR;
            endcase
        end
    end
endmodule
